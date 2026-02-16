#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import CompressedImage
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

import threading
import time
import io
import os
import signal
import subprocess
import base64
import math
import glob
import json
import numpy as np
from PIL import Image
import yaml
import cv2
from pathlib import Path

from room_monitor.map_processor import MapProcessor

try:
    import shapely  # noqa: F401
    _HAS_SHAPELY = True
except Exception:
    _HAS_SHAPELY = False

WEB_PORT = 5010
IMG_WIDTH = 640
PREVIEW_W = 640
MAP_DIR = os.path.join(os.environ["HOME"], "kobuki/src/kobuki/maps")

PKG_NAME = "kobuki"
CMD_CORE_MAPPING = ["ros2", "launch", PKG_NAME, "navigation_slam_sim.launch.py"]
CMD_EXPLORE = ["ros2", "launch", PKG_NAME, "component_explore_sim.launch.py"]
CMD_PATROL_BASE = ["ros2", "launch", PKG_NAME, "navigation_sim.launch.py"]

PKG = "kobuki_control_center"

SHARE_DIR = None
WEB_DIR = None
TEMPLATES_DIR = None
STATIC_DIR = None

try:
    from ament_index_python.packages import get_package_share_directory

    SHARE_DIR = get_package_share_directory(PKG)
    WEB_DIR = os.path.join(SHARE_DIR, "web")
    TEMPLATES_DIR = os.path.join(WEB_DIR, "templates")
    STATIC_DIR = os.path.join(WEB_DIR, "static")
except Exception:
    SHARE_DIR = None

_pkg_dir = Path(__file__).resolve().parent
_fallback_web = _pkg_dir / "web"
_fallback_templates = _fallback_web / "templates"
_fallback_static = _fallback_web / "static"


def _has_required_web_files(tpl_dir: str, st_dir: str) -> bool:
    return (
        os.path.exists(os.path.join(tpl_dir, "index.html"))
        and os.path.exists(os.path.join(st_dir, "app.js"))
        and os.path.exists(os.path.join(st_dir, "styles.css"))
    )


if not (TEMPLATES_DIR and STATIC_DIR and _has_required_web_files(TEMPLATES_DIR, STATIC_DIR)):
    TEMPLATES_DIR = str(_fallback_templates)
    STATIC_DIR = str(_fallback_static)

print("SHARE_DIR     =", SHARE_DIR)
print(
    "TEMPLATES_DIR =",
    TEMPLATES_DIR,
    "index.html exists =",
    os.path.exists(os.path.join(TEMPLATES_DIR, "index.html")),
)
print(
    "STATIC_DIR    =",
    STATIC_DIR,
    "app.js exists =",
    os.path.exists(os.path.join(STATIC_DIR, "app.js")),
    "styles.css exists =",
    os.path.exists(os.path.join(STATIC_DIR, "styles.css")),
)

from fastapi import FastAPI, Request
from fastapi.responses import (
    HTMLResponse,
    JSONResponse,
    PlainTextResponse,
    StreamingResponse,
    FileResponse,
)
from fastapi.staticfiles import StaticFiles
from starlette.templating import Jinja2Templates
import uvicorn

app = FastAPI()
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
templates = Jinja2Templates(directory=TEMPLATES_DIR)


@app.get("/styles.css")
async def legacy_styles_css():
    return FileResponse(
        os.path.join(STATIC_DIR, "styles.css"),
        media_type="text/css",
    )


@app.get("/app.js")
async def legacy_app_js():
    return FileResponse(
        os.path.join(STATIC_DIR, "app.js"),
        media_type="application/javascript",
    )


@app.get("/favicon.ico")
async def favicon():
    return PlainTextResponse("", status_code=204)


node_instance = None


class RobotApp(Node):
    def __init__(self):
        super().__init__("robot_master")
        self._lock = threading.Lock()
        self.proc_core = None
        self.proc_explore = None
        self.mode = "IDLE"
        if not os.path.exists(MAP_DIR):
            os.makedirs(MAP_DIR)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, "/map", self.cb_map, qos)
        self.create_subscription(NavPath, "/plan", self.cb_plan, 10)

        self.latest_cam_frame = None
        self.cam_active = False
        cam_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            CompressedImage, "/rgbd_camera/image/compressed", self.cb_cam, cam_qos
        )

        self._vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self._init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf = Buffer()
        TransformListener(self.tf, self)

        self.b64 = None
        self.info = None
        self.seq = 0
        self.rob = None
        self.cached_path = []
        self._nav_goal_handle = None

        self.patrol_running = False
        self.patrol_points = []
        self.patrol_index = 0

    def cb_cam(self, msg):
        if not self.cam_active:
            return
        with self._lock:
            self.latest_cam_frame = bytes(msg.data)

    def start_mapping_core(self):
        self.stop_all()
        print("啟動 Mapping Core...")
        self.proc_core = subprocess.Popen(CMD_CORE_MAPPING, preexec_fn=os.setsid)
        self.mode = "MAPPING"
        with self._lock:
            self.b64 = None
            self.seq = 0
            self.cached_path = []
        time.sleep(3)
        return True

    def toggle_explore(self, mode):
        if mode == "manual":
            self.cancel_nav()
            self.cached_path = []

        if mode != "auto" and self.proc_explore:
            try:
                os.killpg(os.getpgid(self.proc_explore.pid), signal.SIGTERM)
            except Exception:
                pass
            self.proc_explore = None

        if mode == "auto":
            self.cancel_nav()
            if not self.proc_explore:
                self.proc_explore = subprocess.Popen(CMD_EXPLORE, preexec_fn=os.setsid)
        return True

    def start_patrol_core(self, map_name):
        self.stop_all()
        map_path = os.path.join(MAP_DIR, map_name + ".yaml")
        cmd = CMD_PATROL_BASE + [f"map:={map_path}"]
        self.proc_core = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.mode = "PATROL"
        with self._lock:
            self.b64 = None
            self.seq = 0
            self.cached_path = []
        return True

    def stop_all(self):
        self.cancel_nav()
        self.patrol_running = False
        self.patrol_points = []
        self.patrol_index = 0

        if self.proc_explore:
            try:
                os.killpg(os.getpgid(self.proc_explore.pid), signal.SIGTERM)
            except Exception:
                pass
            self.proc_explore = None

        if self.proc_core:
            try:
                os.killpg(os.getpgid(self.proc_core.pid), signal.SIGTERM)
            except Exception:
                pass
            self.proc_core = None

        for _ in range(3):
            self.stop_robot()
            time.sleep(0.1)

        self.mode = "IDLE"
        with self._lock:
            self.cached_path = []

    def stop_robot(self):
        self._vel.publish(Twist())

    def cancel_nav(self):
        if self._nav_goal_handle:
            try:
                self._nav_goal_handle.cancel_goal_async()
                self._nav_goal_handle = None
            except Exception:
                pass
        self.stop_robot()

    def set_initial_pose(self, rx, ry, yaw):
        if not self.info:
            return
        wx, wy = self._ratio_to_world(rx, ry)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = wx
        msg.pose.pose.position.y = wy
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance = [
            0.25 if i in [0, 7] else 0.06 if i == 35 else 0.0 for i in range(36)
        ]
        self._init_pose_pub.publish(msg)

    def navigate_to(self, rx, ry, yaw=0.0):
        if not self.info:
            return
        wx, wy = self._ratio_to_world(rx, ry)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wx
        goal.pose.pose.position.y = wy
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._nav_future = self._nav.send_goal_async(goal)
        self._nav_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        self._nav_goal_handle = goal_handle

    def start_new_patrol(self, pts):
        real_pts = []
        with self._lock:
            if not self.info:
                return
            for p in pts:
                wx, wy = self._ratio_to_world(p["rx"], p["ry"])
                real_pts.append((wx, wy, p.get("yaw", 0.0)))

        self.patrol_points = real_pts
        self.patrol_index = 0
        self.patrol_running = True
        threading.Thread(target=self._patrol_loop, daemon=True).start()

    def resume_patrol(self):
        if not self.patrol_points:
            return
        self.patrol_running = True
        threading.Thread(target=self._patrol_loop, daemon=True).start()

    def pause_patrol(self):
        self.patrol_running = False
        self.cancel_nav()

    def stop_clear_patrol(self):
        self.patrol_running = False
        self.cancel_nav()
        self.patrol_points = []
        self.patrol_index = 0

    def _patrol_loop(self):
        while self.patrol_running and self.patrol_index < len(self.patrol_points):
            x, y, yaw = self.patrol_points[self.patrol_index]
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

            self._nav.wait_for_server()
            future = self._nav.send_goal_async(goal)
            while rclpy.ok() and not future.done():
                if not self.patrol_running:
                    return
                time.sleep(0.1)

            goal_handle = future.result()
            if not goal_handle.accepted:
                self.patrol_index += 1
                continue

            self._nav_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            while rclpy.ok() and not result_future.done():
                if not self.patrol_running:
                    return
                time.sleep(0.1)

            self.patrol_index = (self.patrol_index + 1) % len(self.patrol_points)

    def _ratio_to_world(self, rx, ry):
        i = self.info
        w = i.get("w", IMG_WIDTH)
        px = rx * w
        py = ry * i["h"]
        wx = px * i["res"] + i["ox"]
        wy = i["oy"] + (i["h"] - 1 - py) * i["res"]
        return wx, wy

    def list_maps(self):
        files = glob.glob(os.path.join(MAP_DIR, "*.yaml"))
        return [os.path.splitext(os.path.basename(f))[0] for f in files]

    def save_map(self, name):
        safe_name = (
            "".join([c for c in name if c.isalpha() or c.isdigit() or c == "_"]).strip()
            or "map_def"
        )
        full_path = os.path.join(MAP_DIR, safe_name)
        subprocess.run(
            [
                "ros2",
                "run",
                "nav2_map_server",
                "map_saver_cli",
                "-f",
                full_path,
                "--free",
                "0.196",
                "--fmt",
                "pgm",
            ]
        )
        return f"已存: {safe_name}"

    def cb_map(self, msg):
        if self.mode == "PATROL" and self.b64:
            return

        w, h = msg.info.width, msg.info.height
        arr = np.array(msg.data, dtype=np.int8).reshape((h, w))
        arr = np.flipud(
            np.where(arr == 0, 255, np.where(arr == 100, 0, 128)).astype(np.uint8)
        )

        with self._lock:
            pil = Image.fromarray(arr, mode="L").convert("RGB")
            sc = IMG_WIDTH / float(pil.size[0])
            pil = pil.resize((IMG_WIDTH, int(pil.size[1] * sc)), Image.NEAREST)

            buf = io.BytesIO()
            pil.save(buf, format="JPEG")
            self.b64 = base64.b64encode(buf.getvalue()).decode("utf-8")

            self.seq += 1
            self.info = {
                "seq": self.seq,
                "res": msg.info.resolution / sc,
                "ox": msg.info.origin.position.x,
                "oy": msg.info.origin.position.y,
                "h": pil.height,
                "w": pil.width,
            }

    def cb_plan(self, msg):
        if not self.info:
            return
        tmp_path = []
        with self._lock:
            i = self.info
            for p in msg.poses[::5]:
                px = (p.pose.position.x - i["ox"]) / i["res"]
                py = i["h"] - 1 - (p.pose.position.y - i["oy"]) / i["res"]
                tmp_path.append((int(px), int(py)))
        self.cached_path = tmp_path

    def get_data(self, c_seq):
        try:
            t = self.tf.lookup_transform(
                "map", "base_footprint", rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
            self.rob = {
                "x": t.transform.translation.x,
                "y": t.transform.translation.y,
                "yaw": math.atan2(
                    2.0 * (t.transform.rotation.w * t.transform.rotation.z),
                    1.0 - 2.0 * (t.transform.rotation.z**2),
                ),
            }
        except Exception:
            pass

        ret = {"rob": self.rob, "path": self.cached_path}
        with self._lock:
            if self.info and self.seq != c_seq:
                ret["upd"] = True
                ret["img"] = self.b64
                ret["info"] = self.info
        return ret


@app.get("/", response_class=HTMLResponse)
async def idx(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})


@app.post("/sys/start_mapping")
async def sm():
    ok = node_instance.start_mapping_core()
    return PlainTextResponse("OK" if ok else "ERR")


@app.post("/sys/mapping_toggle")
async def mt(request: Request):
    d = await request.json()
    ok = node_instance.toggle_explore(d["mode"])
    return PlainTextResponse("OK" if ok else "ERR")


@app.post("/sys/stop")
async def st():
    node_instance.stop_all()
    return PlainTextResponse("OK")


@app.get("/map/list")
async def ml():
    return JSONResponse(node_instance.list_maps())


@app.post("/map/save")
async def ms(request: Request):
    d = await request.json()
    return PlainTextResponse(node_instance.save_map(d["name"]), status_code=200)


@app.get("/data")
async def dt(seq: int = -1):
    return JSONResponse(node_instance.get_data(seq))


@app.post("/sys/start_patrol")
async def sp(request: Request):
    d = await request.json()
    map_name = d["map"]
    if node_instance.start_patrol_core(map_name):
        json_path = os.path.join(MAP_DIR, map_name + "_zones.json")
        saved_zones = None
        if os.path.exists(json_path):
            try:
                with open(json_path, "r", encoding="utf-8") as f:
                    saved_zones = json.load(f)
            except Exception:
                saved_zones = None
        return JSONResponse({"status": "OK", "zones": saved_zones})
    return JSONResponse({"status": "ERR"}, status_code=500)


@app.post("/map/auto_segment")
async def auto_segment_map(request: Request):
    d = await request.json()
    map_name = d.get("map_name")
    if not map_name:
        return JSONResponse({"success": False, "error": "No map name"}, status_code=400)

    yaml_path = os.path.join(MAP_DIR, map_name + ".yaml")
    json_path = os.path.join(MAP_DIR, map_name + "_zones.json")

    processor = MapProcessor(yaml_path)
    if not processor.load_map():
        return JSONResponse({"success": False, "error": "Map not found"}, status_code=404)

    try:
        data = processor.auto_segment()
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return JSONResponse({"success": True, "data": data})
    except Exception as e:
        return JSONResponse({"success": False, "error": str(e)}, status_code=500)


@app.post("/map/rename_zone")
async def rename_zone(request: Request):
    d = await request.json()
    map_name = d.get("map_name")
    old_name = d.get("old_name")
    new_name = d.get("new_name")

    if not map_name or not old_name or not new_name:
        return JSONResponse({"success": False, "error": "Missing params"}, status_code=400)

    new_name = str(new_name).strip()
    if not new_name:
        return JSONResponse({"success": False, "error": "Empty new_name"}, status_code=400)

    json_path = os.path.join(MAP_DIR, map_name + "_zones.json")
    if not os.path.exists(json_path):
        return JSONResponse({"success": False, "error": "File not found"}, status_code=404)

    try:
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        zones = data.get("zones", {})
        if old_name not in zones:
            return JSONResponse({"success": False, "error": "Zone not found"}, status_code=404)

        if new_name in zones and new_name != old_name:
            return JSONResponse({"success": False, "error": "Zone name exists"}, status_code=409)

        zones[new_name] = zones.pop(old_name)
        data["zones"] = zones

        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

        return JSONResponse({"success": True, "data": data})

    except Exception as e:
        return JSONResponse({"success": False, "error": str(e)}, status_code=500)


@app.post("/map/split_zone")
async def split_zone(request: Request):
    from collections import deque

    d = await request.json()
    map_name = d.get("map_name")
    zone_name = d.get("zone_name")
    line = d.get("line")
    new_names = d.get("new_names")

    if not map_name or not zone_name or not line or not new_names:
        return JSONResponse({"success": False, "error": "Missing params"}, status_code=400)
    if len(line) != 2 or len(new_names) != 2:
        return JSONResponse({"success": False, "error": "Bad line/new_names"}, status_code=400)

    n1 = str(new_names[0]).strip()
    n2 = str(new_names[1]).strip()
    if not n1 or not n2:
        return JSONResponse({"success": False, "error": "Empty new zone name"}, status_code=400)
    if n1 == n2:
        return JSONResponse({"success": False, "error": "兩個新名稱不可相同"}, status_code=400)

    json_path = os.path.join(MAP_DIR, map_name + "_zones.json")
    yaml_path = os.path.join(MAP_DIR, map_name + ".yaml")
    if not os.path.exists(json_path):
        return JSONResponse({"success": False, "error": "Zones file not found"}, status_code=404)
    if not os.path.exists(yaml_path):
        return JSONResponse({"success": False, "error": "Map yaml not found"}, status_code=404)

    try:
        with open(yaml_path, "r", encoding="utf-8") as f:
            y = yaml.safe_load(f)

        res = float(y.get("resolution"))
        ox, oy, _ = y.get("origin", [0.0, 0.0, 0.0])

        img_file = y.get("image")
        if not img_file:
            return JSONResponse({"success": False, "error": "yaml missing image field"}, status_code=500)

        img_path = img_file
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(yaml_path), img_file)

        original_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if original_img is None:
            return JSONResponse({"success": False, "error": f"Map image not found: {img_path}"}, status_code=404)

        H, W = original_img.shape[:2]
        _, free = cv2.threshold(original_img, 250, 255, cv2.THRESH_BINARY)
        free_mask = (free == 255).astype(np.uint8) * 255
    except Exception as e:
        return JSONResponse({"success": False, "error": f"Load map failed: {e}"}, status_code=500)

    try:
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        zones = data.get("zones", {})
        if zone_name not in zones:
            return JSONResponse({"success": False, "error": "Zone not found"}, status_code=404)
        if n1 in zones or n2 in zones:
            return JSONResponse({"success": False, "error": "New zone name exists"}, status_code=409)

        pts_world = zones[zone_name]
        if not pts_world or len(pts_world) < 3:
            return JSONResponse({"success": False, "error": "Invalid polygon"}, status_code=400)

        def clip_int(v, lo, hi):
            return lo if v < lo else (hi if v > hi else v)

        def world_to_px(x, y):
            px = (float(x) - ox) / res
            py = (H - 1) - ((float(y) - oy) / res)
            px = int(round(px))
            py = int(round(py))
            px = clip_int(px, 0, W - 1)
            py = clip_int(py, 0, H - 1)
            return px, py

        poly_px = [list(world_to_px(x, y)) for x, y in pts_world]
        zone_mask_full = np.zeros((H, W), dtype=np.uint8)
        cv2.fillPoly(zone_mask_full, [np.array(poly_px, dtype=np.int32)], 255)
        zone_mask_full = cv2.bitwise_and(zone_mask_full, free_mask)

        if cv2.countNonZero(zone_mask_full) == 0:
            return JSONResponse(
                {"success": False, "error": "Zone mask is empty (check coords/res/origin)"},
                status_code=400,
            )

        ys, xs = np.where(zone_mask_full > 0)
        y0, y1 = int(ys.min()), int(ys.max())
        x0, x1 = int(xs.min()), int(xs.max())
        pad = 3
        y0 = max(0, y0 - pad)
        y1 = min(H - 1, y1 + pad)
        x0 = max(0, x0 - pad)
        x1 = min(W - 1, x1 + pad)

        zone_mask = zone_mask_full[y0 : y1 + 1, x0 : x1 + 1].copy()
        free_roi = free_mask[y0 : y1 + 1, x0 : x1 + 1].copy()
        h, w = zone_mask.shape[:2]

        (wx1, wy1), (wx2, wy2) = line[0], line[1]
        p1 = world_to_px(wx1, wy1)
        p2 = world_to_px(wx2, wy2)
        p1 = (p1[0] - x0, p1[1] - y0)
        p2 = (p2[0] - x0, p2[1] - y0)

        SPLIT_LINE_PX = 2
        barrier = np.zeros((h, w), dtype=np.uint8)
        cv2.line(barrier, p1, p2, 255, SPLIT_LINE_PX, lineType=cv2.LINE_8)

        cut_mask = zone_mask.copy()
        cut_mask[barrier > 0] = 0
        cut_mask = cv2.bitwise_and(cut_mask, free_roi)

        bin_img = (cut_mask > 0).astype(np.uint8)
        num, labels = cv2.connectedComponents(bin_img, connectivity=8)
        if num <= 2:
            return JSONResponse(
                {"success": False, "error": "切割失敗：切割線未把區域切成至少兩塊（請畫穿過整個房間的一刀）"},
                status_code=400,
            )

        counts = np.bincount(labels.ravel())
        comp_ids = [i for i in range(1, num) if counts[i] > 0]
        comp_ids.sort(key=lambda i: counts[i], reverse=True)
        if len(comp_ids) < 2:
            return JSONResponse({"success": False, "error": "切割失敗：有效區塊不足 2"}, status_code=400)

        A, B = comp_ids[0], comp_ids[1]
        others = comp_ids[2:]

        def centroid(lab):
            ys2, xs2 = np.where(labels == lab)
            if len(xs2) == 0:
                return None
            return (float(xs2.mean()), float(ys2.mean()))

        cA = centroid(A)
        cB = centroid(B)
        if cA is None or cB is None:
            return JSONResponse({"success": False, "error": "切割失敗：centroid 計算失敗"}, status_code=500)

        seed_group = np.zeros_like(labels, dtype=np.uint8)
        seed_group[labels == A] = 1
        seed_group[labels == B] = 2

        for lab in others:
            c = centroid(lab)
            if c is None:
                continue
            dA = (c[0] - cA[0]) ** 2 + (c[1] - cA[1]) ** 2
            dB = (c[0] - cB[0]) ** 2 + (c[1] - cB[1]) ** 2
            seed_group[labels == lab] = 1 if dA <= dB else 2

        seed1 = (seed_group == 1) & (cut_mask > 0)
        seed2 = (seed_group == 2) & (cut_mask > 0)
        if not np.any(seed1) or not np.any(seed2):
            return JSONResponse({"success": False, "error": "切割失敗：其中一側 seed 為空"}, status_code=400)

        label_map = np.zeros((h, w), dtype=np.uint8)
        q = deque()

        ys1, xs1 = np.where(seed1)
        for yv, xv in zip(ys1, xs1):
            label_map[yv, xv] = 1
            q.append((yv, xv))

        ys2, xs2 = np.where(seed2)
        for yv, xv in zip(ys2, xs2):
            label_map[yv, xv] = 2
            q.append((yv, xv))

        shifts = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

        while q:
            yv, xv = q.popleft()
            lv = label_map[yv, xv]
            for dy, dx in shifts:
                ny, nx = yv + dy, xv + dx
                if 0 <= ny < h and 0 <= nx < w:
                    if label_map[ny, nx] != 0:
                        continue
                    if zone_mask[ny, nx] == 0:
                        continue
                    if barrier[ny, nx] > 0:
                        continue
                    label_map[ny, nx] = lv
                    q.append((ny, nx))

        for _ in range(10):
            unknown = (zone_mask > 0) & (label_map == 0)
            if not np.any(unknown):
                break
            changed = False
            for dy, dx in shifts:
                rolled = np.roll(np.roll(label_map, dy, axis=0), dx, axis=1)
                fill = unknown & (rolled > 0)
                if np.any(fill):
                    label_map[fill] = rolled[fill]
                    changed = True
            if not changed:
                break

        unknown = (zone_mask > 0) & (label_map == 0)
        if np.any(unknown):
            uy, ux = np.where(unknown)
            for yv, xv in zip(uy, ux):
                dA = (xv - cA[0]) ** 2 + (yv - cA[1]) ** 2
                dB = (xv - cB[0]) ** 2 + (yv - cB[1]) ** 2
                label_map[yv, xv] = 1 if dA <= dB else 2

        m1 = (label_map == 1).astype(np.uint8) * 255
        m2 = (label_map == 2).astype(np.uint8) * 255

        m1 = cv2.bitwise_and(m1, zone_mask)
        m2 = cv2.bitwise_and(m2, zone_mask)
        m1 = cv2.bitwise_and(m1, free_roi)
        m2 = cv2.bitwise_and(m2, free_roi)

        if cv2.countNonZero(m1) == 0 or cv2.countNonZero(m2) == 0:
            return JSONResponse({"success": False, "error": "切割失敗：其中一塊為空（切割線可能太靠邊）"}, status_code=400)

        def mask_to_world_polygon(mask):
            conts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if not conts:
                return None
            c = max(conts, key=cv2.contourArea)
            if len(c) < 3:
                return None
            eps = 0.0001 * cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, eps, True)

            out = []
            for pt in approx:
                px, py = pt[0]
                px = px + x0
                py = py + y0
                wx = px * res + ox
                wy = (H - 1 - py) * res + oy
                out.append([float(f"{wx:.4f}"), float(f"{wy:.4f}")])
            return out if len(out) >= 3 else None

        p1w = mask_to_world_polygon(m1)
        p2w = mask_to_world_polygon(m2)
        if p1w is None or p2w is None:
            return JSONResponse({"success": False, "error": "切割失敗：輪廓轉換失敗"}, status_code=500)

        zones.pop(zone_name, None)
        zones[n1] = p1w
        zones[n2] = p2w
        data["zones"] = zones

        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

        return JSONResponse({"success": True, "data": data})

    except Exception as e:
        return JSONResponse({"success": False, "error": str(e)}, status_code=500)


@app.post("/map/merge_zones")
async def merge_zones(request: Request):
    if not _HAS_SHAPELY:
        return JSONResponse({"success": False, "error": "缺少 shapely，請先 pip3 install shapely"}, status_code=500)

    d = await request.json()
    map_name = d.get("map_name")
    zones_in = d.get("zones")
    new_name = (d.get("new_name") or "").strip()

    if not map_name or not zones_in or len(zones_in) != 2 or not new_name:
        return JSONResponse({"success": False, "error": "Missing params"}, status_code=400)

    z1, z2 = zones_in[0], zones_in[1]
    if z1 == z2:
        return JSONResponse({"success": False, "error": "zones must be different"}, status_code=400)

    json_path = os.path.join(MAP_DIR, map_name + "_zones.json")
    if not os.path.exists(json_path):
        return JSONResponse({"success": False, "error": "Zones file not found"}, status_code=404)

    yaml_path = os.path.join(MAP_DIR, map_name + ".yaml")
    if not os.path.exists(yaml_path):
        return JSONResponse({"success": False, "error": "Map yaml not found"}, status_code=404)

    try:
        with open(yaml_path, "r", encoding="utf-8") as f:
            y = yaml.safe_load(f)

        res = float(y.get("resolution"))
        ox, oy, _ = y.get("origin", [0.0, 0.0, 0.0])
        img_file = y.get("image")
        if not img_file:
            return JSONResponse({"success": False, "error": "yaml missing image field"}, status_code=500)

        img_path = img_file
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(yaml_path), img_file)

        original_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if original_img is None:
            return JSONResponse({"success": False, "error": f"Map image not found: {img_path}"}, status_code=404)

        h, w = original_img.shape[:2]
        _, free = cv2.threshold(original_img, 250, 255, cv2.THRESH_BINARY)
        free_mask = (free == 255).astype(np.uint8) * 255

    except Exception as e:
        return JSONResponse({"success": False, "error": f"Load map failed: {e}"}, status_code=500)

    try:
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        zones = data.get("zones", {})
        if z1 not in zones or z2 not in zones:
            return JSONResponse({"success": False, "error": "Zone not found"}, status_code=404)

        if new_name in zones and new_name not in (z1, z2):
            return JSONResponse({"success": False, "error": "New zone name exists"}, status_code=409)

        def world_to_px(x, y):
            px = (x - ox) / res
            py = (h - 1) - ((y - oy) / res)
            return int(round(px)), int(round(py))

        def poly_to_mask(pts_world):
            pts_px = []
            for x, y in pts_world:
                px, py = world_to_px(float(x), float(y))
                px = 0 if px < 0 else (w - 1 if px >= w else px)
                py = 0 if py < 0 else (h - 1 if py >= h else py)
                pts_px.append([px, py])

            if len(pts_px) < 3:
                return None

            mask = np.zeros((h, w), dtype=np.uint8)
            cv2.fillPoly(mask, [np.array(pts_px, dtype=np.int32)], 255)
            mask = cv2.bitwise_and(mask, free_mask)
            return mask

        m1 = poly_to_mask(zones[z1])
        m2 = poly_to_mask(zones[z2])
        if m1 is None or m2 is None:
            return JSONResponse({"success": False, "error": "Invalid polygon"}, status_code=400)

        k = 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1))

        d1 = cv2.dilate(m1, kernel, iterations=1)
        d2 = cv2.dilate(m2, kernel, iterations=1)

        seam = cv2.bitwise_and(d1, d2)
        seam = cv2.bitwise_and(seam, free_mask)

        if cv2.countNonZero(seam) == 0:
            touch = (
                cv2.countNonZero(cv2.bitwise_and(d1, m2)) > 0
                or cv2.countNonZero(cv2.bitwise_and(d2, m1)) > 0
            )
            if not touch:
                return JSONResponse(
                    {"success": False, "error": "兩個分區未接觸（且中間不是 1px 縫），為避免副作用拒絕合併"},
                    status_code=400,
                )

        merged_mask = cv2.bitwise_or(m1, m2)
        merged_mask = cv2.bitwise_or(merged_mask, seam)
        merged_mask = cv2.bitwise_and(merged_mask, free_mask)

        merged_mask = cv2.morphologyEx(
            merged_mask,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
            iterations=1,
        )
        merged_mask = cv2.bitwise_and(merged_mask, free_mask)

        contours, _ = cv2.findContours(merged_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return JSONResponse({"success": False, "error": "合併後找不到輪廓"}, status_code=400)

        contour = max(contours, key=cv2.contourArea)
        if len(contour) < 3:
            return JSONResponse({"success": False, "error": "合併結果輪廓點不足"}, status_code=400)

        merged_pts = []
        for pt in contour:
            px, py = pt[0]
            wx = px * res + ox
            wy = (h - 1 - py) * res + oy
            merged_pts.append([float(wx), float(wy)])

        zones.pop(z1, None)
        zones.pop(z2, None)
        zones[new_name] = merged_pts
        data["zones"] = zones

        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

        return JSONResponse({"success": True, "data": data})

    except Exception as e:
        return JSONResponse({"success": False, "error": str(e)}, status_code=500)


@app.post("/map/rename_map")
async def rename_map(request: Request):
    d = await request.json()
    old_name = d.get("old_name")
    new_name = d.get("new_name")
    if not old_name or not new_name:
        return JSONResponse({"success": False, "error": "Missing params"}, status_code=400)

    safe_new_name = "".join(
        [c for c in str(new_name) if c.isalpha() or c.isdigit() or c == "_"]
    ).strip()
    if not safe_new_name:
        return JSONResponse({"success": False, "error": "Invalid Name"}, status_code=400)

    old_yaml = os.path.join(MAP_DIR, old_name + ".yaml")
    if not os.path.exists(old_yaml):
        return JSONResponse({"success": False, "error": "Old map not found"}, status_code=404)

    new_yaml = os.path.join(MAP_DIR, safe_new_name + ".yaml")
    if os.path.exists(new_yaml):
        return JSONResponse({"success": False, "error": "New map name already exists"}, status_code=409)

    try:
        extensions = [".yaml", ".pgm", "_zones.json"]

        for ext in extensions:
            old_path = os.path.join(MAP_DIR, old_name + ext)
            new_path = os.path.join(MAP_DIR, safe_new_name + ext)

            if not os.path.exists(old_path):
                continue

            if ext == ".yaml":
                with open(old_path, "r", encoding="utf-8") as f:
                    lines = f.readlines()

                out = []
                for line in lines:
                    if line.strip().startswith("image:"):
                        out.append(f"image: {safe_new_name}.pgm\n")
                    else:
                        out.append(line)

                with open(new_path, "w", encoding="utf-8") as f:
                    f.writelines(out)

                os.remove(old_path)
            else:
                os.rename(old_path, new_path)

        node_instance.stop_all()
        return JSONResponse({"success": True, "new_name": safe_new_name})

    except Exception as e:
        return JSONResponse({"success": False, "error": str(e)}, status_code=500)


@app.post("/ctrl/move")
async def cm(request: Request):
    d = await request.json()
    m = Twist()
    m.linear.x = float(d["l"])
    m.angular.z = float(d["a"])
    node_instance._vel.publish(m)
    return PlainTextResponse("OK")


@app.post("/ctrl/navigate_to_pose")
async def cnavpose(request: Request):
    d = await request.json()
    node_instance.navigate_to(float(d["rx"]), float(d["ry"]), float(d.get("yaw", 0)))
    return PlainTextResponse("OK")


@app.post("/ctrl/set_pose")
async def csetpose(request: Request):
    d = await request.json()
    node_instance.set_initial_pose(float(d["rx"]), float(d["ry"]), float(d.get("yaw", 0)))
    return PlainTextResponse("OK")


@app.post("/ctrl/patrol_start")
async def cp_start(request: Request):
    d = await request.json()
    node_instance.start_new_patrol(d["pts"])
    return PlainTextResponse("OK")


@app.post("/ctrl/patrol_pause")
async def cp_pause():
    node_instance.pause_patrol()
    return PlainTextResponse("OK")


@app.post("/ctrl/patrol_resume")
async def cp_resume():
    node_instance.resume_patrol()
    return PlainTextResponse("OK")


@app.post("/ctrl/patrol_stop")
async def cp_stop():
    node_instance.stop_clear_patrol()
    return PlainTextResponse("OK")


def gen_cam_frames():
    while True:
        frame = None
        if node_instance:
            with node_instance._lock:
                frame = node_instance.latest_cam_frame
        if frame:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        time.sleep(0.04)


@app.get("/cam/feed")
async def cmfd():
    return StreamingResponse(
        gen_cam_frames(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.post("/cam/on")
async def cmon():
    if node_instance:
        node_instance.cam_active = True
    return PlainTextResponse("OK")


@app.post("/cam/off")
async def cmoff():
    if node_instance:
        node_instance.cam_active = False
    return PlainTextResponse("OK")


def _safe_map_name(name: str) -> str:
    return "".join([c for c in str(name) if c.isalpha() or c.isdigit() or c == "_"]).strip()


@app.post("/map/preview")
async def map_preview(request: Request):
    d = await request.json()
    map_name = d.get("name") or d.get("map_name")
    if not map_name:
        return JSONResponse({"success": False, "error": "Missing name"}, status_code=400)

    safe = _safe_map_name(map_name)
    if not safe:
        return JSONResponse({"success": False, "error": "Invalid name"}, status_code=400)

    yaml_path = os.path.join(MAP_DIR, safe + ".yaml")
    if not os.path.exists(yaml_path):
        return JSONResponse({"success": False, "error": "Map yaml not found"}, status_code=404)

    try:
        with open(yaml_path, "r", encoding="utf-8") as f:
            y = yaml.safe_load(f) or {}

        img_file = y.get("image")
        if not img_file:
            return JSONResponse({"success": False, "error": "yaml missing image field"}, status_code=500)

        img_path = img_file
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(yaml_path), img_file)

        gray = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            return JSONResponse({"success": False, "error": f"Map image not found: {img_path}"}, status_code=404)

        free = gray >= 250
        occ = gray <= 10
        out = np.full_like(gray, 128, dtype=np.uint8)
        out[free] = 255
        out[occ] = 0

        H, W = out.shape[:2]
        if W > PREVIEW_W:
            sc = PREVIEW_W / float(W)
            out = cv2.resize(out, (PREVIEW_W, int(H * sc)), interpolation=cv2.INTER_NEAREST)

        ok, jpg = cv2.imencode(".jpg", out, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            return JSONResponse({"success": False, "error": "JPEG encode failed"}, status_code=500)

        b64 = base64.b64encode(jpg.tobytes()).decode("utf-8")
        return JSONResponse({"success": True, "img": b64})

    except Exception as e:
        return JSONResponse({"success": False, "error": str(e)}, status_code=500)


def main():
    global node_instance
    rclpy.init()
    node_instance = RobotApp()
    threading.Thread(target=lambda: rclpy.spin(node_instance), daemon=True).start()

    uvicorn.run(app, host="0.0.0.0", port=WEB_PORT, log_level="info")


if __name__ == "__main__":
    main()
