import os
import cv2
import yaml
import numpy as np
from typing import Dict, List, Any

# 引入您提供的 Voronoi 模組
from .voronoi_segmentation import VoronoiSegmentation


class MapProcessor:
    def __init__(self, yaml_path: str):
        self.yaml_path = yaml_path
        self.map_data = None
        self.map_info = {}

    def load_map(self) -> bool:
        """
        讀取 ROS 地圖的 YAML 與 PGM 檔
        """
        if not os.path.exists(self.yaml_path):
            print(f"YAML not found: {self.yaml_path}")
            return False

        try:
            # 1. 解析 YAML
            with open(self.yaml_path, 'r') as f:
                yaml_data = yaml.safe_load(f)

            self.map_info['resolution'] = float(yaml_data['resolution'])
            self.map_info['origin'] = yaml_data['origin']  # [x, y, yaw]

            # 2. 處理影像路徑
            img_filename = yaml_data['image']
            map_dir = os.path.dirname(self.yaml_path)
            img_path = os.path.join(map_dir, img_filename)

            if not os.path.exists(img_path):
                print(f"Map image not found: {img_path}")
                return False

            # 3. 讀取影像 (Grayscale)
            original_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

            if original_img is None:
                return False

            self.map_info['width'] = original_img.shape[1]
            self.map_info['height'] = original_img.shape[0]

            # 4. 二值化處理
            # 確保與原始腳本邏輯一致：>250 為自由空間(255)，其餘為障礙(0)
            _, self.map_data = cv2.threshold(original_img, 250, 255, cv2.THRESH_BINARY)

            return True

        except Exception as e:
            print(f"Error loading map: {e}")
            return False

    def auto_segment(self) -> Dict[str, Any]:
        """
        執行 Voronoi 分割並回傳前端可用的資料格式
        """
        if self.map_data is None:
            raise ValueError("Map not loaded")

        # ==========================================
        # 修正：參數完全還原至原始 Python Script 的設定
        # ==========================================
        segmenter = VoronoiSegmentation(
            room_area_factor_lower_limit=0.1,        # 原始：0.1
            room_area_factor_upper_limit=1000.0,     # 原始：1000.0
            neighborhood_index=280,                  # 原始：280
            max_iterations=100,                      # 原始：100
            min_critical_point_distance_factor=1.6,  # 原始：1.6
            max_area_for_merging=12.5                # 原始：12.5
        )

        # 執行分割
        res = self.map_info['resolution']
        segmented_map = segmenter.segment_map(self.map_data, map_resolution=res, display_map=False)

        # 將分割結果轉換為座標點
        zones = self._extract_zones(segmented_map)

        return {
            "info": self.map_info,
            "zones": zones
        }

    def _extract_zones(self, segmented_map: np.ndarray) -> Dict[str, List[List[float]]]:
        """
        從分割後的 int32 地圖中提取每個房間的輪廓
        """
        zones = {}
        unique_ids = np.unique(segmented_map)

        res = self.map_info['resolution']
        ox = self.map_info['origin'][0]
        oy = self.map_info['origin'][1]
        height = self.map_info['height']

        room_count = 1

        for uid in unique_ids:
            # 0 是障礙物，> 65279 是未分配區域
            if uid == 0 or uid > 65279:
                continue

            # 建立該房間的遮罩
            mask = np.zeros_like(segmented_map, dtype=np.uint8)
            mask[segmented_map == uid] = 255

            # 尋找輪廓
            # 使用 CHAIN_APPROX_NONE 獲取所有點，不進行任何壓縮
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            for contour in contours:
                # ==========================================
                # 修正：極低程度的簡化，保留原始鋸齒狀邊緣
                # ==========================================
                # 將 epsilon 設得非常小 (0.0001)，幾乎等同於原始像素邊界
                epsilon = 0.0001 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                # 轉換座標 (Pixel -> World Meter)
                world_points = []
                for pt in approx:
                    px, py = pt[0]

                    wx = px * res + ox
                    # 再次確認 Y 軸翻轉公式：
                    # 圖像座標 (0,0) 在左上，地圖座標 (0,0) 在左下
                    wy = (height - 1 - py) * res + oy

                    # 為了精確度，保留 4 位小數
                    world_points.append([float(f"{wx:.4f}"), float(f"{wy:.4f}")])

                # 只有當輪廓點夠多才視為有效房間
                if len(world_points) > 2:
                    zone_name = f"Zone_{room_count}"
                    zones[zone_name] = world_points
                    room_count += 1

        return zones
