"""
Voronoi Segmentation for Room Segmentation
基於 Voronoi 圖的房間分割模組

本模組實現了基於 Voronoi 圖的房間分割演算法，
參考自 ipa_room_segmentation 的 C++ 實作。

演算法步驟:
1. 建立廣義 Voronoi 圖 (Generalized Voronoi Diagram, GVD)
2. 修剪 Voronoi 圖（移除短分支/毛刺）
3. 找出臨界點（沿 Voronoi 圖到障礙物距離的局部最小值，通常位於門口區域）
4. 繪製臨界線（從臨界點向最近的兩側障礙物連線，模擬門的分隔線）
5. 過濾臨界線（依角度過濾——移除牆角處的線，保留門口處的線）
6. 區域增長（Wavefront Region Growing，將分割後的區域填滿）
7. 合併房間（將過小的碎片區域合併到相鄰房間）
"""

import numpy as np
import cv2
from typing import List, Tuple, Set, Dict
from dataclasses import dataclass, field


@dataclass
class Room:
    """
    代表一個被分割出來的房間。

    Attributes:
    -----------
    room_id : int
        房間的唯一識別碼（同時也是該房間在 segmented_map 中的像素值/顏色）
    members : List[Tuple[int, int]]
        房間所包含的所有像素座標 (x, y)
    neighbor_ids : Set[int]
        相鄰房間的 ID 集合（不含牆壁 ID=0）
    neighbor_statistics : Dict[int, int]
        鄰居 ID -> 共享邊界長度的統計（包含 ID=0 代表牆壁的邊界）
    area : float
        房間面積（平方公尺）
    perimeter : int
        房間周長（邊界像素數，包括與牆壁接觸的部分）
    wall_perimeter : int
        與牆壁接觸的周長（邊界像素中鄰居為 0/障礙物的數量）
    """
    room_id: int
    members: List[Tuple[int, int]] = field(default_factory=list)
    neighbor_ids: Set[int] = field(default_factory=set)
    neighbor_statistics: Dict[int, int] = field(default_factory=dict)
    area: float = 0.0
    perimeter: int = 0
    wall_perimeter: int = 0

    def insert_member_point(self, point: Tuple[int, int], map_resolution: float):
        """
        將一個像素點加入此房間。

        Parameters:
        -----------
        point : Tuple[int, int]
            像素座標 (x, y)
        map_resolution : float
            地圖解析度（公尺/像素），用於計算實際面積
        """
        self.members.append(point)
        # 面積 = 像素數 × 每像素面積（resolution²）
        self.area = len(self.members) * map_resolution * map_resolution

    def add_neighbor(self, neighbor_id: int):
        """
        累加鄰居的邊界統計。

        每次呼叫代表在邊界上又發現一個與 neighbor_id 相鄰的像素，
        因此 perimeter 加 1。如果鄰居是牆壁（ID=0），wall_perimeter 也加 1。

        Parameters:
        -----------
        neighbor_id : int
            鄰居房間的 ID（0 代表牆壁/障礙物）
        """
        if neighbor_id in self.neighbor_statistics:
            self.neighbor_statistics[neighbor_id] += 1
        else:
            self.neighbor_statistics[neighbor_id] = 1
        self.perimeter += 1
        if neighbor_id == 0:
            self.wall_perimeter += 1

    def add_neighbor_id(self, neighbor_id: int):
        """
        將一個鄰居 ID 加入鄰居集合。

        Parameters:
        -----------
        neighbor_id : int
            鄰居房間的 ID
        """
        self.neighbor_ids.add(neighbor_id)

    def get_wall_to_perimeter_ratio(self) -> float:
        """
        取得牆壁接觸比例。

        Returns:
        --------
        float
            wall_perimeter / perimeter，即該房間周長中有多少比例是與牆壁接觸的。
            值越高代表房間越「封閉」（被牆壁包圍），
            值越低代表房間與其他房間有較多共享邊界。
        """
        if self.perimeter == 0:
            return 0.0
        return self.wall_perimeter / self.perimeter

    def get_neighbor_with_largest_common_border(self) -> int:
        """
        取得與本房間共享邊界最長的鄰居 ID。

        遍歷 neighbor_statistics，找出（排除牆壁 ID=0 後）
        共享邊界像素數最多的鄰居。

        Returns:
        --------
        int
            共享邊界最長的鄰居 room_id；若無有效鄰居則回傳 0
        """
        max_border = 0
        max_neighbor = 0
        for neighbor_id, border_len in self.neighbor_statistics.items():
            if neighbor_id != 0 and border_len > max_border:
                max_border = border_len
                max_neighbor = neighbor_id
        return max_neighbor

    def merge_room(self, other: 'Room', map_resolution: float):
        """
        將另一個房間合併到本房間中。

        合併後：
        - 本房間的 members 會包含 other 的所有成員像素
        - 面積重新計算
        - 鄰居集合取聯集，但排除自己和被合併房間的 ID

        Parameters:
        -----------
        other : Room
            要被合併的房間
        map_resolution : float
            地圖解析度
        """
        self.members.extend(other.members)
        self.area = len(self.members) * map_resolution * map_resolution
        self.neighbor_ids.update(other.neighbor_ids)
        # 排除自己和被合併的房間 ID，避免「自己是自己的鄰居」
        self.neighbor_ids.discard(self.room_id)
        self.neighbor_ids.discard(other.room_id)


class VoronoiSegmentation:
    """
    基於 Voronoi 圖的房間分割演算法。

    核心思路：
    - Voronoi 圖的骨架線（等距線）自然地穿過門口區域
    - 在骨架線上找到「到障礙物距離最小」的臨界點（通常在門口處）
    - 從臨界點向兩側最近障礙物畫線，形成房間分隔線
    - 用區域增長填滿各個分割區域
    - 最後合併過小的碎片

    Parameters:
    -----------
    room_area_factor_lower_limit : float
        最小房間面積（平方公尺），小於此值的區域不被視為房間
    room_area_factor_upper_limit : float
        最大房間面積（平方公尺），大於此值的區域不被視為房間
    neighborhood_index : int
        臨界點搜尋時的鄰域大小參數。值越大，搜尋範圍越廣，
        與 distance_map 值配合：eps = neighborhood_index / distance
    max_iterations : int
        臨界點鄰域搜尋的最大迭代次數，防止無限迴圈
    min_critical_point_distance_factor : float
        過濾過近臨界點的距離因子。若兩臨界點間距 < distance * factor，
        則只保留角度較大的那個
    max_area_for_merging : float
        合併閾值（平方公尺），面積小於此值的房間會被考慮合併
    """

    def __init__(self,
                 room_area_factor_lower_limit: float = 0.1,
                 room_area_factor_upper_limit: float = 1000.0,
                 neighborhood_index: int = 280,
                 max_iterations: int = 100,
                 min_critical_point_distance_factor: float = 1.6,
                 max_area_for_merging: float = 12.5):
        self.room_area_factor_lower_limit = room_area_factor_lower_limit
        self.room_area_factor_upper_limit = room_area_factor_upper_limit
        self.neighborhood_index = neighborhood_index
        self.max_iterations = max_iterations
        self.min_critical_point_distance_factor = min_critical_point_distance_factor
        self.max_area_for_merging = max_area_for_merging

    def segment_map(self, map_image: np.ndarray, map_resolution: float = 0.05,
                    display_map: bool = False) -> np.ndarray:
        # ===== 步驟 I: 建立 Voronoi 圖 =====
        voronoi_map = self._create_voronoi_graph(map_image.copy())

        if display_map:
            cv2.imshow("Voronoi Graph", voronoi_map)
            cv2.waitKey(1)

        # ===== 步驟 II: 修剪 Voronoi 圖並找出節點 =====
        voronoi_map, _node_points = self._prune_voronoi_graph(voronoi_map)

        if display_map:
            cv2.imshow("Pruned Voronoi", voronoi_map)
            cv2.waitKey(1)

        # ===== 步驟 III: 找出臨界點 =====
        distance_map = cv2.distanceTransform(map_image, cv2.DIST_L2, 5)
        distance_map = cv2.convertScaleAbs(distance_map)
        critical_points = self._find_critical_points(voronoi_map, distance_map)

        if display_map:
            display = map_image.copy()
            for pt in critical_points:
                cv2.circle(display, pt, 2, 128, -1)
            cv2.imshow("Critical Points", display)
            cv2.waitKey(1)

        # ===== 步驟 IV: 繪製臨界線 =====
        segmented_map = map_image.astype(np.int32) * 256
        contours, _ = cv2.findContours(map_image.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        critical_lines_map = voronoi_map.copy()
        self._draw_critical_lines(critical_lines_map, critical_points, contours, distance_map)

        if display_map:
            cv2.imshow("Critical Lines", critical_lines_map)
            cv2.waitKey(1)

        # ===== 步驟 V: 尋找並填充房間輪廓 =====
        rooms = self._find_and_fill_rooms(critical_lines_map, segmented_map, map_resolution)
        print(f"Found {len(rooms)} rooms.")

        # ===== 步驟 VI: 波前區域增長 =====
        self._wavefront_region_growing(segmented_map)

        if display_map:
            cv2.imshow("Before Merge", self._colorize_map(segmented_map))
            cv2.waitKey(1)

        # ===== 步驟 VII: 合併房間 =====
        self._merge_rooms(segmented_map, rooms, map_resolution)

        if display_map:
            cv2.imshow("After Merge", self._colorize_map(segmented_map))
            cv2.waitKey(0)

        return segmented_map

    def _create_voronoi_graph(self, map_image: np.ndarray) -> np.ndarray:
        map_to_draw = map_image.copy()
        temp_map = map_image.copy()

        temp_map = cv2.erode(temp_map, None)
        temp_map = cv2.dilate(temp_map, None)

        rect = (0, 0, map_to_draw.shape[1], map_to_draw.shape[0])
        subdiv = cv2.Subdiv2D(rect)

        contours, _hierarchy = cv2.findContours(temp_map, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(map_to_draw, contours, -1, 255, cv2.FILLED)

        for contour in contours:
            for point in contour:
                try:
                    subdiv.insert((float(point[0][0]), float(point[0][1])))
                except cv2.error:
                    pass

        (facets, _centers) = subdiv.getVoronoiFacetList([])
        eroded_map = cv2.erode(temp_map, None, iterations=2)
        self._draw_voronoi(map_to_draw, facets, 127, eroded_map)
        map_to_draw[map_image == 0] = 0

        return map_to_draw

    def _draw_voronoi(self, img: np.ndarray, facets: List, color: int, eroded_map: np.ndarray):
        for facet in facets:
            if len(facet) == 0:
                continue
            for i in range(len(facet)):
                p1 = facet[i]
                p2 = facet[(i + 1) % len(facet)]

                x1, y1 = int(p1[0]), int(p1[1])
                x2, y2 = int(p2[0]), int(p2[1])

                if (0 <= x1 < eroded_map.shape[1] and 0 <= y1 < eroded_map.shape[0] and
                    0 <= x2 < eroded_map.shape[1] and 0 <= y2 < eroded_map.shape[0] and
                    eroded_map[y1, x1] != 0 and eroded_map[y2, x2] != 0):
                    cv2.line(img, (x1, y1), (x2, y2), color, 1)

    def _prune_voronoi_graph(self, voronoi_map: np.ndarray) -> Tuple[np.ndarray, Set[Tuple[int, int]]]:
        node_points: Set[Tuple[int, int]] = set()

        for v in range(1, voronoi_map.shape[0] - 1):
            for u in range(1, voronoi_map.shape[1] - 1):
                if voronoi_map[v, u] == 127:
                    neighbor_count = 0
                    for dv in range(-1, 2):
                        for du in range(-1, 2):
                            if dv == 0 and du == 0:
                                continue
                            if voronoi_map[v + dv, u + du] == 127:
                                neighbor_count += 1
                    if neighbor_count >= 3:
                        node_points.add((u, v))

        print(f"    Found {len(node_points)} node points")

        for step in range(100):
            changed = False
            for v in range(voronoi_map.shape[0]):
                for u in range(voronoi_map.shape[1]):
                    if voronoi_map[v, u] == 127:
                        neighbor_count = 0
                        for dv in range(-1, 2):
                            for du in range(-1, 2):
                                if dv == 0 and du == 0:
                                    continue
                                nv, nu = v + dv, u + du
                                if (0 <= nv < voronoi_map.shape[0] and
                                    0 <= nu < voronoi_map.shape[1] and
                                    voronoi_map[nv, nu] == 127):
                                    neighbor_count += 1

                        if neighbor_count <= 1 and (u, v) not in node_points:
                            voronoi_map[v, u] = 255
                            changed = True

            if not changed:
                break

        print(f"    Pruning done in {step + 1} iterations")

        return voronoi_map, node_points

    def _find_critical_points(self, voronoi_map: np.ndarray,
                              distance_map: np.ndarray) -> List[Tuple[int, int]]:
        critical_points = []

        for v in range(voronoi_map.shape[0]):
            for u in range(voronoi_map.shape[1]):
                if voronoi_map[v, u] == 127:
                    dist_val = max(1, int(distance_map[v, u]))
                    eps = self.neighborhood_index // dist_val

                    neighbor_points: Set[Tuple[int, int]] = {(u, v)}
                    temp_points: List[Tuple[int, int]] = []
                    neighbor_count = 0
                    loop_counter = 0

                    while True:
                        loop_counter += 1

                        for pt in list(neighbor_points):
                            for dv in range(-1, 2):
                                for du in range(-1, 2):
                                    if dv == 0 and du == 0:
                                        continue

                                    nv, nu = pt[1] + dv, pt[0] + du
                                    if (0 <= nv < voronoi_map.shape[0] and
                                        0 <= nu < voronoi_map.shape[1] and
                                        voronoi_map[nv, nu] == 127 and
                                        (nu, nv) not in neighbor_points):
                                        neighbor_count += 1
                                        temp_points.append((nu, nv))

                        for pt in temp_points:
                            neighbor_points.add(pt)
                            voronoi_map[pt[1], pt[0]] = 255
                            voronoi_map[v, u] = 255

                        temp_points.clear()

                        if not (neighbor_count <= eps and loop_counter < self.max_iterations):
                            break

                    current_critical = (u, v)
                    for pt in neighbor_points:
                        if distance_map[pt[1], pt[0]] < distance_map[current_critical[1], current_critical[0]]:
                            current_critical = pt

                    critical_points.append(current_critical)

        return critical_points

    def _draw_critical_lines(self, voronoi_map: np.ndarray,
                             critical_points: List[Tuple[int, int]],
                             contours: List[np.ndarray],
                             distance_map: np.ndarray):
        all_contour_points = []
        for contour in contours:
            for point in contour:
                all_contour_points.append((point[0, 0], point[0, 1]))

        if len(all_contour_points) < 2:
            return

        basis_points_1 = []
        basis_points_2 = []
        angles = []
        lengths = []

        for cp in critical_points:
            basis_1 = all_contour_points[0]
            vec_1_x = all_contour_points[0][0] - cp[0]
            vec_1_y = all_contour_points[0][1] - cp[1]
            distance_basis_1 = np.sqrt(vec_1_x * vec_1_x + vec_1_y * vec_1_y)

            basis_2 = all_contour_points[1] if len(all_contour_points) > 1 else all_contour_points[0]
            vec_2_x = basis_2[0] - cp[0]
            vec_2_y = basis_2[1] - cp[1]
            distance_basis_2 = np.sqrt(vec_2_x * vec_2_x + vec_2_y * vec_2_y)

            for pt in all_contour_points:
                vector_x = pt[0] - cp[0]
                vector_y = pt[1] - cp[1]
                current_distance = np.sqrt(vector_x * vector_x + vector_y * vector_y)

                if current_distance < distance_basis_1:
                    distance_basis_1 = current_distance
                    basis_1 = pt
                    vec_1_x = vector_x
                    vec_1_y = vector_y

            cp_dist = float(distance_map[cp[1], cp[0]])

            for pt in all_contour_points:
                vector_x = pt[0] - cp[0]
                vector_y = pt[1] - cp[1]
                current_distance = np.sqrt(vector_x * vector_x + vector_y * vector_y)

                vector_x_basis = basis_1[0] - pt[0]
                vector_y_basis = basis_1[1] - pt[1]
                basis_distance = np.sqrt(vector_x_basis * vector_x_basis + vector_y_basis * vector_y_basis)

                if (current_distance > distance_basis_1 and
                    current_distance < distance_basis_2 and
                    basis_distance > cp_dist):
                    distance_basis_2 = current_distance
                    basis_2 = pt
                    vec_2_x = vector_x
                    vec_2_y = vector_y

            if distance_basis_1 > 0 and distance_basis_2 > 0:
                dot = vec_1_x * vec_2_x + vec_1_y * vec_2_y
                cos_angle = dot / (distance_basis_1 * distance_basis_2)
                cos_angle = np.clip(cos_angle, -1.0, 1.0)
                angle = np.degrees(np.arccos(cos_angle))
            else:
                angle = 0.0

            basis_points_1.append(basis_1)
            basis_points_2.append(basis_2)
            angles.append(angle)
            lengths.append(distance_basis_1 + distance_basis_2)

        for i in range(len(critical_points)):
            cp = critical_points[i]
            draw = True

            for j in range(len(critical_points)):
                if j == i:
                    continue

                cp2 = critical_points[j]
                vector_x = cp2[0] - cp[0]
                vector_y = cp2[1] - cp[1]
                critical_point_distance = np.sqrt(vector_x * vector_x + vector_y * vector_y)

                threshold = int(distance_map[cp[1], cp[0]]) * self.min_critical_point_distance_factor
                if critical_point_distance < threshold:
                    if angles[i] < angles[j]:
                        draw = False

                    if (angles[i] == angles[j] and
                        lengths[i] > lengths[j] and
                        (lengths[j] > 3 or i > j)):
                        draw = False

            if draw:
                cv2.line(voronoi_map, cp, basis_points_1[i], 0, 2)
                cv2.line(voronoi_map, cp, basis_points_2[i], 0, 2)

    def _find_and_fill_rooms(self, critical_lines_map: np.ndarray,
                             segmented_map: np.ndarray,
                             map_resolution: float) -> List[Room]:
        rooms = []
        used_colors = set()

        contours, hierarchy = cv2.findContours(critical_lines_map, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        if hierarchy is None:
            return rooms

        for i, contour in enumerate(contours):
            if hierarchy[0][i][3] == -1:
                area = map_resolution * map_resolution * cv2.contourArea(contour)

                if self.room_area_factor_lower_limit <= area <= self.room_area_factor_upper_limit:
                    for _ in range(1000):
                        color = np.random.randint(13056, 65280)
                        if color not in used_colors:
                            break

                    used_colors.add(color)
                    cv2.drawContours(segmented_map, [contour], 0, color, 1)

                    room = Room(room_id=color)
                    for pt in contour:
                        room.insert_member_point((pt[0][0], pt[0][1]), map_resolution)
                    rooms.append(room)

        return rooms

    def _wavefront_region_growing(self, segmented_map: np.ndarray):
        max_iterations = 500
        for iteration in range(max_iterations):
            unassigned = segmented_map > 65279
            if not np.any(unassigned):
                break

            changed = False
            shifts = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

            for dy, dx in shifts:
                shifted = np.roll(np.roll(segmented_map, dy, axis=0), dx, axis=1)
                valid_fill = unassigned & (shifted > 0) & (shifted <= 65279)

                if np.any(valid_fill):
                    segmented_map[valid_fill] = shifted[valid_fill]
                    changed = True

            if not changed:
                break

        print(f"    Region growing done in {iteration+1} iterations")

    def _merge_rooms(self, segmented_map: np.ndarray, rooms: List[Room], map_resolution: float):
        self._update_room_statistics(segmented_map, rooms, map_resolution)
        rooms.sort(key=lambda r: r.area)

        def get_room_by_id(room_id):
            for r in rooms:
                if r.room_id == room_id:
                    return r
            return None

        print("Merge step a) dead-ends (1 neighbor, wall < 75%)...")
        self._merge_by_criteria(segmented_map, rooms, map_resolution,
            lambda r: (len([n for n in r.neighbor_ids if n != 0]) == 1 and
                      r.area < self.max_area_for_merging and
                      r.get_wall_to_perimeter_ratio() < 0.75))

        print("Merge step b) small noise (area < 2m², border >= 20%)...")
        self._merge_by_criteria(segmented_map, rooms, map_resolution,
            lambda r: (r.area < 2.0 and r.perimeter > 0 and
                      r.neighbor_statistics.get(r.get_neighbor_with_largest_common_border(), 0) / r.perimeter >= 0.2))

        print("Merge step c) room interior parts (1 neighbor with <=2 neighbors, wall >= 50%)...")
        def criteria_c(r):
            if r.get_wall_to_perimeter_ratio() < 0.5:
                return False
            non_wall_neighbors = [n for n in r.neighbor_ids if n != 0]
            if len(non_wall_neighbors) != 1:
                return False
            neighbor = get_room_by_id(non_wall_neighbors[0])
            if neighbor is None:
                return False
            neighbor_count = len([n for n in neighbor.neighbor_ids if n != 0])
            return neighbor_count <= 2

        self._merge_by_criteria(segmented_map, rooms, map_resolution, criteria_c)

        print("Merge step d) shared border (small >= 20%, large >= 10%)...")
        def criteria_d(r):
            if r.perimeter == 0:
                return False
            max_neighbor_id = r.get_neighbor_with_largest_common_border()
            if max_neighbor_id == 0:
                return False

            small_ratio = r.neighbor_statistics.get(max_neighbor_id, 0) / r.perimeter
            if small_ratio < 0.2:
                return False

            neighbor = get_room_by_id(max_neighbor_id)
            if neighbor is None or neighbor.perimeter == 0:
                return False

            large_ratio = neighbor.neighbor_statistics.get(r.room_id, 0) / neighbor.perimeter
            return large_ratio >= 0.1

        self._merge_by_criteria(segmented_map, rooms, map_resolution, criteria_d)

        print("Merge step e) furniture fragments (border > 40%)...")
        self._merge_by_criteria(segmented_map, rooms, map_resolution,
            lambda r: (r.perimeter > 0 and
                      r.neighbor_statistics.get(r.get_neighbor_with_largest_common_border(), 0) / r.perimeter > 0.4))

        print("Merge step f) final cleanup of small areas...")
        self._merge_by_criteria(segmented_map, rooms, map_resolution,
            lambda r: r.area < self.max_area_for_merging * 0.5)

        print(f"Final room count: {len(rooms)}")

    def _update_room_statistics(self, segmented_map: np.ndarray, rooms: List[Room], map_resolution: float):
        from scipy import ndimage

        for room in rooms:
            room.members.clear()
            room.neighbor_ids.clear()
            room.neighbor_statistics.clear()
            room.perimeter = 0
            room.wall_perimeter = 0
            room.area = 0

        id_to_room = {room.room_id: room for room in rooms}

        unique_ids, counts = np.unique(segmented_map, return_counts=True)
        for uid, count in zip(unique_ids, counts):
            if uid in id_to_room:
                id_to_room[uid].area = count * map_resolution * map_resolution

        for room in rooms:
            room_id = room.room_id
            mask = (segmented_map == room_id)
            eroded = ndimage.binary_erosion(mask)
            boundary = mask & ~eroded

            boundary_coords = np.where(boundary)
            for y, x in zip(boundary_coords[0], boundary_coords[1]):
                for dy in range(-1, 2):
                    for dx in range(-1, 2):
                        if dy == 0 and dx == 0:
                            continue
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < segmented_map.shape[0] and 0 <= nx < segmented_map.shape[1]:
                            neighbor_id = segmented_map[ny, nx]
                            if neighbor_id != room_id:
                                room.add_neighbor(neighbor_id)
                                if neighbor_id != 0:
                                    room.add_neighbor_id(neighbor_id)

    def _merge_by_criteria(self, segmented_map: np.ndarray, rooms: List[Room],
                           map_resolution: float, criteria_func):
        i = 0
        while i < len(rooms):
            room = rooms[i]

            if criteria_func(room):
                merge_id = room.get_neighbor_with_largest_common_border()
                merge_idx = None

                for j, r in enumerate(rooms):
                    if r.room_id == merge_id:
                        merge_idx = j
                        break

                if merge_idx is not None:
                    target = rooms[merge_idx]
                    target.merge_room(room, map_resolution)
                    segmented_map[segmented_map == room.room_id] = target.room_id
                    rooms.pop(i)
                    self._update_room_statistics(segmented_map, rooms, map_resolution)
                    rooms.sort(key=lambda r: r.area)
                    i = 0
                    continue

            i += 1

    def _colorize_map(self, segmented_map: np.ndarray) -> np.ndarray:
        colored = np.zeros((*segmented_map.shape, 3), dtype=np.uint8)

        unique_ids = np.unique(segmented_map)
        for uid in unique_ids:
            if uid == 0:
                continue
            mask = segmented_map == uid
            color = np.random.randint(50, 255, 3)
            colored[mask] = color

        return colored


def main():
    import sys

    if len(sys.argv) < 2:
        print("Usage: python voronoi_segmentation.py <map_image>")
        print("Creating test map...")

        test_map = np.zeros((300, 400), dtype=np.uint8)
        test_map[20:280, 20:380] = 255

        test_map[20:200, 150:160] = 0
        test_map[100:110, 20:150] = 0
        test_map[100:110, 160:300] = 0

        test_map[140:160, 150:160] = 255
        test_map[100:110, 60:80] = 255
        test_map[100:110, 200:220] = 255

        map_image = test_map
    else:
        map_image = cv2.imread(sys.argv[1], cv2.IMREAD_GRAYSCALE)
        if map_image is None:
            print(f"Error: Could not load image {sys.argv[1]}")
            return

        _, map_image = cv2.threshold(map_image, 250, 255, cv2.THRESH_BINARY)

    segmenter = VoronoiSegmentation(
        room_area_factor_lower_limit=0.1,
        room_area_factor_upper_limit=1000.0,
        neighborhood_index=280,
        max_iterations=100,
        min_critical_point_distance_factor=1.6,
        max_area_for_merging=12.5
    )

    _segmented = segmenter.segment_map(map_image, map_resolution=0.05, display_map=True)


if __name__ == "__main__":
    main()
