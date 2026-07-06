#!/usr/bin/env python3
"""
完整的 PGM 貼牆路徑生成器
輸入：PGM 地圖檔案
輸出：YAML 格式的航點清單，可直接用於 Nav2 Waypoint Follower
"""

import cv2
import numpy as np
import yaml
import argparse
import os
from pathlib import Path


class WallFollowingPathGenerator:
    def __init__(self, pgm_path, offset_meters, interval_meters, resolution=0.05):
        """
        pgm_path: PGM 地圖檔案路徑
        offset_meters: 刷子距離牆面的距離（公尺）
        interval_meters: 航點間隔（公尺）
        resolution: 地圖解析度（公尺/像素），預設 0.05
        """
        self.pgm_path = pgm_path
        self.offset_m = offset_meters
        self.interval_m = interval_meters
        self.resolution = resolution
        
        self.offset_px = offset_meters / resolution
        self.interval_px = interval_meters / resolution
        
    def load_and_extract_contour(self):
        """讀取 PGM 並提取最外層牆壁輪廓"""
        # 讀取 PGM
        img = cv2.imread(self.pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"無法讀取 PGM 檔案: {self.pgm_path}")
        
        self.img_height, self.img_width = img.shape
        print(f"PGM 尺寸: {self.img_width} x {self.img_height} 像素")
        print(f"實際尺寸: {self.img_width * self.resolution:.2f} x {self.img_height * self.resolution:.2f} 公尺")
        
        # 反相：讓牆壁變白色 (255)，可行區域變黑色 (0)
        # PGM 通常 254=可行，0=障礙，但我們統一處理
        _, thresh = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY_INV)
        
        # 形態學操作：填補牆壁中的小洞，確保輪廓連續
        kernel = np.ones((3, 3), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        # 提取最外層輪廓
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        if not contours:
            raise ValueError("找不到任何輪廓，請檢查 PGM 檔案")
        
        # 取最大的輪廓（應該是外牆）
        self.wall_contour = max(contours, key=cv2.contourArea)
        self.wall_contour = self.wall_contour.reshape(-1, 2)
        
        print(f"牆壁輪廓點數: {len(self.wall_contour)}")
        
        # 保存用於可視化
        self.raw_img = img.copy()
        
    def compute_centroid_offset_path(self):
        """用質心法計算內縮路徑"""
        # 計算質心
        self.center = np.mean(self.wall_contour, axis=0)
        print(f"質心座標 (像素): ({self.center[0]:.1f}, {self.center[1]:.1f})")
        
        self.inner_path = []
        for p in self.wall_contour:
            vec = p - self.center
            norm = np.linalg.norm(vec)
            if norm < 1e-6:
                continue
            n = vec / norm  # 從中心指向牆壁的單位向量（即向內的法向量）
            inner_point = p - n * self.offset_px
            self.inner_path.append(inner_point)
        
        self.inner_path = np.array(self.inner_path)
        print(f"內縮路徑點數: {len(self.inner_path)}")
        
    def smooth_path(self, window_size=5):
        """對路徑做簡單的移動平均平滑化"""
        if len(self.inner_path) < window_size:
            return
        
        smoothed = np.copy(self.inner_path)
        for i in range(len(self.inner_path)):
            start = max(0, i - window_size // 2)
            end = min(len(self.inner_path), i + window_size // 2 + 1)
            smoothed[i] = np.mean(self.inner_path[start:end], axis=0)
        
        self.inner_path = smoothed
        print("路徑平滑化完成")
    
    def generate_waypoints(self):
        """計算航點：(x_meters, y_meters, yaw_rad)"""
        
        if len(self.inner_path) < 3:
            raise ValueError("內縮路徑點數不足，無法生成航點")
        
        # 確保路徑是封閉的（首尾相連）
        if np.linalg.norm(self.inner_path[0] - self.inner_path[-1]) > 2.0:
            self.inner_path = np.vstack([self.inner_path, self.inner_path[0]])
        
        self.waypoints_px = []
        accumulated_dist = 0.0
        
        # 第一個航點：用前兩個點的切線計算法向量
        p0 = self.inner_path[0]
        p1 = self.inner_path[1]
        tangent = p1 - p0
        # 順時針法向量（假設路徑是順時針）
        normal = np.array([tangent[1], -tangent[0]])
        norm = np.linalg.norm(normal)
        if norm > 1e-6:
            normal = normal / norm
        yaw = np.arctan2(normal[1], normal[0])
        
        self.waypoints_px.append((p0[0], p0[1], yaw))
        accumulated_dist = 0.0
        prev_point = p0
        
        # 遍歷路徑點，按照間隔取樣
        for i in range(1, len(self.inner_path)):
            p_curr = self.inner_path[i]
            dist = np.linalg.norm(p_curr - prev_point)
            accumulated_dist += dist
            
            if accumulated_dist >= self.interval_px:
                # 計算法向量（垂直於路徑方向，指向牆壁）
                if i < len(self.inner_path) - 1:
                    tangent = self.inner_path[i + 1] - p_curr
                else:
                    tangent = p_curr - self.inner_path[i - 1]
                
                normal = np.array([tangent[1], -tangent[0]])
                norm = np.linalg.norm(normal)
                if norm > 1e-6:
                    normal = normal / norm
                
                yaw = np.arctan2(normal[1], normal[0])
                self.waypoints_px.append((p_curr[0], p_curr[1], yaw))
                accumulated_dist = 0.0
            
            prev_point = p_curr
        
        print(f"生成航點數: {len(self.waypoints_px)}")
        
        # 轉換為世界座標（公尺）
        self.waypoints_world = []
        for wx, wy, wyaw in self.waypoints_px:
            x_m = wx * self.resolution
            y_m = (self.img_height - wy) * self.resolution  # 影像 y 軸翻轉
            self.waypoints_world.append({
                'x': float(x_m),
                'y': float(y_m),
                'yaw': float(wyaw)
            })
    
    def save_waypoints_yaml(self, output_path):
        """儲存為 Nav2 相容的 YAML 格式"""
        data = {
            'waypoints': []
        }
        
        for i, wp in enumerate(self.waypoints_world):
            data['waypoints'].append({
                'point': [wp['x'], wp['y']],
                'yaw': wp['yaw'],
                'id': i,
                'frame_id': 'map'
            })
        
        with open(output_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
        
        print(f"航點已儲存至: {output_path}")
        print(f"航點總數: {len(data['waypoints'])}")
    
    def visualize(self, output_path=None):
        """可視化結果（用 matplotlib）"""
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("需要 matplotlib 來可視化，請執行: pip install matplotlib")
            return
        
        fig, ax = plt.subplots(1, 1, figsize=(10, 10))
        
        # 顯示原始地圖
        ax.imshow(self.raw_img, cmap='gray', origin='upper')
        
        # 繪製牆壁輪廓
        contour_arr = np.array(self.wall_contour)
        ax.plot(contour_arr[:, 0], contour_arr[:, 1], 'b-', linewidth=1, alpha=0.5, label='牆壁輪廓')
        
        # 繪製內縮路徑
        ax.plot(self.inner_path[:, 0], self.inner_path[:, 1], 'g-', linewidth=2, label='內縮路徑')
        
        # 繪製航點和朝向
        wp_x = [wp[0] for wp in self.waypoints_px]
        wp_y = [wp[1] for wp in self.waypoints_px]
        wp_yaw = [wp[2] for wp in self.waypoints_px]
        
        ax.scatter(wp_x, wp_y, c='red', s=30, zorder=5, label='航點')
        
        # 繪製箭頭（船頭方向）
        # arrow_len = 15  # 像素
        # for x, y, yaw in self.waypoints_px[::max(1, len(self.waypoints_px)//20)]:  # 只畫部分箭頭，避免太密
        #     dx = arrow_len * np.cos(yaw)
        #     dy = arrow_len * np.sin(yaw)
        #     ax.arrow(x, y, dx, dy, head_width=3, head_length=4, fc='red', ec='red', alpha=0.8)
        
        # 標示質心
        ax.scatter(self.center[0], self.center[1], c='yellow', s=100, marker='*', label='質心')
        
        ax.set_title(f'貼牆路徑生成結果\n航點數: {len(self.waypoints_px)}, 間距: {self.interval_m:.2f}m, 偏移: {self.offset_m:.2f}m')
        ax.legend()
        ax.set_aspect('equal')
        
        if output_path:
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            print(f"可視化圖檔已儲存至: {output_path}")
        
        plt.show()
    
    def run(self, output_yaml, output_vis=None):
        """執行完整流程"""
        print("=" * 50)
        print("貼牆路徑生成器開始執行")
        print("=" * 50)
        
        # 步驟 1：讀取 PGM 並提取輪廓
        print("\n[1/5] 讀取 PGM 並提取輪廓...")
        self.load_and_extract_contour()
        
        # 步驟 2：計算內縮路徑
        print("\n[2/5] 計算內縮路徑...")
        self.compute_centroid_offset_path()
        
        # 步驟 3：平滑化（可選）
        print("\n[3/5] 路徑平滑化...")
        self.smooth_path(window_size=7)
        
        # 步驟 4：生成航點
        print("\n[4/5] 生成航點...")
        self.generate_waypoints()
        
        # 步驟 5：儲存與可視化
        print("\n[5/5] 儲存結果...")
        self.save_waypoints_yaml(output_yaml)
        
        if output_vis:
            self.visualize(output_vis)
        else:
            self.visualize()
        
        print("\n✅ 完成！")


def main():
    parser = argparse.ArgumentParser(description='從 PGM 地圖生成貼牆清掃路徑')
    parser.add_argument('pgm_file', help='輸入的 PGM 地圖檔案路徑')
    parser.add_argument('-o', '--output', default='wall_waypoints.yaml', help='輸出的 YAML 航點檔案 (預設: wall_waypoints.yaml)')
    parser.add_argument('--offset', type=float, default=0.15, help='刷子距牆面的距離(公尺) (預設: 0.15)')
    parser.add_argument('--interval', type=float, default=0.3, help='航點間距(公尺) (預設: 0.3)')
    parser.add_argument('--resolution', type=float, default=0.05, help='地圖解析度(公尺/像素) (預設: 0.05)')
    parser.add_argument('--vis', default='wall_path_visualization.png', help='可視化輸出圖檔路徑 (預設: wall_path_visualization.png)')
    
    args = parser.parse_args()
    
    # 建立生成器並執行
    generator = WallFollowingPathGenerator(
        pgm_path=args.pgm_file,
        offset_meters=args.offset,
        interval_meters=args.interval,
        resolution=args.resolution
    )
    
    generator.run(
        output_yaml=args.output,
        output_vis=args.vis
    )


if __name__ == '__main__':
    main()