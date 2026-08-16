import time
import numpy as np
import open3d as o3d
from collections import deque
from lidar_udp_receiver import LidarStream

VIS_SIZE = 100
REFRESH_HZ = 120
MAX_TIME_GAP = 0.01


def rotate_points(points_xyz, quaternion):
    """points_xyz: Nx3 array. quaternion: (x, y, z, w). Returns Nx3 rotated array."""
    x, y, z, w = quaternion
    q_xyz = np.array([x, y, z])
    t = 2 * np.cross(q_xyz, points_xyz)
    rotated = points_xyz + w * t + np.cross(q_xyz, t)
    return rotated


class RotatedScanAccumulator:
    """Rolling window of already-rotated scans. Each scan is rotated exactly
    once, no matter how many frames it stays on screen."""

    def __init__(self, max_scans=VIS_SIZE, max_time_gap=MAX_TIME_GAP):
        self.max_time_gap = max_time_gap
        self._rotated_scans = deque(maxlen=max_scans)
        self.points_total = 0

    def update(self, lidar_stream):
        new_scans = lidar_stream.scans.drain()
        if not new_scans:
            return

        imu_samples = lidar_stream.recent_imu(lidar_stream.imu_capacity)
        if not imu_samples:
            return

        for scan in new_scans:
            closest = min(imu_samples, key=lambda s: abs(s.stamp - scan.stamp))
            if abs(closest.stamp - scan.stamp) > self.max_time_gap:
                continue
            xyz, _intensity = scan.xyz_intensity()
            self._rotated_scans.append(rotate_points(xyz, closest.quaternion))
            self.points_total += len(xyz)

    def get_points(self):
        if not self._rotated_scans:
            return np.empty((0, 3), dtype=np.float64)
        return np.concatenate(self._rotated_scans, axis=0).astype(np.float64)


lidar = LidarStream(scan_maxlen=round(180/REFRESH_HZ)+50, imu_maxlen=round(250/REFRESH_HZ)+100)
lidar.start()
print("LiDAR started.")
start_time = time.time()

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Unitree LiDAR Point Cloud", width=2000, height=1400)
pcd = o3d.geometry.PointCloud()
vis.add_geometry(pcd)
vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0]))

ctr = vis.get_view_control()
ctr.set_lookat([0, 0, 1.5])
ctr.set_front([0, -1, 0.3])
ctr.set_up([0, 0, 1])
ctr.set_zoom(2)

lidar.wait_until_ready()

accumulator = RotatedScanAccumulator(max_scans=VIS_SIZE, max_time_gap=MAX_TIME_GAP)
refresh_period = 1.0 / REFRESH_HZ

try:
    while True:
        accumulator.update(lidar)
        points = accumulator.get_points()
        if len(points) > 0:
            pcd.points = o3d.utility.Vector3dVector(points)
            vis.update_geometry(pcd)
        if not vis.poll_events():
          break
        vis.update_renderer()
        print(f"Valid points per second: {round(accumulator.total_points / (time.time() - start_time}")
        time.sleep(refresh_period)
except KeyboardInterrupt:
    print("\nStopped by Ctrl+C.")
finally:
    vis.destroy_window()
    lidar.stop()
