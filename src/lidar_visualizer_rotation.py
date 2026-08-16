import time
import numpy as np
import open3d as o3d
from lidar_udp_receiver import LidarStream

VIS_SIZE = 100
REFRESH_HZ = 120

lidar = LidarStream()
lidar.start()
print("LiDAR started.")

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Unitree LiDAR Point Cloud", width=2000, height=1400)
pcd = o3d.geometry.PointCloud()
vis.add_geometry(pcd)  
vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0]))
ctr = vis.get_view_control()
ctr.set_lookat([0, 0, 1.5])       # point the camera is aimed at
ctr.set_front([0, -1, 0.3])     # direction the camera faces, pointing toward lookat
ctr.set_up([0, 0, 1])           # which way is "up" on screen
ctr.set_zoom(2)               # smaller = zoomed in closer
lidar.wait_until_ready()



def rotate_points(points_xyz, quaternion):
    """points_xyz: Nx3 array. quaternion: (x, y, z, w). Returns Nx3 rotated array."""
    x, y, z, w = quaternion
    q_xyz = np.array([x, y, z])

    t = 2 * np.cross(q_xyz, points_xyz)          # Nx3
    rotated = points_xyz + w * t + np.cross(q_xyz, t)
    return rotated

def get_compensated_points(lidar_stream, n_scans=100, max_time_gap=0.01):
    """Grab the n most recent scans, rotate each into world orientation
    using its closest-in-time IMU sample, and merge into one point cloud."""
    scans = lidar_stream.recent_scans(n_scans)
    if not scans:
        return np.empty((0, 3), dtype=np.float64)

    # Pull one IMU window that comfortably covers this whole batch of scans,
    # rather than re-querying the buffer per scan -- cheaper, and the search
    # below just picks whichever sample is closest for each individual scan.
    imu_samples = lidar_stream.recent_imu(lidar_stream.imu_capacity)

    rotated_chunks = []
    for scan in scans:
        if not imu_samples:
            continue
        closest = min(imu_samples, key=lambda s: abs(s.stamp - scan.stamp))
        if abs(closest.stamp - scan.stamp) > max_time_gap:
            continue  # no trustworthy IMU match -- skip this scan rather than guess

        xyz, _intensity = scan.xyz_intensity()
        rotated_chunks.append(rotate_points(xyz, closest.quaternion))

    if not rotated_chunks:
        return np.empty((0, 3), dtype=np.float64)

    return np.concatenate(rotated_chunks, axis=0).astype(np.float64)



refresh_period = 1.0 / REFRESH_HZ
try:
    while True:
        points = get_compensated_points(lidar, n_scans=VIS_SIZE)
        if len(points) > 0:
            pcd.points = o3d.utility.Vector3dVector(points)
            vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(refresh_period)
except KeyboardInterrupt:
    print("\nStopped by Ctrl+C.")
finally:
    vis.destroy_window()
    lidar.stop()



  
