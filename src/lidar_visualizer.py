import time
import numpy as np
import open3d as o3d
from lidar_udp_receiver import LidarStream, LidarUDPReceiver

VIS_BLOCK = 17

lidar = LidarStream()
lidar.start()
print("LiDAR started.")

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Unitree LiDAR Point Cloud", width=1024, height=768)
pcd = o3d.geometry.PointCloud()
vis.add_geometry(pcd)  
vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0]))
ctr = vis.get_view_control()
ctr.set_lookat([0, 0, 0])       # point the camera is aimed at
ctr.set_front([0, -1, 0.3])     # direction the camera faces, pointing toward lookat
ctr.set_up([0, 0, 1])           # which way is "up" on screen
ctr.set_zoom(0.5)               # smaller = zoomed in closer

try:
    while True:
        scans = lidar.recent_scans(17)
        xyz_list = [s.xyz_intensity()[0] for s in scans]
        points = np.concatenate(xyz_list, axis=0).astype(np.float64)

        pcd.points = o3d.utility.Vector3dVector(points)
        vis.update_geometry(pcd)

        if not vis.poll_events():   # False if the user closed the window
            break
        vis.update_renderer()
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopped by Ctrl+C.")

finally:
    lidar.stop()
    vis.destroy_window()

  
