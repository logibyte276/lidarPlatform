import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from datetime import datetime

# Try to import open3d for better visualization (optional)
try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("Open3D not installed. Install with: pip install open3d")
    print("Using matplotlib for visualization instead.")

# IP and Port
UDP_IP = "0.0.0.0"
UDP_PORT = 12345

# Point Type
class PointUnitree:
    def __init__(self, x, y, z, intensity, time, ring):
        self.x = x
        self.y = y
        self.z = z
        self.intensity = intensity
        self.time = time
        self.ring = ring

# Scan Type
class ScanUnitree:
    def __init__(self, stamp, id, validPointsNum, points):
        self.stamp = stamp
        self.id = id
        self.validPointsNum = validPointsNum
        self.points = points
    
    def to_numpy_array(self):
        """Convert points to numpy array for easier processing"""
        points_array = np.array([[p.x, p.y, p.z] for p in self.points[:self.validPointsNum]])
        intensities = np.array([p.intensity for p in self.points[:self.validPointsNum]])
        return points_array, intensities

# IMU Type
class IMUUnitree:
    def __init__(self, stamp, id, quaternion, angular_velocity, linear_acceleration):
        self.stamp = stamp
        self.id = id
        self.quaternion = quaternion
        self.angular_velocity = angular_velocity
        self.linear_acceleration = linear_acceleration

# Point Cloud Visualizer
class PointCloudVisualizer:
    def __init__(self, use_open3d=False):
        self.use_open3d = use_open3d and HAS_OPEN3D
        self.fig = None
        self.ax = None
        self.scatter = None
        
        if self.use_open3d:
            self.init_open3d()
        else:
            self.init_matplotlib()
    
    def init_open3d(self):
        """Initialize Open3D visualizer"""
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="Unitree LiDAR Point Cloud", width=1024, height=768)
        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)
        
        # Set view control
        ctr = self.vis.get_view_control()
        ctr.set_front([0, 0, -1])
        ctr.set_lookat([0, 0, 0])
        ctr.set_up([0, -1, 0])
        ctr.set_zoom(0.8)
    
    def init_matplotlib(self):
        """Initialize matplotlib 3D plot"""
        plt.ion()  # Interactive mode
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title('Unitree LiDAR Point Cloud')
        
        # Set equal aspect ratio
        self.ax.set_box_aspect([1, 1, 1])
        
        plt.tight_layout()
        plt.show()
    
    def update_open3d(self, points, intensities=None):
        """Update Open3D point cloud"""
        if len(points) == 0:
            return
        
        self.pcd.points = o3d.utility.Vector3dVector(points)
        
        if intensities is not None and len(intensities) > 0:
            # Normalize intensities for coloring
            intensities_norm = (intensities - intensities.min()) / (intensities.max() - intensities.min() + 1e-6)
            colors = plt.cm.jet(intensities_norm)[:, :3]
            self.pcd.colors = o3d.utility.Vector3dVector(colors)
        
        self.vis.update_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()
    
    def update_matplotlib(self, points, intensities=None):
        """Update matplotlib plot"""
        if len(points) == 0:
            return
        
        self.ax.clear()
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title('Unitree LiDAR Point Cloud')
        
        # Color by intensity or by height
        if intensities is not None and len(intensities) > 0:
            scatter = self.ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                                     c=intensities, cmap='viridis', s=1, alpha=0.6)
            #self.fig.colorbar(scatter, ax=self.ax, label='Intensity')
        else:
            # Color by height (Z)
            scatter = self.ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                                     c=points[:, 2], cmap='viridis', s=1, alpha=0.6)
            #self.fig.colorbar(scatter, ax=self.ax, label='Height (m)')
        
        # Set equal aspect ratio
        if len(points) > 0:
            max_range = max(points[:, 0].max() - points[:, 0].min(),
                           points[:, 1].max() - points[:, 1].min(),
                           points[:, 2].max() - points[:, 2].min()) / 2.0
            
            mid_x = (points[:, 0].max() + points[:, 0].min()) * 0.5
            mid_y = (points[:, 1].max() + points[:, 1].min()) * 0.5
            mid_z = (points[:, 2].max() + points[:, 2].min()) * 0.5
            
            self.ax.set_xlim(mid_x - max_range, mid_x + max_range)
            self.ax.set_ylim(mid_y - max_range, mid_y + max_range)
            self.ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        plt.draw()
        plt.pause(0.01)
    
    def close(self):
        """Close visualizer"""
        if self.use_open3d:
            self.vis.destroy_window()
        else:
            plt.ioff()
            plt.close()

def filter_points(points, intensities, distance_threshold=5000.0):
    """Filter points by distance and remove NaN/invalid points"""
    # Calculate distance from origin
    distances = np.linalg.norm(points, axis=1)
    
    # Filter points within threshold
    valid_mask = (distances < distance_threshold) & (~np.isnan(points).any(axis=1))
    
    return points[valid_mask], intensities[valid_mask]

def main():
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(1.0)  # Set timeout to 1 second
    
    # Calculate Struct Sizes
    imuDataStr = "=dI4f3f3f"
    imuDataSize = struct.calcsize(imuDataStr)
    
    pointDataStr = "=fffffI"
    pointSize = struct.calcsize(pointDataStr)
    
    scanDataStr = "=dII" + 120 * "fffffI"
    scanDataSize = struct.calcsize(scanDataStr)
    
    print("=" * 60)
    print("Unitree LiDAR Point Cloud Receiver")
    print("=" * 60)
    print("Listening on {}:{}".format(UDP_IP, UDP_PORT))
    print("pointSize = {}, scanDataSize = {}, imuDataSize = {}".format(pointSize, scanDataSize, imuDataSize))
    print("=" * 60)
    
    # Initialize visualizer
    use_open3d = raw_input("Use Open3D for better visualization? (y/n): ").lower() == 'y' if hasattr(__builtins__, 'raw_input') else input("Use Open3D for better visualization? (y/n): ").lower() == 'y'
    visualizer = PointCloudVisualizer(use_open3d=use_open3d)
    
    # Statistics
    frame_count = 0
    total_points = 0
    start_time = time.time()
    
    try:
        while True:
            try:
                # Receive data
                data, addr = sock.recvfrom(65536)  # Increased buffer size
                
                msgType = struct.unpack("=I", data[:4])[0]
                
                if msgType == 101:  # IMU Message
                    length = struct.unpack("=I", data[4:8])[0]
                    imuData = struct.unpack(imuDataStr, data[8:8+imuDataSize])
                    imuMsg = IMUUnitree(imuData[0], imuData[1], imuData[2:6], 
                                       imuData[6:9], imuData[9:12])
                    
                    # Optionally print IMU data (comment out for performance)
                    # print("IMU - stamp: {}, id: {}".format(imuMsg.stamp, imuMsg.id))
                
                elif msgType == 102:  # Scan Message
                    length = struct.unpack("=I", data[4:8])[0]
                    stamp = struct.unpack("=d", data[8:16])[0]
                    id_val = struct.unpack("=I", data[16:20])[0]
                    validPointsNum = struct.unpack("=I", data[20:24])[0]
                    
                    scanPoints = []
                    pointStartAddr = 24
                    
                    for i in range(validPointsNum):
                        pointData = struct.unpack(pointDataStr, 
                                                data[pointStartAddr: pointStartAddr+pointSize])
                        pointStartAddr += pointSize
                        point = PointUnitree(*pointData)
                        scanPoints.append(point)
                    
                    scanMsg = ScanUnitree(stamp, id_val, validPointsNum, scanPoints)
                    
                    # Convert to numpy array
                    points, intensities = scanMsg.to_numpy_array()
                    
                    # Filter points
                    points, intensities = filter_points(points, intensities, distance_threshold=500.0)
                    
                    # Update statistics
                    frame_count += 1
                    total_points += len(points)
                    elapsed_time = time.time() - start_time
                    fps = frame_count / elapsed_time if elapsed_time > 0 else 0
                    
                    # Print scan info (using string formatting instead of f-strings)
                    print("\n[Scan #{}] ID: {}, Timestamp: {:.3f}".format(frame_count, id_val, stamp))
                    print("  Valid points: {}, Filtered points: {}".format(validPointsNum, len(points)))
                    print("  FPS: {:.2f}, Total points: {}".format(fps, total_points))
                    
                    # Update visualization
                    if len(points) > 0:
                        if visualizer.use_open3d:
                            visualizer.update_open3d(points, intensities)
                        else:
                            visualizer.update_matplotlib(points, intensities)
                    
                    # Print point range
                    if len(points) > 0:
                        print("  X range: [{:.2f}, {:.2f}]".format(points[:, 0].min(), points[:, 0].max()))
                        print("  Y range: [{:.2f}, {:.2f}]".format(points[:, 1].min(), points[:, 1].max()))
                        print("  Z range: [{:.2f}, {:.2f}]".format(points[:, 2].min(), points[:, 2].max()))
                        print("  Intensity range: [{:.2f}, {:.2f}]".format(intensities.min(), intensities.max()))
                
                else:
                    print("Unknown message type: {}".format(msgType))
                    
            except socket.timeout:
                # Timeout is normal, continue listening
                continue
            except Exception as e:
                print("Error processing message: {}".format(e))
                continue
                
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        print("Final statistics: {} frames, {} points".format(frame_count, total_points))
    
    finally:
        visualizer.close()
        sock.close()
        print("Cleanup complete")

if __name__ == "__main__":
    main()
