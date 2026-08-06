import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from scipy.spatial.transform import Rotation as R

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
        times = np.array([p.time for p in self.points[:self.validPointsNum]])
        return points_array, intensities, times

# IMU Type
class IMUUnitree:
    def __init__(self, stamp, id, quaternion, angular_velocity, linear_acceleration):
        self.stamp = stamp
        self.id = id
        self.quaternion = quaternion  # [x, y, z, w]
        self.angular_velocity = angular_velocity
        self.linear_acceleration = linear_acceleration

# Rotation Compensator
class RotationCompensator:
    def __init__(self):
        self.last_imu_stamp = None
        self.last_quaternion = None
        self.reference_quaternion = None
        self.angular_velocity_history = []
        
    def update_imu(self, stamp, quaternion, angular_velocity):
        """Update IMU data and compute rotation compensation"""
        # Convert quaternion from [x, y, z, w] to [w, x, y, z] for scipy
        q = np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])
        
        if self.reference_quaternion is None:
            # Set first IMU reading as reference
            self.reference_quaternion = q
            self.last_quaternion = q
            self.last_imu_stamp = stamp
            return np.eye(3)  # Identity rotation
        
        # Compute relative rotation from reference
        # R_rel = R_current * R_reference^(-1)
        r_ref = R.from_quat(self.reference_quaternion)
        r_current = R.from_quat(q)
        
        # Rotation that transforms current frame to reference frame
        r_compensation = r_current * r_ref.inv()
        
        self.last_quaternion = q
        self.last_imu_stamp = stamp
        
        # Store angular velocity for interpolation
        self.angular_velocity_history.append((stamp, angular_velocity))
        if len(self.angular_velocity_history) > 100:
            self.angular_velocity_history.pop(0)
        
        return r_compensation.as_matrix()
    
    def compensate_points(self, points, point_times, scan_stamp, imu_interpolator=None):
        """Compensate points for rotation using IMU data"""
        if self.reference_quaternion is None or len(points) == 0:
            return points
        
        # Get rotation at scan timestamp
        if imu_interpolator is not None:
            # Interpolate rotation for each point based on its timestamp
            compensated_points = []
            for i, (point, point_time) in enumerate(zip(points, point_times)):
                # Calculate rotation at this specific point time
                rot_matrix = imu_interpolator.get_rotation_at_time(point_time)
                if rot_matrix is not None:
                    compensated_point = rot_matrix.dot(point)
                else:
                    compensated_point = point
                compensated_points.append(compensated_point)
            return np.array(compensated_points)
        else:
            # Apply same rotation to all points (simpler but less accurate)
            rot_matrix = self.get_rotation_at_time(scan_stamp)
            if rot_matrix is not None:
                return np.dot(points, rot_matrix.T)
            return points
    
    def get_rotation_at_time(self, timestamp):
        """Get rotation matrix at specific timestamp (for future interpolation)"""
        # This will be implemented with IMU interpolator
        return None

# IMU Interpolator for per-point compensation
class IMUInterpolator:
    def __init__(self):
        self.imu_data = []  # List of (timestamp, quaternion, angular_velocity)
        self.reference_quaternion = None
        
    def add_imu_data(self, stamp, quaternion, angular_velocity):
        """Add IMU data to history"""
        # Convert quaternion to scipy format [w, x, y, z]
        q = np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])
        self.imu_data.append((stamp, q, angular_velocity))
        
        # Keep last 1000 IMU readings
        if len(self.imu_data) > 1000:
            self.imu_data.pop(0)
        
        # Set reference if not set
        if self.reference_quaternion is None:
            self.reference_quaternion = q
    
    def get_rotation_at_time(self, timestamp):
        """Get rotation matrix that transforms points at timestamp to reference frame"""
        if len(self.imu_data) < 2 or self.reference_quaternion is None:
            return None
        
        # Find IMU readings before and after timestamp
        idx = np.searchsorted([t for t, _, _ in self.imu_data], timestamp)
        
        if idx == 0:
            # Before first IMU reading, use first available
            q = self.imu_data[0][1]
        elif idx >= len(self.imu_data):
            # After last IMU reading, use last available
            q = self.imu_data[-1][1]
        else:
            # Interpolate between two IMU readings
            t1, q1, _ = self.imu_data[idx - 1]
            t2, q2, _ = self.imu_data[idx]
            
            # Spherical linear interpolation (slerp) for quaternions
            alpha = (timestamp - t1) / (t2 - t1)
            q = self.slerp(q1, q2, alpha)
        
        # Compute relative rotation from reference
        r_current = R.from_quat(q)
        r_ref = R.from_quat(self.reference_quaternion)
        r_compensation = r_current * r_ref.inv()
        
        return r_compensation.as_matrix()
    
    def slerp(self, q1, q2, alpha):
        """Spherical linear interpolation between two quaternions"""
        # Ensure quaternions are normalized
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        # Compute dot product
        dot = np.dot(q1, q2)
        
        # If dot is negative, flip one quaternion for shortest path
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        
        # If dot is close to 1, use linear interpolation
        if dot > 0.9995:
            q = q1 + alpha * (q2 - q1)
            return q / np.linalg.norm(q)
        
        # Spherical interpolation
        theta_0 = np.arccos(np.clip(dot, -1, 1))
        theta = theta_0 * alpha
        sin_theta = np.sin(theta)
        sin_theta_0 = np.sin(theta_0)
        
        s1 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s2 = sin_theta / sin_theta_0
        
        q = s1 * q1 + s2 * q2
        return q / np.linalg.norm(q)

# Point Cloud Visualizer with compensation
class PointCloudVisualizer:
    def __init__(self):
        plt.ion()
        self.fig = plt.figure(figsize=(12, 10))
        
        # Create two subplots: original and compensated
        self.ax_original = self.fig.add_subplot(121, projection='3d')
        self.ax_compensated = self.fig.add_subplot(122, projection='3d')
        
        self.ax_original.set_title('Original Point Cloud (Rotating)')
        self.ax_compensated.set_title('Compensated Point Cloud (Stable)')
        
        self.ax_original.set_xlabel('X (m)')
        self.ax_original.set_ylabel('Y (m)')
        self.ax_original.set_zlabel('Z (m)')
        
        self.ax_compensated.set_xlabel('X (m)')
        self.ax_compensated.set_ylabel('Y (m)')
        self.ax_compensated.set_zlabel('Z (m)')
        
        plt.tight_layout()
        plt.show()
        
        self.scatter_original = None
        self.scatter_compensated = None
        
    def update(self, original_points, compensated_points, intensities=None):
        """Update both visualizations"""
        if len(original_points) == 0:
            return
        
        # Clear axes
        self.ax_original.clear()
        self.ax_compensated.clear()
        
        # Set titles and labels
        self.ax_original.set_title('Original Point Cloud (Rotating)')
        self.ax_compensated.set_title('Compensated Point Cloud (Stable)')
        self.ax_original.set_xlabel('X (m)')
        self.ax_original.set_ylabel('Y (m)')
        self.ax_original.set_zlabel('Z (m)')
        self.ax_compensated.set_xlabel('X (m)')
        self.ax_compensated.set_ylabel('Y (m)')
        self.ax_compensated.set_zlabel('Z (m)')
        
        # Plot original points
        if intensities is not None:
            self.scatter_original = self.ax_original.scatter(
                original_points[:, 0], original_points[:, 1], original_points[:, 2],
                c=intensities, cmap='viridis', s=1, alpha=0.6
            )
            self.scatter_compensated = self.ax_compensated.scatter(
                compensated_points[:, 0], compensated_points[:, 1], compensated_points[:, 2],
                c=intensities, cmap='viridis', s=1, alpha=0.6
            )
        else:
            self.scatter_original = self.ax_original.scatter(
                original_points[:, 0], original_points[:, 1], original_points[:, 2],
                c=original_points[:, 2], cmap='viridis', s=1, alpha=0.6
            )
            self.scatter_compensated = self.ax_compensated.scatter(
                compensated_points[:, 0], compensated_points[:, 1], compensated_points[:, 2],
                c=compensated_points[:, 2], cmap='viridis', s=1, alpha=0.6
            )
        
        # Set equal aspect ratio for compensated view
        if len(compensated_points) > 0:
            max_range = max(
                compensated_points[:, 0].max() - compensated_points[:, 0].min(),
                compensated_points[:, 1].max() - compensated_points[:, 1].min(),
                compensated_points[:, 2].max() - compensated_points[:, 2].min()
            ) / 2.0
            
            mid_x = (compensated_points[:, 0].max() + compensated_points[:, 0].min()) * 0.5
            mid_y = (compensated_points[:, 1].max() + compensated_points[:, 1].min()) * 0.5
            mid_z = (compensated_points[:, 2].max() + compensated_points[:, 2].min()) * 0.5
            
            self.ax_compensated.set_xlim(mid_x - max_range, mid_x + max_range)
            self.ax_compensated.set_ylim(mid_y - max_range, mid_y + max_range)
            self.ax_compensated.set_zlim(mid_z - max_range, mid_z + max_range)
        
        plt.draw()
        plt.pause(0.01)
    
    def close(self):
        plt.ioff()
        plt.close()

def filter_points(points, intensities, times, distance_threshold=500.0):
    """Filter points by distance and remove NaN/invalid points"""
    if len(points) == 0:
        return points, intensities, times
    
    # Calculate distance from origin
    distances = np.linalg.norm(points, axis=1)
    
    # Filter points within threshold
    valid_mask = (distances < distance_threshold) & (~np.isnan(points).any(axis=1))
    
    return points[valid_mask], intensities[valid_mask], times[valid_mask]

def main():
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(0.1)
    
    # Calculate Struct Sizes
    imuDataStr = "=dI4f3f3f"
    imuDataSize = struct.calcsize(imuDataStr)
    
    pointDataStr = "=fffffI"
    pointSize = struct.calcsize(pointDataStr)
    
    print("=" * 60)
    print("Unitree LiDAR Point Cloud Receiver with Rotation Compensation")
    print("=" * 60)
    print("Listening on {}:{}".format(UDP_IP, UDP_PORT))
    print("=" * 60)
    
    # Initialize components
    visualizer = PointCloudVisualizer()
    imu_interpolator = IMUInterpolator()
    
    # Statistics
    frame_count = 0
    scan_count = 0
    start_time = time.time()
    
    # Store latest scan data
    latest_scan = None
    latest_scan_points = None
    latest_scan_intensities = None
    latest_scan_times = None
    
    try:
        while True:
            try:
                # Receive data
                data, addr = sock.recvfrom(65536)
                
                msgType = struct.unpack("=I", data[:4])[0]
                
                if msgType == 101:  # IMU Message
                    length = struct.unpack("=I", data[4:8])[0]
                    imuData = struct.unpack(imuDataStr, data[8:8+imuDataSize])
                    imuMsg = IMUUnitree(
                        imuData[0], imuData[1], 
                        imuData[2:6], imuData[6:9], imuData[9:12]
                    )
                    
                    # Add IMU data to interpolator
                    imu_interpolator.add_imu_data(
                        imuMsg.stamp, 
                        imuMsg.quaternion, 
                        imuMsg.angular_velocity
                    )
                    
                    # Print IMU info periodically
                    if frame_count % 100 == 0:
                        print("IMU - stamp: {:.3f}, quat: [{:.3f}, {:.3f}, {:.3f}, {:.3f}]".format(
                            imuMsg.stamp, imuMsg.quaternion[0], imuMsg.quaternion[1],
                            imuMsg.quaternion[2], imuMsg.quaternion[3]
                        ))
                    
                    frame_count += 1
                
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
                    
                    # Convert to numpy arrays
                    points, intensities, times = ScanUnitree(stamp, id_val, validPointsNum, scanPoints).to_numpy_array()
                    
                    # Filter points
                    points, intensities, times = filter_points(points, intensities, times, distance_threshold=50.0)
                    
                    if len(points) > 0:
                        # Compensate for rotation using IMU data
                        compensated_points = []
                        
                        for i, (point, point_time) in enumerate(zip(points, times)):
                            # Get rotation matrix for this point's timestamp
                            rot_matrix = imu_interpolator.get_rotation_at_time(point_time)
                            
                            if rot_matrix is not None:
                                # Apply rotation compensation
                                compensated_point = rot_matrix.dot(point)
                            else:
                                # No IMU data available yet
                                compensated_point = point
                            
                            compensated_points.append(compensated_point)
                        
                        compensated_points = np.array(compensated_points)
                        
                        # Update visualization
                        visualizer.update(points, compensated_points, intensities)
                        
                        # Print statistics
                        scan_count += 1
                        elapsed_time = time.time() - start_time
                        fps = scan_count / elapsed_time if elapsed_time > 0 else 0
                        
                        print("\n[Scan #{}] Timestamp: {:.3f}, Points: {}".format(scan_count, stamp, len(points)))
                        print("  FPS: {:.2f}".format(fps))
                        print("  IMU data points: {}".format(len(imu_interpolator.imu_data)))
                        
                        if len(compensated_points) > 0:
                            print("  Compensated point cloud range:")
                            print("    X: [{:.2f}, {:.2f}]".format(compensated_points[:, 0].min(), compensated_points[:, 0].max()))
                            print("    Y: [{:.2f}, {:.2f}]".format(compensated_points[:, 1].min(), compensated_points[:, 1].max()))
                            print("    Z: [{:.2f}, {:.2f}]".format(compensated_points[:, 2].min(), compensated_points[:, 2].max()))
                    
            except socket.timeout:
                continue
            except Exception as e:
                print("Error: {}".format(e))
                continue
                
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        print("Processed {} IMU messages and {} scans".format(frame_count, scan_count))
    
    finally:
        visualizer.close()
        sock.close()
        print("Cleanup complete")

if __name__ == "__main__":
    main()