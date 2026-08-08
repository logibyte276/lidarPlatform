"""
newSubscriber.py
 
UDP receiver + parser for Unitree L1 LiDAR point-cloud and IMU data.
No visualization here on purpose -- this module is meant to be dropped
into another script (SLAM, obstacle avoidance, logging, etc.) via a
plain `import`.

Wire format (sent by the C++ bridge on the Orin Nano):
    [msgType: uint32][length: uint32][payload...]

    msgType 101 -> IMU packet.     payload = "=dI4f3f3f"
    msgType 102 -> Scan packet.    payload = "=dII" + up to 120 * "fffffI"

Example usage from another script:

    from lidar_udp_receiver import LidarUDPReceiver, LidarScan, LidarIMU

    with LidarUDPReceiver(port=12345) as receiver:
        for message in receiver.stream():
            if isinstance(message, LidarScan):
                xyz, intensity = message.to_numpy()
                # ... feed xyz into your obstacle-avoidance / SLAM code
            elif isinstance(message, LidarIMU):
                # ... use message.angular_velocity, etc.
                pass
"""
"""
A big difference between the old subscriber and this is that this uses a structured numpy array to store scan data instead of just a normal array of point objects. A lot more efficient.
Ex: notice how binary data is parsed directly into a structured numpy array. Or notice how the LidarPoint object is only used when trying to grab a single point. Nothing more. 
Also uses dataclasses for cleaner class writing as well as typing for type hinting and stuff. If you see : or ->, that is type hinting.
"""

import socket
import struct
import logging
from dataclasses import dataclass
from typing import Optional, Union, Iterator, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# --- protocol constants -----------------------------------------------------

MSG_TYPE_IMU = 101
MSG_TYPE_SCAN = 102

"""The C++ side packs a fixed-size buffer of up to this many points per scan
packet (only the first `valid_points_num` of them are meaningful). Used
here as a safety cap so a corrupted/short packet can't make us read past
the end of the buffer."""
MAX_POINTS_PER_SCAN = 120


# --- data containers ---------------------------------------------------------

@dataclass
class LidarPoint:
    """A single LiDAR return. Mainly a convenience view onto one row of a
    LidarScan's `points` array -- for bulk processing, work with
    LidarScan.to_numpy() instead, it's much faster."""
    x: float
    y: float
    z: float
    intensity: float
    time: float
    ring: int


@dataclass
class LidarScan:
    stamp: float
    id: int
    valid_points_num: int
    points: np.ndarray  # structured array, dtype = POINT_DTYPE, length == valid_points_num

    def to_numpy(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return (Nx3 xyz array, N-length intensity array) for this scan."""
        xyz = np.stack([self.points['x'], self.points['y'], self.points['z']], axis=1)
        return xyz, self.points['intensity']

    def point(self, i: int) -> LidarPoint:
        """Convenience accessor for a single point as a LidarPoint object.
        Fine for occasional single-point lookups; use to_numpy() instead if
        you're processing the whole scan, since building a LidarPoint per
        point in a loop defeats the point of the vectorized parsing below."""
        p = self.points[i]
        return LidarPoint(float(p['x']), float(p['y']), float(p['z']),
                           float(p['intensity']), float(p['time']), int(p['ring']))


@dataclass
class LidarIMU:
    stamp: float
    id: int
    quaternion: Tuple[float, float, float, float]
    angular_velocity: Tuple[float, float, float]
    linear_acceleration: Tuple[float, float, float]


# --- binary layout -----------------------------------------------------------

# One point = 5 little-endian float32s (x, y, z, intensity, time) + 1 uint32 (ring). For the structured numpy array.
_POINT_DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
    ('intensity', '<f4'), ('time', '<f4'), ('ring', '<u4'),
])

_HEADER_STRUCT = struct.Struct("=I")         # msgType (length field follows but isn't needed to parse)
_IMU_STRUCT = struct.Struct("=dI4f3f3f")
_SCAN_HEADER_STRUCT = struct.Struct("=dII")  # stamp, id, validPointsNum


def parse_imu(payload: bytes) -> LidarIMU:
    """Parse an IMU packet body (everything after the 8-byte msgType+length header)."""
    data = _IMU_STRUCT.unpack(payload[:_IMU_STRUCT.size])
    return LidarIMU(
        stamp=data[0],
        id=data[1],
        quaternion=data[2:6],
        angular_velocity=data[6:9],
        linear_acceleration=data[9:12],
    )


def parse_scan(payload: bytes) -> LidarScan:
    """Parse a Scan packet body (everything after the 8-byte msgType+length header)."""
    stamp, scan_id, valid_points_num = _SCAN_HEADER_STRUCT.unpack(
        payload[:_SCAN_HEADER_STRUCT.size]
    )
    points_bytes = payload[_SCAN_HEADER_STRUCT.size:]
    count = min(valid_points_num, MAX_POINTS_PER_SCAN)
    points = np.frombuffer(points_bytes, dtype=_POINT_DTYPE, count=count)
    return LidarScan(stamp=stamp, id=scan_id, valid_points_num=count, points=points)


# --- receiver ------------------------------------------------------------------

class LidarUDPReceiver:
    """
    Owns a UDP socket and hands back parsed LidarScan / LidarIMU objects.
    Pure data plumbing -- no plotting, no printing by default (uses `logging`
    so the importing script controls verbosity).

    Usage:
        with LidarUDPReceiver(port=12345) as receiver:
            for message in receiver.stream():
                ...

    or, without the context manager:
        receiver = LidarUDPReceiver(port=12345)
        receiver.open()
        msg = receiver.receive_once()
        ...
        receiver.close()
    """

    def __init__(self, port: int = 12345, ip: str = "0.0.0.0",
                 timeout: float = 1.0, buffer_size: int = 65536):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.buffer_size = buffer_size
        self._sock: Optional[socket.socket] = None

    def open(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self.ip, self.port))
        self._sock.settimeout(self.timeout)
        logger.info("Listening for LiDAR UDP data on %s:%d", self.ip, self.port)

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None
            logger.info("Socket closed.")

    def __enter__(self) -> "LidarUDPReceiver":
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    def receive_once(self) -> Optional[Union[LidarScan, LidarIMU]]:
        """
        Block up to `timeout` seconds for one UDP packet.
        Returns the parsed message, or None on timeout / unknown / malformed packet.
        """
        if self._sock is None:
            raise RuntimeError("Socket not open -- call open() or use a 'with' block first.")

        try:
            data, _addr = self._sock.recvfrom(self.buffer_size)
        except socket.timeout:
            return None

        if len(data) < 8:
            logger.warning("Received undersized packet (%d bytes), ignoring.", len(data))
            return None

        msg_type = _HEADER_STRUCT.unpack(data[:4])[0]
        payload = data[8:]  # skip the 4-byte msgType + 4-byte length header

        try:
            if msg_type == MSG_TYPE_IMU:
                return parse_imu(payload)
            elif msg_type == MSG_TYPE_SCAN:
                return parse_scan(payload)
            else:
                logger.warning("Unknown message type: %d", msg_type)
                return None
        except struct.error as e:
            logger.warning("Failed to parse packet (msgType=%d): %s", msg_type, e)
            return None

    def stream(self) -> Iterator[Union[LidarScan, LidarIMU]]:
        """Generator that yields parsed messages forever, skipping timeouts."""
        while True:
            msg = self.receive_once()
            if msg is not None:
                yield msg


if __name__ == "__main__":
    # Minimal smoke test when this file is run directly (not for visualization,
    # just to confirm packets are arriving and parsing correctly).
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    with LidarUDPReceiver() as receiver:
        try:
            for message in receiver.stream():
                if isinstance(message, LidarScan):
                    print(f"Scan #{message.id}: {message.valid_points_num} points, "
                          f"stamp={message.stamp:.3f}")
                elif isinstance(message, LidarIMU):
                    print(f"IMU  #{message.id}: stamp={message.stamp:.3f}, "
                          f"ang_vel={message.angular_velocity}")
        except KeyboardInterrupt:
            print("\nStopped.")
