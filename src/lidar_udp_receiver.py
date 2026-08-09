"""
lidar_udp_receiver.py

UDP receiver + parser for Unitree L1 LiDAR point-cloud and IMU data.
No visualization here on purpose -- this module is meant to be dropped
into another script (SLAM, obstacle avoidance, logging, etc.) via a
plain `import`.

Wire format (sent by the C++ bridge on the Orin Nano):
    [msgType: uint32][length: uint32][payload...]

    msgType 101 -> IMU packet.     payload = "=dI4f3f3f"
    msgType 102 -> Scan packet.    payload = "=dII" + up to 120 * "fffffI"

There are two ways to use this module:

1. PULL / one-at-a-time (LidarUDPReceiver). You call receive_once() or
   iterate stream(), and you get messages as they arrive. Simple, but your
   loop has to keep up with the sensor or packets pile up in the OS socket
   buffer and eventually get dropped by the kernel.

2. PUSH / buffered (LidarStream). A background thread constantly drains the
   socket and drops parsed messages into two separate bounded ring buffers
   (one for scans, one for IMU). Your main loop then grabs the latest data
   whenever it's ready, at whatever rate it wants. This is what you want if
   your consumer runs slower than the sensor -- e.g. an obstacle-avoidance
   loop at 20Hz reading from a ~500Hz IMU stream.

Example usage (buffered):

    from lidar_udp_receiver import LidarStream

    with LidarStream(port=12345, scan_maxlen=100, imu_maxlen=100) as lidar:
        while True:
            # single newest reading
            scan = lidar.latest_scan
            if scan is not None:
                xyz, intensity = scan.xyz_intensity()
                # ... feed xyz into your obstacle-avoidance / SLAM code

            # or the N most recent, oldest-first
            for s in lidar.recent_scans(25):
                ...
            recent_imu = lidar.recent_imu(10)

            time.sleep(0.05)

    Asking for more than the buffer's capacity raises ValueError (it can never
    be satisfied). Asking for more than have arrived *so far* just returns what
    is available, since that's normal right after startup.

Example usage (pull, unchanged from before):

    from lidar_udp_receiver import LidarUDPReceiver, LidarScan, LidarIMU

    with LidarUDPReceiver(port=12345) as receiver:
        for message in receiver.stream():
            ...
"""

import socket
import struct
import time
import logging
import threading
from collections import deque
from itertools import islice
from dataclasses import dataclass
from typing import Optional, Union, Iterator, Tuple, List, Deque

import numpy as np

logger = logging.getLogger(__name__)

# --- protocol constants -----------------------------------------------------

MSG_TYPE_IMU = 101
MSG_TYPE_SCAN = 102

# The C++ side packs a fixed-size buffer of up to this many points per scan
# packet (only the first `valid_points_num` of them are meaningful). Used
# here as a safety cap so a corrupted/short packet can't make us read past
# the end of the buffer.
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
    points: np.ndarray  # structured array, dtype = _POINT_DTYPE, length == valid_points_num

    def xyz_intensity(self) -> Tuple[np.ndarray, np.ndarray]:
        """Split this scan's structured points into (Nx3 xyz array, N-length
        intensity array). Use this instead of indexing .points directly when
        you want plain arrays for math / plotting / SLAM libraries."""
        xyz = np.stack([self.points['x'], self.points['y'], self.points['z']], axis=1)
        return xyz, self.points['intensity']

    def point(self, i: int) -> LidarPoint:
        """Convenience accessor for a single point as a LidarPoint object.
        Fine for occasional single-point lookups; use xyz_intensity() instead if
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

# One point = 5 little-endian float32s (x, y, z, intensity, time) + 1 uint32 (ring).
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

    # Clamp against BOTH the protocol max and the bytes we actually received.
    # Without the second clamp, a truncated packet makes np.frombuffer raise
    # ValueError (not struct.error), which the caller's except clause missed.
    available = len(points_bytes) // _POINT_DTYPE.itemsize
    count = min(valid_points_num, MAX_POINTS_PER_SCAN, available)

    # .copy() is deliberate: np.frombuffer returns a READ-ONLY view that also
    # keeps the whole original packet alive in memory. Both are bad once these
    # arrays get stored in a buffer -- callers can't modify the array in place,
    # and memory usage balloons. ~3KB per scan, so the copy is cheap.
    points = np.frombuffer(points_bytes, dtype=_POINT_DTYPE, count=count).copy()

    return LidarScan(stamp=stamp, id=scan_id, valid_points_num=count, points=points)


def parse_packet(data: bytes) -> Optional[Union[LidarScan, LidarIMU]]:
    """Parse one full UDP datagram. Returns None if it's too short, an unknown
    message type, or malformed."""
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
    except (struct.error, ValueError) as e:
        # ValueError matters: np.frombuffer raises it on a short/corrupt scan
        # packet. In a background thread an uncaught exception would silently
        # kill the reader and the buffers would just stop updating forever.
        logger.warning("Failed to parse packet (msgType=%d): %s", msg_type, e)
        return None


# --- bounded ring buffer -------------------------------------------------------

class RingBuffer:
    """
    Thread-safe bounded buffer of the most recent N messages.

    Backed by collections.deque(maxlen=N): appending is O(1) and once it's
    full, adding a new item automatically discards the OLDEST one. That's the
    behavior you want for live sensor data -- if the consumer falls behind,
    stale readings get dropped rather than growing memory without limit.

    Every method takes a lock, because the producer (background socket thread)
    and the consumer (your main loop) touch this concurrently.
    """

    def __init__(self, maxlen: int):
        if maxlen < 1:
            raise ValueError("maxlen must be >= 1")
        self._buf: Deque = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._total_received = 0
        self._total_dropped = 0

    @property
    def maxlen(self) -> int:
        return self._buf.maxlen

    def append(self, item) -> None:
        """Add a message. Called by the reader thread; you shouldn't need this."""
        with self._condition:
            if len(self._buf) == self._buf.maxlen:
                self._total_dropped += 1
            self._buf.append(item)
            self._total_received += 1
            self._condition.notify_all()

    def latest(self):
        """Most recent message, or None if nothing has arrived yet."""
        with self._lock:
            return self._buf[-1] if self._buf else None

    def latest_n(self, n: int, allow_partial: bool = True) -> List:
        """
        The n most recent messages, oldest-first, as a plain list (a snapshot --
        safe to iterate while the reader thread keeps appending).

        Guardrails, and why they differ:

        - n > maxlen  -> ALWAYS raises ValueError. This can never be satisfied
          no matter how long you wait, so it's a bug in the calling code, not a
          transient condition. Failing loudly here is the whole point; silently
          handing back fewer would hide the mistake.

        - n > however many are buffered RIGHT NOW (but still <= maxlen) -> by
          default returns what's available. This happens constantly and
          legitimately: right after start(), during a brief sensor dropout, or
          after drain(). Raising here would crash your program on every startup.
          Pass allow_partial=False if your math genuinely requires exactly n
          samples (e.g. a fixed-window filter) and short data would silently
          corrupt the result.

        - n < 1 or non-integer -> raises, since that's always a caller bug.
        """
        # bool is a subclass of int, so True would sneak through as n=1.
        if isinstance(n, bool) or not isinstance(n, int):
            raise TypeError(f"n must be an int, got {type(n).__name__}")
        if n < 1:
            raise ValueError(f"n must be >= 1, got {n}")
        if n > self.maxlen:
            raise ValueError(
                f"Asked for the {n} most recent messages, but this buffer only "
                f"holds {self.maxlen}. Either lower n, or construct the stream "
                f"with a bigger maxlen."
            )

        with self._lock:
            available = len(self._buf)
            if n > available and not allow_partial:
                raise ValueError(
                    f"Asked for exactly {n} messages but only {available} have "
                    f"arrived so far (buffer capacity {self.maxlen}). Wait for "
                    f"more data, or use allow_partial=True."
                )
            if n >= available:
                return list(self._buf)
            # islice walks the deque once (O(n)); indexing it in a loop instead
            # would be O(n^2), since deque indexing is O(n) toward the middle.
            return list(islice(self._buf, available - n, available))

    def snapshot(self) -> List:
        """Everything currently buffered, oldest-first, as a plain list."""
        with self._lock:
            return list(self._buf)

    def drain(self) -> List:
        """Everything currently buffered, oldest-first, AND clear the buffer.
        Use this if you want each message exactly once instead of just the
        newest -- e.g. integrating every IMU sample rather than sampling it."""
        with self._lock:
            items = list(self._buf)
            self._buf.clear()
            return items

    def wait_for_new(self, timeout: Optional[float] = None):
        """Block until a new message arrives, then return it (or None on
        timeout). Cheaper and lower-latency than polling latest() in a busy
        loop, since it sleeps until the reader thread actually wakes it."""
        with self._condition:
            count_before = self._total_received
            got_one = self._condition.wait_for(
                lambda: self._total_received != count_before, timeout=timeout
            )
            if not got_one:
                return None
            return self._buf[-1] if self._buf else None

    def clear(self) -> None:
        with self._lock:
            self._buf.clear()

    @property
    def total_received(self) -> int:
        """How many messages have ever been put in (not how many are held now)."""
        with self._lock:
            return self._total_received

    @property
    def total_dropped(self) -> int:
        """How many were evicted because the buffer was full. If this climbs
        steadily, your consumer loop is slower than the sensor -- either make
        the buffer bigger or accept that you're sampling, not capturing."""
        with self._lock:
            return self._total_dropped

    def __len__(self) -> int:
        with self._lock:
            return len(self._buf)


# --- pull-style receiver (original behavior, kept) --------------------------------

class LidarUDPReceiver:
    """
    Owns a UDP socket and hands back parsed LidarScan / LidarIMU objects
    one at a time. Pure data plumbing -- no plotting, no printing by default.

    If your consumer can't keep up with the sensor, use LidarStream instead.

    Usage:
        with LidarUDPReceiver(port=12345) as receiver:
            for message in receiver.stream():
                ...
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

        return parse_packet(data)

    def stream(self) -> Iterator[Union[LidarScan, LidarIMU]]:
        """Generator that yields parsed messages forever, skipping timeouts."""
        while True:
            msg = self.receive_once()
            if msg is not None:
                yield msg


# --- push-style buffered stream ---------------------------------------------------

class LidarStream:
    """
    Runs a background thread that continuously drains the UDP socket and files
    parsed messages into two SEPARATE bounded ring buffers:

        .scans -> RingBuffer of LidarScan
        .imu   -> RingBuffer of LidarIMU

    Your main loop reads whichever it wants, whenever it wants:

        with LidarStream() as lidar:
            scan = lidar.scans.latest()      # newest scan, or None
            recent = lidar.imu.latest_n(10)  # last 10 IMU samples
            everything = lidar.imu.drain()   # all buffered IMU, and clear

    Sizing the buffers: pick roughly (sensor rate) x (how many seconds of
    history you care about). IMU comes in far faster than scans, so it wants a
    bigger buffer -- hence the separate defaults. Bigger buffers cost memory
    but reduce dropped messages if your loop stutters.

    Note on threading: this is genuinely useful here despite Python's GIL,
    because the reader thread spends nearly all its time blocked in
    socket.recvfrom() waiting on the OS, which releases the GIL. It is not
    doing CPU work that would compete with your main loop.
    """

    def __init__(self, port: int = 12345, ip: str = "0.0.0.0",
                 scan_maxlen: int = 180, imu_maxlen: int = 250,
                 timeout: float = 1.0, buffer_size: int = 65536):
        self._receiver = LidarUDPReceiver(port=port, ip=ip,
                                          timeout=timeout, buffer_size=buffer_size)
        self.scans = RingBuffer(scan_maxlen)
        self.imu = RingBuffer(imu_maxlen)

        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    # --- lifecycle ---

    def start(self) -> "LidarStream":
        if self._thread is not None and self._thread.is_alive():
            logger.warning("LidarStream already running; start() ignored.")
            return self

        self._receiver.open()
        self._stop_event.clear()
        # daemon=True so a forgotten stop() can't hang interpreter shutdown.
        self._thread = threading.Thread(target=self._run, name="lidar-udp-reader",
                                        daemon=True)
        self._thread.start()
        logger.info("LidarStream reader thread started.")
        return self

    def stop(self, join_timeout: float = 2.0) -> None:
        self._stop_event.set()
        if self._thread is not None:
            # The thread can be sitting in recvfrom() for up to `timeout`
            # seconds, so give it at least that long to notice the stop flag.
            self._thread.join(timeout=join_timeout)
            if self._thread.is_alive():
                logger.warning("Reader thread did not exit within %.1fs.", join_timeout)
            self._thread = None
        self._receiver.close()
        logger.info("LidarStream stopped.")

    def __enter__(self) -> "LidarStream":
        return self.start()

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.stop()

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # --- non-blocking "just give me the current value" accessors ---
    #
    # These are plain attribute reads: they never wait on the network, never
    # wait on the reader thread, and return immediately. If no data has
    # arrived yet they return None.
    #
    # IMPORTANT: these are SNAPSHOTS, not live references. Doing
    #     scan = lidar.latest_scan
    # copies the current value into `scan` once. `scan` will NOT change on its
    # own afterwards -- you have to read the property again to see newer data.
    # (This is just how Python names work; nothing can make a local variable
    # auto-refresh.) So read it fresh at the top of each loop iteration.

    @property
    def latest_scan(self) -> Optional[LidarScan]:
        """Most recent point cloud, or None. Non-blocking."""
        return self.scans.latest()

    @property
    def latest_imu(self) -> Optional[LidarIMU]:
        """Most recent IMU sample, or None. Non-blocking."""
        return self.imu.latest()

    def recent_scans(self, n: int, allow_partial: bool = True) -> List[LidarScan]:
        """The n most recent scans, oldest-first. Non-blocking.

        Raises ValueError if n exceeds the scan buffer's capacity (see
        scan_capacity). Returns fewer than n if fewer have arrived yet, unless
        allow_partial=False."""
        return self.scans.latest_n(n, allow_partial=allow_partial)

    def recent_imu(self, n: int, allow_partial: bool = True) -> List[LidarIMU]:
        """The n most recent IMU samples, oldest-first. Non-blocking.
        Same guardrails as recent_scans()."""
        return self.imu.latest_n(n, allow_partial=allow_partial)

    @property
    def scan_capacity(self) -> int:
        """Max scans the buffer can hold -- the ceiling for recent_scans(n)."""
        return self.scans.maxlen

    @property
    def imu_capacity(self) -> int:
        """Max IMU samples the buffer can hold -- the ceiling for recent_imu(n)."""
        return self.imu.maxlen

    # --- reader thread ---

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                msg = self._receiver.receive_once()
            except OSError as e:
                # Socket closed out from under us during shutdown is expected.
                if not self._stop_event.is_set():
                    logger.error("Socket error in reader thread: %s", e)
                break
            except Exception as e:
                # Catch-all so one weird packet can't silently kill the thread
                # and leave the buffers frozen with no visible error.
                logger.exception("Unexpected error in reader thread: %s", e)
                continue

            if msg is None:
                continue  # timeout or malformed; loop and check stop flag again

            if isinstance(msg, LidarScan):
                self.scans.append(msg)
            elif isinstance(msg, LidarIMU):
                self.imu.append(msg)

    # --- convenience ---

    def wait_until_ready(self, timeout: float = 5.0) -> bool:
        """Block until at least one scan AND one IMU message have arrived.
        Returns False on timeout. Handy at startup so your first loop iteration
        isn't full of Nones."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if len(self.scans) > 0 and len(self.imu) > 0:
                return True
            time.sleep(0.05)
        return False

    def stats(self) -> dict:
        """Counters for debugging throughput / whether you're dropping data."""
        return {
            "running": self.is_running,
            "scans_held": len(self.scans),
            "scans_total": self.scans.total_received,
            "scans_dropped": self.scans.total_dropped,
            "imu_held": len(self.imu),
            "imu_total": self.imu.total_received,
            "imu_dropped": self.imu.total_dropped,
        }


if __name__ == "__main__":
    # Minimal smoke test when run directly: confirms packets are arriving,
    # parsing, and buffering correctly.


    logging.basicConfig(level=logging.INFO, format="%(message)s")

    with LidarStream() as lidar:
        print("Waiting for first data...")
        if not lidar.wait_until_ready(timeout=5.0):
            print("No data within 5s -- is the C++ bridge running?")

        try:
            while True:
                scan = lidar.scans.latest()
                imu = lidar.imu.latest()

                if scan is not None:
                    print(f"Scan #{scan.id}: {scan.valid_points_num} points, "
                          f"stamp={scan.stamp:.3f}")
                if imu is not None:
                    print(f"IMU  #{imu.id}: stamp={imu.stamp:.3f}, "
                          f"ang_vel={imu.angular_velocity}")

                print(f"  {lidar.stats()}")
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nStopped.")
