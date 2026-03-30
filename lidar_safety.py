import threading
import time
from typing import Dict, List, Optional, Tuple

from serial.tools import list_ports
from rplidar import RPLidar


def _in_zone(angle_deg: float, zone: Tuple[int, int]) -> bool:
    lo, hi = zone
    if lo <= hi:
        return lo <= angle_deg <= hi
    # Wrapped zone, e.g. 330..30
    return angle_deg >= lo or angle_deg <= hi


class LidarSafetyMonitor:
    def __init__(
        self,
        port: str = "auto",
        stop_mm: int = 800,
        clear_scans_required: int = 3,
        block_scans_required: int = 1,
        front_zones: Optional[List[Tuple[int, int]]] = None,
        rear_zones: Optional[List[Tuple[int, int]]] = None,
    ):
        self.requested_port = port
        self.stop_mm = int(stop_mm)
        self.clear_scans_required = max(1, int(clear_scans_required))
        self.block_scans_required = max(1, int(block_scans_required))
        self.front_zones = front_zones or [(330, 359), (0, 30)]
        self.rear_zones = rear_zones or [(150, 210)]

        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self.front_blocked = False
        self.rear_blocked = False
        self._front_block_scan_count = 0
        self._rear_block_scan_count = 0
        self._front_clear_scan_count = 0
        self._rear_clear_scan_count = 0
        self.front_min_mm: Optional[float] = None
        self.rear_min_mm: Optional[float] = None
        self.last_scan_time = 0.0
        self.connected_port: Optional[str] = None
        self.last_error: Optional[str] = None

    def _pick_port(self) -> Optional[str]:
        if self.requested_port and self.requested_port != "auto":
            return self.requested_port

        ports = list(list_ports.comports())
        # Prefer Slamtec/RPLidar-identifiable devices.
        for p in ports:
            hay = " ".join(
                x for x in [p.manufacturer, p.product, p.description] if x
            ).lower()
            if "slamtec" in hay or "rplidar" in hay:
                return p.device

        # Fallback: first ACM/USB serial device.
        for p in ports:
            if p.device.startswith("/dev/ttyACM") or p.device.startswith("/dev/ttyUSB"):
                return p.device
        return None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def _set_status(self, front_min: Optional[float], rear_min: Optional[float]) -> None:
        front_hit = front_min is not None and front_min < self.stop_mm
        rear_hit = rear_min is not None and rear_min < self.stop_mm

        with self._lock:
            prev_f = self.front_blocked
            prev_r = self.rear_blocked

            if front_hit:
                self._front_block_scan_count += 1
                self._front_clear_scan_count = 0
            else:
                self._front_clear_scan_count += 1
                self._front_block_scan_count = 0

            if rear_hit:
                self._rear_block_scan_count += 1
                self._rear_clear_scan_count = 0
            else:
                self._rear_clear_scan_count += 1
                self._rear_block_scan_count = 0

            # Hysteresis: block quickly, unblock only after N clear scans.
            if not self.front_blocked and self._front_block_scan_count >= self.block_scans_required:
                self.front_blocked = True
            elif self.front_blocked and self._front_clear_scan_count >= self.clear_scans_required:
                self.front_blocked = False

            if not self.rear_blocked and self._rear_block_scan_count >= self.block_scans_required:
                self.rear_blocked = True
            elif self.rear_blocked and self._rear_clear_scan_count >= self.clear_scans_required:
                self.rear_blocked = False

            self.front_min_mm = front_min
            self.rear_min_mm = rear_min
            self.last_scan_time = time.time()

        if prev_f != self.front_blocked or prev_r != self.rear_blocked:
            print(
                f"[LIDAR] front_blocked={self.front_blocked} rear_blocked={self.rear_blocked} "
                f"front_min={front_min} rear_min={rear_min} "
                f"front_clear_scans={self._front_clear_scan_count} "
                f"rear_clear_scans={self._rear_clear_scan_count}"
            )

    def _run(self) -> None:
        while not self._stop_evt.is_set():
            port = self._pick_port()
            if not port:
                with self._lock:
                    self.last_error = "No lidar serial device found"
                    self.connected_port = None
                time.sleep(1.0)
                continue

            lidar = None
            try:
                print(f"[LIDAR] connecting on {port}")
                lidar = RPLidar(port, timeout=1)
                with self._lock:
                    self.connected_port = port
                    self.last_error = None

                for scan in lidar.iter_scans(max_buf_meas=1000):
                    if self._stop_evt.is_set():
                        break

                    front_min = None
                    rear_min = None
                    for _quality, angle, distance in scan:
                        if distance <= 0:
                            continue
                        a = int(angle) % 360

                        if any(_in_zone(a, z) for z in self.front_zones):
                            if front_min is None or distance < front_min:
                                front_min = distance
                        if any(_in_zone(a, z) for z in self.rear_zones):
                            if rear_min is None or distance < rear_min:
                                rear_min = distance

                    self._set_status(front_min, rear_min)

            except Exception as ex:
                with self._lock:
                    self.last_error = str(ex)
                print(f"[LIDAR WARN] {ex}")
                time.sleep(1.0)
            finally:
                if lidar is not None:
                    try:
                        lidar.stop()
                    except Exception:
                        pass
                    try:
                        lidar.stop_motor()
                    except Exception:
                        pass
                    try:
                        lidar.disconnect()
                    except Exception:
                        pass
                with self._lock:
                    self.connected_port = None

    def status(self) -> Dict[str, object]:
        with self._lock:
            return {
                "front_blocked": self.front_blocked,
                "rear_blocked": self.rear_blocked,
                "front_min_mm": self.front_min_mm,
                "rear_min_mm": self.rear_min_mm,
                "stop_mm": self.stop_mm,
                "clear_scans_required": self.clear_scans_required,
                "block_scans_required": self.block_scans_required,
                "front_clear_scan_count": self._front_clear_scan_count,
                "rear_clear_scan_count": self._rear_clear_scan_count,
                "front_zones": self.front_zones,
                "rear_zones": self.rear_zones,
                "last_scan_time": self.last_scan_time,
                "connected_port": self.connected_port,
                "last_error": self.last_error,
            }
