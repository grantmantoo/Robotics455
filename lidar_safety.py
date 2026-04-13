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
        # Narrow center-forward cone for wall-follow front obstacle checks.
        self.front_center_zones = [(350, 359), (0, 10)]
        self.rear_zones = rear_zones or [(180, 210)]
        # Robot self-echo angles to ignore globally.
        self.ignore_zones = [(108, 122), (144, 150)]
        # Extra zones for wall-follow control.
        self.right_zones = [(70, 110)]
        self.front_right_zones = [(20, 70)]
        self.back_right_zones = [(110, 150)]
        self.left_zones = [(250, 290)]
        self.front_left_zones = [(290, 340)]
        self.back_left_zones = [(210, 250)]
        self.scan_buf_meas = 400
        self.partial_update_every_meas = 40

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
        self.zone_mins_mm: Dict[str, Optional[float]] = {
            "front": None,
            "front_center": None,
            "rear": None,
            "right": None,
            "front_right": None,
            "back_right": None,
            "left": None,
            "front_left": None,
            "back_left": None,
        }
        self.last_scan_time = 0.0
        self.connected_port: Optional[str] = None
        self.last_error: Optional[str] = None

    def _set_status_fast_block(
        self,
        front_min: Optional[float],
        rear_min: Optional[float],
        zone_mins: Dict[str, Optional[float]],
    ) -> None:
        """
        Mid-scan update for lower latency blocking.
        This only forces blocked=True on hits; unblocking remains governed by
        full-scan hysteresis in _set_status().
        """
        front_hit = front_min is not None and front_min < self.stop_mm
        rear_hit = rear_min is not None and rear_min < self.stop_mm
        with self._lock:
            changed = False
            if front_hit and not self.front_blocked:
                self.front_blocked = True
                changed = True
            if rear_hit and not self.rear_blocked:
                self.rear_blocked = True
                changed = True
            self.front_min_mm = front_min
            self.rear_min_mm = rear_min
            self.zone_mins_mm = dict(zone_mins)
            self.last_scan_time = time.time()
        if changed:
            print(
                f"[LIDAR FAST] front_blocked={self.front_blocked} rear_blocked={self.rear_blocked} "
                f"front_min={front_min} rear_min={rear_min}"
            )

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

    def _set_status(
        self,
        front_min: Optional[float],
        rear_min: Optional[float],
        zone_mins: Dict[str, Optional[float]],
    ) -> None:
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
            self.zone_mins_mm = dict(zone_mins)
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

                for scan in lidar.iter_scans(max_buf_meas=self.scan_buf_meas):
                    if self._stop_evt.is_set():
                        break

                    front_min = None
                    rear_min = None
                    zone_mins: Dict[str, Optional[float]] = {
                        "front": None,
                        "front_center": None,
                        "rear": None,
                        "right": None,
                        "front_right": None,
                        "back_right": None,
                        "left": None,
                        "front_left": None,
                        "back_left": None,
                    }

                    def update_zone(name: str, distance: float):
                        current = zone_mins[name]
                        if current is None or distance < current:
                            zone_mins[name] = distance

                    meas_count = 0
                    for _quality, angle, distance in scan:
                        if distance <= 0:
                            continue
                        meas_count += 1
                        a = int(angle) % 360
                        if any(_in_zone(a, z) for z in self.ignore_zones):
                            continue

                        if any(_in_zone(a, z) for z in self.front_zones):
                            if front_min is None or distance < front_min:
                                front_min = distance
                            update_zone("front", distance)
                        if any(_in_zone(a, z) for z in self.front_center_zones):
                            update_zone("front_center", distance)
                        if any(_in_zone(a, z) for z in self.rear_zones):
                            if rear_min is None or distance < rear_min:
                                rear_min = distance
                            update_zone("rear", distance)

                        if any(_in_zone(a, z) for z in self.right_zones):
                            update_zone("right", distance)
                        if any(_in_zone(a, z) for z in self.front_right_zones):
                            update_zone("front_right", distance)
                        if any(_in_zone(a, z) for z in self.back_right_zones):
                            update_zone("back_right", distance)

                        if any(_in_zone(a, z) for z in self.left_zones):
                            update_zone("left", distance)
                        if any(_in_zone(a, z) for z in self.front_left_zones):
                            update_zone("front_left", distance)
                        if any(_in_zone(a, z) for z in self.back_left_zones):
                            update_zone("back_left", distance)

                        if (
                            self.partial_update_every_meas > 0
                            and (meas_count % self.partial_update_every_meas) == 0
                        ):
                            self._set_status_fast_block(front_min, rear_min, zone_mins)

                    self._set_status(front_min, rear_min, zone_mins)

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
                "zone_mins_mm": dict(self.zone_mins_mm),
                "stop_mm": self.stop_mm,
                "clear_scans_required": self.clear_scans_required,
                "block_scans_required": self.block_scans_required,
                "front_clear_scan_count": self._front_clear_scan_count,
                "rear_clear_scan_count": self._rear_clear_scan_count,
                "front_zones": self.front_zones,
                "front_center_zones": self.front_center_zones,
                "rear_zones": self.rear_zones,
                "ignore_zones": self.ignore_zones,
                "right_zones": self.right_zones,
                "front_right_zones": self.front_right_zones,
                "back_right_zones": self.back_right_zones,
                "left_zones": self.left_zones,
                "front_left_zones": self.front_left_zones,
                "back_left_zones": self.back_left_zones,
                "last_scan_time": self.last_scan_time,
                "connected_port": self.connected_port,
                "last_error": self.last_error,
                "scan_buf_meas": self.scan_buf_meas,
                "partial_update_every_meas": self.partial_update_every_meas,
            }
