import threading
import time
from typing import Dict, Optional


def _clamp(x: int, lo: int, hi: int) -> int:
    return lo if x < lo else hi if x > hi else x


class WallFollower:
    """
    Simple autonomous wall follower using lidar zone mins.
    """

    def __init__(self, ctrl, lidar_monitor):
        self.ctrl = ctrl
        self.lidar = lidar_monitor
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None

        # Runtime config
        self.side = "right"  # "right" or "left"
        self.target_mm = 1000
        self.tolerance_mm = 150
        self.base_speed = 1000
        self.min_base_speed = 1000
        self.correction_band = 200
        self.steer_delta = 260
        self.steer_min = 60
        self.steer_max = 170
        self.steer_kp = 0.25  # delta ~= kp * |distance_error_mm|
        self.turn_gain = 1.20
        self.dist_kp = 0.12
        self.heading_kp = 0.35
        self.turn_deadband = 2
        self.search_delta = 120
        self.wheel_step_min = 10
        self.corner_turn_boost = 0.75
        self.corner_jump_factor = 1.40
        self.front_stop_mm = 320
        self.front_emergency_mm = 300
        self.front_slow_mm = 800
        self.front_min_scale = 0.30
        self.front_turn_start_mm = 950
        self.front_turn_full_mm = 420
        self.front_turn_max = 1100
        self.front_turn_center_weight = 0.65
        self.front_turn_curve = 2.2
        self.front_diag_weight = 0.90
        self.lost_wall_mm = 2400
        self.loop_period_s = 0.06
        self.reverse_time_s = 0.35
        self.turn_time_s = 0.45
        self._last_recovery_turn = "left"
        self._last_error_mm: Optional[float] = None
        self._prev_side_dist_mm: Optional[float] = None

        # Status
        self.active = False
        self.last_state = "IDLE"
        self.last_side_mm: Optional[float] = None
        self.last_front_mm: Optional[float] = None
        self.last_cmd = {"left": 0, "right": 0}
        self.last_update_ts = 0.0

    def start(
        self,
        side: str = "left",
        target_mm: int = 800,
        tolerance_mm: int = 150,
        base_speed: int = 1000,
        correction_band: int = 200,
        steer_delta: int = 260,
        steer_min: int = 60,
        steer_max: int = 170,
        steer_kp: float = 0.25,
        turn_gain: float = 1.20,
        dist_kp: float = 0.12,
        heading_kp: float = 0.35,
        turn_deadband: int = 2,
        search_delta: int = 120,
        wheel_step_min: int = 10,
        corner_turn_boost: float = 0.75,
        corner_jump_factor: float = 1.40,
        front_stop_mm: int = 320,
        front_emergency_mm: int = 300,
        front_slow_mm: int = 800,
        front_min_scale: float = 0.30,
        front_turn_start_mm: int = 950,
        front_turn_full_mm: int = 420,
        front_turn_max: int = 1100,
        front_turn_center_weight: float = 0.65,
        front_turn_curve: float = 2.2,
        front_diag_weight: float = 0.90,
        lost_wall_mm: int = 2400,
        reverse_time_s: float = 0.35,
        turn_time_s: float = 0.45,
    ) -> None:
        side = side.lower().strip()
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")

        with self._lock:
            self.side = side
            self.target_mm = int(target_mm)
            self.tolerance_mm = int(tolerance_mm)
            self.base_speed = max(self.min_base_speed, int(base_speed))
            self.correction_band = max(50, int(correction_band))
            self.steer_delta = int(steer_delta)
            self.steer_min = int(steer_min)
            self.steer_max = min(int(steer_max), self.correction_band)
            self.steer_kp = float(steer_kp)
            self.turn_gain = max(1.0, float(turn_gain))
            self.dist_kp = float(dist_kp)
            self.heading_kp = float(heading_kp)
            self.turn_deadband = max(0, int(turn_deadband))
            self.search_delta = min(int(search_delta), self.correction_band)
            self.wheel_step_min = max(0, int(wheel_step_min))
            self.corner_turn_boost = _clamp(int(corner_turn_boost * 100), 50, 100) / 100.0
            self.corner_jump_factor = _clamp(int(corner_jump_factor * 100), 100, 300) / 100.0
            self.front_stop_mm = int(front_stop_mm)
            self.front_emergency_mm = min(self.front_stop_mm, int(front_emergency_mm))
            self.front_slow_mm = max(self.front_stop_mm + 50, int(front_slow_mm))
            self.front_min_scale = _clamp(int(front_min_scale * 100), 10, 90) / 100.0
            self.front_turn_start_mm = max(self.front_stop_mm + 100, int(front_turn_start_mm))
            self.front_turn_full_mm = max(
                self.front_emergency_mm + 20,
                min(int(front_turn_full_mm), self.front_turn_start_mm - 50),
            )
            self.front_turn_max = int(_clamp(int(front_turn_max), 0, 1600))
            self.front_turn_center_weight = _clamp(int(front_turn_center_weight * 100), 0, 100) / 100.0
            self.front_turn_curve = max(1.0, float(front_turn_curve))
            self.front_diag_weight = _clamp(int(front_diag_weight * 100), 25, 150) / 100.0
            self.lost_wall_mm = int(lost_wall_mm)
            self.reverse_time_s = float(reverse_time_s)
            self.turn_time_s = float(turn_time_s)
            self._last_error_mm = None
            self._prev_side_dist_mm = None

        if self._thread and self._thread.is_alive():
            return

        self._stop_evt.clear()
        self.active = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print(
            f"[WALL] started side={self.side} target_mm={self.target_mm} "
            f"tol={self.tolerance_mm} set_speed={self.base_speed} delta={self.correction_band}"
        )

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.2)
        self.active = False
        self.last_state = "IDLE"
        self.last_cmd = {"left": 0, "right": 0}
        try:
            self.ctrl.stop()
        except Exception:
            pass
        print("[WALL] stopped")

    def _issue_drive(self, left: int, right: int, state: str) -> None:
        left = int(_clamp(left, -1600, 1600))
        right = int(_clamp(right, -1600, 1600))

        # For wall/search correction states, enforce a minimum per-cycle step so
        # commands don't dither with tiny changes.
        if self.wheel_step_min > 0 and ("TOO_" in state or "SEARCH_" in state):
            prev_left = int(self.last_cmd.get("left", 0))
            prev_right = int(self.last_cmd.get("right", 0))

            if left != prev_left and abs(left - prev_left) < self.wheel_step_min:
                left = prev_left + (self.wheel_step_min if left > prev_left else -self.wheel_step_min)
            if right != prev_right and abs(right - prev_right) < self.wheel_step_min:
                right = prev_right + (self.wheel_step_min if right > prev_right else -self.wheel_step_min)

            left = int(_clamp(left, -1600, 1600))
            right = int(_clamp(right, -1600, 1600))

        if hasattr(self.ctrl, "drive_autonomous"):
            self.ctrl.drive_autonomous(left, right)
        else:
            self.ctrl.drive(left, right)
        self.last_cmd = {"left": left, "right": right}
        self.last_state = state
        self.last_update_ts = time.time()

    def _band_clamp(self, speed: int) -> int:
        lo = self.base_speed - self.correction_band
        hi = self.base_speed + self.correction_band
        return int(_clamp(speed, lo, hi))

    def _steer_from_error(self, error_mm: float) -> int:
        # Responsive near target + capped farther away.
        abs_err = abs(float(error_mm))
        span = max(1.0, float(self.tolerance_mm))
        ratio = min(1.0, abs_err / span)
        shaped = ratio ** 0.85
        steer = int(round(self.steer_min + (self.steer_max - self.steer_min) * shaped))
        return int(_clamp(steer, self.steer_min, self.steer_max))

    def _front_speed_scale(self, front_mm: Optional[float]) -> float:
        # Gradual decel as front distance enters slow zone.
        if front_mm is None:
            return 1.0
        d = float(front_mm)
        if d >= float(self.front_slow_mm):
            return 1.0
        if d <= float(self.front_stop_mm):
            return self.front_min_scale
        span = float(self.front_slow_mm - self.front_stop_mm)
        ratio = (d - float(self.front_stop_mm)) / span
        return self.front_min_scale + (1.0 - self.front_min_scale) * ratio

    def _issue_forward_drive(self, left: int, right: int, front_mm: Optional[float], state: str) -> None:
        # Apply front-based slowdown only for forward wall-follow motion.
        if left > 0 and right > 0:
            scale = self._front_speed_scale(front_mm)
            left = int(round(left * scale))
            right = int(round(right * scale))
        self._issue_drive(left, right, state)

    def _effective_front_distance(
        self,
        front_mm: Optional[float],
        front_left_mm: Optional[float],
        front_right_mm: Optional[float],
    ) -> Optional[float]:
        distances = []
        if front_mm is not None:
            distances.append(float(front_mm))
        if front_left_mm is not None:
            distances.append(float(front_left_mm) / self.front_diag_weight)
        if front_right_mm is not None:
            distances.append(float(front_right_mm) / self.front_diag_weight)
        if not distances:
            return None
        return min(distances)

    def _proximity_ratio(self, distance_mm: Optional[float], start_mm: int, full_mm: int) -> float:
        if distance_mm is None:
            return 0.0
        d = float(distance_mm)
        if d >= float(start_mm):
            return 0.0
        if d <= float(full_mm):
            return 1.0
        span = max(1.0, float(start_mm - full_mm))
        return (float(start_mm) - d) / span

    def _front_turn_bias(self, front_mm: Optional[float], front_side_mm: Optional[float], front_corner_mm: Optional[float]) -> int:
        side_ratio = self._proximity_ratio(front_side_mm, self.front_turn_start_mm, self.front_turn_full_mm)
        center_ratio = self._proximity_ratio(front_mm, self.front_turn_start_mm, self.front_turn_full_mm)
        corner_ratio = self._proximity_ratio(front_corner_mm, self.front_turn_start_mm, self.front_turn_full_mm)
        combined = max(side_ratio, center_ratio * self.front_turn_center_weight)
        combined = max(combined, corner_ratio)
        if combined <= 0.0:
            return 0
        shaped = combined ** self.front_turn_curve
        return int(round(self.front_turn_max * shaped))

    def _compose_track_drive(self, turn_cmd: int) -> tuple[int, int]:
        cmd = int(_clamp(turn_cmd, -1600, 1600))
        base_delta = min(abs(cmd), self.correction_band)
        extra_pivot = max(0, abs(cmd) - self.correction_band)

        if cmd >= 0:
            fast = self.base_speed + base_delta
            slow = self.base_speed - base_delta - extra_pivot
        else:
            fast = self.base_speed + base_delta
            slow = self.base_speed - base_delta - extra_pivot

        fast = int(_clamp(fast, -1600, 1600))
        slow = int(_clamp(slow, -1600, 1600))

        if self.side == "right":
            return (fast, slow) if cmd >= 0 else (slow, fast)
        return (slow, fast) if cmd >= 0 else (fast, slow)

    def _log_cycle(
        self,
        left_mm: Optional[float],
        right_mm: Optional[float],
        side_mm: Optional[float],
        front_mm: Optional[float],
    ) -> None:
        cmd = self.last_cmd
        print(
            "[WALL] "
            f"state={self.last_state} "
            f"left_mm={left_mm} right_mm={right_mm} side_mm={side_mm} front_mm={front_mm} "
            f"wheel_left={cmd.get('left')} wheel_right={cmd.get('right')}"
        )

    def _run(self) -> None:
        while not self._stop_evt.is_set():
            try:
                s = self.lidar.status()
                z = s.get("zone_mins_mm", {}) or {}
                front = z.get("front_center")
                if front is None:
                    front = z.get("front")
                front_left = z.get("front_left")
                front_right = z.get("front_right")
                back_left = z.get("back_left")
                back_right = z.get("back_right")
                left_dist = z.get("left")
                right_dist = z.get("right")
                side_key = "right" if self.side == "right" else "left"
                side_dist = z.get(side_key)
                if self.side == "right":
                    front_side = front_right
                    back_side = back_right
                else:
                    front_side = front_left
                    back_side = back_left
                front_effective = self._effective_front_distance(front, front_left, front_right)
                front_corner = front_left
                if front_corner is None or (front_right is not None and float(front_right) < float(front_corner)):
                    front_corner = front_right

                self.last_front_mm = front
                self.last_side_mm = side_dist

                # Global emergency override only for imminent collision.
                if front_effective is not None and front_effective < self.front_emergency_mm:
                    self._issue_drive(-self.base_speed, -self.base_speed, "FRONT_EMERGENCY_REVERSE")
                    self._log_cycle(left_dist, right_dist, side_dist, front)
                    time.sleep(self.reverse_time_s)
                    if self._stop_evt.is_set():
                        break

                    left_open = float(front_left) if front_left is not None else -1.0
                    right_open = float(front_right) if front_right is not None else -1.0
                    if right_open > left_open:
                        turn_dir = "right"
                    elif left_open > right_open:
                        turn_dir = "left"
                    else:
                        turn_dir = self._last_recovery_turn

                    self._last_recovery_turn = turn_dir
                    if turn_dir == "right":
                        self._issue_drive(self.base_speed, -self.base_speed, "FRONT_EMERGENCY_TURN_RIGHT")
                    else:
                        self._issue_drive(-self.base_speed, self.base_speed, "FRONT_EMERGENCY_TURN_LEFT")
                    self._log_cycle(left_dist, right_dist, side_dist, front)
                    time.sleep(self.turn_time_s)
                    continue

                # Front stop fallback only when side wall data is unavailable.
                if (
                    (side_dist is None or side_dist > self.lost_wall_mm)
                    and front_effective is not None
                    and front_effective < self.front_stop_mm
                ):
                    # Perpendicular-wall recovery:
                    # 1) back up briefly
                    # 2) commit to one turn direction (prefer open side)
                    self._issue_drive(-self.base_speed, -self.base_speed, "FRONT_BLOCKED_REVERSE")
                    self._log_cycle(left_dist, right_dist, side_dist, front)
                    time.sleep(self.reverse_time_s)
                    if self._stop_evt.is_set():
                        break

                    left_open = float(front_left) if front_left is not None else -1.0
                    right_open = float(front_right) if front_right is not None else -1.0
                    if right_open > left_open:
                        turn_dir = "right"
                    elif left_open > right_open:
                        turn_dir = "left"
                    else:
                        # Tie/unknown: keep previous commit to avoid flip-flop.
                        turn_dir = self._last_recovery_turn

                    self._last_recovery_turn = turn_dir
                    if turn_dir == "right":
                        self._issue_drive(self.base_speed, -self.base_speed, "FRONT_BLOCKED_TURN_RIGHT")
                    else:
                        self._issue_drive(-self.base_speed, self.base_speed, "FRONT_BLOCKED_TURN_LEFT")
                    self._log_cycle(left_dist, right_dist, side_dist, front)
                    time.sleep(self.turn_time_s)
                    continue

                # Case 4: wall lost/open -> gentle search toward wall side.
                if side_dist is None or side_dist > self.lost_wall_mm:
                    steer = int(_clamp(round(self.search_delta * self.turn_gain), self.steer_min, self.correction_band))
                    if self.side == "right":
                        self._issue_forward_drive(
                            self._band_clamp(self.base_speed + steer),
                            self._band_clamp(self.base_speed - steer),
                            front,
                            "SEARCH_RIGHT_COMMIT",
                        )
                    else:
                        self._issue_forward_drive(
                            self._band_clamp(self.base_speed - steer),
                            self._band_clamp(self.base_speed + steer),
                            front,
                            "SEARCH_LEFT_COMMIT",
                        )
                    self._log_cycle(left_dist, right_dist, side_dist, front)
                    time.sleep(self.loop_period_s)
                    continue

                # Continuous wall control (distance + heading):
                # toward_cmd > 0 means "turn toward followed wall"
                # heading_err = front_side - back_side:
                #   positive -> nose points away from wall (should turn toward wall)
                dist_err = float(side_dist) - float(self.target_mm)
                heading_err = 0.0
                if front_side is not None and back_side is not None:
                    heading_err = float(front_side) - float(back_side)

                toward_cmd = (self.dist_kp * dist_err) + (self.heading_kp * heading_err)
                toward_cmd *= self.turn_gain
                toward_cmd = _clamp(int(round(toward_cmd)), -self.steer_max, self.steer_max)
                if abs(toward_cmd) <= self.turn_deadband:
                    toward_cmd = 0

                front_turn_bias = self._front_turn_bias(front_effective, front_side, front_corner)
                if front_turn_bias > 0:
                    toward_cmd = min(toward_cmd, -front_turn_bias)

                # Corner boost: if side distance jumps by more than target in one cycle,
                # force a strong turn to account for sharp corners.
                corner_triggered = False
                if self._prev_side_dist_mm is not None:
                    side_jump = float(side_dist) - float(self._prev_side_dist_mm)
                    jump_trigger = float(self.target_mm) * self.corner_jump_factor
                    if side_jump > jump_trigger:
                        # Sudden opening on followed side -> turn toward wall strongly.
                        forced = int(round(self.steer_max * self.corner_turn_boost))
                        toward_cmd = max(toward_cmd, forced)
                        corner_triggered = True
                    elif side_jump < -jump_trigger:
                        # Sudden closing on followed side -> turn away strongly.
                        forced = int(round(self.steer_max * self.corner_turn_boost))
                        toward_cmd = min(toward_cmd, -forced)
                        corner_triggered = True
                self._prev_side_dist_mm = float(side_dist)

                left_cmd, right_cmd = self._compose_track_drive(toward_cmd)

                if toward_cmd == 0:
                    state = "TRACK_OK_FORWARD"
                elif toward_cmd > 0:
                    state = "TRACK_TOWARD_WALL"
                else:
                    state = "TRACK_AWAY_FROM_WALL"
                if corner_triggered:
                    state = f"{state}_CORNER_BOOST"
                elif front_turn_bias > 0:
                    state = f"{state}_FRONT_BIAS"

                self._issue_forward_drive(left_cmd, right_cmd, front_effective, state)
                self._last_error_mm = dist_err
                self._log_cycle(left_dist, right_dist, side_dist, front)

            except Exception as ex:
                self.last_state = f"ERROR: {ex}"
                self.last_update_ts = time.time()
                print(f"[WALL WARN] {ex}")
                try:
                    self.ctrl.stop()
                except Exception:
                    pass
                time.sleep(0.25)

            time.sleep(self.loop_period_s)

    def status(self) -> Dict[str, object]:
        with self._lock:
            cfg = {
                "side": self.side,
                "target_mm": self.target_mm,
                "tolerance_mm": self.tolerance_mm,
                "base_speed": self.base_speed,
                "correction_band": self.correction_band,
                "min_speed": self.base_speed - self.correction_band,
                "max_speed": self.base_speed + self.correction_band,
                "steer_delta": self.steer_delta,
                "steer_min": self.steer_min,
                "steer_max": self.steer_max,
                "steer_kp": self.steer_kp,
                "turn_gain": self.turn_gain,
                "dist_kp": self.dist_kp,
                "heading_kp": self.heading_kp,
                "turn_deadband": self.turn_deadband,
                "search_delta": self.search_delta,
                "wheel_step_min": self.wheel_step_min,
                "corner_turn_boost": self.corner_turn_boost,
                "corner_jump_factor": self.corner_jump_factor,
                "front_stop_mm": self.front_stop_mm,
                "front_emergency_mm": self.front_emergency_mm,
                "front_slow_mm": self.front_slow_mm,
                "front_min_scale": self.front_min_scale,
                "front_turn_start_mm": self.front_turn_start_mm,
                "front_turn_full_mm": self.front_turn_full_mm,
                "front_turn_max": self.front_turn_max,
                "front_turn_center_weight": self.front_turn_center_weight,
                "front_turn_curve": self.front_turn_curve,
                "front_diag_weight": self.front_diag_weight,
                "lost_wall_mm": self.lost_wall_mm,
                "reverse_time_s": self.reverse_time_s,
                "turn_time_s": self.turn_time_s,
                "loop_period_s": self.loop_period_s,
            }
        return {
            "active": self.active,
            "config": cfg,
            "last_state": self.last_state,
            "last_side_mm": self.last_side_mm,
            "last_front_mm": self.last_front_mm,
            "last_cmd": dict(self.last_cmd),
            "last_update_ts": self.last_update_ts,
        }
