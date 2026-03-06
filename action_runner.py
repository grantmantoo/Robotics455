import queue
import threading
import time
from typing import Callable, List, Optional


class ActionRunner:
    def __init__(self, ctrl, on_state_change: Optional[Callable[[Optional[str]], None]] = None):
        self.ctrl = ctrl
        self.on_state_change = on_state_change
        self.q: "queue.Queue[List[str]]" = queue.Queue()
        self.lock = threading.Lock()
        self.cancel_event = threading.Event()
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

    def enqueue(self, actions: List[str]) -> None:
        if not actions:
            return
        self.q.put(actions)

    def interrupt(self) -> None:
        with self.lock:
            self.cancel_event.set()
            while True:
                try:
                    self.q.get_nowait()
                except queue.Empty:
                    break
        try:
            self.ctrl.stop()
        except Exception as ex:
            print(f"[ACTION] stop failed during interrupt: {ex}")

    def _set_state(self, value: Optional[str]) -> None:
        if self.on_state_change:
            self.on_state_change(value)

    def _sleep_with_cancel(self, seconds: float, deadline: float) -> bool:
        end = time.time() + seconds
        while time.time() < end:
            if self.cancel_event.is_set() or time.time() > deadline:
                return False
            time.sleep(0.03)
        return True

    def _run_action(self, action: str) -> None:
        caps = {
            "head_yes": 3.0,
            "head_no": 3.0,
            "arm_raise": 4.0,
            "dance90": 6.0,
        }
        cap = caps.get(action)
        if cap is None:
            print(f"[ACTION] warning: unknown action <{action}> ignored")
            return

        deadline = time.time() + cap
        print(f"[ACTION] start <{action}> cap={cap:.1f}s")
        self._set_state("EXEC_ACTIONS")

        if action == "head_yes":
            base = self.ctrl.robot.servo_neutral("head_tilt")
            self.ctrl.head_tilt(min(8000, base + 900))
            if not self._sleep_with_cancel(0.4, deadline):
                return
            self.ctrl.head_tilt(max(2000, base - 900))
            if not self._sleep_with_cancel(0.4, deadline):
                return
            self.ctrl.head_tilt(base)
            self._sleep_with_cancel(0.2, deadline)
            return

        if action == "head_no":
            base = self.ctrl.robot.servo_neutral("head_pan")
            self.ctrl.head_pan(min(8000, base + 1400))
            if not self._sleep_with_cancel(0.35, deadline):
                return
            self.ctrl.head_pan(max(2000, base - 1400))
            if not self._sleep_with_cancel(0.45, deadline):
                return
            self.ctrl.head_pan(base)
            self._sleep_with_cancel(0.2, deadline)
            return

        if action == "arm_raise":
            self.ctrl.arm_raise_sequence(deadline=deadline, cancel_event=self.cancel_event)
            return

        if action == "dance90":
            # Wheel deadman safety: always stop wheels on exit.
            try:
                # Add waist dance motion during rotation.
                self.ctrl.waist(2000)
                if not self._sleep_with_cancel(0.2, deadline):
                    return
                self.ctrl.turn_left(1000)
                if not self._sleep_with_cancel(0.6, deadline):
                    return
                self.ctrl.stop()
                if not self._sleep_with_cancel(0.15, deadline):
                    return
                self.ctrl.waist(8000)
                if not self._sleep_with_cancel(0.2, deadline):
                    return
                self.ctrl.turn_right(1300)
                if not self._sleep_with_cancel(0.6, deadline):
                    return
                self.ctrl.waist(self.ctrl.robot.servo_neutral("waist"))
                if not self._sleep_with_cancel(0.15, deadline):
                    return
            finally:
                self.ctrl.stop()
                self.ctrl.waist(self.ctrl.robot.servo_neutral("waist"))

    def _worker_loop(self) -> None:
        while True:
            actions = self.q.get()
            self.cancel_event.clear()
            for action in actions:
                if self.cancel_event.is_set():
                    break
                try:
                    self._run_action(action)
                except Exception as ex:
                    print(f"[ACTION] error in <{action}>: {ex}")
                    try:
                        self.ctrl.stop()
                    except Exception:
                        pass
            # Clear override so state falls back to dialog engine state.
            self._set_state(None)
