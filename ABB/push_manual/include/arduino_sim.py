# include/arduino_sim.py
import threading, time
from typing import Callable, List

class ArduinoSim:
    """100msごとに9値をストリーム配信．GUIの送信ボタンで5フレームだけ“指定値”を流し，その後は -1 に戻す"""
    def __init__(self, hz: float = 10.0, forced_frames: int = 5):
        self.period = 1.0 / max(1e-3, hz)
        self.forced_frames = forced_frames
        self._subscribers: List[Callable[[List[int]], None]] = []
        self._lock = threading.Lock()
        self._forced_values = None    # type: List[int] | None
        self._remain = 0
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def subscribe(self, cb: Callable[[list], None]):
        with self._lock:
            self._subscribers.append(cb)

    def send_once_for_next_frames(self, values9: List[int]):
        if len(values9) != 9:
            raise ValueError("9値が必要です")
        with self._lock:
            self._forced_values = list(map(int, values9))
            self._remain = self.forced_frames

    def _loop(self):
        while not self._stop.is_set():
            with self._lock:
                if self._forced_values is not None and self._remain > 0:
                    payload = list(self._forced_values)
                    self._remain -= 1
                    if self._remain <= 0:
                        self._forced_values = None
                else:
                    payload = [-1]*9
                subs = list(self._subscribers)
            # 配信
            for cb in subs:
                try:
                    cb(payload)
                except Exception:
                    pass
            time.sleep(self.period)

    def stop(self):
        self._stop.set()
        self._th.join(timeout=1.0)
