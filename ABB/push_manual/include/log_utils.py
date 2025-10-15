# include/log_utils.py
import os, json, threading
from datetime import datetime

_log_lock = threading.Lock()

def append_json_line(log_dir: str, filename: str, obj: dict):
    os.makedirs(log_dir, exist_ok=True)
    path = os.path.join(log_dir, filename)
    rec = {"ts_iso": datetime.utcnow().isoformat() + "Z", **obj}
    with _log_lock:
        with open(path, "a", encoding="utf-8") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
