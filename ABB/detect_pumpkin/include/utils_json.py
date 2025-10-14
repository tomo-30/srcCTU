# -*- coding: utf-8 -*-
# include/utils_json.py

import os
import json
from datetime import datetime

def timestamp_str() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

def make_log_filename(log_dir: str) -> str:
    """
    logフォルダ内に日時付きファイル名を生成（存在しないパスを返すだけ；作成はしない）
    例: log/log_20251009_135501.json
    """
    os.makedirs(log_dir, exist_ok=True)
    base = datetime.now().strftime("log_%Y%m%d_%H%M%S.json")
    return os.path.join(log_dir, base)

def write_log_entry(filepath: str, entry: dict, mode: str = "append"):
    """
    JSONログに1件追記する．
    mode:
      - "new"    : 新規作成（既存は上書き）して1件目として保存
      - "append" : 既存に追記（存在しなければ"new"相当で作成）
    """
    os.makedirs(os.path.dirname(filepath), exist_ok=True)

    if mode == "new" or not os.path.exists(filepath):
        data = {"logs": [entry]}
    else:
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)
            if "logs" not in data or not isinstance(data["logs"], list):
                data = {"logs": [entry]}
            else:
                data["logs"].append(entry)
        except Exception:
            data = {"logs": [entry]}

    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
