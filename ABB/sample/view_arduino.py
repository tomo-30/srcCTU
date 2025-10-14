# -*- coding: utf-8 -*-
# monitor_9slide.py
# 目的:
#   Arduino Due から 9軸リニアポテンショメータの値を受信し，
#   各センサの値をリアルタイム表示する．
#   形式: 1:23,2:-1,3:17,...,9:5

import serial
import time

# ====== シリアル設定 ======
PORT = "COM4"          # ← 環境に合わせて変更（例: '/dev/ttyACM0'）
BAUDRATE = 115200

# ====== 初期化 ======
try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print(f"[INFO] Connected to {PORT} ({BAUDRATE} bps)")
except serial.SerialException:
    print("[ERROR] シリアルポートが開けませんでした．")
    exit(1)

# ====== メインループ ======
try:
    while True:
        line = ser.readline().decode("utf-8").strip()
        if not line:
            continue

        # 例: "1:23,2:-1,3:17,4:0,5:12,6:8,7:-1,8:30,9:5"
        try:
            pairs = line.split(",")
            values = {}
            for p in pairs:
                id_str, val_str = p.split(":")
                values[int(id_str)] = int(val_str)

            # 整形して表示
            display = "  ".join([f"{i}:{values[i]:>3}" for i in sorted(values.keys())])
            print(display)

        except Exception as e:
            print(f"[WARN] 受信形式エラー: {line} ({e})")

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n[INFO] 停止しました．")

finally:
    ser.close()
