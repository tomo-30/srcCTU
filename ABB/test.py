# -*- coding: utf-8 -*-
# move_direct_socket.py
# open_abb の abb.py を使わず，IRC5 の SERVER.mod に「生コマンド」を送る最小完全版
# CASE 対応:
#   8: 速度設定  → "8 v_tcp v_ori #"
#   9: ゾーン設定→ "9 0 p_tcp p_ori z_ori #"
#   1: MoveL     → "1 x y z q1 q2 q3 q4 #"
#   3: 現在TCP   → "3 #"

import socket
import time

ROBOT_IP    = "192.168.125.1"   # IRC5 の実IP
ROBOT_PORT  = 5000              # SERVER.mod の serverPort と一致させる

# 目標TCP（x,y,z[mm], q1,q2,q3,q4）※姿勢は現場toolに合わせて要調整
TARGET      = [350, 0, 300, 0, 1, 0, 0]

# 安全のため超低速＆ゆるいゾーンから
V_TCP, V_ORI          = 50, 50          # mm/s, deg/s （CASE 8 の2引数版）
P_TCP, P_ORI, Z_ORI   = 10, 10, 10      # CASE 9（fine=0で有効）

def send_cmd(sock: socket.socket, parts, pause=0.05, recv_bytes=1024):
    """
    parts を "a b c #" にして ASCII 送信．
    その後 1 回だけ recv して文字列で返す．
    """
    msg = " ".join(str(p) for p in parts) + " #"
    sock.sendall(msg.encode("ascii"))
    time.sleep(pause)
    try:
        data = sock.recv(recv_bytes)
        text = data.decode("ascii", errors="ignore").strip()
    except socket.timeout:
        text = ""
    print(f"[SEND] {msg.strip()}   [RECV] {text}")
    return text

def main():
    print(f"[INFO] Connecting to {ROBOT_IP}:{ROBOT_PORT}")
    with socket.create_connection((ROBOT_IP, ROBOT_PORT), timeout=5.0) as sock:
        sock.settimeout(2.0)
        print("[INFO] Connected")

        # 通信確認：現在TCP
        send_cmd(sock, [3])

        # 速度設定（CASE 8：v_tcp, v_ori）
        send_cmd(sock, [8, V_TCP, V_ORI])

        # ゾーン設定（CASE 9：fine=0, p_tcp, p_ori, z_ori）
        send_cmd(sock, [9, 0, P_TCP, P_ORI, Z_ORI])

        # MoveL（CASE 1）
        print("[INFO] MoveL to:", TARGET)
        reply = send_cmd(sock, [1] + TARGET)

        # 到達後の現在TCP
        send_cmd(sock, [3])

        print("[INFO] Done")

if __name__ == "__main__":
    main()
