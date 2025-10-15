# -*- coding: utf-8 -*-
# move_by_joints.py
# 目的: 関節角(J1..J6, deg)を指定して MoveJ で動作させる（open_abb想定）
# CASE:
#   0: Ping
#   2: MoveJ j1 j2 j3 j4 j5 j6
#   3: Get Cartesian
#   4/10: Get Joints（実装依存）
#   6: Set Tool
#   7: Set WorkObject
#   8: Set Speed (v_tcp, v_ori)
#   9: Set Zone (blend)

import os, sys, importlib.util, time, math

BASE = os.path.dirname(__file__)
OPEN_ABB_BASE = os.path.join(BASE, "open_abb")
ROBOT_IP, ROBOT_PORT = "192.168.125.1", 5000

# ====== ユーザ設定 ======
# 絶対関節角（deg）: 例として各軸を安全な範囲の値に
JOINT_TARGET_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# 速度・ゾーン（MoveJでも open_abb の共通コマンドを使用する実装が多い）
V_TCP, V_ORI = 100, 100     # 位置/姿勢の目標速度（単位は実装依存，typ. mm/s, deg/s）
P_TCP, P_ORI, Z_ORI = 10, 10, 10  # ゾーン（ブレンド量）
POST_MOVE_WAIT = 0.8
# Tool/WObj は必要に応じて設定（ここでは既定: フランジTCP，wobj0）
TOOL_TXYZ = (0.0, 0.0, 0.0)      # [tx,ty,tz] (mm)
TOOL_Q = (1.0, 0.0, 0.0, 0.0)    # [qx,qy,qz,qw] ABB形式の単位姿勢
USE_SET_WOBJ = True              # Trueなら wobj0 を明示
# ========================

def find_abb_py(start_dir):
    for root, _, files in os.walk(start_dir):
        if "abb.py" in files:
            return os.path.join(root, "abb.py")
    raise FileNotFoundError("abb.py not found")

spec = importlib.util.spec_from_file_location("abb", find_abb_py(OPEN_ABB_BASE))
abb = importlib.util.module_from_spec(spec); spec.loader.exec_module(abb)

# --- Python3互換パッチ（bytes/str，*args/**kwargs対応） ---
def _patch_bytes_io():
    if hasattr(abb, "Robot"):
        if hasattr(abb.Robot, "send"):
            _orig_send = abb.Robot.send
            def _send(self, *args, **kwargs):
                a2 = [a.encode("ascii") if isinstance(a, str) else a for a in args]
                kw2 = {k:(v.encode("ascii") if isinstance(v, str) else v) for k,v in kwargs.items()}
                return _orig_send(self, *a2, **kw2)
            abb.Robot.send = _send
        if hasattr(abb.Robot, "receive"):
            _orig_recv = abb.Robot.receive
            def _recv(self, *args, **kwargs):
                d = _orig_recv(self, *args, **kwargs)
                if isinstance(d, bytes):
                    try: return d.decode("ascii")
                    except: return d.decode("utf-8", errors="ignore")
                return d
            abb.Robot.receive = _recv
_patch_bytes_io()

# --- PORT固定（5000） ---
if hasattr(abb, "Robot") and hasattr(abb.Robot, "connect_motion"):
    _orig_connect_motion = abb.Robot.connect_motion
    def _connect_motion(self, remote):
        host, _ = remote
        return _orig_connect_motion(self, (host, ROBOT_PORT))
    abb.Robot.connect_motion = _connect_motion

def fmt(n): return str(n) if isinstance(n,int) else f"{float(n):.6f}"

def send_cmd(sock, parts, pause=0.18, expect=None):
    msg = " ".join(fmt(p) for p in parts) + " #"
    sock.sendall(msg.encode("ascii"))
    time.sleep(pause)
    data = sock.recv(2048).decode("ascii","ignore").strip()
    print(f"[SEND] {msg.strip()}   [RECV] {data}")
    toks = data.split()
    instr = int(toks[0]) if len(toks)>=1 and toks[0].lstrip("-").isdigit() else None
    ok    = int(toks[1]) if len(toks)>=2 and toks[1].lstrip("-").isdigit() else None
    if expect is not None and instr != expect:
        raise RuntimeError(f"ACK instr mismatch: expected {expect}, got {instr}, raw='{data}'")
    return instr, ok, toks

def try_get_joints(sock):
    # まず CASE 10，次に CASE 4
    for case_id in (10, 4):
        try:
            _, ok, toks = send_cmd(sock, [case_id], expect=case_id)
            if ok == 1 and len(toks) >= 8:
                return [float(toks[i]) for i in range(2,8)]
        except Exception:
            pass
    return None

def fallback_set_joints(robot, joints_deg):
    if hasattr(robot, "set_joints"):
        # 実装により deg/rad 異なる可能性に対応（閾値で判定）
        vals = joints_deg
        try:
            robot.set_joints(vals); return True
        except Exception:
            try:
                robot.set_joints([math.radians(v) for v in vals]); return True
            except Exception:
                return False
    return False

def main():
    tgt = JOINT_TARGET_DEG
    if len(tgt) != 6:
        raise ValueError("JOINT_TARGET_DEG must have 6 elements")
    print(f"[INFO] Connecting to {ROBOT_IP}:{ROBOT_PORT}")
    with abb.Robot(ip=ROBOT_IP) as robot:
        sock = robot.sock; sock.settimeout(3.0)
        print("[INFO] Connected")

        # 0) Ping
        send_cmd(sock, [0], expect=0)
        # 1) WObj = wobj0（必要なら）
        if USE_SET_WOBJ:
            try:
                send_cmd(sock, [7, 0, 0, 0, 1, 0, 0, 0], expect=7)
            except Exception as e:
                print("[WARN] Set WorkObject skipped:", e)

        # 2) Tool = 指定（必要な場合のみ。ここでは identity 相当）
        tx,ty,tz = TOOL_TXYZ
        qx,qy,qz,qw = TOOL_Q
        send_cmd(sock, [6, tx,ty,tz, qx,qy,qz,qw], expect=6)

        # 3) 速度・ゾーン
        send_cmd(sock, [8, V_TCP, V_ORI], expect=8)
        send_cmd(sock, [9, 0, P_TCP, P_ORI, Z_ORI], expect=9)

        # 4) 現在ジョイント
        js_now = try_get_joints(sock)
        if js_now:
            print("[CURR JOINTS(deg)] " + ", ".join(f"J{i+1}={js_now[i]:.3f}" for i in range(6)))

        # 5) MoveJ：CASE 2（フォーマット: 2 j1 j2 j3 j4 j5 j6）
        print("[INFO] MoveJ to JOINT_TARGET_DEG =", tgt)
        try:
            instr, ok, _ = send_cmd(sock, [2] + list(tgt), expect=2)
            if ok != 1:
                print("[WARN] CASE 2 MoveJ was not accepted (ok!=1). Trying API fallback...")
                okfb = fallback_set_joints(robot, tgt)
                if not okfb:
                    print("[ERROR] MoveJ fallback failed.")
        except Exception as e:
            print("[WARN] CASE 2 MoveJ raised:", e)
            okfb = fallback_set_joints(robot, tgt)
            if not okfb:
                print("[ERROR] MoveJ fallback failed.")

        time.sleep(POST_MOVE_WAIT)

        # 6) 到達確認（ジョイント）
        js_after = try_get_joints(sock)
        if js_after:
            print("[POST JOINTS(deg)] " + ", ".join(f"J{i+1}={js_after[i]:.3f}" for i in range(6)))
            if js_now:
                diff = [js_after[i]-js_now[i] for i in range(6)]
                print("[ΔJOINTS(deg)] " + ", ".join(f"dJ{i+1}={diff[i]:.3f}" for i in range(6)))

        print("[INFO] Done.")

if __name__ == "__main__":
    main()
