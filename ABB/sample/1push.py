# -*- coding: utf-8 -*-
# move_with_tool_and_wobj_p1p2p1.py
# 目的: 「WObj」「Tool」を明示設定し，開始から指定秒待って p1→p2→p1 を MoveL
# CASE:
#   0: Ping
#   6: Set Tool            -> "6 tx ty tz qx qy qz qw #"
#   7: Set WorkObject      -> "7 wx wy wz qx qy qz qw #"
#   8: Set Speed           -> "8 v_tcp v_ori #"
#   9: Set Zone            -> "9 0 p_tcp p_ori z_ori #"
#   1: MoveL               -> "1 x y z qx qy qz qw #"
#   3: Get Cartesian (TCP) -> "3 #"

import socket, time, math

ROBOT_IP, ROBOT_PORT = "192.168.125.1", 5000

# ====== 実機設定 ======
# Tool（フランジ→TCP）
TOOL_TX, TOOL_TY, TOOL_TZ = 0.0, 0.0, 0.0
TOOL_R_DEG, TOOL_P_DEG, TOOL_Y_DEG = 0.0, 0.0, 0.0

# WObj（ベース→作業物座標）
IDENTITY_WOBJ = True
WOBJ_WX, WOBJ_WY, WOBJ_WZ = 0.0, 0.0, 0.0
WOBJ_R_DEG, WOBJ_P_DEG, WOBJ_Y_DEG = 0.0, 0.0, 0.0

# ====== 動作パラメータ ======
# 目標点（mm/deg）
p1 = [350.0, 0.0, 300.0,  0.0, 180.0, 0.0]
p2 = [350.0, 0.0, 150.0,  0.0, 180.0, 0.0]

START_DELAY_SEC = 10.0       # 実行開始からの待機秒（任意に変更可）
V_TCP, V_ORI = 50, 50
P_TCP, P_ORI, Z_ORI = 10, 10, 10
POST_MOVEL_WAIT = 1.0
# ===========================

def fmt(n): return str(n) if isinstance(n,int) else f"{float(n):.6f}"

def rpy_deg_to_quat(r, p, y):
    rx, ry, rz = map(math.radians, (r, p, y))
    cx, sx = math.cos(rx/2), math.sin(rx/2)
    cy, sy = math.cos(ry/2), math.sin(ry/2)
    cz, sz = math.cos(rz/2), math.sin(rz/2)
    qw = cz*cy*cx + sz*sy*sx
    qx = cz*cy*sx - sz*sy*cx
    qy = cz*sy*cx + sz*cy*sx
    qz = sz*cy*cx - cz*sy*sx
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz) or 1.0
    return [qx/n, qy/n, qz/n, qw/n]  # ABB: [qx,qy,qz,qw]

def send_cmd(sock, parts, pause=0.18, expect=None):
    msg = " ".join(fmt(p) for p in parts) + " #"
    sock.sendall(msg.encode("ascii"))
    time.sleep(pause)
    data = sock.recv(2048).decode("ascii", errors="ignore").strip()
    print(f"[SEND] {msg.strip()}   [RECV] {data}")
    toks = data.split()
    instr = int(toks[0]) if len(toks)>=1 and toks[0].lstrip("-").isdigit() else None
    ok    = int(toks[1]) if len(toks)>=2 and toks[1].lstrip("-").isdigit() else None
    if expect is not None and instr != expect:
        raise RuntimeError(f"ACK instr mismatch: expected {expect}, got {instr}, raw='{data}'")
    return instr, ok, toks

def parse_case3_tokens(toks):
    # 期待: 3 1 x y z qx qy qz qw
    if len(toks) >= 9:
        return (float(toks[2]), float(toks[3]), float(toks[4]),
                float(toks[5]), float(toks[6]), float(toks[7]), float(toks[8]))
    raise ValueError("CASE3 reply too short")

def quat_angle_error_deg(qt, qm):
    qx, qy, qz, qw = qt
    ax, ay, az, aw = qm
    dq = [
        qw*(-ax) + qx*aw + qy*(-az) - qz*(-ay),
        qw*(-ay) - qx*(-az) + qy*aw + qz*(-ax),
        qw*(-az) + qx*(-ay) - qy*(-ax) + qz*aw,
        qw*aw    - qx*(-ax) - qy*(-ay) - qz*(-az)
    ]
    v = math.sqrt(dq[0]**2 + dq[1]**2 + dq[2]**2)
    ang = 2*math.atan2(v, max(-1.0, min(1.0, dq[3])))
    ang = (ang + math.pi) % (2*math.pi) - math.pi
    return abs(math.degrees(ang))

def movel_xyzrpy(sock, xyzrpy):
    x, y, z, r, p, yaw = xyzrpy
    qx, qy, qz, qw = rpy_deg_to_quat(r, p, yaw)
    print(f"[INFO] MoveL to XYZRPY={xyzrpy}  (quat=({fmt(qx)},{fmt(qy)},{fmt(qz)},{fmt(qw)}))")
    instr, ok, _ = send_cmd(sock, [1, x, y, z, qx, qy, qz, qw], expect=1)
    if ok != 1:
        print("[WARN] MoveL ack != 1")

def main():
    # Tool
    tqx,tqy,tqz,tqw = rpy_deg_to_quat(TOOL_R_DEG, TOOL_P_DEG, TOOL_Y_DEG)
    # WObj
    if IDENTITY_WOBJ:
        wqx,wqy,wqz,wqw = 1.0, 0.0, 0.0, 0.0
    else:
        wqx,wqy,wqz,wqw = rpy_deg_to_quat(WOBJ_R_DEG, WOBJ_P_DEG, WOBJ_Y_DEG)

    print(f"[INFO] Connecting {ROBOT_IP}:{ROBOT_PORT}")
    with socket.create_connection((ROBOT_IP, ROBOT_PORT), timeout=5.0) as sock:
        sock.settimeout(3.0)
        print("[INFO] Connected")

        # 0) Ping
        send_cmd(sock, [0], expect=0)

        # 1) WObj 明示
        print(f"[INFO] Set WObj: tframe=[{WOBJ_WX},{WOBJ_WY},{WOBJ_WZ}], quat=[{wqx},{wqy},{wqz},{wqw}]")
        send_cmd(sock, [7, WOBJ_WX, WOBJ_WY, WOBJ_WZ, wqx, wqy, wqz, wqw], expect=7)

        # 2) Tool 明示
        print(f"[INFO] Set Tool: tframe=[{TOOL_TX},{TOOL_TY},{TOOL_TZ}], quat=[{tqx},{tqy},{tqz},{tqw}]")
        send_cmd(sock, [6, TOOL_TX, TOOL_TY, TOOL_TZ, tqx, tqy, tqz, tqw], expect=6)

        # 3) 現在TCP（確認）
        _, ok, toks = send_cmd(sock, [3], expect=3)
        if ok == 1:
            cx, cy, cz, cqx, cqy, cqz, cqw = parse_case3_tokens(toks)
            print(f"[CURR TCP] pos=({cx:.3f},{cy:.3f},{cz:.3f}), quat=({cqx:.6f},{cqy:.6f},{cqz:.6f},{cqw:.6f})")

        # 4) 速度・ゾーン
        send_cmd(sock, [8, V_TCP, V_ORI], expect=8)
        send_cmd(sock, [9, 0, P_TCP, P_ORI, Z_ORI], expect=9)

        # 5) 開始待ち
        print(f"[INFO] Waiting {START_DELAY_SEC:.2f} sec before motion ...")
        time.sleep(START_DELAY_SEC)

        # 6) p1 → p2 → p1
        for target in (p1, p2, p1):
            movel_xyzrpy(sock, target)
            time.sleep(POST_MOVEL_WAIT)
            # 目標到達の簡易確認
            _, ok, toks = send_cmd(sock, [3], expect=3)
            if ok == 1:
                mx, my, mz, mqx, mqy, mqz, mqw = parse_case3_tokens(toks)
                qx, qy, qz, qw = rpy_deg_to_quat(target[3], target[4], target[5])
                pos_err = math.sqrt((mx-target[0])**2 + (my-target[1])**2 + (mz-target[2])**2)
                ang_err = quat_angle_error_deg([qx,qy,qz,qw], [mqx,mqy,mqz,mqw])
                print(f"[CHK] pos_err={pos_err:.3f} mm, ang_err={ang_err:.3f} deg")

        print("[INFO] Done")

if __name__ == "__main__":
    main()
