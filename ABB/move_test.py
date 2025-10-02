# -*- coding: utf-8 -*-
# move_with_tool_and_wobj.py
# 目的: 「WObj（作業物座標）」「Tool（TCP）」を明示設定した上で，指定姿勢にMoveL
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

# ====== 必ずあなたの実機に合わせて設定する部分 ======
# 1) Tool（フランジ→TCP）。ツール先端原点をTCPにしたい場合は実寸を入れる（mm, deg）
TOOL_TX, TOOL_TY, TOOL_TZ = 0.0, 0.0, 0.0   # 例: +Zに150mm先が先端
TOOL_R_DEG, TOOL_P_DEG, TOOL_Y_DEG = 0.0, 0.0, 0.0  # フランジ基準RPY（ZYX）

# 2) WObj（ベース→作業物座標）。実験系に揃えたいならここを実測で定義（mm, deg）
#    「ベース＝世界そのまま」で良ければ IDENTITY_WOBJ=True にする
IDENTITY_WOBJ = True
WOBJ_WX, WOBJ_WY, WOBJ_WZ = 0.0, 0.0, 0.0
WOBJ_R_DEG, WOBJ_P_DEG, WOBJ_Y_DEG = 0.0, 0.0, 0.0  # ベース基準RPY（ZYX）

# 3) 目標姿勢（このスクリプトでは「アクティブWObj座標系」解釈）
TARGET_XYZRPY = [350.0, 0.0, 300.0,   0.0, 180.0, 0.0]  # x,y,z, roll, pitch, yaw（mm/deg）
# 速度・ゾーン
V_TCP, V_ORI = 50, 50
P_TCP, P_ORI, Z_ORI = 10, 10, 10
POST_MOVEL_WAIT = 1.0
# =====================================================

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
    # 角度誤差 = 2*atan2(||v||, w) for Δq = qt * inv(qm)
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
    # [-pi,pi]へ正規化
    ang = (ang + math.pi) % (2*math.pi) - math.pi
    return abs(math.degrees(ang))

def main():
    tx,ty,tz = TOOL_TX, TOOL_TY, TOOL_TZ
    tqx,tqy,tqz,tqw = rpy_deg_to_quat(TOOL_R_DEG, TOOL_P_DEG, TOOL_Y_DEG)

    if IDENTITY_WOBJ:
        wx,wy,wz = 0.0, 0.0, 0.0
        wqx,wqy,wqz,wqw = 1.0, 0.0, 0.0, 0.0       # 単位姿勢（ABB式で[x,y,z,w]→[1,0,0,0]）
    else:
        wx,wy,wz = WOBJ_WX, WOBJ_WY, WOBJ_WZ
        wqx,wqy,wqz,wqw = rpy_deg_to_quat(WOBJ_R_DEG, WOBJ_P_DEG, WOBJ_Y_DEG)

    x, y, z, r, p, yw = TARGET_XYZRPY
    qx,qy,qz,qw = rpy_deg_to_quat(r, p, yw)

    print(f"[INFO] Connecting {ROBOT_IP}:{ROBOT_PORT}")
    with socket.create_connection((ROBOT_IP, ROBOT_PORT), timeout=5.0) as sock:
        sock.settimeout(3.0)
        print("[INFO] Connected")

        # 0) Ping
        send_cmd(sock, [0], expect=0)

        # 1) WorkObject を明示（これが“目標XYZの解釈座標系”になる）
        print(f"[INFO] Set WObj: tframe=[{wx},{wy},{wz}], quat=[{wqx},{wqy},{wqz},{wqw}]")
        send_cmd(sock, [7, wx, wy, wz, wqx, wqy, wqz, wqw], expect=7)

        # 2) Tool（取り付けた実物のTCP）を明示（これが“移動・読み取りのTCP”になる）
        print(f"[INFO] Set Tool: tframe=[{tx},{ty},{tz}], quat=[{tqx},{tqy},{tqz},{tqw}]")
        send_cmd(sock, [6, tx, ty, tz, tqx, tqy, tqz, tqw], expect=6)

        # 3) 現在のTCP（この時点で WObj & Tool 基準）
        _, ok, toks = send_cmd(sock, [3], expect=3)
        if ok == 1:
            cx, cy, cz, cqx, cqy, cqz, cqw = parse_case3_tokens(toks)
            print(f"[CURR TCP] pos=({cx:.3f},{cy:.3f},{cz:.3f}), quat=({cqx:.6f},{cqy:.6f},{cqz:.6f},{cqw:.6f})")

        # 4) 速度・ゾーン
        send_cmd(sock, [8, V_TCP, V_ORI], expect=8)
        send_cmd(sock, [9, 0, P_TCP, P_ORI, Z_ORI], expect=9)

        # 5) MoveL（目標は“アクティブWObj”座標で解釈される！）
        print(f"[INFO] MoveL to XYZRPY(WObj)={TARGET_XYZRPY}")
        print(f"[INFO]  -> quat_target=({fmt(qx)},{fmt(qy)},{fmt(qz)},{fmt(qw)})")
        instr, ok, _ = send_cmd(sock, [1, x, y, z, qx, qy, qz, qw], expect=1)
        if ok != 1:
            print("[WARN] MoveL ack != 1")

        time.sleep(POST_MOVEL_WAIT)

        # 6) 到達確認 ＆ 誤差の可視化（座標系の食い違い検出用）
        _, ok, toks = send_cmd(sock, [3], expect=3)
        if ok == 1:
            mx, my, mz, mqx, mqy, mqz, mqw = parse_case3_tokens(toks)
            pos_err = math.sqrt((mx-x)**2 + (my-y)**2 + (mz-z)**2)
            ang_err = quat_angle_error_deg([qx,qy,qz,qw], [mqx,mqy,mqz,mqw])
            print(f"[POST TCP] pos=({mx:.3f},{my:.3f},{mz:.3f}), quat=({mqx:.6f},{mqy:.6f},{mqz:.6f},{mqw:.6f})")
            print(f"[ERROR] pos_err={pos_err:.3f} mm, ang_err={ang_err:.3f} deg")

        print("[INFO] Done")

if __name__ == "__main__":
    main()
