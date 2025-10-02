# --- connect_get_pose_and_joints_with_tooltip.py
# J6フランジ姿勢＋ジョイント角に加え，フランジ→Toolの剛体変換を合成して
# 工具先端(TCP)の座標・姿勢（XYZ, Quaternion, RPY）を表示する版

import os, sys, importlib.util, math, time

BASE = os.path.dirname(__file__)
OPEN_ABB_BASE = os.path.join(BASE, "open_abb")

# ===== Tool定義 =====
# 1) 簡易モード：フランジ+Z方向オフセット（mm）
TCP_OFFSET_Z_MM = 150.0   # 例: 150mm（フランジ中心→先端）
# 2) フルToolモード：フランジ→Toolの並進 + 回転（RPY指定）
#    ※フルToolを使うなら USE_FULL_TOOL = True にして tx,ty,tz と RPY を設定
USE_FULL_TOOL = False
TOOL_TX, TOOL_TY, TOOL_TZ = 0.0, 0.0, 150.0  # mm
TOOL_R_DEG, TOOL_P_DEG, TOOL_Y_DEG = 0.0, 0.0, 0.0  # deg
# ====================

def find_abb_py(start_dir):
    for root, _, files in os.walk(start_dir):
        if "abb.py" in files:
            return os.path.join(root, "abb.py")
    return None

abb_py_path = find_abb_py(OPEN_ABB_BASE)
if not abb_py_path:
    raise FileNotFoundError("abb.py not found under open_abb")
spec = importlib.util.spec_from_file_location("abb", abb_py_path)
abb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(abb)

# -------- Python3: bytes/str パッチ --------
def _patch_bytes_io():
    if hasattr(abb, "Robot"):
        if hasattr(abb.Robot, "send"):
            _orig_send = abb.Robot.send
            def _send(self, *args, **kwargs):
                a2 = [a.encode("ascii") if isinstance(a, str) else a for a in args]
                kw2 = {k: (v.encode("ascii") if isinstance(v, str) else v) for k, v in kwargs.items()}
                return _orig_send(self, *a2, **kw2)
            abb.Robot.send = _send
        if hasattr(abb.Robot, "receive"):
            _orig_recv = abb.Robot.receive
            def _recv(self, *args, **kwargs):
                data = _orig_recv(self, *args, **kwargs)
                if isinstance(data, bytes):
                    try:    return data.decode("ascii")
                    except: return data.decode("utf-8", errors="ignore")
                return data
            abb.Robot.receive = _recv
_patch_bytes_io()

# -------- ポート固定 --------
TARGET_PORT = 5000
if hasattr(abb, "Robot") and hasattr(abb.Robot, "connect_motion"):
    _orig_connect_motion = abb.Robot.connect_motion
    def _connect_motion(self, remote):
        host, _ = remote
        return _orig_connect_motion(self, (host, TARGET_PORT))
    abb.Robot.connect_motion = _connect_motion

ROBOT_IP = "192.168.125.1"

# ---- utility ----
def fmt(n): return str(n) if isinstance(n, int) else f"{float(n):.6f}"

def rpy_to_quat(r_deg, p_deg, y_deg):
    rx, ry, rz = map(math.radians, (r_deg, p_deg, y_deg))
    cx, sx = math.cos(rx/2), math.sin(rx/2)
    cy, sy = math.cos(ry/2), math.sin(ry/2)
    cz, sz = math.cos(rz/2), math.sin(rz/2)
    qw = cz*cy*cx + sz*sy*sx
    qx = cz*cy*sx - sz*sy*cx
    qy = cz*sy*cx + sz*cy*sx
    qz = sz*cy*cx - cz*sy*sx
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz) or 1.0
    return [qx/n, qy/n, qz/n, qw/n]  # ABB形式 [x,y,z,w]

def quat_to_matrix(q):
    qx, qy, qz, qw = q  # [x,y,z,w]
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    return [
        [1 - 2*(yy + zz),     2*(xy - wz),         2*(xz + wy)],
        [    2*(xy + wz),  1 - 2*(xx + zz),        2*(yz - wx)],
        [    2*(xz - wy),      2*(yz + wx),     1 - 2*(xx + yy)],
    ]

def quat_to_rpy_ZYX_deg(q):
    R = quat_to_matrix(q)
    r20 = max(-1.0, min(1.0, R[2][0]))
    pitch = math.degrees(math.asin(-r20))             # Y
    yaw   = math.degrees(math.atan2(R[1][0], R[0][0]))# Z
    roll  = math.degrees(math.atan2(R[2][1], R[2][2]))# X
    return roll, pitch, yaw  # (X,Y,Z)

def R_mul(A,B):
    return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

def R_apply(R, v):
    return [R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2],
            R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2],
            R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2]]

def R_to_quat(R):
    m00,m01,m02 = R[0]; m10,m11,m12 = R[1]; m20,m21,m22 = R[2]
    tr = m00 + m11 + m22
    if tr > 0:
        S = math.sqrt(tr+1.0)*2
        qw = 0.25*S
        qx = (m21 - m12)/S
        qy = (m02 - m20)/S
        qz = (m10 - m01)/S
    elif (m00 > m11) and (m00 > m22):
        S = math.sqrt(1.0 + m00 - m11 - m22)*2
        qw = (m21 - m12)/S
        qx = 0.25*S
        qy = (m01 + m10)/S
        qz = (m02 + m20)/S
    elif m11 > m22:
        S = math.sqrt(1.0 + m11 - m00 - m22)*2
        qw = (m02 - m20)/S
        qx = (m01 + m10)/S
        qy = 0.25*S
        qz = (m12 + m21)/S
    else:
        S = math.sqrt(1.0 + m22 - m00 - m11)*2
        qw = (m10 - m01)/S
        qx = (m02 + m20)/S
        qy = (m12 + m21)/S
        qz = 0.25*S
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz) or 1.0
    return [qx/n, qy/n, qz/n, qw/n]

def send_raw(sock, parts, pause=0.12):
    msg = " ".join(fmt(p) for p in parts) + " #"
    sock.sendall(msg.encode("ascii"))
    time.sleep(pause)
    data = sock.recv(1024).decode("ascii", errors="ignore").strip()
    print(f"[SEND] {msg.strip()}   [RECV] {data}")
    return data.split()

def parse_case3_tokens(toks):
    if len(toks) >= 9:
        xyz  = [float(toks[2]), float(toks[3]), float(toks[4])]
        quat = [float(toks[5]), float(toks[6]), float(toks[7]), float(toks[8])]  # [x,y,z,w]
        return xyz, quat
    raise ValueError("CASE3 reply too short: " + " ".join(toks))

# --- joints helpers ---
def try_get_joints_with_case(sock, case_id):
    try:
        toks = send_raw(sock, [case_id])
        if len(toks) >= 8 and int(toks[0]) == case_id and int(toks[1]) == 1:
            js = [float(toks[i]) for i in range(2, 8)]
            return js
    except Exception:
        pass
    return None

def get_joint_angles(sock, robot_obj=None):
    js = try_get_joints_with_case(sock, 10)
    if js is not None: return js
    js = try_get_joints_with_case(sock, 4)
    if js is not None: return js
    if robot_obj is not None and hasattr(robot_obj, "get_joints"):
        try:
            js = robot_obj.get_joints()
            if max(abs(v) for v in js) <= math.pi + 1e-6:
                js = [math.degrees(v) for v in js]
            return js
        except Exception:
            pass
    return None

def set_wobj_base(sock):
    try:
        send_raw(sock, [7, 0, 0, 0, 1, 0, 0, 0])
    except Exception as e:
        print("[WARN] set wobj via raw failed:", e)

def set_tool(sock, tz_mm):
    send_raw(sock, [6, 0, 0, tz_mm, 1, 0, 0, 0])

def get_current_pose(sock):
    toks = send_raw(sock, [3])
    return parse_case3_tokens(toks)

# ====== メイン ======
ROBOT_IP = "192.168.125.1"

try:
    with abb.Robot(ip=ROBOT_IP) as robot:
        sock = robot.sock
        sock.settimeout(2.0)

        # 0) 同期
        send_raw(sock, [0])
        # 1) WObj固定
        set_wobj_base(sock)

        # === A) J6フランジ（工具なし）姿勢 ===
        print("[STEP] Read J6 flange pose (Tool = identity)")
        set_tool(sock, 0.0)  # TCP=J6原点
        j6_xyz, j6_q = get_current_pose(sock)
        j6_r, j6_p, j6_y = quat_to_rpy_ZYX_deg(j6_q)
        print(f"[J6] x(mm)={j6_xyz[0]:.3f}, y(mm)={j6_xyz[1]:.3f}, z(mm)={j6_xyz[2]:.3f}, "
              f"rollX(deg)={j6_r:.3f}, pitchY(deg)={j6_p:.3f}, yawZ(deg)={j6_y:.3f}")
        print(f"[J6-QUAT] qx={j6_q[0]:.6f}, qy={j6_q[1]:.6f}, qz={j6_q[2]:.6f}, qw={j6_q[3]:.6f}")

        # === B) ジョイント角 ===
        print("[STEP] Read joint angles (J1..J6)")
        joints = get_joint_angles(sock, robot_obj=robot)
        if joints is not None:
            j1,j2,j3,j4,j5,j6 = joints
            print(f"[JOINTS(deg)] J1={j1:.3f}, J2={j2:.3f}, J3={j3:.3f}, J4={j4:.3f}, J5={j5:.3f}, J6={j6:.3f}")
        else:
            print("[WARN] ジョイント角の取得に失敗しました．")

        # === C) 工具先端(TCP)の座標・姿勢（J6→Toolを合成） ===
        print("[STEP] Compute tool-tip pose by compounding J6 -> Tool transform")
        # W_T_J6
        R_wj6 = quat_to_matrix(j6_q)
        t_wj6 = j6_xyz  # [x,y,z]

        if USE_FULL_TOOL:
            # フルTool
            q_tool = rpy_to_quat(TOOL_R_DEG, TOOL_P_DEG, TOOL_Y_DEG)
            R_j6t = quat_to_matrix(q_tool)
            t_j6t = [TOOL_TX, TOOL_TY, TOOL_TZ]
        else:
            # 簡易：+Zオフセットのみ
            R_j6t = [[1,0,0],[0,1,0],[0,0,1]]
            t_j6t = [0.0, 0.0, TCP_OFFSET_Z_MM]

        # 合成：W_T_TCP = W_T_J6 * J6_T_Tool
        t_wtcp = [t_wj6[i] + R_apply(R_wj6, t_j6t)[i] for i in range(3)]
        R_wtcp = R_mul(R_wj6, R_j6t)
        q_wtcp = R_to_quat(R_wtcp)
        r_tcp, p_tcp, y_tcp = quat_to_rpy_ZYX_deg(q_wtcp)

        print(f"[TCP(compound)] x(mm)={t_wtcp[0]:.3f}, y(mm)={t_wtcp[1]:.3f}, z(mm)={t_wtcp[2]:.3f}")
        print(f"[TCP-QUAT(compound)] qx={q_wtcp[0]:.6f}, qy={q_wtcp[1]:.6f}, qz={q_wtcp[2]:.6f}, qw={q_wtcp[3]:.6f}")
        print(f"[TCP-RPY(compound)] rollX={r_tcp:.3f}, pitchY={p_tcp:.3f}, yawZ={y_tcp:.3f}")

        # 参考：open_abbのCASE6でTool設定してからCASE3を読む方法とも一致するはず
        #      検証用（任意）：実機のToolに合わせてCASE6→CASE3を行い、上の値と比較して下さい。

        print("[INFO] Done.")

except Exception as e:
    print("[ERROR]", repr(e))
