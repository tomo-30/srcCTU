# info/config.py
ROBOT_IP = "192.168.125.1"
ROBOT_PORT = 5000

# 速度・ゾーン（必要に応じてGUIから上書き可能）
DEFAULT_STEP_MM = 4.0
DEFAULT_V_TCP   = 40
DEFAULT_V_ORI   = 80
DEFAULT_P_TCP   = 1.5
DEFAULT_P_ORI   = 8
DEFAULT_Z_ORI   = 8
DEFAULT_LOOKAHEAD = 3
DEFAULT_FINAL_FINE = True
DEFAULT_FINAL_P_TCP = 0.5

# 受信系
MOVEL_ACK_TIMEOUT_SEC = 60.0
DEFAULT_RECV_TIMEOUT   = 3.0
IDLE_FRAME_WINDOW_SEC  = 0.07

# 工具・作業物（必要なら調整）
TOOL_TXYZ = (0.0, 0.0, 0.0)
TOOL_RPY  = (0.0, 0.0, 0.0)
IDENTITY_WOBJ = True
WOBJ_TXYZ = (0.0, 0.0, 0.0)
WOBJ_RPY  = (0.0, 0.0, 0.0)

# Arduino（GUIシミュレータ）
ARDUINO_STREAM_HZ = 10          # 100ms周期
SIM_FORCED_FRAMES = 5           # 送信ボタン後の“固定値”フレーム数

# 停止判定しきい値（ユーザが調整）
PIN_ANY_CONTACT_MM = 5.0        # ② 任意ピンがこのmm以下
PIN_ALL_CONTACT_MM = 10.0       # ③ 全ピンがこのmm以下

# ②③時に“押し付けず手を持ち上げる”ための退避量
LIFT_DZ_MM = 30.0

# ファイル
PLAN_FILENAME = "push_plan.json"
LOG_FILENAME  = "log.json"
