# -*- coding: utf-8 -*-
# gui_move_zyx_halt_debug.py
#
# 目的:
#  - 起動時に自動接続（IP/PORTは下の定数）
#  - GUIで XYZ + RZ/RY/RX (deg) を入力し CASE 21 で MoveL（OrientZYX）送信
#  - 停止ボタン（CASE 11: HALT）
#  - 受信/送信/例外を GUI とターミナルの両方に詳細ログ出力
#
# SERVER.mod 側 前提:
#  - CASE 11: HALT
#  - CASE 21: MoveL_ZYX (x y z rz ry rx)

import socket
import threading
import time
import logging
import traceback
import tkinter as tk
from tkinter import ttk, messagebox

# ===== 接続先 =====
ROBOT_IP   = "192.168.125.1"  # 実機
# ROBOT_IP = "127.0.0.1"      # RobotStudio の場合はこちら
ROBOT_PORT = 5000

# ===== ロギング設定（ターミナルへ詳細出力）=====
logging.basicConfig(
    level=logging.DEBUG,  # 必要に応じて INFO/DEBUG 切替
    format="%(asctime)s [%(levelname)s] %(message)s"
)
log = logging.getLogger("open_abb_gui")

def hexdump(b: bytes, width=16):
    try:
        s = b.decode("ascii", errors="ignore")
        return s
    except Exception:
        return repr(b)

class ABBClient:
    def __init__(self, ip, port, gui_log_cb):
        self.ip = ip
        self.port = port
        self.gui_log_cb = gui_log_cb
        self.sock = None
        self.rx_thread = None
        self._stop = threading.Event()
        self._connected = threading.Event()

    # GUI とターミナルの両方に出す
    def log_both(self, level, msg):
        getattr(log, level)(msg)
        if self.gui_log_cb:
            self.gui_log_cb(msg)

    def connect(self):
        """自動再接続付きの受信スレッドを開始"""
        def _run():
            attempt = 0
            while not self._stop.is_set():
                attempt += 1
                try:
                    self.log_both("info", f"Connecting to {self.ip}:{self.port} (try {attempt})...")
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    s.settimeout(5.0)

                    # connect_ex で errno を拾いやすくする
                    err = s.connect_ex((self.ip, self.port))
                    if err != 0:
                        raise OSError(err, f"connect_ex failed errno={err}")

                    s.settimeout(None)
                    self.sock = s
                    self._connected.set()
                    self.log_both("info", "Connected.")

                    # 最初に Ping（CASE 0）で疎通確認
                    try:
                        self.send_raw("0 #")
                    except Exception as e:
                        self.log_both("warning", f"Ping send failed: {e}")

                    # 受信ループ
                    buf = b""
                    while not self._stop.is_set():
                        data = self.sock.recv(4096)
                        if not data:
                            raise ConnectionError("socket closed by peer")
                        buf += data
                        # サーバは「1 回の送信で 1 レコード」が基本だが、念のため区切りで分割
                        # ここでは簡易：改行なし想定。都度 flush。
                        msg = buf.decode("ascii", errors="ignore").strip()
                        if msg:
                            self.log_both("debug", "[RECV RAW] " + hexdump(buf))
                            self.gui_log_cb("[RECV] " + msg)
                            buf = b""
                except Exception as e:
                    self._connected.clear()
                    self.log_both("error", f"Disconnected/Error: {e}")
                    # 例外の詳細（トレースバック）も出す
                    tb = traceback.format_exc()
                    for line in tb.rstrip().splitlines():
                        self.gui_log_cb("[TRACE] " + line)
                        log.debug(line)
                    # 後始末
                    try:
                        if self.sock:
                            self.sock.close()
                    except Exception:
                        pass
                    self.sock = None

                    if self._stop.is_set():
                        break
                    time.sleep(1.0)  # リトライ間隔
            self.log_both("info", "RX thread exit.")

        self.rx_thread = threading.Thread(target=_run, daemon=True)
        self.rx_thread.start()

    def close(self):
        self._stop.set()
        try:
            if self.sock:
                self.sock.shutdown(socket.SHUT_RDWR)
                self.sock.close()
        except Exception:
            pass

    def ensure_connected(self):
        if not self._connected.is_set() or not self.sock:
            raise RuntimeError("Not connected to robot (socket not ready)")

    def send_raw(self, text: str):
        """生テキスト送信 + 送信ログ（GUI & 端末）"""
        self.ensure_connected()
        wire = text.encode("ascii")
        self.log_both("debug", "[SEND RAW] " + text.strip())
        self.sock.sendall(wire)

    # ---- プロトコル: CASE 21 (MoveL_ZYX), CASE 11 (HALT), CASE 0 (Ping) ----
    def moveL_zyx(self, x,y,z, rz,ry,rx):
        self.send_raw(f"21 {x:.3f} {y:.3f} {z:.3f} {rz:.3f} {ry:.3f} {rx:.3f} #")

    def halt(self):
        self.send_raw("11 #")

    def ping(self):
        self.send_raw("0 #")

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ABB MoveL ZYX + HALT (debug logger)")
        self.geometry("600x480")

        self.client = ABBClient(ROBOT_IP, ROBOT_PORT, self._append_log)
        self.create_widgets()
        self.after(100, self._auto_connect)

    def create_widgets(self):
        pad = {"padx":8, "pady":6}

        grid = ttk.LabelFrame(self, text="Target (XYZ mm, RZ/RY/RX deg)")
        grid.pack(fill="x", **pad)

        self.var_x  = tk.DoubleVar(value=450.0)
        self.var_y  = tk.DoubleVar(value=  0.0)
        self.var_z  = tk.DoubleVar(value=300.0)
        self.var_rz = tk.DoubleVar(value=180.0)
        self.var_ry = tk.DoubleVar(value=  0.0)
        self.var_rx = tk.DoubleVar(value=180.0)

        def add_row(parent, label, var, col):
            ttk.Label(parent, text=label, width=6).grid(row=col, column=0, sticky="w", padx=5)
            ttk.Entry(parent, textvariable=var, width=12).grid(row=col, column=1, sticky="w", padx=5)

        add_row(grid, "X",  self.var_x,  0)
        add_row(grid, "Y",  self.var_y,  1)
        add_row(grid, "Z",  self.var_z,  2)
        add_row(grid, "RZ", self.var_rz, 3)
        add_row(grid, "RY", self.var_ry, 4)
        add_row(grid, "RX", self.var_rx, 5)

        btns = ttk.Frame(self)
        btns.pack(fill="x", **pad)
        ttk.Button(btns, text="Send Move (CASE 21)", command=self._on_send).pack(side="left", padx=6)
        ttk.Button(btns, text="HALT (CASE 11)",      command=self._on_halt).pack(side="left", padx=6)
        ttk.Button(btns, text="Ping (CASE 0)",       command=self._on_ping).pack(side="left", padx=6)

        conn = ttk.Frame(self)
        conn.pack(fill="x", **pad)
        ttk.Label(conn, text=f"Target: {ROBOT_IP}:{ROBOT_PORT}").pack(side="left")
        self.lbl_conn = ttk.Label(conn, text="Disconnected", foreground="red")
        self.lbl_conn.pack(side="left", padx=12)

        self.txt = tk.Text(self, height=14)
        self.txt.pack(fill="both", expand=True, padx=8, pady=8)

    def _auto_connect(self):
        self._append_log("[INFO] Auto-connect starting...")
        self.client.connect()
        self._poll_connection()

    def _poll_connection(self):
        if self.client._connected.is_set():
            self.lbl_conn.config(text="Connected", foreground="green")
        else:
            self.lbl_conn.config(text="Disconnected (auto-retry...)", foreground="red")
        self.after(500, self._poll_connection)

    def _on_send(self):
        try:
            x  = float(self.var_x.get())
            y  = float(self.var_y.get())
            z  = float(self.var_z.get())
            rz = float(self.var_rz.get())
            ry = float(self.var_ry.get())
            rx = float(self.var_rx.get())
            self.client.moveL_zyx(x,y,z, rz,ry,rx)
        except Exception as e:
            self._append_log("[ERROR] Send failed: " + str(e))
            traceback.print_exc()

    def _on_halt(self):
        try:
            self.client.halt()
        except Exception as e:
            self._append_log("[ERROR] HALT failed: " + str(e))
            traceback.print_exc()

    def _on_ping(self):
        try:
            self.client.ping()
        except Exception as e:
            self._append_log("[ERROR] Ping failed: " + str(e))
            traceback.print_exc()

    def _append_log(self, s):
        self.txt.insert("end", s + "\n")
        self.txt.see("end")
        # ターミナルにも INFO レベルでミラー
        log.info(s)

    def destroy(self):
        try:
            self.client.close()
        except Exception:
            pass
        super().destroy()

if __name__ == "__main__":
    App().mainloop()
