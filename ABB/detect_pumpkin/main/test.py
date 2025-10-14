# quick_probe.py
import socket, time

HOST, PORT = "192.168.125.1", 5000  # ← ここをRAPID側の実ポートに合わせる
with socket.create_connection((HOST, PORT), timeout=3) as s:
    s.settimeout(2.0)
    # open_abbは「' #' 終端」のテキストプロトコル
    s.sendall(b"0 #")   # CASE 0: Ping
    time.sleep(0.05)
    try:
        data = s.recv(4096)
        print("RAW RECV:", data)
    except Exception as e:
        print("RECV ERROR:", e)
