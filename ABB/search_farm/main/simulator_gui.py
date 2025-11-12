# main/simulator_gui.py
# 依存: tkinter, matplotlib, numpy（任意: trimesh または numpy-stl）
import os, sys, tkinter as tk
from tkinter import ttk, filedialog, messagebox
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import matplotlib.pyplot as plt
import traceback
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# === 相対パスで include/ をimportできるように設定 ===
BASE_DIR = os.path.dirname(os.path.abspath(__file__))         # main/
ROOT_DIR = os.path.dirname(BASE_DIR)                          # project/
INCLUDE_DIR = os.path.join(ROOT_DIR, "include")               # project/include
if INCLUDE_DIR not in sys.path:
    sys.path.append(INCLUDE_DIR)

from io_utils import load_json, aabb_from_stl
from transform_utils import (
    rpy_xyz_deg_to_rot, make_T, transform_points,
    aabb_from_points, aabb_corners
)

BROWN = (0.59, 0.29, 0.0, 0.3)  # 地面色（α付き）
ROBOT_COLOR = (0.1, 0.4, 0.9, 0.25)
STL_BOX_COLOR = (0.1, 0.6, 0.1, 0.15)
EXP_BOX_COLOR = (0.95, 0.2, 0.2, 0.18)
EDGE_COLOR = "k"

class SimulatorGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Pumpkin Field Sim (Pin-Art explorer bbox)")
        self.geometry("1360x860")

        # ====== 状態 ======
        self.field_x = tk.DoubleVar(value=3000.0)  # mm
        self.field_y = tk.DoubleVar(value=3000.0)  # mm
        self.ground_z = tk.DoubleVar(value=0.0)

        self.path_robot_json = tk.StringVar(value="")
        self.path_push_json  = tk.StringVar(value="")
        self.stl_paths = []

        self.robot_pose = {  # world系でのスタート姿勢（mm, deg）
            "x": tk.DoubleVar(value=0.0),
            "y": tk.DoubleVar(value=0.0),
            "z": tk.DoubleVar(value=0.0),
            "r": tk.DoubleVar(value=0.0),
            "p": tk.DoubleVar(value=0.0),
            "y_": tk.DoubleVar(value=0.0),
        }

        self.robot_size_xyz = np.array([900.0, 700.0, 500.0], dtype=float)  # 既定．JSON読込で上書き
        self.pin_mount_T_robot = np.eye(4)  # pin_art_moduleの取り付け姿勢（Robot系）
        self.rpy_order = "XYZ"  # JSON読込で確認

        # ====== UI ======
        self._build_ui()

        # ====== Matplotlib Axes ======
        self.fig = plt.Figure(figsize=(8,6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.center)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self._set_equal_aspect()

    # ---------- UI ----------
    def _build_ui(self):
        self.grid_columnconfigure(1, weight=1)
        # 左：操作
        left = ttk.Frame(self)
        left.grid(row=0, column=0, sticky="nsw", padx=6, pady=6)
        # 中央：描画
        self.center = ttk.Frame(self)
        self.center.grid(row=0, column=1, sticky="nsew", padx=6, pady=6)
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)

        # フィールド
        f_field = ttk.LabelFrame(left, text="Field (mm)")
        f_field.pack(fill="x", pady=6)
        for name, var in [("X Len", self.field_x), ("Y Len", self.field_y), ("Ground Z", self.ground_z)]:
            row = ttk.Frame(f_field); row.pack(fill="x", pady=2)
            ttk.Label(row, text=name, width=10).pack(side="left")
            ttk.Entry(row, textvariable=var, width=12).pack(side="left")

        # ファイル指定
        f_files = ttk.LabelFrame(left, text="Config / STL")
        f_files.pack(fill="x", pady=6)

        row = ttk.Frame(f_files); row.pack(fill="x", pady=2)
        ttk.Label(row, text="robot_composition.json", width=22).pack(side="left")
        ttk.Entry(row, textvariable=self.path_robot_json).pack(side="left", fill="x", expand=True)
        ttk.Button(row, text="参照", command=self._browse_robot_json).pack(side="left", padx=2)

        row = ttk.Frame(f_files); row.pack(fill="x", pady=2)
        ttk.Label(row, text="push_plan.json", width=22).pack(side="left")
        ttk.Entry(row, textvariable=self.path_push_json).pack(side="left", fill="x", expand=True)
        ttk.Button(row, text="参照", command=self._browse_push_json).pack(side="left", padx=2)

        row = ttk.Frame(f_files); row.pack(fill="x", pady=2)
        ttk.Label(row, text="STL files", width=22).pack(side="left")
        ttk.Button(row, text="追加", command=self._add_stl).pack(side="left")
        ttk.Button(row, text="削除", command=self._del_stl).pack(side="left", padx=2)
        self.lb_stl = tk.Listbox(f_files, height=5)
        self.lb_stl.pack(fill="both", expand=True, pady=4)

        # ロボット姿勢
        f_pose = ttk.LabelFrame(left, text="Robot Pose in World (mm, deg)")
        f_pose.pack(fill="x", pady=6)

        # 2段組（x,y / z,r / p,yaw）
        grid = ttk.Frame(f_pose)
        grid.pack(fill="x", pady=4)

        pairs = [
            (("x","x"), ("y","y")),
            (("z","z"), ("r","roll")),
            (("p","pitch"), ("y_","yaw")),
        ]
        for r_idx, ((k1,l1),(k2,l2)) in enumerate(pairs):
            ttk.Label(grid, text=l1, width=6).grid(row=r_idx, column=0, padx=2, pady=2, sticky="w")
            ttk.Entry(grid, textvariable=self.robot_pose[k1], width=10).grid(row=r_idx, column=1, padx=2, pady=2, sticky="w")
            ttk.Label(grid, text=l2, width=6).grid(row=r_idx, column=2, padx=10, pady=2, sticky="w")
            ttk.Entry(grid, textvariable=self.robot_pose[k2], width=10).grid(row=r_idx, column=3, padx=2, pady=2, sticky="w")

        # リモコン：探索AABB幅でのステップ移動（ロボット座標系）
        f_step = ttk.LabelFrame(left, text="Step Move by Explore AABB (Robot frame)")
        f_step.pack(fill="x", pady=6)

        remote = ttk.Frame(f_step)
        remote.pack(pady=4)

        # 上(+x)
        btn_up = ttk.Button(remote, text="▲ (+x)", width=9, command=lambda: self._on_step("+x"))
        btn_up.grid(row=0, column=1, pady=2)

        # 左(-y), 右(+y)
        btn_left  = ttk.Button(remote, text="◀ (+y)", width=9, command=lambda: self._on_step("+y"))
        btn_right = ttk.Button(remote, text="▶ (-y)", width=9, command=lambda: self._on_step("-y"))
        btn_left.grid(row=1, column=0, padx=2, pady=2)
        btn_right.grid(row=1, column=2, padx=2, pady=2)

        # 下(-x)
        btn_down = ttk.Button(remote, text="▼ (-x)", width=9, command=lambda: self._on_step("-x"))
        btn_down.grid(row=2, column=1, pady=2)

        # 説明
        ttk.Label(f_step, text="push_plan の探索範囲AABB(ピン座標系)の幅を\nロボット座標系の±x/±y方向に適用して移動").pack(padx=2, pady=2)

        # 操作ボタン
        row = ttk.Frame(left); row.pack(fill="x", pady=8)
        ttk.Button(row, text="読込＆描画", command=self.draw_all).pack(side="left", padx=2)
        ttk.Button(row, text="リセット表示", command=self._reset_view).pack(side="left", padx=2)

    def _browse_robot_json(self):
        path = filedialog.askopenfilename(filetypes=[("JSON","*.json")])
        if path:
            self.path_robot_json.set(path)

    def _browse_push_json(self):
        path = filedialog.askopenfilename(filetypes=[("JSON","*.json")])
        if path:
            self.path_push_json.set(path)

    def _add_stl(self):
        paths = filedialog.askopenfilenames(filetypes=[("STL","*.stl"),("All","*.*")])
        for p in paths:
            self.stl_paths.append(p)
            self.lb_stl.insert(tk.END, p)

    def _del_stl(self):
        sel = list(self.lb_stl.curselection())
        sel.reverse()
        for i in sel:
            self.stl_paths.pop(i)
            self.lb_stl.delete(i)

    def _get_explore_span_xy(self):
        """
        push_plan.json から Pa/Pb を集約し，探索範囲AABB（ピン座標系）の
        8頂点を Robot 座標系へ変換してから，Robot座標系での幅(dx, dy)を返す．
        これにより，描画（Pin→Robot→World）と整合した“ロボット目線の”ステップ量になる．
        """
        ppath = self.path_push_json.get()
        if not ppath or not os.path.exists(ppath):
            return 0.0, 0.0

        data = load_json(ppath)
        pts = []
        for item in data.get("plan", []):
            Pa = item.get("Pa", [])
            Pb = item.get("Pb", [])
            pts.extend(Pa); pts.extend(Pb)
        if not pts:
            return 0.0, 0.0

        arr = np.asarray(pts, dtype=float)  # pin座標系
        # pin座標系でのAABB8頂点
        mn = arr.min(axis=0); mx = arr.max(axis=0)
        corners_pin = np.array([
            [mn[0], mn[1], mn[2]],
            [mx[0], mn[1], mn[2]],
            [mx[0], mx[1], mn[2]],
            [mn[0], mx[1], mn[2]],
            [mn[0], mn[1], mx[2]],
            [mx[0], mn[1], mx[2]],
            [mx[0], mx[1], mx[2]],
            [mn[0], mx[1], mx[2]],
        ], dtype=float)

        # Robot←Pin（取り付け姿勢）のみを反映（Worldは関係ない）
        # self.pin_mount_T_robot は Robot←Pin の同次変換
        corners_robot = transform_points(self.pin_mount_T_robot, corners_pin)

        # Robot座標系でのx/y幅
        dx = float(corners_robot[:,0].max() - corners_robot[:,0].min())
        dy = float(corners_robot[:,1].max() - corners_robot[:,1].min())

        # 退化対策（幅ゼロはわずかに膨らませる）
        if dx == 0.0:
            dx = 1e-6
        if dy == 0.0:
            dy = 1e-6
        return dx, dy


    def _move_robot_by(self, dx_robot, dy_robot):
        """
        ロボット座標系の並進 (dx_robot, dy_robot, 0) を，現在のロボット姿勢で
        ワールド座標系の並進ベクトルに変換して Robot Pose の (x,y,z) を更新．
        """
        # 現在姿勢
        r = self.robot_pose["r"].get()
        p = self.robot_pose["p"].get()
        yw= self.robot_pose["y_"].get()
        Rw = rpy_xyz_deg_to_rot(r, p, yw)  # world←robot 回転
        d_robot = np.array([dx_robot, dy_robot, 0.0], dtype=float)
        d_world = Rw @ d_robot  # 3x3 * 3x1

        # 位置更新
        self.robot_pose["x"].set(self.robot_pose["x"].get() + float(d_world[0]))
        self.robot_pose["y"].set(self.robot_pose["y"].get() + float(d_world[1]))
        self.robot_pose["z"].set(self.robot_pose["z"].get() + float(d_world[2]))

        # 再描画
        self.draw_all()

    def _on_step(self, direction):
        """
        direction: '+x','-x','+y','-y'（ロボット座標系）に1ステップ移動
        ステップ量は push_plan の探索AABB幅 (dx, dy) を使用
        """
        try:
            dx_span, dy_span = self._get_explore_span_xy()
            if dx_span == 0.0 and dy_span == 0.0:
                messagebox.showwarning("Warning", "push_plan.json の探索範囲が未定義です．")
                return

            if direction == "+x":
                self._move_robot_by(+dx_span, 0.0)
            elif direction == "-x":
                self._move_robot_by(-dx_span, 0.0)
            elif direction == "+y":
                self._move_robot_by(0.0, +dy_span)
            elif direction == "-y":
                self._move_robot_by(0.0, -dy_span)
            else:
                messagebox.showwarning("Warning", f"未知の方向: {direction}")
        except Exception as e:
            # ターミナルにも出力
            import sys, traceback
            print("[ERROR] _on_step failed:", file=sys.stderr)
            traceback.print_exc()
            messagebox.showerror("Error", f"{e}\n\n詳しくはターミナル出力を参照してください．")


    # ---------- 描画 ----------
    def _set_equal_aspect(self):
        self.ax.set_box_aspect([1,1,0.35])

    def _reset_view(self):
        self.ax.cla()
        self._set_equal_aspect()
        self.canvas.draw()

    def draw_all(self):
        try:
            self.ax.cla()
            self._set_equal_aspect()
            self._draw_field()
            self._read_robot_json()
            self._draw_robot_box()
            self._draw_stl_bboxes()
            self._draw_pin_explore_bbox()
            self._decorate_axes()
            self.canvas.draw()
        except Exception as e:
            # --- ターミナルにも詳細を出す ---
            print("[ERROR] draw_all failed:", file=sys.stderr)
            traceback.print_exc()
            # --- GUIにも出す ---
            messagebox.showerror("Error", f"{e}\n\n詳細はターミナル出力を参照してください．")


    def _decorate_axes(self):
        self.ax.set_xlabel("X [mm]")
        self.ax.set_ylabel("Y [mm]")
        self.ax.set_zlabel("Z [mm]")
        self.ax.grid(True)

    def _draw_field(self):
        X = float(self.field_x.get())
        Y = float(self.field_y.get())
        Z = float(self.ground_z.get())

        # 枠線
        xs = [0, X, X, 0, 0]
        ys = [0, 0, Y, Y, 0]
        zs = [Z]*5
        self.ax.plot(xs, ys, zs, color="saddlebrown")

        # 面（四角ポリゴンで描画）: X,Yが正の値のときのみ塗る
        if X > 0 and Y > 0:
            quad = [
                [0, 0, Z],
                [X, 0, Z],
                [X, Y, Z],
                [0, Y, Z],
            ]
            poly = Poly3DCollection([quad], facecolors=(0.59, 0.29, 0.0, 0.30), edgecolors='none')
            self.ax.add_collection3d(poly)

        # 表示範囲
        self.ax.set_xlim(0, max(X, 1.0))
        self.ax.set_ylim(0, max(Y, 1.0))
        self.ax.set_zlim(Z, Z + max(300.0, float(self.robot_size_xyz[2]) * 2.0))

    def _read_robot_json(self):
        path = self.path_robot_json.get()
        if not path or not os.path.exists(path):
            raise RuntimeError("robot_composition.json を指定してください．")
        data = load_json(path)
        # 単位系＆順序
        self.rpy_order = data.get("units",{}).get("rpy_order","XYZ")
        if self.rpy_order != "XYZ":
            raise RuntimeError(f"rpy_order={self.rpy_order} は未対応（XYZのみ）")
        # ロボット本体サイズ
        base = data["world"]["robot_base"]
        self.robot_size_xyz = np.array(base["size_xyz"], dtype=float)

        # pin_art_module 取り付け姿勢（Robot系）
        pin = data["robot_frame_mounts"]["pin_art_module"]
        pos = np.array(pin["mount_pose_robot"]["position"], dtype=float)
        r,p,y = pin["mount_pose_robot"]["rpy_deg"]
        Rm = rpy_xyz_deg_to_rot(r,p,y)
        self.pin_mount_T_robot = make_T(Rm, pos)

    def _draw_robot_box(self):
        # ワールド系でのロボット姿勢
        x = self.robot_pose["x"].get()
        y = self.robot_pose["y"].get()
        z = self.robot_pose["z"].get()
        r = self.robot_pose["r"].get()
        p = self.robot_pose["p"].get()
        yw= self.robot_pose["y_"].get()
        Rw = rpy_xyz_deg_to_rot(r,p,yw)
        Tw = make_T(Rw, [x,y,z])

        # アンカーは -Z面中心（下面）をワールド位置に合わせる想定
        sx, sy, sz = self.robot_size_xyz
        # ロボット座標系での直方体コーナ（下下面中心=原点→-Z面中心を原点に置くため，z:0..sz）
        # x,yは中心から±
        hx, hy = sx/2, sy/2
        local_corners = np.array([
            [-hx,-hy, 0],[ hx,-hy, 0],[ hx, hy, 0],[-hx, hy, 0],
            [-hx,-hy, sz],[ hx,-hy, sz],[ hx, hy, sz],[-hx, hy, sz]
        ], dtype=float)

        world_corners = transform_points(Tw, local_corners)
        self._draw_box_edges(world_corners, color=EDGE_COLOR, face_color=ROBOT_COLOR)

        # pin_art_moduleの取り付け位置も目安として点を描いておく
        pin_origin = transform_points(Tw @ self.pin_mount_T_robot, np.array([[0,0,0]]))[0]
        self.ax.scatter([pin_origin[0]], [pin_origin[1]], [pin_origin[2]], s=40, c="b", marker="o")

        # 保存：内部で後続計算に使う
        self.Tw_robot = Tw  # world←robot
        self.Tw_pin   = Tw @ self.pin_mount_T_robot  # world←pin

    def _draw_stl_bboxes(self):
        for path in self.stl_paths:
            try:
                mn, mx = aabb_from_stl(path)
                corners = aabb_corners(mn, mx)
                self._draw_box_edges(corners, color=EDGE_COLOR, face_color=STL_BOX_COLOR)
            except Exception as e:
                messagebox.showwarning("STL読み込み警告", f"{os.path.basename(path)}: {e}")

    def _draw_pin_explore_bbox(self):
        # push_plan.json から Pa, Pb を全部集めて pin系AABB を作成 → worldへ変換 →描画
        ppath = self.path_push_json.get()
        if not ppath or not os.path.exists(ppath):
            return
        data = load_json(ppath)
        all_pts = []
        for item in data.get("plan", []):
            Pa = item.get("Pa", [])
            Pb = item.get("Pb", [])
            all_pts.extend(Pa); all_pts.extend(Pb)
        if not all_pts:
            return
        all_pts = np.asarray(all_pts, dtype=float)  # pin_art_module座標系
        mn_pin, mx_pin = aabb_from_points(all_pts)
        corners_pin = aabb_corners(mn_pin, mx_pin)
        corners_world = transform_points(self.Tw_pin, corners_pin)
        self._draw_box_edges(corners_world, color=EDGE_COLOR, face_color=EXP_BOX_COLOR)

    # 立方体のエッジと半透明面を描く
    def _draw_box_edges(self, corners8, color="k", face_color=(0,0,1,0.15)):
        C = np.asarray(corners8, dtype=float)
        if C.shape != (8, 3):
            # 退化や頂点不足時はスキップ
            return

        # 面のインデックス（四角形）
        faces = [
            [0,1,2,3],  # bottom
            [4,5,6,7],  # top
            [0,1,5,4],  # side
            [1,2,6,5],
            [2,3,7,6],
            [3,0,4,7],
        ]

        # 面（半透明）
        quads = [C[idx_list] for idx_list in faces]

        # 退化（同一点や同一直線）を避ける：x,yで一意点が3未満なら面描画をスキップ
        valid_quads = []
        for q in quads:
            xy = q[:, :2]
            if len(np.unique(xy, axis=0)) >= 3:
                valid_quads.append(q)

        if valid_quads:
            poly = Poly3DCollection(valid_quads, facecolors=(face_color[0], face_color[1], face_color[2], face_color[3]), edgecolors='none')
            self.ax.add_collection3d(poly)

        # エッジ
        edges = [
            (0,1),(1,2),(2,3),(3,0),
            (4,5),(5,6),(6,7),(7,4),
            (0,4),(1,5),(2,6),(3,7)
        ]
        for i, j in edges:
            self.ax.plot([C[i,0], C[j,0]], [C[i,1], C[j,1]], [C[i,2], C[j,2]], color=color, linewidth=1.0)


if __name__ == "__main__":
    app = SimulatorGUI()
    app.mainloop()
