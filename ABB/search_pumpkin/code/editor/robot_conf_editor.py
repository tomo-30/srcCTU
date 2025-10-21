# -*- coding: utf-8 -*-
"""
robot_composer_gui_v5_2.py  (座標系をすべて同時表示: Base/Storage/Harvest/PinArt)
- 2DオーバーレイArrow3Dで各フレームの座標軸(X/Y/Z)を常に最前面に表示
- 一度だけクリアし、複数フレーム分を追加するように修正
- 各フレームに名前ラベル(例: Base, Storage...)を原点付近に表示
"""

import os
import json
import math
import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ========= 数学ユーティリティ =========
def rpy_deg_to_rot_matrix(roll_deg, pitch_deg, yaw_deg):
    rx = math.radians(roll_deg)
    ry = math.radians(pitch_deg)
    rz = math.radians(yaw_deg)
    Rx = np.array([[1, 0, 0],
                   [0, math.cos(rx), -math.sin(rx)],
                   [0, math.sin(rx),  math.cos(rx)]], dtype=float)
    Ry = np.array([[ math.cos(ry), 0, math.sin(ry)],
                   [0,             1, 0           ],
                   [-math.sin(ry), 0, math.cos(ry)]], dtype=float)
    Rz = np.array([[math.cos(rz), -math.sin(rz), 0],
                   [math.sin(rz),  math.cos(rz), 0],
                   [0,             0,            1]], dtype=float)
    return Rz @ Ry @ Rx

def anchor_offset(size_xyz, anchor_name):
    sx, sy, sz = size_xyz
    hx, hy, hz = sx/2, sy/2, sz/2
    mapping = {
        "中心":             np.array([0,   0,   0  ]),
        "+Z面中心(上面)":   np.array([0,   0,   hz ]),
        "-Z面中心(下面)":   np.array([0,   0,  -hz ]),
        "+X面中心":         np.array([hx,  0,   0  ]),
        "-X面中心":         np.array([-hx, 0,   0  ]),
        "+Y面中心":         np.array([0,   hy,  0  ]),
        "-Y面中心":         np.array([0,  -hy,  0  ]),
        "最小角(-X,-Y,-Z)": np.array([-hx, -hy, -hz]),
        "最大角(+X,+Y,+Z)": np.array([ hx,  hy,  hz]),
    }
    return mapping.get(anchor_name, np.zeros(3))

def cuboid_vertices_local(size_xyz, anchor_name):
    sx, sy, sz = size_xyz
    hx, hy, hz = sx/2, sy/2, sz/2
    ctr = np.array([
        [-hx, -hy, -hz],
        [ hx, -hy, -hz],
        [ hx,  hy, -hz],
        [-hx,  hy, -hz],
        [-hx, -hy,  hz],
        [ hx, -hy,  hz],
        [ hx,  hy,  hz],
        [-hx,  hy,  hz],
    ], dtype=float)
    return ctr - anchor_offset(size_xyz, anchor_name)

def transform_points(pts, R, t):
    return (R @ pts.T).T + t

def faces_from_vertices(verts):
    idx = [
        [0,1,2,3],  # -Z
        [4,5,6,7],  # +Z
        [0,1,5,4],  # -Y
        [2,3,7,6],  # +Y
        [1,2,6,5],  # +X
        [0,3,7,4],  # -X
    ]
    return [[verts[i] for i in face] for face in idx]

# ========= Arrow3D（2Dに投影して常に最前面） =========
class Arrow3D(FancyArrowPatch):
    """3Dの端点を受け取り、描画時に2Dへ投影して矢印を描く（2Dアーティスト）。"""
    def __init__(self, xs, ys, zs, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._verts3d = (np.asarray(xs), np.asarray(ys), np.asarray(zs))

    def set_positions_3d(self, xs, ys, zs):
        self._verts3d = (np.asarray(xs), np.asarray(ys), np.asarray(zs))

    def _project(self):
        ax = self.axes
        xs3d, ys3d, zs3d = self._verts3d
        proj = ax.get_proj()
        xs2d, ys2d, zs2d = proj3d.proj_transform(xs3d, ys3d, zs3d, proj)
        return xs2d, ys2d, zs2d

    def draw(self, renderer):
        xs2d, ys2d, _ = self._project()
        self.set_positions((xs2d[0], ys2d[0]), (xs2d[1], ys2d[1]))
        super().draw(renderer)

    def do_3d_projection(self, renderer=None):
        xs2d, ys2d, zs2d = self._project()
        self.set_positions((xs2d[0], ys2d[0]), (xs2d[1], ys2d[1]))
        return np.min(zs2d)

def clear_axes_overlays(ax):
    """既存の座標系オーバーレイ（矢印・ラベル）を一括削除"""
    if hasattr(ax, "_axis_overlay_artists"):
        for art in ax._axis_overlay_artists:
            try:
                art.remove()
            except Exception:
                pass
        ax._axis_overlay_artists.clear()
    else:
        ax._axis_overlay_artists = []

def draw_axes_overlay(ax, R, origin, length=150.0, zorder=10000, frame_label=None):
    """
    1つのフレームの座標軸(X/Y/Z)を追加描画（既存は消さない）。
    frame_label を与えると、原点にフレーム名ラベルを描く。
    """
    origin = np.array(origin, dtype=float)
    ex = R @ np.array([1,0,0], dtype=float)
    ey = R @ np.array([0,1,0], dtype=float)
    ez = R @ np.array([0,0,1], dtype=float)

    colors = {"X": "#cc0000", "Y": "#007700", "Z": "#0000cc"}  # 濃色
    widths = {"X": 5.0, "Y": 5.0, "Z": 6.0}                    # Zを太く

    # 各軸の矢印
    tip_x = origin + ex*length
    tip_y = origin + ey*length
    tip_z = origin + ez*length

    arr_x = Arrow3D([origin[0], tip_x[0]], [origin[1], tip_x[1]], [origin[2], tip_x[2]],
                    mutation_scale=18, lw=widths["X"], arrowstyle='-|>', color=colors["X"], zorder=zorder)
    arr_y = Arrow3D([origin[0], tip_y[0]], [origin[1], tip_y[1]], [origin[2], tip_y[2]],
                    mutation_scale=18, lw=widths["Y"], arrowstyle='-|>', color=colors["Y"], zorder=zorder)
    arr_z = Arrow3D([origin[0], tip_z[0]], [origin[1], tip_z[1]], [origin[2], tip_z[2]],
                    mutation_scale=18, lw=widths["Z"], arrowstyle='-|>', color=colors["Z"], zorder=zorder)
    ax.add_artist(arr_x); ax.add_artist(arr_y); ax.add_artist(arr_z)

    # 軸ラベル
    tx = ax.text(tip_x[0], tip_x[1], tip_x[2], "X", color=colors["X"], fontsize=11, weight="bold", zorder=zorder+1)
    ty = ax.text(tip_y[0], tip_y[1], tip_y[2], "Y", color=colors["Y"], fontsize=11, weight="bold", zorder=zorder+1)
    tz = ax.text(tip_z[0], tip_z[1], tip_z[2], "Z", color=colors["Z"], fontsize=11, weight="bold", zorder=zorder+1)

    # フレーム名ラベル（原点近傍）
    frame_text = None
    if frame_label:
        frame_text = ax.text(origin[0], origin[1], origin[2],
                             f"{frame_label}", color="k", fontsize=11, weight="bold",
                             ha="left", va="bottom", zorder=zorder+2)

    # 登録（あとでまとめて消せるように）
    ax._axis_overlay_artists.extend([arr_x, arr_y, arr_z, tx, ty, tz] + ([frame_text] if frame_text else []))

# ========= 立方体（軸はここでは描かない） =========
def draw_cuboid_with_R(ax, size_xyz, anchor_name, world_pos, R_world,
                       face_color, alpha=0.50, label=None, zorder_poly=1):
    verts_local = cuboid_vertices_local(size_xyz, anchor_name)
    verts_world = transform_points(verts_local, R_world, np.array(world_pos, dtype=float))
    faces = faces_from_vertices(verts_world)

    poly = Poly3DCollection(faces, alpha=alpha, facecolor=face_color, edgecolor="k", linewidths=1.6)
    poly.set_zsort('min')
    poly.set_zorder(zorder_poly)
    ax.add_collection3d(poly)

    if label:
        ax.text(world_pos[0], world_pos[1], world_pos[2], label,
                fontsize=11, ha='center', va='bottom', color="k", zorder=zorder_poly+1)

    return (np.array(world_pos, dtype=float), R_world)

def set_equal_aspect_3d(ax, all_points):
    if len(all_points) == 0:
        ax.set_xlim(-500, 500); ax.set_ylim(-500, 500); ax.set_zlim(-500, 500); return
    P = np.array(all_points)
    x_min,x_max = P[:,0].min(), P[:,0].max()
    y_min,y_max = P[:,1].min(), P[:,1].max()
    z_min,z_max = P[:,2].min(), P[:,2].max()
    dx,dy,dz = x_max-x_min, y_max-y_min, z_max-z_min
    rng = max(dx,dy,dz, 1.0)
    cx,cy,cz = (x_min+x_max)/2, (y_min+y_max)/2, (z_min+z_max)/2
    r = rng*0.65
    ax.set_xlim(cx-r, cx+r); ax.set_ylim(cy-r, cy+r); ax.set_zlim(cz-r, cz+r)
    ax.set_xlabel("X [mm]"); ax.set_ylabel("Y [mm]"); ax.set_zlabel("Z [mm]")

# ========= GUI =========
ANCHOR_CHOICES = [
    "中心",
    "+Z面中心(上面)",
    "-Z面中心(下面)",
    "+X面中心",
    "-X面中心",
    "+Y面中心",
    "-Y面中心",
    "最小角(-X,-Y,-Z)",
    "最大角(+X,+Y,+Z)",
]

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Robot Composition GUI v5.2 (All frames axes visible)")
        self.geometry("980x840")
        self.fig = None
        self.ax = None

        main = ttk.Frame(self, padding=10); main.pack(fill=tk.BOTH, expand=True)
        nb = ttk.Notebook(main); nb.pack(fill=tk.BOTH, expand=True)

        self.page_world  = ttk.Frame(nb, padding=10)
        self.page_robot  = ttk.Frame(nb, padding=10)
        self.page_mounts = ttk.Frame(nb, padding=10)
        self.page_actions= ttk.Frame(nb, padding=10)

        nb.add(self.page_world,  text="① サイズ＆ローカル座標(ワールド基準)")
        nb.add(self.page_robot,  text="② ロボット座標系(ワールド基準)")
        nb.add(self.page_mounts, text="② 各要素の取付(ロボット基準)")
        nb.add(self.page_actions,text="③ 出力/描画")

        self._build_world_page()
        self._build_robot_page()
        self._build_mounts_page()
        self._build_actions_page()
        self._set_defaults()

    # ---- UI builders ----
    def _entry_labeled(self, parent, text, r, c, width=10):
        ttk.Label(parent, text=text).grid(row=r, column=c, sticky="e", padx=(0,4), pady=2)
        var = tk.StringVar()
        ent = ttk.Entry(parent, textvariable=var, width=width)
        ent.grid(row=r, column=c+1, sticky="w", padx=(0,12), pady=2)
        return var

    def _combo_labeled(self, parent, text, r, c, choices):
        ttk.Label(parent, text=text).grid(row=r, column=c, sticky="e", padx=(0,4))
        var = tk.StringVar()
        cb = ttk.Combobox(parent, textvariable=var, values=choices, width=18, state="readonly")
        cb.grid(row=r, column=c+1, sticky="w", padx=(0,12))
        return var

    # ---- ページ構築 ----
    def _build_world_page(self):
        f = self.page_world
        # Base
        ttk.Label(f, text="移動ロボット(ベース) サイズ(mm) / ローカル座標アンカー").grid(row=0, column=0, sticky="w", pady=(0,4))
        fb = ttk.Frame(f); fb.grid(row=1, column=0, sticky="w")
        self.base_x = self._entry_labeled(fb, "X:", 0, 0)
        self.base_y = self._entry_labeled(fb, "Y:", 0, 2)
        self.base_z = self._entry_labeled(fb, "Z:", 0, 4)
        self.base_anchor = self._combo_labeled(fb, "アンカー:", 0, 6, ANCHOR_CHOICES)
        # PinArt
        ttk.Label(f, text="ピンアート+アーム サイズ(mm) / ローカル座標アンカー").grid(row=2, column=0, sticky="w", pady=(12,4))
        fp = ttk.Frame(f); fp.grid(row=3, column=0, sticky="w")
        self.pin_x = self._entry_labeled(fp, "X:", 0, 0)
        self.pin_y = self._entry_labeled(fp, "Y:", 0, 2)
        self.pin_z = self._entry_labeled(fp, "Z:", 0, 4)
        self.pin_anchor = self._combo_labeled(fp, "アンカー:", 0, 6, ANCHOR_CHOICES)
        # Harvest
        ttk.Label(f, text="収穫機構+アーム サイズ(mm) / ローカル座標アンカー").grid(row=4, column=0, sticky="w", pady=(12,4))
        fh = ttk.Frame(f); fh.grid(row=5, column=0, sticky="w")
        self.har_x = self._entry_labeled(fh, "X:", 0, 0)
        self.har_y = self._entry_labeled(fh, "Y:", 0, 2)
        self.har_z = self._entry_labeled(fh, "Z:", 0, 4)
        self.har_anchor = self._combo_labeled(fh, "アンカー:", 0, 6, ANCHOR_CHOICES)
        # Storage
        ttk.Label(f, text="ストレージ サイズ(mm) / ローカル座標アンカー（回転は後述で0固定）").grid(row=6, column=0, sticky="w", pady=(12,4))
        fs = ttk.Frame(f); fs.grid(row=7, column=0, sticky="w")
        self.sto_x = self._entry_labeled(fs, "X:", 0, 0)
        self.sto_y = self._entry_labeled(fs, "Y:", 0, 2)
        self.sto_z = self._entry_labeled(fs, "Z:", 0, 4)
        self.sto_anchor = self._combo_labeled(fs, "アンカー:", 0, 6, ANCHOR_CHOICES)

    def _build_robot_page(self):
        f = self.page_robot
        ttk.Label(f, text="ロボットベース(移動ロボット)のワールド姿勢").grid(row=0, column=0, sticky="w", pady=(0,6))
        fr = ttk.Frame(f); fr.grid(row=1, column=0, sticky="w")
        self.robot_wx = self._entry_labeled(fr, "World X:", 0, 0)
        self.robot_wy = self._entry_labeled(fr, "World Y:", 0, 2)
        self.robot_wz = self._entry_labeled(fr, "World Z:", 0, 4)
        self.robot_wr = self._entry_labeled(fr, "Roll:",   1, 0)
        self.robot_wp = self._entry_labeled(fr, "Pitch:",  1, 2)
        self.robot_wyaw = self._entry_labeled(fr, "Yaw:",  1, 4)

    def _build_mounts_page(self):
        f = self.page_mounts
        # PinArt mount
        ttk.Label(f, text="ピンアート+アーム 取付（ロボット座標系基準）").grid(row=0, column=0, sticky="w", pady=(0,6))
        mp = ttk.Frame(f); mp.grid(row=1, column=0, sticky="w")
        self.pin_px = self._entry_labeled(mp, "X:", 0, 0)
        self.pin_py = self._entry_labeled(mp, "Y:", 0, 2)
        self.pin_pz = self._entry_labeled(mp, "Z:", 0, 4)
        self.pin_rr = self._entry_labeled(mp, "Roll:", 1, 0)
        self.pin_rp = self._entry_labeled(mp, "Pitch:",1, 2)
        self.pin_ry = self._entry_labeled(mp, "Yaw:",  1, 4)
        # Harvest mount
        ttk.Label(f, text="収穫機構+アーム 取付（ロボット座標系基準）").grid(row=2, column=0, sticky="w", pady=(12,6))
        mh = ttk.Frame(f); mh.grid(row=3, column=0, sticky="w")
        self.har_px = self._entry_labeled(mh, "X:", 0, 0)
        self.har_py = self._entry_labeled(mh, "Y:", 0, 2)
        self.har_pz = self._entry_labeled(mh, "Z:", 0, 4)
        self.har_rr = self._entry_labeled(mh, "Roll:", 1, 0)
        self.har_rp = self._entry_labeled(mh, "Pitch:",1, 2)
        self.har_ry = self._entry_labeled(mh, "Yaw:",  1, 4)
        # Storage mount (no rotation)
        ttk.Label(f, text="ストレージ 取付（ロボット座標系基準／回転なし）").grid(row=4, column=0, sticky="w", pady=(12,6))
        ms = ttk.Frame(f); ms.grid(row=5, column=0, sticky="w")
        self.sto_px = self._entry_labeled(ms, "X:", 0, 0)
        self.sto_py = self._entry_labeled(ms, "Y:", 0, 2)
        self.sto_pz = self._entry_labeled(ms, "Z:", 0, 4)

    def _build_actions_page(self):
        f = self.page_actions
        ttk.Button(f, text="描画＆JSON保存", command=self.on_plot_and_save).grid(row=0, column=0, sticky="w", pady=6)
        ttk.Button(f, text="描画のみ", command=self.on_plot_only).grid(row=1, column=0, sticky="w", pady=6)
        ttk.Button(f, text="更新（再描画）", command=self.on_update_plot).grid(row=2, column=0, sticky="w", pady=6)
        ttk.Button(f, text="終了", command=self.destroy).grid(row=3, column=0, sticky="w", pady=6)
        ttk.Label(f, text="保存先: 実行ファイルと同じディレクトリに robot_composition.json").grid(row=4, column=0, sticky="w", pady=(8,0))

    # ---- 既定値 ----
    def _set_defaults(self):
        # sizes & anchors
        self.base_x.set("900"); self.base_y.set("700"); self.base_z.set("300"); self.base_anchor.set("+Z面中心(上面)")
        self.pin_x.set("350");  self.pin_y.set("350");  self.pin_z.set("300");  self.pin_anchor.set("中心")
        self.har_x.set("300");  self.har_y.set("300");  self.har_z.set("300");  self.har_anchor.set("中心")
        self.sto_x.set("400");  self.sto_y.set("300");  self.sto_z.set("250");  self.sto_anchor.set("中心")
        # robot (world pose)
        self.robot_wx.set("0"); self.robot_wy.set("0"); self.robot_wz.set("0")
        self.robot_wr.set("0"); self.robot_wp.set("0"); self.robot_wyaw.set("0")
        # mounts (robot frame)
        self.pin_px.set("150"); self.pin_py.set("0"); self.pin_pz.set("-150")
        self.pin_rr.set("0");   self.pin_rp.set("0");  self.pin_ry.set("0")
        self.har_px.set("-200"); self.har_py.set("0"); self.har_pz.set("-160")
        self.har_rr.set("0");    self.har_rp.set("0"); self.har_ry.set("0")
        self.sto_px.set("0");    self.sto_py.set("0"); self.sto_pz.set("-120")

    # ---- 値取得 & JSON ----
    def _values_to_dict(self):
        try:
            base_xyz = [float(self.base_x.get()), float(self.base_y.get()), float(self.base_z.get())]
            pin_xyz  = [float(self.pin_x.get()),  float(self.pin_y.get()),  float(self.pin_z.get())]
            har_xyz  = [float(self.har_x.get()),  float(self.har_y.get()),  float(self.har_z.get())]
            sto_xyz  = [float(self.sto_x.get()),  float(self.sto_y.get()),  float(self.sto_z.get())]
            base_anchor = self.base_anchor.get()
            pin_anchor  = self.pin_anchor.get()
            har_anchor  = self.har_anchor.get()
            sto_anchor  = self.sto_anchor.get()
            robot_wpos = [float(self.robot_wx.get()), float(self.robot_wy.get()), float(self.robot_wz.get())]
            robot_wrpy = [float(self.robot_wr.get()), float(self.robot_wp.get()), float(self.robot_wyaw.get())]
            pin_mount_pos = [float(self.pin_px.get()), float(self.pin_py.get()), float(self.pin_pz.get())]
            pin_mount_rpy = [float(self.pin_rr.get()), float(self.pin_rp.get()), float(self.pin_ry.get())]
            har_mount_pos = [float(self.har_px.get()), float(self.har_py.get()), float(self.har_pz.get())]
            har_mount_rpy = [float(self.har_rr.get()), float(self.har_rp.get()), float(self.har_ry.get())]
            sto_mount_pos = [float(self.sto_px.get()), float(self.sto_py.get()), float(self.sto_pz.get())]
        except ValueError:
            messagebox.showerror("入力エラー", "数値を正しく入力してください。")
            return None

        return {
            "units": {"length":"mm", "angle":"deg", "rpy_order":"XYZ"},
            "world": {
                "robot_base": {
                    "size_xyz": base_xyz,
                    "anchor": base_anchor,
                    "pose_world": {"position": robot_wpos, "rpy_deg": robot_wrpy}
                }
            },
            "robot_frame_mounts": {
                "pin_art_module": {
                    "size_xyz": pin_xyz,
                    "anchor": pin_anchor,
                    "mount_pose_robot": {"position": pin_mount_pos, "rpy_deg": pin_mount_rpy}
                },
                "harvest_module": {
                    "size_xyz": har_xyz,
                    "anchor": har_anchor,
                    "mount_pose_robot": {"position": har_mount_pos, "rpy_deg": har_mount_rpy}
                },
                "storage": {
                    "size_xyz": sto_xyz,
                    "anchor": sto_anchor,
                    "mount_pose_robot": {"position": sto_mount_pos, "rpy_deg": [0.0,0.0,0.0]}
                }
            }
        }

    def _save_json_same_dir(self, data):
        try:
            here = os.path.dirname(os.path.abspath(__file__))
            path = os.path.join(here, "robot_composition.json")
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            return path
        except Exception as e:
            messagebox.showwarning("保存警告", f"JSON保存に失敗: {e}")
            return None

    # ---- ボタン動作 ----
    def on_plot_and_save(self):
        data = self._values_to_dict()
        if data is None: return
        path = self._save_json_same_dir(data)
        if path:
            messagebox.showinfo("保存完了", f"保存しました: {path}")
        self._plot_world(data, fresh=True)

    def on_plot_only(self):
        data = self._values_to_dict()
        if data is None: return
        self._plot_world(data, fresh=True)

    def on_update_plot(self):
        data = self._values_to_dict()
        if data is None: return
        self._plot_world(data, fresh=False)

    # ---- 描画 ----
    def _plot_world(self, data, fresh=True):
        base = data["world"]["robot_base"]
        base_xyz   = base["size_xyz"]
        base_anchor= base["anchor"]
        base_wpos  = base["pose_world"]["position"]
        base_wrpy  = base["pose_world"]["rpy_deg"]

        pin  = data["robot_frame_mounts"]["pin_art_module"]
        har  = data["robot_frame_mounts"]["harvest_module"]
        sto  = data["robot_frame_mounts"]["storage"]

        Rwr = rpy_deg_to_rot_matrix(*base_wrpy)
        def robot_to_world(pos_robot, rpy_robot):
            Rre = rpy_deg_to_rot_matrix(*rpy_robot)
            Rwe = Rwr @ Rre
            twe = np.array(base_wpos, dtype=float) + (Rwr @ np.array(pos_robot, dtype=float))
            return twe, Rwe

        if fresh or self.fig is None or self.ax is None:
            self.fig = plt.figure(figsize=(10.0, 8.8))
            self.ax = self.fig.add_subplot(111, projection="3d")
            plt.ion(); self.fig.show()
        else:
            self.ax.cla()

        ax = self.ax
        ax.set_title("Autonomous Harvester Composition (World Frame) [mm]")
        ax.set_facecolor("#f2f2f2")

        # Base
        base_Rwe = rpy_deg_to_rot_matrix(*base_wrpy)
        base_axes = draw_cuboid_with_R(ax, base_xyz, base_anchor, base_wpos, base_Rwe,
                                       face_color="#a5d8ff", alpha=0.45, label="Base (Robot Frame)",
                                       zorder_poly=1)
        # PinArt
        pin_xyz, pin_anchor = pin["size_xyz"], pin["anchor"]
        pin_pos_r, pin_rpy_r = pin["mount_pose_robot"]["position"], pin["mount_pose_robot"]["rpy_deg"]
        pin_wpos, pin_Rwe = robot_to_world(pin_pos_r, pin_rpy_r)
        pin_axes = draw_cuboid_with_R(ax, pin_xyz, pin_anchor, pin_wpos, pin_Rwe,
                                      face_color="#9cf0b0", alpha=0.55, label="PinArt+Arm",
                                      zorder_poly=1)
        # Harvest
        har_xyz, har_anchor = har["size_xyz"], har["anchor"]
        har_pos_r, har_rpy_r = har["mount_pose_robot"]["position"], har["mount_pose_robot"]["rpy_deg"]
        har_wpos, har_Rwe = robot_to_world(har_pos_r, har_rpy_r)
        har_axes = draw_cuboid_with_R(ax, har_xyz, har_anchor, har_wpos, har_Rwe,
                                      face_color="#ffc07a", alpha=0.55, label="Harvest+Arm",
                                      zorder_poly=1)
        # Storage (no rotation)
        sto_xyz, sto_anchor = sto["size_xyz"], sto["anchor"]
        sto_pos_r = sto["mount_pose_robot"]["position"]
        sto_wpos, sto_Rwe = robot_to_world(sto_pos_r, [0.0,0.0,0.0])
        sto_axes = draw_cuboid_with_R(ax, sto_xyz, sto_anchor, sto_wpos, sto_Rwe,
                                      face_color="#d6ccff", alpha=0.55, label="Storage",
                                      zorder_poly=1)

        # Ground (Z=0)
        xs = np.linspace(-1500, 1500, 2); ys = np.linspace(-1500, 1500, 2)
        Xg, Yg = np.meshgrid(xs, ys); Zg = np.zeros_like(Xg)
        ax.plot_surface(Xg, Yg, Zg, alpha=0.06, shade=False, edgecolor="none", zorder=0)

        # 等倍
        all_pts = []
        for (size, anchor, wpos, R) in [
            (base_xyz, base_anchor, base_wpos, base_Rwe),
            (pin_xyz,  pin_anchor,  pin_wpos,  pin_Rwe),
            (har_xyz,  har_anchor,  har_wpos,  har_Rwe),
            (sto_xyz,  sto_anchor,  sto_wpos,  sto_Rwe),
        ]:
            v_local = cuboid_vertices_local(size, anchor)
            v_world = transform_points(v_local, R, np.array(wpos, dtype=float))
            all_pts.extend(list(v_world))
        set_equal_aspect_3d(ax, all_pts)

        ax.view_init(elev=22, azim=45)
        ax.grid(True)

        # 凡例
        proxy = [
            plt.Line2D([0],[0], color="#a5d8ff", lw=10),
            plt.Line2D([0],[0], color="#9cf0b0", lw=10),
            plt.Line2D([0],[0], color="#ffc07a", lw=10),
            plt.Line2D([0],[0], color="#d6ccff", lw=10),
        ]
        ax.legend(proxy, ["Base (Robot Frame)", "PinArt+Arm", "Harvest+Arm", "Storage"], loc="upper left")

        # === ここが重要：一度だけクリア → 全フレームを順に追加 ===
        clear_axes_overlays(ax)
        draw_axes_overlay(ax, base_axes[1], base_axes[0], length=150.0, zorder=10000, frame_label="Base")
        draw_axes_overlay(ax, sto_axes[1],  sto_axes[0],  length=150.0, zorder=10000, frame_label="Storage")
        draw_axes_overlay(ax, har_axes[1],  har_axes[0],  length=150.0, zorder=10000, frame_label="Harvest")
        draw_axes_overlay(ax, pin_axes[1],  pin_axes[0],  length=150.0, zorder=10000, frame_label="PinArt")

        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

if __name__ == "__main__":
    App().mainloop()
