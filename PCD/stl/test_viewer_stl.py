import os
import pyvista as pv

# STL ファイルの読み込み
current_dir = os.path.dirname(os.path.abspath(__file__))
stl_path = os.path.join(current_dir, "pumpkin1.stl")

mesh = pv.read(stl_path)

# 情報表示
n_faces = mesh.n_cells
print(f"Faces: {n_faces}, Points: {mesh.n_points}")

# しきい値より多ければ自動デシメーション（例: 10万面を超えたら90%削減）
if n_faces > 100_000:
    target_reduction = 0.90  # 90%削減（0~1）
    mesh = mesh.decimate(target_reduction=target_reduction)  # preserve_topology は削除
    print(f"Decimated -> Faces: {mesh.n_cells}, Points: {mesh.n_points}")

# 高速描画
plotter = pv.Plotter()
plotter.add_mesh(mesh, show_edges=False, smooth_shading=True)
plotter.add_axes()
plotter.show_grid()
plotter.show()
