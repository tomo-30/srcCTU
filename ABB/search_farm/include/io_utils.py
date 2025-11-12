# include/io_utils.py
import json, os, struct, numpy as np

def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

def try_load_stl_vertices(path):
    """
    STLの頂点をnp.ndarray(N,3)で返す．
    1) trimesh 2) numpy-stl を優先使用．
    3) バイナリSTL簡易リーダ．
    4) ASCIIの簡易パーサ．
    """
    # 1) trimesh
    try:
        import trimesh  # type: ignore
        mesh = trimesh.load(path, force='mesh')
        return np.asarray(mesh.vertices, dtype=float)
    except Exception:
        pass
    # 2) numpy-stl
    try:
        from stl import mesh as stlmesh  # type: ignore
        m = stlmesh.Mesh.from_file(path)
        return m.vectors.reshape(-1,3).astype(float)
    except Exception:
        pass
    # 3) binary STL quick read
    try:
        with open(path, "rb") as f:
            header = f.read(80)
            tri_count = struct.unpack("<I", f.read(4))[0]
            rec = f.read()
        # 50 bytes per triangle: 12 floats (48B) + attr(2B)
        expected = tri_count * 50
        if len(rec) == expected:
            data = np.frombuffer(rec, dtype=np.dtype('<f4'))
            data = data.reshape(-1, 12)[:, 1:10]  # skip normal, keep 9 floats=3 verts
            return data.reshape(-1, 3).astype(float)
    except Exception:
        pass
    # 4) ascii fallback (very slow, but last resort)
    try:
        verts = []
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                line = line.strip()
                if line.lower().startswith("vertex"):
                    _, x, y, z = line.split()
                    verts.append([float(x), float(y), float(z)])
        if verts:
            return np.asarray(verts, dtype=float)
    except Exception:
        pass
    raise RuntimeError(f"Failed to read STL vertices: {path}")

def aabb_from_stl(path):
    V = try_load_stl_vertices(path)
    mn = V.min(axis=0)
    mx = V.max(axis=0)
    return mn, mx
