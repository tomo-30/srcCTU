# include/math_utils.py
import math

def fmt(n):
    return str(n) if isinstance(n, int) else f"{float(n):.6f}"

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
    return [qx/n, qy/n, qz/n, qw/n]

def make_via_points_by_step(curr_xyz, target_xyz, step_mm: float):
    cx, cy, cz = curr_xyz
    tx, ty, tz = target_xyz
    dx, dy, dz = tx - cx, ty - cy, tz - cz
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    if dist <= 1e-6:
        return [target_xyz]
    n_seg = max(1, int(dist // max(step_mm, 1e-6)))
    via = []
    for i in range(1, n_seg + 1):
        s = i / n_seg
        via.append((cx + dx*s, cy + dy*s, cz + dz*s))
    via[-1] = (tx, ty, tz)
    return via
