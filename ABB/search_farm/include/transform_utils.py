# include/transform_utils.py
import numpy as np
from math import cos, sin, radians

def rpy_xyz_deg_to_rot(r_deg, p_deg, y_deg):
    """RPY=XYZ（roll->pitch->yaw）[deg] の回転行列（3x3）"""
    rx, ry, rz = map(radians, (r_deg, p_deg, y_deg))
    Rx = np.array([[1, 0, 0],
                   [0, cos(rx), -sin(rx)],
                   [0, sin(rx),  cos(rx)]], dtype=float)
    Ry = np.array([[ cos(ry), 0, sin(ry)],
                   [       0, 1,       0],
                   [-sin(ry), 0, cos(ry)]], dtype=float)
    Rz = np.array([[cos(rz), -sin(rz), 0],
                   [sin(rz),  cos(rz), 0],
                   [     0,       0,   1]], dtype=float)
    # XYZ順：R = Rz @ Ry @ Rx ではなく，右手系で roll(X)→pitch(Y)→yaw(Z) の順に適用
    # ベクトルに左から適用する前提では R = Rz * Ry * Rx
    return Rz @ Ry @ Rx

def make_T(R, t):
    """回転R(3x3)，並進t(3,)から4x4同次変換行列"""
    T = np.eye(4, dtype=float)
    T[:3,:3] = R
    T[:3, 3] = np.asarray(t, dtype=float)
    return T

def transform_points(T, pts):
    """4x4同次変換Tで Nx3 点群を変換"""
    pts = np.asarray(pts, dtype=float)
    ones = np.ones((pts.shape[0], 1), dtype=float)
    P4 = np.hstack([pts, ones])
    Q4 = (T @ P4.T).T
    return Q4[:, :3]

def aabb_from_points(pts):
    """Nx3から(min,max)のAABBを返す"""
    pts = np.asarray(pts, dtype=float)
    mn = pts.min(axis=0)
    mx = pts.max(axis=0)
    return mn, mx

def aabb_corners(min_xyz, max_xyz):
    x0,y0,z0 = min_xyz
    x1,y1,z1 = max_xyz
    return np.array([
        [x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0],
        [x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]
    ], dtype=float)
