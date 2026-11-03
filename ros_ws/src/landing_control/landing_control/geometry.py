"""Frames: odom/world (W), base_link (B), Gazebo camera link (L), optical (C).
Assumes camera link +X forward, +Y left, +Z up; optical +Z forward,
+X right, +Y down. Confirm image axis directions in your Gazebo build.
"""
import numpy as np

def clip(value, max_val):
    return float(np.clip(value, -max_val, max_val))
    

def quat_rotation(q):
    x, y, z, w = [float(v) for v in q]
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        raise ValueError('Invalid odometry quaternion')
    s = 2.0 / n
    return np.array([
        [1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
        [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
        [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)]
    ])


def rpy_rotation(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return rz @ ry @ rx


# optical axes written as columns in Gazebo camera-link coordinates
R_LINK_OPTICAL = np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])


def marker_in_odom(p_cam, p_base, q_base, cam_xyz=(0,0,-0.015), cam_rpy=(0,1.5708,0)):
    """Transform a solvePnP marker origin (optical frame) into odom frame."""
    r_wb = quat_rotation(q_base)
    r_bc = rpy_rotation(*cam_rpy) @ R_LINK_OPTICAL
    p_b = np.asarray(cam_xyz, float) + r_bc @ np.asarray(p_cam, float)
    return np.asarray(p_base, float) + r_wb @ p_b


def generate_zigzag_waypoints(xmin, xmax, ymin, ymax, spacing, altitude):
    """Return alternating search-row endpoints, including the upper boundary."""
    if xmax <= xmin or ymax <= ymin or spacing <= 0:
        raise ValueError('Invalid coverage bounds or spacing')
    rows = []
    y = ymin
    while y <= ymax + 1e-8:
        rows.append(y)
        y += spacing
    if rows[-1] < ymax - 1e-8:
        rows.append(ymax)
    waypoints = []
    for i, y in enumerate(rows):
        x1, x2 = (xmin, xmax) if i % 2 == 0 else (xmax, xmin)
        waypoints += [(x1, y, altitude), (x2, y, altitude)]
    return waypoints
