import math
import numpy as np


# -------- Waypoint patrol for leader --------
WAYPOINTS = [
    (1.0, 2.0),
    (5.0, 5.0),
    (6.0, 2.0),
    (9.0, 9.0),
]
WP_TOL = 0.15                   # Waypoint switch tolerance (m)
WP_SPEED = 1.0                  # Leader target speed along the waypoint vector
_WP_IDX = {}                    # Per-robot waypoint index map

def _robot_id(ego):
    rid = getattr(ego, "id", None)
    return int(rid) if rid is not None else 0

# -------- Line movement --------
def _waypoint_ve1(ego, x, y, th, V_MAX ,W_MAX ):

    rid = _robot_id(ego)
    idx = _WP_IDX.get(rid, 0)
    n = len(WAYPOINTS)

    if idx >= n - 1:
        idx = n - 2

    xg, yg = WAYPOINTS[idx + 1]
    dx, dy = (xg - x), (yg - y)
    d = math.hypot(dx, dy) + 1e-9

    if d < WP_TOL and idx < n - 1:
        idx += 1
        _WP_IDX[rid] = idx
        if idx < n - 1:
            xg, yg = WAYPOINTS[idx + 1]
            dx, dy = (xg - x), (yg - y)
            d = math.hypot(dx, dy) + 1e-9

    vx = WP_SPEED * (dx / d)
    vy = WP_SPEED * (dy / d)
    yaw = math.atan2(vy, vx)
    e = (yaw - th + math.pi) % (2 * math.pi) - math.pi    # converted to [-pi,pi]
    v = np.clip(1.0 * math.cos(e) * 1.2, 0.0, V_MAX)
    w = np.clip(1.0 * e, -W_MAX, W_MAX)
    return v, w

# -------- Circle movement --------
def _circle_vel(x, y, th, V_MAX, W_MAX, center=(5.0, 5.0), radius=4.0):
    cx, cy = center
    dx, dy = x - cx, y - cy
    r = math.hypot(dx, dy) + 1e-9
    tx, ty = dy / r, -dx / r
    rx, ry =  dx / r, dy / r
    v_t = 0.8
    v_r = -0.8 * (r - radius)         
    vx = v_t * tx + v_r * rx
    vy = v_t * ty + v_r * ry
    yaw = math.atan2(vy, vx)
    e = (yaw - th + math.pi) % (2 * math.pi) - math.pi    # converted to [-pi,pi]
    v = np.clip(1.0 * math.cos(e) * 1.2, 0.0, V_MAX)
    w = np.clip(1.0 * e, -W_MAX, W_MAX)
    return v, w