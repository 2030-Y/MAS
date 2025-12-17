import math
import numpy as np

# -------- Time-To-Collision (TTC) parameters & robot geometry --------
TTC_TRIG = 1.2                  # Enter slowdown when min TTC below this (s)
TTC_HARD = 0.6                  # Hard stop threshold (s)
ESC_TTC_HARD = 0.6              # Engage escape when TTC < this (s)
ESC_TTC_RELEASE = 0.8           # Release escape when TTC > this (s)
ESC_W = 1.2                     # Escape angular velocity magnitude (rad/s)
ESC_CONE = math.radians(100)    # Consider beams within ± this angle for escape
ROBOT_R = 0.1                  # Robot radius (m)
A_MAX = 1.5                     # Linear acceleration for speed limiting (m/s^2)


# -------- Read lidar scan --------
def _lidar(ego):
    if hasattr(ego, "get_lidar_scan") and callable(getattr(ego, "get_lidar_scan")):
        sc = ego.get_lidar_scan()
        r = np.asarray(sc.get("ranges", []), float)
        if r.size:
            amin = float(sc.get("angle_min", -math.pi / 2))
            ainc = float(sc.get("angle_increment", sc.get("angle_inc", 0.0)))
            if ainc == 0.0:
                amax = float(sc.get("angle_max", amin + math.pi))
                n = r.size
                a = amin + (amax - amin) * (np.arange(n, dtype=float) / max(1, n - 1))
            else:
                a = amin + ainc * np.arange(r.size, dtype=float)
            return a, r
    return np.asarray([], float), np.asarray([], float)


# -------- Limit forward speed based on 1D TTC computed along lidar beams. --------
def _ttc_v_limit_from_lidar(ego, v_cmd):
    ang, rng = _lidar(ego)
    v_allow = v_cmd
    ttc_min = 1e9
    rmin = 1e9

    if ang.size > 0 and rng.size > 0 and np.isfinite(rng).any():
        for a, r in zip(ang, rng):
            if not np.isfinite(r):
                continue
            # Clearance in front of the robot (deduct robot radius + small margin)
            d = max(0.0, r - ROBOT_R - 0.05)
            rmin = min(rmin, d)
            closing = max(0.0, v_cmd * math.cos(float(a)))
            if closing > 1e-6:
                ttc = d / closing
                ttc_min = min(ttc_min, ttc)

    if ttc_min < TTC_HARD:
        v_allow = 0.0
    elif ttc_min < TTC_TRIG:
        v_allow = min(v_cmd, max(0.0, A_MAX * (ttc_min - 0.1)))

    return v_allow, ttc_min, rmin

# -------- Compute an escape angular velocity based on the closest obstacle within ESC_CONE. --------
def _escape_turn_from_lidar(ang, rng):
    if ang.size == 0 or rng.size == 0 or not np.isfinite(rng).any():
        return +ESC_W
    mask = np.isfinite(rng) & (np.abs(ang) <= ESC_CONE)
    if not np.any(mask):
        return +ESC_W
    a_sel = float(ang[mask][np.argmin(rng[mask])])
    return -math.copysign(ESC_W, a_sel)

def _collision_avoidance_vel(ego, v ,w, W_MAX):
    v, ttc_min, rmin = _ttc_v_limit_from_lidar(ego, v)
    ang, rng = _lidar(ego)
    w_escape = _escape_turn_from_lidar(ang, rng)
    if ttc_min < ESC_TTC_HARD:
        alpha = 1.0
    elif ttc_min < ESC_TTC_RELEASE:
        alpha = (ESC_TTC_RELEASE - ttc_min) / (ESC_TTC_RELEASE - ESC_TTC_HARD)
    else:
        alpha = 0.0

    w = (1.0 - alpha) * w + alpha * w_escape

    # Escape-turn state (enter/exit with hysteresis on TTC)
    if not hasattr(ego, "_esc_lock"):
        ego._esc_lock = False
    if ttc_min < ESC_TTC_HARD:
        ego._esc_lock = True
    if ego._esc_lock:
        ang, rng = _lidar(ego)
        w = _escape_turn_from_lidar(ang, rng)
        if ttc_min >= ESC_TTC_RELEASE:
            ego._esc_lock = False

    w = np.clip(w, -W_MAX, W_MAX)
    return v,w