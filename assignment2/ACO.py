from irsim.lib import register_behavior
import numpy as np
import math
import weakref
import random
from collections import deque
from movement_style import _waypoint_ve1,_circle_vel
from collision_avoidance import _collision_avoidance_vel,_lidar


# -------------------- Global parameters --------------------
PH_SIZE = (50, 50)            # Pheromone grid resolution (ny, nx)
PH_EVAP = 0.99                  # Evaporation factor: tau(t+1) = (1 - rho) * tau(t)
PH_DIFF = 0.01                  # Diffusion blend (4-neighbour averaging)
PH_DEP_L = 50.0                # Leader deposit Δτ^k
PH_DEP_F = 5.0                  # Follower deposit Δτ^k

# ACO exponents and policy
ALPHA = 2.0                     # Weight of pheromone τ^α
BETA = 1.0                      # Weight of heuristic η^β
Q0 = 0.5                        # Pseudo-greedy prob: with q0 choose argmax, else roulette
N_DIR = 16                      # Candidate headings for followers (discretisation)
LOOKAHEAD = 0.1                # Evaluate τ,η at a small forward look
MIN_TAU = 1e-6                  # Numerical floor for τ

# Heuristic: weights
ETA_W_CLEAR = 1.0               # Clearance from lidar in that direction
ETA_W_LEADER = 0.8              # Attraction to leader (inverse distance at lookahead point)

# Kinematics
VN, KW = 0.75, 2.0              # Nominal forward speed scale & turn gain

# Spacing (used in simple proximity penalty metric)
DESIRED_D = 0.5                 # Desired minimum separation between agents


# Shared state
_LEADER_POSE = None             # Cached leader pose (x, y, theta)
_FIELD = None                   # Global pheromone field instance
_STEP_OWNER = 0                 # Robot id responsible for advancing the field step()

PATH_BAND = 0.5                # To decide whether follower is on the path
_LEAD_PATH = deque(maxlen=500)  # leader path storage


# -------------------- Pheromone field & metrics --------------------
class _Field:
    """Simple 2D grid representing pheromone intensity with evaporation and diffusion."""

    def __init__(self, bounds):
        self.bounds = tuple(bounds)
        ny, nx = PH_SIZE
        self.g = np.zeros((ny, nx), np.float32)

    def _idx(self, x, y):
        """
        Convert world coordinates to grid indices clamped to grid extent.
        """
        xmin, xmax, ymin, ymax = self.bounds
        ny, nx = PH_SIZE
        u = (x - xmin) / max(1e-9, (xmax - xmin))
        v = (y - ymin) / max(1e-9, (ymax - ymin))
        ix = int(np.clip(u * (nx - 1), 0, nx - 1))
        iy = int(np.clip(v * (ny - 1), 0, ny - 1))
        return iy, ix

    def step(self):
        """Apply evaporation and 4-neighbour diffusion in-place."""
        g = self.g
        g *= PH_EVAP
        if PH_DIFF > 0.0:
            up = np.roll(g, -1, 0)
            dn = np.roll(g, +1, 0)
            lf = np.roll(g, -1, 1)
            rt = np.roll(g, +1, 1)
            self.g[:] = (1.0 - PH_DIFF) * g + PH_DIFF * 0.25 * (up + dn + lf + rt)

    def deposit(self, x, y, amt):
        iy, ix = self._idx(x, y)
        self.g[iy, ix] += float(amt)

    def val_at(self, x, y):
        iy, ix = self._idx(x, y)
        return float(self.g[iy, ix])

# -------------------- IR-SIM adapters --------------------
def _pose(ego):
    s = np.asarray(ego.state, float).reshape(-1)
    return float(s[0]), float(s[1]), float(s[2])


def _robot_id(ego):
    rid = getattr(ego, "id", None)
    return int(rid) if rid is not None else 0


def _is_leader(ego, kwargs):
    if "role" in kwargs and isinstance(kwargs["role"], str):
        if kwargs["role"].lower() == "leader":
            return True
    return False


def _get_field(ego):
    """
    Retrieve or lazily create the global pheromone field.
    """
    global _FIELD, _STEP_OWNER
    if _FIELD is None:
        # World bounds are assumed to be [0,10]x[0,10]; adjust if needed.
        bounds = (0.0, 10.0, 0.0, 10.0)
        _FIELD = _Field(bounds)
        _STEP_OWNER = getattr(ego, "id", 0)
    return _FIELD


def _shape_like_ref(u, ref_like):
    """
    Return a numpy array shaped like ref_like (if 2D), else 1D.
    """
    arr = np.asarray(u, np.float32).reshape(-1)
    ref = np.asarray(ref_like) if ref_like is not None else None
    return arr.reshape(-1, 1) if (ref is not None and ref.ndim == 2) else arr


def _agent_limits(ego):
    """
    Read per-agent velocity limits.
    """
    try:
        vmax = float(getattr(ego, "vel_max", [1.0, 1.2])[0])
        wmax = float(getattr(ego, "vel_max", [1.0, 1.2])[1])
    except Exception:
        vmax, wmax = 1.0, 1.2
    return vmax, wmax

# -------------------- ACO helpers (η, p, sampling) --------------------
def _leader_pose(objects):
    """
    Find leader pose.
    """
    if isinstance(objects, (list, tuple)):
        for o in objects:
            if getattr(o, "role", "") == "leader":
                return _pose(o)
    return _LEADER_POSE


def _clearance_in_heading(ego, hdg):
    """
    Return lidar clearance along an absolute heading (metres, clipped).
    """
    th = _pose(ego)[2]
    ang, rng = _lidar(ego)
    if ang is None or len(ang) == 0 or len(rng) == 0:
        return 0.5  # Fallback if lidar not available
    rel = (hdg - th + math.pi) % (2 * math.pi) - math.pi
    idx = int(np.argmin(np.abs(ang - rel)))
    r = float(rng[idx]) if np.isfinite(rng[idx]) and rng[idx] > 0.0 else 0.0
    return max(0.0, r)


def _heuristic_eta(ego, objects, x, y, hdg):
    """
    Heuristic η combines clearance (lidar) and attractiveness to the leader.
    """
    clr = _clearance_in_heading(ego, hdg)
    clr_score = clr
    lp = _leader_pose(objects)
    if lp is None:
        lead_score = 0.0
    else:
        xl, yl, _ = lp
        x2 = x + LOOKAHEAD * math.cos(hdg)
        y2 = y + LOOKAHEAD * math.sin(hdg)
        d = math.hypot(x2 - xl, y2 - yl)
        lead_score = 1.0 / (1e-6 + d)
    eta = ETA_W_CLEAR * clr_score + ETA_W_LEADER * lead_score
    return max(1e-9, eta)


def _sample_heading_by_tau_eta(field, ego, objects, x, y, th):
    """
    Discretise candidate headings; compute τ,η then sample with p ∝ τ^α η^β.
    """
    angles = th + np.linspace(-math.pi, math.pi, N_DIR, endpoint=False)
    tau = np.zeros(N_DIR, float)
    eta = np.zeros(N_DIR, float)

    for i, hdg in enumerate(angles):
        x2 = x + LOOKAHEAD* math.cos(hdg)
        y2 = y + LOOKAHEAD* math.sin(hdg)
        tau[i] = max(MIN_TAU, field.val_at(x2, y2))
        eta[i] = _heuristic_eta(ego, objects, x, y, hdg)

    if np.max(eta) > 0:
        eta = eta / (np.max(eta) + 1e-9)

    score = np.power(tau, ALPHA) * np.power(eta, BETA)

    if random.random() < Q0:
        k = int(np.argmax(score))
    else:
        s = np.sum(score)
        if s <= 0.0 or not np.isfinite(s):
            k = int(np.argmax(score))
        else:
            p = score / s
            k = int(np.random.choice(np.arange(N_DIR), p=p))
    return float(angles[k])

# -------------------- Evaluation --------------------
_M = {
    "steps": 0,
    "sum_dist_to_leader": 0.0,
    "sum_inter_member_dist": 0.0,
    "follower_count": 0,
    "onpath_hits": 0,
    "follower_samples": 0,
    "pheromone_sum": 0.0,
}

_PEERS = []

def _register_peer(ego):
    for w in _PEERS:
        if w() is ego:
            return
    _PEERS.append(weakref.ref(ego))

def _average_sep_from_peers(ego):
    count = 1
    sum_sep = 0
    if not hasattr(ego, "state"):
        return None
    ex, ey = float(ego.state[0]), float(ego.state[1])
    for w in _PEERS:
        other = w()
        if (other is None) or (other is ego) or (not hasattr(other, "state")):
            continue
        ox, oy = float(other.state[0]), float(other.state[1])
        d = np.hypot(ox - ex, oy - ey)
        sum_sep += d
        count +=1
    return sum_sep/count

def _min_dist_to_polyline(px, py, poly):

    if not poly:
        return float("inf")
    it = iter(poly)
    x1, y1 = next(it)
    best = math.hypot(px - x1, py - y1)  
    for x2, y2 in it:
        dx, dy = (x2 - x1), (y2 - y1)
        seg_len2 = dx*dx + dy*dy
        if seg_len2 <= 1e-12:
            d = math.hypot(px - x1, py - y1)
        else:
            t = ((px - x1)*dx + (py - y1)*dy) / seg_len2
            t = max(0.0, min(1.0, t))
            qx, qy = (x1 + t*dx, y1 + t*dy)
            d = math.hypot(px - qx, py - qy)
        if d < best:
            best = d
        x1, y1 = x2, y2
    return best

def metrics(ego, is_leader):
    global _M, _PEERS

    _M["steps"] += 1
    _register_peer(ego)
    x, y, _ = _pose(ego)
    field = _get_field(ego)

    # leader-follower distance 
    if not is_leader:
        lx , ly, _= _LEADER_POSE
        _M["sum_dist_to_leader"] += math.hypot(x - lx, y - ly)
        _M["sum_inter_member_dist"] += _average_sep_from_peers(ego)
        _M["pheromone_sum"] += field.val_at(x,y)
        _M["follower_count"] += 1
        #whether follower is on the path
        if len(_LEAD_PATH) >= 1:
            d_path = _min_dist_to_polyline(x, y, _LEAD_PATH)
            _M["follower_samples"] += 1
            if d_path <= PATH_BAND:
                _M["onpath_hits"] += 1
 

def metrics_report():
    steps = max(_M["steps"], 1)
    report = {
        "mean_dist_to_leader": _M["sum_dist_to_leader"] / max(_M["follower_count"], 1),
        "mean_inter_member_dist": _M["sum_inter_member_dist"] / max(_M["follower_count"], 1),
        "follower_onpath_ratio": _M["onpath_hits"] / max(_M["follower_samples"], 1),
        "mean_pheromone_conc": _M["pheromone_sum"] / max(_M["follower_count"], 1),
        "steps": steps,
    }

    print("[Metrics]", {k: round(v, 4) if isinstance(v, float) else v for k, v in report.items()})
    return report

# -------------------- Main behavior --------------------
# Followers sample headings by combining pheromone τ and heuristic η 
@register_behavior("diff", "aco_follow_line")
def beh_diff_aco_follow(ego_object, objects=None, *args, **kwargs):
    field = _get_field(ego_object)
    x, y, th = _pose(ego_object)
    is_leader = _is_leader(ego_object, kwargs)
    V_MAX_STEP, W_MAX_STEP = _agent_limits(ego_object)

    # Ensure ego_object.role reflects kwargs for external tools/plots
    if "role" in kwargs and isinstance(kwargs["role"], str):
        if getattr(ego_object, "role", None) != kwargs["role"]:
            ego_object.role = kwargs["role"]

    # Pheromone update (single step-owner advances the field dynamics)
    if getattr(ego_object, "id", None) == _STEP_OWNER:
        field.step()
    field.deposit(x, y, PH_DEP_L if is_leader else PH_DEP_F)

    # Desired motion
    global _LEADER_POSE
    if is_leader:
        _LEADER_POSE = (x, y, th)
        v, w = _waypoint_ve1(ego_object, x, y, th, V_MAX_STEP ,W_MAX_STEP)
        # v, w =_circle_vel(x, y, th, V_MAX_STEP, W_MAX_STEP , (5,5) , 3)
        v, w = _collision_avoidance_vel(ego_object, v ,w, W_MAX_STEP)
        return _shape_like_ref([v, w], getattr(ego_object, "vel_min", None))
    
    hdg = _sample_heading_by_tau_eta(field, ego_object, objects, x, y, th)
    vx, vy = math.cos(hdg), math.sin(hdg)

    # Convert desired vector to (v, w) with turn-aware speed scheduling
    yaw = math.atan2(vy, vx)
    e = (yaw - th + math.pi) % (2 * math.pi) - math.pi
    w = KW * e
    v = np.clip(VN * (0.30 + 0.70 * max(0.0, math.cos(e))), 0.0, V_MAX_STEP)

    # TTC-based forward speed limiting
    v, w = _collision_avoidance_vel(ego_object, v, w, W_MAX_STEP)
    w = np.clip(w, -W_MAX_STEP, W_MAX_STEP)

    # evaluate
    metrics(ego_object, is_leader)

    return _shape_like_ref([v, w], getattr(ego_object, "vel_min", None))
