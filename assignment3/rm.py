import math
import random
import csv
import os
import numpy as np
from collections import defaultdict
from irsim.lib import register_behavior
from collision_avoidance import _collision_avoidance_vel

# -------------------- Global parameters --------------------
GRID_SIZE = 0.5                 # meters per cell
COLLECT_RADIUS = 0.3            # radius to collect reward patch
STEP_PENALTY = 0.01             # living cost per step
REWARD_PATCHES = [              # Predefined reward “patches” (center_x, center_y, reward_value)
    ( 1.0,  1.0,  5.0),
    ( 2.0,  2.0,  3.0),
    ( 3.0,  3.0,  4.0),
    ( 4.0,  4.0,  6.0),
    ( 5.0,  5.0,  5.0),
    ( 6.0,  6.0,  3.0),
    ( 7.0,  7.0,  4.0),
    ( 8.0,  8.0,  6.0),
    ( 9.0,  9.0,  4.0),
    ( 1.0,  9.0,  5.0),
    ( 2.0,  8.0,  3.0),
    ( 3.0,  7.0,  4.0),
    ( 4.0,  6.0,  6.0),
    ( 6.0,  4.0,  3.0),
    ( 7.0,  3.0,  4.0),
    ( 8.0,  2.0,  6.0),
    ( 9.0,  1.0,  4.0),
]
ACTIONS = [                 # Action set: (v, w) pairs
    (0.35,  0.0),           # forward
    (0.20,  1.2),           # forward + left
    (0.20, -1.2),           # forward + right
    (0.00,  1.8),           # rotate left
    (0.00, -1.8),           # rotate right
]
NA=len(ACTIONS)
RM_MEMORY = {}              # Global RM memory 

# Global tracker for all collected rewards (module-level singleton)
GLOBAL_COLLECTED_REWARDS = set()

def all_rewards_collected():
    return len(GLOBAL_COLLECTED_REWARDS) >= len(REWARD_PATCHES)

# -------------------- Helper functions --------------------
def _pose(ego):
    s = np.asarray(ego.state, float).reshape(-1)
    return float(s[0]), float(s[1]), float(s[2])

def state_key_from_xy(x, y):
    gx = int(math.floor(x / GRID_SIZE))
    gy = int(math.floor(y / GRID_SIZE))
    return (gx, gy)

def _shape_like_ref(u, ref_like):
    arr = np.asarray(u, np.float32).reshape(-1)
    ref = np.asarray(ref_like) if ref_like is not None else None
    return arr.reshape(-1, 1) if (ref is not None and ref.ndim == 2) else arr

def _agent_limits(ego):
    try:
        vmax = float(getattr(ego, "vel_max", [1.0, 1.2])[0])
        wmax = float(getattr(ego, "vel_max", [1.0, 1.2])[1])
    except Exception:
        vmax, wmax = 1.0, 1.2
    return vmax, wmax

def get_mem(ego):
    rid = getattr(ego, "id", None)
    rid = rid if rid is not None else id(ego)  
    if rid not in RM_MEMORY:
        RM_MEMORY[rid] = {
            "collected": set(),
            "N": defaultdict(lambda: [0] * NA),
            "sumR": defaultdict(lambda: [0.0] * NA),
            "regret": defaultdict(lambda: [0.0] * NA),
            "last_sa": None
        }
    return RM_MEMORY[rid]

def reward_computation(ego_object, mem):
    x, y, th = _pose(ego_object)
    r = -STEP_PENALTY  # small living cost
    if "prev_dmin" not in mem:
        mem["prev_dmin"] = None

    dmin = dist_to_nearest_reward(x, y, mem)
    if dmin >= 0 and mem["prev_dmin"] is not None:
        r += 0.2 * (mem["prev_dmin"] - dmin)
    mem["prev_dmin"] = dmin
    collect_now = False

    for i, (rx, ry, val) in enumerate(REWARD_PATCHES):
        # ** This logic makes sure NO patch is collected more than once **
        if i in mem["collected"] or i in GLOBAL_COLLECTED_REWARDS:
            continue
        if (x - rx) ** 2 + (y - ry) ** 2 <= COLLECT_RADIUS ** 2:
            r += float(val)
            mem["collected"].add(i)
            GLOBAL_COLLECTED_REWARDS.add(i)
            collect_now = True

    return r, collect_now

def update_regrets(mem, s_prev, a_prev, r):
    mem["N"][s_prev][a_prev] += 1
    mem["sumR"][s_prev][a_prev] += r

    means = []
    for a in range(NA):
        n = mem["N"][s_prev][a]
        means.append(mem["sumR"][s_prev][a] / n if n > 0 else 0.0)
    baseline = sum(means) / float(NA)
    for a in range(NA):
        mem["regret"][s_prev][a] += (means[a] - baseline)

def choose_new_action(ego_object, mem):
    x, y, th = _pose(ego_object)
    s = state_key_from_xy(x, y)
    EPS = 0.10  # exploration probability
    if random.random() < EPS:
        a = random.randrange(NA)
        mem["last_sa"] = (s, a)
        v, w = ACTIONS[a]
        return v, w
    
    reg = mem["regret"][s]
    pos = [max(0.0, v) for v in reg]
    Z = sum(pos)
    if Z <= 1e-12:
        a = random.randrange(NA)
    else:
        thresh = random.random() * Z
        c = 0.0
        a = 0
        for i, p in enumerate(pos):
            c += p
            if c >= thresh:
                a = i
                break
    mem["last_sa"] = (s, a)
    v, w = ACTIONS[a]
    return v, w

def dist_to_nearest_reward(x, y, mem):
    best = 1e9
    for i, (rx, ry, val) in enumerate(REWARD_PATCHES):
        if i in mem["collected"] or i in GLOBAL_COLLECTED_REWARDS:
            continue
        d = math.hypot(x - rx, y - ry)
        best = min(best, d)
    return best if best < 1e8 else -1.0

def _get_csv_writer(mem, ego):
    if "csv_writer" in mem:
        return mem["csv_writer"]
    rid = getattr(ego, "id", None)
    rid = rid if rid is not None else id(ego)
    filename = f"rm_metrics_agent_{rid}.csv"
    file_exists = os.path.isfile(filename)
    f = open(filename, "a", newline="", encoding="utf-8")
    writer = csv.writer(f)
    if not file_exists:
        writer.writerow([
            "agent_id", "t", "x", "y", "dmin",
            "episode_return", "ema_return", "recent_mean",
            "collected", "since_last_collect",
            "pos_regret_sum", "regret_max","action_dist"
        ])
    mem["csv_file"] = f
    mem["csv_writer"] = writer
    return writer

METRIC_STEP = 50  
def init_metrics(mem):
    if "metrics" in mem:
        return
    mem["metrics"] = {
        "t": 0,
        "episode_return": 0.0,
        "avg_return_ema": 0.0,     
        "ema_beta": 0.98,
        "collected_cnt": 0,
        "last_collect_t": -1,
        "since_last_collect": 0,
        "action_cnt": [0] * NA,
        "state_visits": defaultdict(int),
        "pos_regret_sum": 0.0,
        "regret_max": 0.0,
        "recent_rewards": [],
        "recent_window": 200,
    }

def update_metrics_pre_action(ego_object, mem, r, s):
    m = mem["metrics"]
    m["t"] += 1
    m["episode_return"] += float(r)
    beta = m["ema_beta"]
    m["avg_return_ema"] = beta * m["avg_return_ema"] + (1 - beta) * float(r)
    m["state_visits"][s] += 1
    m["recent_rewards"].append(float(r))
    if len(m["recent_rewards"]) > m["recent_window"]:
        m["recent_rewards"].pop(0)

def update_metrics_post_action(mem, s, a):
    m = mem["metrics"]
    m["action_cnt"][a] += 1
    reg = mem["regret"][s]
    pos = [max(0.0, v) for v in reg]
    m["pos_regret_sum"] = float(sum(pos))
    m["regret_max"] = float(max(reg)) if len(reg) else 0.0

def update_collect_metrics(mem, reward_before_penalty, collected_now):
    m = mem["metrics"]
    if collected_now:
        m["collected_cnt"] += 1
        m["last_collect_t"] = m["t"]
        m["since_last_collect"] = 0
    else:
        if m["last_collect_t"] >= 0:
            m["since_last_collect"] = m["t"] - m["last_collect_t"]

def print_metrics(mem, ego):
    rid = getattr(ego, "id", None)
    rid = rid if rid is not None else id(ego)
    m = mem["metrics"]
    if m["t"] % METRIC_STEP != 0:
        return

    x, y, th = _pose(ego)
    dmin = dist_to_nearest_reward(x, y, mem)
    total_actions = sum(m["action_cnt"]) + 1e-9
    action_dist = [c / total_actions for c in m["action_cnt"]]
    recent_mean = sum(m["recent_rewards"]) / max(1, len(m["recent_rewards"]))

    print(
        f"[RM][id={rid}] t={m['t']:6d} "
        f"pos=({x:.2f},{y:.2f}) dmin={dmin:.2f} "
        f"return={m['episode_return']:.2f} ema_r={m['avg_return_ema']:.4f} "
        f"recent_mean={recent_mean:.4f} collected={m['collected_cnt']} "
        f"since_last={m['since_last_collect']} "
        f"posRegSum={m['pos_regret_sum']:.3f} regMax={m['regret_max']:.3f} "
        f"actDist={[round(x,3) for x in action_dist]}"
    )
    writer = _get_csv_writer(mem, ego)
    writer.writerow([
        rid, m["t"],
        round(x, 4), round(y, 4), round(dmin, 4),
        round(m["episode_return"], 6), round(m["avg_return_ema"], 6), round(recent_mean, 6),
        m["collected_cnt"], m["since_last_collect"], round(m["pos_regret_sum"], 6),
        round(m["regret_max"], 6),
        [round(v, 6) for v in action_dist]
    ])

# -------------------- Main behavior --------------------
# Regret Matching (RM) on discretized grid world
@register_behavior("diff", "RM")
def beh_rm(ego_object, objects=None, *args, **kwargs):
    mem = get_mem(ego_object)
    last_sa = mem.get("last_sa", None)
    init_metrics(mem)
    r, collected_now = reward_computation(ego_object, mem)
    if last_sa is not None:
        s_prev, a_prev = last_sa
        update_regrets(mem, s_prev, a_prev, r)
    x, y, th = _pose(ego_object)
    s = state_key_from_xy(x, y)
    update_metrics_pre_action(ego_object, mem, r, s)
    update_collect_metrics(mem, r, collected_now)
    v, w = choose_new_action(ego_object, mem)
    _, W_MAX_STEP = _agent_limits(ego_object)
    v, w = _collision_avoidance_vel(ego_object, v, w, W_MAX_STEP)
    _, a = mem["last_sa"]
    update_metrics_post_action(mem, s, a)
    print_metrics(mem, ego_object)
    return _shape_like_ref([v, w], getattr(ego_object, "vel_min", None))