from __future__ import annotations
import math
import weakref
import time
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional, Set
import numpy as np
from irsim.lib import register_behavior
from movement_style import _waypoint_ve1,_circle_vel
from collision_avoidance import _collision_avoidance_vel

# -------- parameters--------
COMM_TTL = 1.5                                # seconds messages stay valid
HEARTBEAT_DT = 0.7                            # leader heartbeat period
ELECTION_TIMEOUT = 1.0                        # time to wait for victory
AUCTION_ENABLED = False
EPS = 1e-6                                     # avoid div-by-zero in bids
CIRCLE_CENTER = (10.0, 10.0)
CIRCLE_RADIUS = 6.0
Side = "right"
_last_election_started_at: Optional[float] = None

# -------- message type--------
ELECTION = "election"
ANSWER = "answer"
VICTORY = "victory"
HEARTBEAT = "hb"
POSE = "pose"             # broadcast leader pose 
AUC_REQ = "auc_req"       # leader -> followers: {"slots":[...]}
BID = "bid"               # follower -> leader: {"slot":int, "price":float}
ASSIGN = "assign"         # leader -> all: {"slot":int, "winner":int, "price":float}

# ---- leader-side auction book-keeping ----
_LEADER_CURRENT: Optional[int] = None        
_ASSIGNED_SLOTS: Dict[int, int] = {}          # slot -> follower id
_ASSIGNED_FOLLOWERS: Set[int] = set()         # followers already assigned

# ==== shared bus for communication ====

class _Bus:
    """
    {"type": str, "src": int, "dst": Optional[int], "data": any,"t": float}.
    """
    def __init__(self):
        self.msgs: List[dict] = []
        self.time_zero = time.monotonic()

    def now(self) -> float:
        return time.monotonic() - self.time_zero

    def send(self, msg: dict):
        self.msgs.append(msg)                                       # dst=None for broadcast ; dst=robot_id for unicast

    def recv(self, dst: int, ttl: float) -> List[dict]:
        now = self.now()
        self.msgs = [m for m in self.msgs if (now - m.get("t", now)) <= ttl]    # Drop expired message by ttl
        return [m for m in self.msgs if m.get("dst") in (None, dst)]

BUS = _Bus()

def _broadcast(src: int, typ: str, data: dict):
    BUS.send({"type": typ, "src": src, "dst": None, "data": data, "t": BUS.now()})

def _unicast(src: int, dst: int, typ: str, data: dict):
    BUS.send({"type": typ, "src": src, "dst": dst, "data": data, "t": BUS.now()})


# ==== tools for irsim ====

def _robot_id(ego):
    rid = getattr(ego, "id", None)
    return int(rid) if rid is not None else 0


def _pose(ego):
    s = np.asarray(ego.state, float).reshape(-1)
    return float(s[0]), float(s[1]), float(s[2])

def _shape_like_ref(u, ref_like):

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

# ==== State per agent identidied by id ====

@dataclass
class AgentState:
    leader_id: Optional[int] = None
    election_mode: bool = False
    waiting_victory_until: float = 0.0
    last_heartbeat_s: float = 0.0
    slot_id: Optional[int] = None

_STATES: Dict[int, AgentState] = {}


def _st(ego) -> AgentState:
    k = id(ego)
    if k not in _STATES:
        _STATES[k] = AgentState()
    return _STATES[k]

# ==== Bully algorithm for leader election ====

def _begin_election(aid: int, state: AgentState, higher_ids: List[int], election_timeout: float):
    global _last_election_started_at
    state.election_mode = True
    state.leader_id = None
    state.waiting_victory_until = BUS.now() + election_timeout
    _last_election_started_at = BUS.now()
    for hid in higher_ids:
        _unicast(aid, hid, ELECTION, {})


def _handle_bully(ego, N: int, comm_ttl: float, election_timeout: float, heartbeat_dt: float):
    # Get id,state,time
    aid = _robot_id(ego)
    st = _st(ego)
    now = BUS.now()

    # Detect leader loss via heartbeat
    if st.leader_id is not None and aid != st.leader_id:
        if (now - st.last_heartbeat_s) > max(heartbeat_dt * 2.5, election_timeout):
            st.leader_id = None  # trigger a new election below

    # If no leader known and not already in election -> start one
    if st.leader_id is None and not st.election_mode:
        higher = [i for i in range(N) if i > aid]
        _begin_election(aid, st, higher, election_timeout)

    # Receive messages destined for me (or broadcast)
    inbox = BUS.recv(aid, ttl=comm_ttl)
    for m in inbox:
        typ, src = m["type"], m["src"]
        if typ == ELECTION:
            if src < aid:
                # If from lower ID : answer and start my own election
                _unicast(aid, src, ANSWER, {})
                if not st.election_mode:
                    higher = [i for i in range(N) if i > aid]
                    _begin_election(aid, st, higher, election_timeout)
            # If from higher ID: do nothing (wait for victory)

        elif typ == ANSWER and st.election_mode:
            # Someone higher exists; wait for VICTORY
            st.waiting_victory_until = now + election_timeout

        elif typ == VICTORY:
            st.leader_id = src
            st.election_mode = False

        elif typ == HEARTBEAT and src == st.leader_id:
            st.last_heartbeat_s = now

    # If we were waiting and no one answered: we are the leader
    if st.election_mode and now >= st.waiting_victory_until:
        st.election_mode = False
        st.leader_id = aid
        _broadcast(aid, VICTORY, {})

    # Leader heartbeat
    if st.leader_id == aid:
        # Broadcast heartbeat to show the existence of leader
        _broadcast(aid, HEARTBEAT, {})
        st.last_heartbeat_s = now

    return st.leader_id

# ==== First Price Sealed Bid(FPSB) Auctions algorithm for follower to decide slot id  ====
def _reset_auction_state():
    global _ASSIGNED_SLOTS, _ASSIGNED_FOLLOWERS
    _ASSIGNED_SLOTS = {}
    _ASSIGNED_FOLLOWERS = set()

def _start_FPSB_auction(ego, N):
    my_id = _robot_id(ego)
    if AUCTION_ENABLED:
        followers = [i for i in range(N) if i != my_id]
        total_slots = len(followers)
        pending_slots = [s for s in range(1, total_slots + 1) if s not in _ASSIGNED_SLOTS]
        if pending_slots:
            # ask for bids for all pending slots at once
            _broadcast(my_id, AUC_REQ, {"slots": pending_slots})
            # collect bids currently on bus
            inbox = BUS.recv(my_id, ttl=COMM_TTL)
            bids_by_slot: Dict[int, List[Tuple[float,int]]] = {s: [] for s in pending_slots}
            for m in inbox:
                if m.get("type") == BID and m.get("dst") in (None, my_id):
                    d = m.get("data", {})
                    slot = int(d.get("slot", -1))
                    price = float(d.get("price", 0.0))
                    bidder = int(m.get("src", -1))
                    if slot in bids_by_slot and bidder in followers and bidder not in _ASSIGNED_FOLLOWERS:
                        bids_by_slot[slot].append((price, bidder))
                # sequentially assign slots to highest bidders; winners don't participate further
            for slot in sorted(pending_slots):
                # remove bidders that already won a previous slot in this same pass
                opts = [t for t in bids_by_slot.get(slot, []) if t[1] not in _ASSIGNED_FOLLOWERS]
                if not opts:
                    continue
                price, winner = max(opts, key=lambda t: (t[0], -t[1]))  # tie-break by smaller id
                _ASSIGNED_SLOTS[slot] = winner
                _ASSIGNED_FOLLOWERS.add(winner)
                _broadcast(my_id, ASSIGN, {"slot": slot, "winner": winner, "price": price})
    
def _handle_auction(ego, lp, leader_id, x, y):
    if AUCTION_ENABLED:
        my_id = _robot_id(ego)
        st = _st(ego)
        inbox = BUS.recv(my_id, ttl=COMM_TTL)
        for m in inbox:
            typ, src = m["type"], m["src"]
            if typ == AUC_REQ and st.slot_id is None and lp is not None:
                # compute sealed bids for each requested slot
                slots = list(m.get("data", {}).get("slots", []))
                for s in slots:
                    side, rank = idx_to_side_rank(st.slot_id)
                    gx, gy = v_anchor_xy(lp, side, rank)
                    dist = math.hypot(gx - x, gy - y)
                    price = 1.0 / (dist + EPS)   
                    _unicast(my_id, leader_id, BID, {"slot": int(s), "price": float(price)})

            elif typ == ASSIGN:
                d = m.get("data", {})
                s = int(d.get("slot", -1))
                winner = int(d.get("winner", -1))
                # if I win, store my slot_id
                if winner == my_id:
                    st.slot_id = s
        return st.slot_id
    return None

# ==== V shape formation of followers ====

def idx_to_side_rank(idx: Optional[int]):
    if idx is None:
        return "right", 1
    k = idx // 2 + 1
    side = "right" if (idx % 2 == 0) else "left"
    return side, k

def v_anchor_xy(leader_xyh, side = Side, rank=1, spacing = 0.5,angle_deg = 45.0,back_offset= 0):

    xL, yL, th = leader_xyh
    phi = math.radians(angle_deg)
    fx, fy = math.cos(th), math.sin(th)
    rx, ry = math.cos(th + math.pi/2.0), math.sin(th + math.pi/2.0)

    s = +1.0 if side.lower().startswith("r") else -1.0
    back = back_offset + rank * spacing * math.cos(phi)
    lat  = s * rank * spacing * math.sin(phi)

    x = xL - back * fx + lat * rx
    y = yL - back * fy + lat * ry
    return (x, y)


def _read_leader_pose_from_bus(my_id: int, leader_id: int, comm_ttl: float) -> Optional[Tuple[float, float, float]]:

    inbox = BUS.recv(my_id, ttl=comm_ttl)
    latest = None
    t_latest = -1.0
    for m in inbox:
        if m.get("type") == POSE and m.get("src") == leader_id:
            t = float(m.get("t", 0.0))
            if t > t_latest:
                t_latest = t
                latest = m
    if latest is not None:
        d = latest["data"]
        return float(d["x"]), float(d["y"]), float(d["th"])
    return None

def _ctrl_to_point(x, y, th, xt, yt, speed: float) -> Tuple[float, float]:
    dx, dy = xt - x, yt - y
    dist = math.hypot(dx, dy)
    tgt_th = math.atan2(dy, dx)
    e_th = (tgt_th - th + math.pi) % (2*math.pi) - math.pi
    v = speed * max(0.0, math.tanh(dist))
    w = 2.0 * e_th  
    return v, w 

# ==== Evaluation ====
_M = {
    "steps": 0,
    "sum_dist_to_leader": 0.0,
    "sum_inter_member_dist": 0.0,
    "sum_pos_err": 0.0,
    "follower_count": 0,
    "sum_election_time": 0.0,
    "election_count": 0,
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

def metrics(ego, leader_id, lp, gx, gy):
    global _M, _PEERS,_last_election_started_at

    _M["steps"] += 1
    _register_peer(ego)
    my_id = _robot_id(ego)
    x, y, _ = _pose(ego)

    # election duration
    if _last_election_started_at is not None and leader_id is not None:
        _M["sum_election_time"] += BUS.now() - _last_election_started_at
        _M["election_count"] += 1
        _last_election_started_at = None 

    # leader-follower distance & position error
    if my_id  != leader_id and lp is not None:
        lx , ly, _= lp
        _M["sum_dist_to_leader"] += math.hypot(x - lx, y - ly)
        _M["sum_pos_err"] += math.hypot(x - gx, y - gy)
        _M["sum_inter_member_dist"] += _average_sep_from_peers(ego)
        _M["follower_count"] += 1
 

def metrics_report():
    steps = max(_M["steps"], 1)
    report = {
        "mean_dist_to_leader": _M["sum_dist_to_leader"] / max(_M["follower_count"], 1),
        "mean_inter_member_dist": _M["sum_inter_member_dist"] / max(_M["follower_count"], 1),
        "avg_election_time": _M["sum_election_time"] / max(_M["election_count"], 1),
        "mean_pos_error": _M["sum_pos_err"] / max(_M["follower_count"], 1),
        "steps": steps,
    }

    print("[Metrics]", {k: round(v, 4) if isinstance(v, float) else v for k, v in report.items()})
    return report

# ==== register behavior : bully fleet ====
@register_behavior("diff", "bully_fleet")
def beh_diff_bully_fleet(ego_object, objects=None, *args, **kwargs):
    global _LEADER_CURRENT, SPEED ,AUCTION_ENABLED
    N = int(kwargs.get("N", 5))
    my_id = _robot_id(ego_object)
    x, y, th = _pose(ego_object)
    st = _st(ego_object)
    V_MAX, W_MAX = _agent_limits(ego_object)

    # 1) Run bully election
    leader_id = _handle_bully(ego_object, N, COMM_TTL, ELECTION_TIMEOUT, HEARTBEAT_DT)    
    if my_id == leader_id:
        ego_object.color = "brown"
        ego_object.traj_color = "brown" 
    else:
        ego_object.color = "green"             
        ego_object.traj_color = "green"  
    
    # 2) Detect leader change to reset auction state
    if leader_id != _LEADER_CURRENT :
        _LEADER_CURRENT = leader_id
        _reset_auction_state()
        st.slot_id = None

    if my_id == leader_id:
        # 3) announce pose
        _broadcast(my_id, POSE, {"x": x, "y": y, "th": th})

        # 4) start auction
        _start_FPSB_auction(ego_object, N)

        # 5) calculate leader v,w considering collision avoidance
        # v, w = _waypoint_ve1(ego_object,x, y, th, V_MAX_STEP, W_MAX_STEP)
        v, w =_circle_vel(x, y, th, V_MAX, W_MAX , CIRCLE_CENTER , CIRCLE_RADIUS)
        v, w = _collision_avoidance_vel(ego_object, v ,w, W_MAX)
        SPEED = abs(v)                                                    # control follower v
        return _shape_like_ref([v, w], getattr(ego_object, "vel_min", None))


    # 6) read leader pose
    lp = _read_leader_pose_from_bus(my_id, leader_id, COMM_TTL)

    # 7) handle auction to get the slot of follower
    st.slot_id = _handle_auction(ego_object, lp, leader_id ,x, y)

    if lp is not None :
        AUCTION_ENABLED = True

    # 8) calculate follower v,w considering collision avoidance
    if st.slot_id is None or lp is None:
        v, w = 0.1, 0.1
        metrics(ego_object, leader_id, lp, x, y)     # evaluate
    else:
        side, rank = idx_to_side_rank(st.slot_id)
        gx, gy = v_anchor_xy(lp, side, rank)
        v, w = _ctrl_to_point(x, y, th, gx, gy, SPEED)
        metrics(ego_object, leader_id, lp, gx, gy)    # evaluate
    v, w = _collision_avoidance_vel(ego_object, v, w, W_MAX)

    return _shape_like_ref([v, w], getattr(ego_object, "vel_min", None))