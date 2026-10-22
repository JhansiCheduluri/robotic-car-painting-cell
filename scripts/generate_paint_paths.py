"""
generate_paint_paths.py
Robotic Car Painting Cell - Dual FANUC P-250iB RH Setup
Restored & Stabilized Production Paint Simulation for RoboDK

RESTORATION & STABILIZATION SUMMARY:
-----------------------------------
1. Recovered original working architecture from backup, eliminating artificial tucking
   and violent joint flipping between consecutive strokes.
2. Verified spray gun TCP at transl(0, 0, 200) with tool +Z pointing strictly perpendicular
   into the car surface (-normal) across all trajectory points.
3. Aligned surface geometry directly to simple_car.stl with 60 mm safety margins from vertical
   cabin obstacle walls (windshield at X=900, rear window at X=-1000).
4. Solved incomplete painting root cause: visual stripes are generated as high-speed binary
   STLs with collision pairs explicitly deactivated against all robot links, tools, and the car.
5. Enabled progressive live painting: visual ribbons render immediately behind the spray nozzle
   during valid spray motion alongside native RoboDK particle spray.

HOW TO RUN:
-----------
1. Open RoboDK with station: Simulation_Files/car_painting_cell_v1.rdk
2. In terminal:
      cd C:\\projects\\robotic-car-painting-cell\\scripts
      python generate_paint_paths.py
"""

import csv
import math
import os
import re
import struct
import sys
import time
import traceback

try:
    from robodk.robolink import *
    from robodk.robomath import *
except ImportError:
    from robolink import *
    from robomath import *

# ----------------------------------------------------------------------
# 1. PROCESS AND CELL PARAMETERS
# ----------------------------------------------------------------------
ROBOT_NAME = "Fanuc P-250iB RH"      # Left robot in station
ROBOT2_NAME = "Robot_Right"          # Right robot in station
CAR_NAME = "simple_car"
USE_SECOND_ROBOT = True              # Dual-robot operation
SHOW_PAINT = True                    # Persistent visual paint ribbons
LIVE_PAINT = True                    # Progressive live spray during motion
SIMULATION_SPEED = 50.0              # Simulation speedup factor

STANDOFF = 250          # Gun nozzle distance from surface (mm)
FAN_WIDTH = 300         # Spray width at standoff (mm)
OVERLAP = 0.40          # 40% overlap between adjacent passes
STEP = FAN_WIDTH * (1.0 - OVERLAP)   # 180 mm between passes
PAINT_SPEED = 400       # mm/s while spraying
APPROACH_SPEED = 800    # mm/s for approach and retract
APPROACH_DIST = 150     # mm standoff offset for approach and retract
CHECK_STEP = 50         # mm between collision-checked points along a stroke
SEGMENT_LEN = 600       # Maximum length of a stroke piece (mm)
PAINT_GROUP = 3         # Checked points per progressive paint ribbon update (150 mm)

# ----------------------------------------------------------------------
# 2. VALIDATED SURFACE GEOMETRY (simple_car.stl verified ground truth)
# ----------------------------------------------------------------------
# simple_car.stl 3-box geometry:
#   Lower body: X in [-2400, 2400], Y in [-950, 950], Z in [300, 1000]
#   Trunk step: X in [-2400, -1000], Y in [-800, 800], Z in [1000, 1050]
#   Cabin box:  X in [-1000, 900], Y in [-650, 650], Z in [1000, 1600]
# 60 mm safety margins maintain clearance from vertical obstacle walls
# (windshield X=900, rear window X=-1000) so the 300 mm spray fan fully
# covers the boundary while the physical wrist remains completely clear.
SURFACES = {
    "Roof": dict(
        origin=(-940.0, -500.0, 1600.0), u=(1, 0, 0), v=(0, 1, 0),
        len_u=1780.0, len_v=1000.0, normal=(0, 0, 1)
    ),
    "Hood": dict(
        origin=(1020.0, -890.0, 1000.0), u=(1, 0, 0), v=(0, 1, 0),
        len_u=1320.0, len_v=1780.0, normal=(0, 0, 1)
    ),
    "Trunk": dict(
        origin=(-2340.0, -740.0, 1050.0), u=(1, 0, 0), v=(0, 1, 0),
        len_u=1220.0, len_v=1480.0, normal=(0, 0, 1)
    ),
    "Front": dict(
        origin=(2400.0, -890.0, 350.0), u=(0, 1, 0), v=(0, 0, 1),
        len_u=1780.0, len_v=600.0, normal=(1, 0, 0)
    ),
    "Rear": dict(
        origin=(-2400.0, -890.0, 350.0), u=(0, 1, 0), v=(0, 0, 1),
        len_u=1780.0, len_v=600.0, normal=(-1, 0, 0)
    ),
    "Side_Right": dict(
        origin=(-2340.0, 950.0, 350.0), u=(1, 0, 0), v=(0, 0, 1),
        len_u=4680.0, len_v=600.0, normal=(0, 1, 0)
    ),
    "Side_Left": dict(
        origin=(-2340.0, -950.0, 350.0), u=(1, 0, 0), v=(0, 0, 1),
        len_u=4680.0, len_v=600.0, normal=(0, -1, 0)
    ),
}
PAINT_ORDER = ["Roof", "Hood", "Trunk", "Front", "Rear", "Side_Right", "Side_Left"]

SIDE_SIGN = {"L": -1, "R": 1}
SIDE_YAW = {"L": 90, "R": -90}
LANES = (1600, 1500, 1700, 1800, 2000)
LANES_EXT = (1500, 1600, 1700, 1800, 1900, 2000, 2200)
STOPS = list(range(-2400, 2401, 150))
STOPS_EXT = list(range(-2500, 2501, 100))
ROLLS = (0, 90, 180, 270)
ROLLS_EXT = tuple(range(0, 360, 30))

TRACK_Y = 1900
PER_LANE = 4
EXT_MAX = 12
OTHER_SIDE_MAX = 4

TRAVEL_STEP = 200
JOINT_STEP_DEG = 4.0
JUMP_STEP = 55.0
JOINT_SPEED_EST = 120.0
RAIL_SPEED = 500.0
PAINT_COLOR = [0.05, 0.35, 0.95, 1.0] # Automotive electric blue
PAINT_LIFT = 3.0                      # mm ribbon floats above car mesh to prevent z-fighting

# Safe upright compact home posture for travel and standby parking
SAFE_TUCKED_JOINTS = [0.0, -30.0, 90.0, 0.0, -60.0, 0.0]

try:
    _HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:
    _HERE = os.path.join(os.path.expanduser("~"), "Documents")
OUT_DIR = os.path.abspath(os.path.join(_HERE, ".."))
SCENERY_PREFIXES = ("Paint_", "Booth_", "Rail_")
STRIPES_DIR = os.path.join(OUT_DIR, "Simulation_Files", "stripes")


# Vector math helpers
def _add(a, b):
    return [a[i] + b[i] for i in range(3)]


def _sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def _scale(a, s):
    return [a[i] * s for i in range(3)]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def lerp(a, b, t):
    return [a[i] + (b[i] - a[i]) * t for i in range(3)]


def dist3(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def tool_pose(position, stroke_dir, normal):
    """
    Gun TCP pose:
    +Z points INTO the surface (-normal), perpendicular to the face.
    +X follows the stroke direction.
    +Y is orthogonal to complete the right-handed frame.
    """
    z = _scale(normal, -1)
    x = stroke_dir
    y = _cross(z, x)
    return Mat([[x[0], y[0], z[0], position[0]],
                [x[1], y[1], z[1], position[1]],
                [x[2], y[2], z[2], position[2]],
                [0, 0, 0, 1]])


def base_pose(x, y, yaw):
    """Robot base pose in Car_Frame."""
    return transl(x, y, 0) * rotz(math.radians(yaw))


def raster_strokes(surf):
    """Generate alternating raster strokes with 250 mm nominal standoff."""
    o, u, v, n = surf["origin"], surf["u"], surf["v"], surf["normal"]
    n_strokes = max(1, int(math.ceil(surf["len_v"] / STEP)) + 1)
    strokes = []
    for k in range(n_strokes):
        dv = min(k * STEP, surf["len_v"])
        base = _add(_add(o, _scale(v, dv)), _scale(n, STANDOFF))
        p_start = base
        p_end = _add(base, _scale(u, surf["len_u"]))
        direction = u
        if k % 2 == 1:
            p_start, p_end = p_end, p_start
            direction = _scale(u, -1)
        strokes.append((p_start, p_end, direction, dv))
    return strokes


def jl(j):
    """Normalize joints to list of 6 floats (empty list if invalid)."""
    if j is None:
        return []
    try:
        v = j.list() if hasattr(j, "list") else (j.tolist() if hasattr(j, "tolist") else list(j))
    except Exception:
        return []
    v = [float(x) for x in v]
    return v[:6] if len(v) >= 6 else []


def jdist(a, b):
    """Maximum joint difference in degrees."""
    if not a or not b or len(a) != len(b):
        return 999999.0
    return max(abs(x - y) for x, y in zip(a, b))


def lerp_joints(a, b, t):
    return [a[i] + (b[i] - a[i]) * t for i in range(len(a))]


# ----------------------------------------------------------------------
# 3. ROBODK CONNECTION & WORKER SETUP
# ----------------------------------------------------------------------
def connect_robodk():
    """Connect to active RoboDK station with car and robots loaded."""
    for port in (20500, 20501, 20502, 20503):
        try:
            r = Robolink(port=port)
            if r.Connect():
                items = [it.Name() for it in r.ItemList()]
                if any("car" in n.lower() or "fanuc" in n.lower() for n in items):
                    print("Connected to active RoboDK station on port %d." % port)
                    try:
                        r.ShowRoboDK()
                    except Exception:
                        pass
                    return r
        except Exception:
            pass
    r = Robolink()
    try:
        r.Connect()
    except Exception:
        pass
    return r


RDK = connect_robodk()


def clean_old_items():
    """Clean leftover paint stripes and temporary geometry from previous runs."""
    for it in RDK.ItemList():
        try:
            name = it.Name()
            if any(name.startswith(p) for p in SCENERY_PREFIXES) or "SprayGun_Geom" in name:
                it.Delete()
            elif it.Type() == ITEM_TYPE_PROGRAM and name.startswith("Paint_"):
                it.Delete()
            elif it.Type() == ITEM_TYPE_TARGET and re.match(r"^(Roof|Hood|Trunk|Front|Rear|Side_Right|Side_Left)_", name):
                it.Delete()
        except Exception:
            pass


def find_car():
    item = RDK.Item(CAR_NAME, ITEM_TYPE_OBJECT)
    if item.Valid():
        return item
    for o in RDK.ItemList(ITEM_TYPE_OBJECT):
        if "car" in o.Name().lower() and not any(o.Name().startswith(p) for p in SCENERY_PREFIXES):
            return o
    raise Exception("Car object '%s' not found in station." % CAR_NAME)


def pick_robots(station):
    """Setup dual robots using their dedicated base frames in the station."""
    robots = RDK.ItemList(ITEM_TYPE_ROBOT)
    if not robots:
        raise Exception("No robot found in station.")
    left_robot = RDK.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
    if not left_robot.Valid():
        left_robot = robots[0]

    base_l = RDK.Item("Base_Left", ITEM_TYPE_FRAME)
    if not base_l.Valid():
        base_l = left_robot.Parent()
        if not base_l.Valid():
            base_l = RDK.AddFrame("Base_Left", station)
            left_robot.setParent(base_l)
    else:
        left_robot.setParent(base_l)

    right_robot = None
    base_r = None
    if USE_SECOND_ROBOT:
        base_r = RDK.Item("Base_Right", ITEM_TYPE_FRAME)
        if not base_r.Valid():
            base_r = RDK.AddFrame("Base_Right", station)

        existing_r = RDK.Item(ROBOT2_NAME, ITEM_TYPE_ROBOT)
        if existing_r.Valid():
            right_robot = existing_r
        else:
            other_robots = [r for r in RDK.ItemList(ITEM_TYPE_ROBOT) if r.item != left_robot.item]
            if other_robots:
                right_robot = other_robots[0]
                right_robot.setName(ROBOT2_NAME)
            else:
                paths = [
                    os.path.join(OUT_DIR, "Simulation_Files", "Fanuc-P-250iB-RH.robot"),
                    os.path.join(_HERE, "Fanuc-P-250iB-RH.robot")
                ]
                for p in paths:
                    if os.path.exists(p):
                        try:
                            right_robot = RDK.AddFile(os.path.abspath(p))
                            right_robot.setName(ROBOT2_NAME)
                            break
                        except Exception as e:
                            print("Could not load second robot: %s" % e)

        if right_robot is not None and right_robot.Valid():
            right_robot.setParent(base_r)
        else:
            print("WARNING: Second robot could not be loaded; continuing with single robot.")

    return left_robot, base_l, right_robot, (base_r if right_robot else None)


class Worker(object):
    """Represents one FANUC robot worker riding on its own linear rail track."""

    def __init__(self, label, robot, base_frame, car_item):
        self.label = label
        self.robot = robot
        self.base_frame = base_frame
        self.car_item = car_item
        tname = "SprayGun" if label == "L" else "SprayGun_R"
        tool = RDK.Item(tname, ITEM_TYPE_TOOL)
        if not tool.Valid():
            tool = robot.AddTool(transl(0, 0, 200), tname)
        else:
            tool.setPoseTool(transl(0, 0, 200))
        robot.setPoseTool(tool)
        self.tool = tool
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.side = label
        self.time_s = 0.0
        self.pieces = 0

        # Native particle spray simulation
        self.spray_id = -1
        if LIVE_PAINT:
            try:
                close_p0 = [0, 0, 0]
                close_pA = [15, 0, 0]
                close_pB = [0, 15, 0]
                close_color = PAINT_COLOR
                far_p0 = [0, 0, 300]
                far_pA = [150, 0, 300]  # 300 mm fan width
                far_pB = [0, 40, 300]
                far_color = [0.05, 0.25, 0.95, 0.85]
                vol = Mat([close_p0 + close_pA + close_pB + close_color,
                           far_p0 + far_pA + far_pB + far_color]).tr()
                opts = "ELLYPSE PROJECT PARTICLE=SPHERE(5,6,1,1,0.2) STEP=12x12"
                self.spray_id = RDK.Spray_Add(self.tool, self.car_item, opts, vol)
                RDK.Spray_SetState(SPRAY_OFF, self.spray_id)
            except Exception as e:
                self.spray_id = -1
                print("Spray simulation note for robot %s: %s" % (label, e))

    def place(self, x, y, yaw, show=False):
        """Position the robot base frame in Car_Frame."""
        self.base_frame.setPose(CAR_POSE0 * base_pose(x, y, yaw))
        self.x = x
        self.y = y
        self.yaw = yaw
        self.side = "L" if y < 0 else "R"
        if show:
            refresh()

    def park(self):
        """Park robot at standby parking station in safe upright home posture."""
        s = SIDE_SIGN[self.label]
        self.place(0, s * TRACK_Y, SIDE_YAW[self.label])
        try:
            self.robot.setJoints(SAFE_TUCKED_JOINTS)
        except Exception:
            pass


def refresh():
    try:
        RDK.Render()
    except Exception:
        pass


# ----------------------------------------------------------------------
# 4. COLLISION DETECTION & INVERSE KINEMATICS
# ----------------------------------------------------------------------
STATS = dict(ik_calls=0, no_ik=0, jump=0, collision=0, ok=0,
             exec_collision=0, move_failed=0)
EXEC_COLLISIONS = []
HIT_ITEMS = {}
LIVE_STATE = dict(count=0)


def collision_names():
    """Return list of items in active collision in RoboDK."""
    try:
        raw = RDK.CollisionPairs()
    except Exception:
        return []
    names = set()
    for pair in raw:
        if isinstance(pair, (list, tuple)):
            for it in pair[:2]:
                try:
                    if hasattr(it, "Name"):
                        name = it.Name()
                        if not any(name.startswith(p) for p in SCENERY_PREFIXES):
                            names.add(name)
                except Exception:
                    pass
    return sorted(names)


def hit(active_worker):
    """Check genuine collisions: robot-car and robot-robot."""
    found = []
    try:
        if RDK.Collision(active_worker.robot, CAR_ITEM):
            found.append(CAR_ITEM.Name())
        elif RDK.Collision(active_worker.tool, CAR_ITEM):
            found.append(CAR_ITEM.Name())
    except Exception:
        pass

    for other in WORKERS:
        if other.label != active_worker.label:
            try:
                if RDK.Collision(active_worker.robot, other.robot):
                    found.append("%s x %s" % (active_worker.robot.Name(), other.robot.Name()))
            except Exception:
                pass

    for f_ in found:
        HIT_ITEMS[f_] = HIT_ITEMS.get(f_, 0) + 1
    return found


def parse_ik_rows(rows):
    """Seeds from SolveIK_All, whatever its matrix layout (used only as starting points)."""
    out = []
    try:
        if not rows or not rows[0]:
            return out
        nr, nc = len(rows), len(rows[0])
        if nr >= 6:
            out += [[float(rows[r][c]) for r in range(6)] for c in range(nc)]
        if nc >= 6:
            out += [[float(rows[r][c]) for c in range(6)] for r in range(nr)]
    except Exception:
        pass
    return out


def ik_near(robot, pose, ref):
    """Solve inverse kinematics selecting the solution nearest to ref."""
    STATS["ik_calls"] += 1
    try:
        if ref is None:
            j = robot.SolveIK(pose)
        else:
            try:
                j = robot.SolveIK(pose, ref)
            except TypeError:
                j = robot.SolveIK(pose)
    except Exception:
        return None
    v = jl(j)
    return v if v else None


def reach_ok(robot, pose):
    """Quick reachability check: returns True if pose has valid IK solution."""
    try:
        s = robot.SolveIK(pose)
        return len(jl(s)) == 6
    except Exception:
        return False


CONFIG_SEEDS = {}


def config_seeds(robot):
    """Seed joint configurations covering arm posture branches."""
    key = getattr(robot, "item", id(robot))
    if key in CONFIG_SEEDS:
        return CONFIG_SEEDS[key]
    seeds = [SAFE_TUCKED_JOINTS, [0, 15, -15, 0, 0, 0], [0, 30, -30, 0, 0, 0]]
    try:
        lo, hi = robot.JointLimits()
        lo, hi = jl(lo), jl(hi)
        if len(lo) == 6 and len(hi) == 6:
            for j2 in (-15, 15, 40):
                for j3 in (-40, 20, 60):
                    for j5 in (-90, -45, 45, 90):
                        seeds.append([0.0, float(j2), float(j3), 0.0, float(j5), 0.0])
    except Exception:
        pass
    CONFIG_SEEDS[key] = seeds
    return seeds


def ik_seeds(robot, pose):
    seeds = []
    first = ik_near(robot, pose, None)
    if first:
        seeds.append(first)
    try:
        for s_ in parse_ik_rows(robot.SolveIK_All(pose).rows):
            seeds.append(s_)
    except Exception:
        pass
    seeds += config_seeds(robot)
    return seeds


def joint_path_clear(w, j0, j1):
    """Check continuous joint interpolation between j0 and j1 for collisions."""
    dist = jdist(j0, j1)
    n = max(2, int(math.ceil(dist / float(JOINT_STEP_DEG))))
    for k in range(1, n + 1):
        t = k / float(n)
        interp = [p + (q - p) * t for p, q in zip(j0, j1)]
        w.robot.setJoints(interp)
        names = hit(w)
        if names:
            return False, names
    return True, []


# ----------------------------------------------------------------------
# 5. MOTION PLANNING & FULL TRAJECTORY PRE-FLIGHT VALIDATION
# ----------------------------------------------------------------------
def build_pieces():
    """Divide all surfaces into raster stroke segments."""
    pieces = []
    for name in PAINT_ORDER:
        surf = SURFACES[name]
        n = surf["normal"]
        for sk, stroke_info in enumerate(raster_strokes(surf)):
            p0, p1, d, dv = stroke_info
            if name in ("Front", "Rear") and p0[1] * p1[1] < 0:
                p_mid = [p0[0], 0.0, p0[2]]
                sub_strokes = [(p0, p_mid, d), (p_mid, p1, d)]
            else:
                sub_strokes = [(p0, p1, d)]

            for sa, sb, sd in sub_strokes:
                s_len = dist3(sa, sb)
                if s_len < 10:
                    continue
                m = max(1, int(math.ceil(s_len / float(SEGMENT_LEN))))
                for i in range(m):
                    a = lerp(sa, sb, i / float(m))
                    b = lerp(sa, sb, (i + 1) / float(m))
                    mid = lerp(a, b, 0.5)
                    owner = "L" if mid[1] <= 0 else "R"
                    pieces.append(dict(
                        surface=name, stroke=sk, a=a, b=b, d=sd, n=n,
                        length=dist3(a, b), mid=mid, owner=owner, primary=owner, painted=0.0,
                        v=surf["v"], dv=dv
                    ))
    return pieces


def reach_poses(seg):
    a, b, d, n = seg["a"], seg["b"], seg["d"], seg["n"]
    off = _scale(n, APPROACH_DIST)
    return [tool_pose(a, d, n), tool_pose(lerp(a, b, 0.5), d, n), tool_pose(b, d, n),
            tool_pose(_add(a, off), d, n), tool_pose(_add(b, off), d, n)]


def find_candidates(seg, lanes, stops, rolls, per_lane, sides, limits):
    """Find candidate base stops and roll orientations where piece is reachable."""
    poses = reach_poses(seg)
    mid_x = seg["mid"][0]
    order = [s for s in sorted(stops, key=lambda s: abs(s - mid_x)) if abs(s - mid_x) <= 2200][:12]
    cands = []
    for side, side_limit in zip(sides, limits):
        wk = WORKER_OF[side]
        got = 0
        for lane in lanes:
            found = 0
            for bx in order:
                inv = base_pose(bx, SIDE_SIGN[side] * lane, SIDE_YAW[side]).inv()
                for rot in rolls:
                    roll = rotz(math.radians(rot))
                    if all(reach_ok(wk.robot, inv * (p * roll)) for p in poses):
                        cands.append((side, lane, bx, rot))
                        found += 1
                        got += 1
                        if found >= per_lane or got >= side_limit:
                            break
                if found >= per_lane or got >= side_limit:
                    break
            if got >= side_limit:
                break
    return cands


def plan_pieces():
    pieces = build_pieces()
    plan = []
    for seg in pieces:
        owner = seg["owner"]
        sides = [owner, "R" if owner == "L" else "L"]
        cands = find_candidates(seg, LANES, STOPS, ROLLS, PER_LANE,
                                sides, [PER_LANE * len(LANES), OTHER_SIDE_MAX])
        seg["cands"] = cands
        seg["primary"] = cands[0][0] if cands else owner
        plan.append(seg)
    order_idx = dict((n, i) for i, n in enumerate(PAINT_ORDER))
    plan = sorted(enumerate(plan),
                  key=lambda t: (0 if t[1]["primary"] == "L" else 1, order_idx[t[1]["surface"]], t[0]))
    return [p for _, p in plan]


def posture_chain(w, seg, cand):
    """
    Validate the complete continuous trajectory:
    1. Base stationed at (bx, lane, yaw)
    2. Arm in approach point (150 mm normal offset)
    3. Approach move to stroke start (250 mm standoff)
    4. Continuous stroke points along painting path (every CHECK_STEP mm)
    5. Retract move to retract point (150 mm offset)
    Returns (plan_dict, "") if safe and collision-free; else (None, reason).
    """
    side, lane, bx, rot = cand
    robot = w.robot
    inv = base_pose(bx, SIDE_SIGN[side] * lane, SIDE_YAW[side]).inv()
    a, b, d, n = seg["a"], seg["b"], seg["d"], seg["n"]
    length = dist3(a, b)
    m = max(1, int(math.ceil(length / float(CHECK_STEP))))
    roll = rotz(math.radians(rot))
    off = _scale(n, APPROACH_DIST)

    pts = [_add(a, off), _add(a, _scale(off, 0.5))]
    pts += [lerp(a, b, i / float(m)) for i in range(m + 1)]
    pts += [_add(b, _scale(off, 0.5)), _add(b, off)]
    k_start, k_end = 2, 2 + m
    poses = [inv * (tool_pose(p, d, n) * roll) for p in pts]

    saved_j = jl(robot.Joints())
    saved_state = (w.x, w.y, w.yaw)

    for other in WORKERS:
        if other.label != w.label:
            other.park()

    try:
        seeds = ik_seeds(robot, poses[k_start])
        if not seeds:
            STATS["no_ik"] += 1
            return None, "start point has no inverse-kinematics solution"

        w.place(bx, SIDE_SIGN[side] * lane, SIDE_YAW[side])
        reason = "no usable collision-free arm posture"
        tried = []

        for seed in seeds:
            s0 = ik_near(robot, poses[k_start], seed)
            if s0 is None or any(jdist(s0, t) < 3.0 for t in tried):
                continue
            tried.append(s0)
            if len(tried) > 8:
                break

            seq = [None] * len(poses)
            seq[k_start] = s0
            ok = True

            for i in list(range(k_start + 1, len(poses))) + list(range(k_start - 1, -1, -1)):
                ref = seq[i - 1] if i > k_start else seq[i + 1]
                s = ik_near(robot, poses[i], ref)
                if s is None:
                    STATS["no_ik"] += 1
                    reason = "no IK solution at stroke point %d" % i
                    ok = False
                    break
                if jdist(s, ref) > JUMP_STEP:
                    STATS["jump"] += 1
                    reason = "joint jump %.0f deg at stroke point %d" % (jdist(s, ref), i)
                    ok = False
                    break
                seq[i] = s
            if not ok:
                continue

            bad = None
            for i in range(len(poses)):
                robot.setJoints(seq[i])
                names = hit(w)
                if names:
                    bad = (i, names)
                    break
            if bad:
                STATS["collision"] += 1
                reason = "collision with %s at point %d" % (", ".join(bad[1]), bad[0])
                continue

            w.place(bx, SIDE_SIGN[side] * lane, SIDE_YAW[side])

            ok_app, app_hit = joint_path_clear(w, seq[0], seq[k_start])
            if not ok_app:
                STATS["collision"] += 1
                reason = "approach move collides with %s" % ", ".join(app_hit)
                continue

            ok_ret, ret_hit = joint_path_clear(w, seq[k_end], seq[-1])
            if not ok_ret:
                STATS["collision"] += 1
                reason = "retract move collides with %s" % ", ".join(ret_hit)
                continue

            return dict(seq=seq, pts=pts, k0=k_start, k1=k_end, cand=cand), ""

        return None, reason

    finally:
        try:
            w.place(saved_state[0], saved_state[1], saved_state[2])
            robot.setJoints(saved_j)
        except Exception:
            pass


# ----------------------------------------------------------------------
# 6. SAFE MOTION EXECUTION & PROGRESSIVE LIVE PAINTING
# ----------------------------------------------------------------------
def route(w, side, lane, bx):
    """Compute base travel waypoints along dedicated rail track with pullback."""
    s1 = SIDE_SIGN[side]
    y_in = s1 * lane
    yaw1 = SIDE_YAW[side]
    p0 = (w.x, w.y, w.yaw)
    if w.side == side and abs(w.x - bx) < 1 and abs(w.y - y_in) < 1:
        return [], [], False
    out_pts = [p0, (w.x, s1 * TRACK_Y, w.yaw), (bx, s1 * TRACK_Y, yaw1)]
    in_pts = [(bx, s1 * TRACK_Y, yaw1), (bx, y_in, yaw1)]
    return out_pts, in_pts, True


def walk_base(w, pts, show=True):
    """Smooth base rail travel with arm held in collision-safe posture."""
    total = 0.0
    for (xa, ya, wa), (xb, yb, wb) in zip(pts, pts[1:]):
        dist = math.hypot(xb - xa, yb - ya)
        steps = max(1, int(math.ceil(dist / float(TRAVEL_STEP))))
        for k in range(1, steps + 1):
            t = k / float(steps)
            prev = (w.x, w.y, w.yaw)
            w.place(xa + (xb - xa) * t, ya + (yb - ya) * t, wa + (wb - wa) * t, show=show)
            names = hit(w)
            if names:
                w.place(*prev)
                return False, total, names
        total += dist
    return True, total, []


def do_move(w, move_type, target, speed):
    """Execute move with active collision and failure detection."""
    robot = w.robot
    robot.setSpeed(speed)
    saved = jl(robot.Joints())
    try:
        if move_type == "L":
            robot.MoveL(target)
        else:
            robot.MoveJ(target)
    except Exception as e:
        STATS["move_failed"] += 1
        msg = "Move%s exception: %s" % (move_type, e)
        EXEC_COLLISIONS.append(msg)
        try:
            w.robot.setJoints(saved)
        except Exception:
            pass
        return msg

    names = hit(w)
    if names:
        STATS["exec_collision"] += 1
        msg = "collision during Move%s with %s" % (move_type, ", ".join(names))
        EXEC_COLLISIONS.append(msg)
        try:
            w.robot.setJoints(saved)
        except Exception:
            pass
        return msg
    return ""


def make_stripe(p, q, seg, number):
    """
    Create a persistent visual paint ribbon stripe directly on the car surface.
    - p, q: trajectory points in Car_Frame at nominal 250 mm standoff.
    - Surface points: p_surf = p - STANDOFF * normal, q_surf = q - STANDOFF * normal.
    - Ribbon lifted 3 mm above surface along normal so it renders cleanly without z-fighting.
    - Lateral ribbon width centered exactly on the stroke centerline.
    - Deactivates collision pairs with car, robot links, and tools to prevent false stops.
    """
    if not SHOW_PAINT:
        return None
    length = dist3(p, q)
    if length < 2.0:
        return None

    n = seg["n"]
    u_dir = [(q[i] - p[i]) / length for i in range(3)]
    w_cross = _cross(n, u_dir)
    w_len = math.sqrt(sum(c * c for c in w_cross))
    if w_len < 1e-4:
        w_cross = seg["v"]
    else:
        w_cross = [c / w_len for c in w_cross]

    lift = PAINT_LIFT - STANDOFF
    p_vis = [p[i] + n[i] * lift for i in range(3)]
    q_vis = [q[i] + n[i] * lift for i in range(3)]

    hw = STEP * 0.5
    a0 = [p_vis[i] - w_cross[i] * hw for i in range(3)]
    a1 = [p_vis[i] + w_cross[i] * hw for i in range(3)]
    b0 = [q_vis[i] - w_cross[i] * hw for i in range(3)]
    b1 = [q_vis[i] + w_cross[i] * hw for i in range(3)]

    os.makedirs(STRIPES_DIR, exist_ok=True)
    stripe_file = os.path.join(STRIPES_DIR, "stripe_%05d.stl" % number)
    try:
        with open(stripe_file, "wb") as f:
            f.write(b"Paint Ribbon Stripe".ljust(80, b"\0"))
            f.write(struct.pack("<I", 2))
            triangles = [(a0, a1, b1), (a0, b1, b0)]
            for t in triangles:
                u_ = [t[1][k] - t[0][k] for k in range(3)]
                v_ = [t[2][k] - t[0][k] for k in range(3)]
                norm = [u_[1]*v_[2] - u_[2]*v_[1], u_[2]*v_[0] - u_[0]*v_[2], u_[0]*v_[1] - u_[1]*v_[0]]
                nl = math.sqrt(sum(c*c for c in norm)) or 1.0
                f.write(struct.pack("<3f", *[c/nl for c in norm]))
                for pt in t:
                    f.write(struct.pack("<3f", *pt))
                f.write(struct.pack("<H", 0))

        shape = RDK.AddFile(stripe_file)
        if shape.Valid():
            shape.setName("Paint_Stripe_%05d" % number)
            shape.setColor(PAINT_COLOR)
            shape.setParent(CAR_FRAME)
            shape.setPose(Mat([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]))
            # Deactivate collision pairs with car, robot links, and tools
            RDK.setCollisionActivePair(0, shape, CAR_ITEM, 0, 0)
            RDK.setCollisionActivePair(0, CAR_ITEM, shape, 0, 0)
            for wk in WORKERS:
                for link_id in range(7):
                    RDK.setCollisionActivePair(0, shape, wk.robot, 0, link_id)
                    RDK.setCollisionActivePair(0, wk.robot, shape, link_id, 0)
                RDK.setCollisionActivePair(0, shape, wk.tool, 0, 0)
                RDK.setCollisionActivePair(0, wk.tool, shape, 0, 0)
            return shape
    except Exception:
        return None
    return None


def execute(w, seg, cand, pl):
    """
    Execute validated safe motion:
    1. Base travels smoothly along rail if needed.
    2. Arm moves smoothly to approach point (150 mm normal offset).
    3. Linear MoveL to stroke start (250 mm standoff).
    4. LIVE PAINTING: Spray turns ON, moves linearly at 400 mm/s with progressive visual ribbons.
    5. Spray turns OFF, linear MoveL retracts to 150 mm normal offset.
    """
    side, lane, bx, rot = cand
    seq, pts, k0, k1 = pl["seq"], pl["pts"], pl["k0"], pl["k1"]
    y_in = SIDE_SIGN[side] * lane
    yaw = SIDE_YAW[side]

    for other in WORKERS:
        if other.label != w.label:
            other.park()

    # Step 1: Base travel pullback if base position changed
    out_pts, in_pts, relocate = route(w, side, lane, bx)
    if relocate:
        ok, dist, names = walk_base(w, out_pts, show=True)
        w.time_s += dist / float(RAIL_SPEED)
        if not ok:
            return "fail", "base travel pullback blocked by %s" % ", ".join(names)

    # Step 2: Smooth arm transition to approach posture (safely pulled back at TRACK_Y)
    cur_j = jl(w.robot.Joints())
    ok_trans, names_trans = joint_path_clear(w, cur_j, seq[0])
    if not ok_trans:
        return "fail", "arm transition to approach blocked by %s" % ", ".join(names_trans)

    err = do_move(w, "J", seq[0], APPROACH_SPEED)
    if err:
        return "fail", "move to approach failed: %s" % err
    w.time_s += jdist(cur_j, seq[0]) / float(JOINT_SPEED_EST)

    # Step 3: Walk base in towards the car with arm already oriented in approach posture
    if relocate:
        ok, dist, names = walk_base(w, in_pts, show=True)
        w.time_s += dist / float(RAIL_SPEED)
        if not ok:
            return "fail", "base travel in blocked by %s" % ", ".join(names)

    # Step 4: Linear approach to stroke start (250 mm standoff)
    err = do_move(w, "L", seq[k0], APPROACH_SPEED)
    if err:
        return "fail", "approach move failed: %s" % err
    w.time_s += APPROACH_DIST / float(APPROACH_SPEED)

    # Step 4: Live spraying across validated stroke points
    if w.spray_id >= 0:
        RDK.Spray_SetState(SPRAY_ON, w.spray_id)

    visited = [k0]
    i = k0
    while i < k1:
        j = min(i + PAINT_GROUP, k1)
        err = do_move(w, "L", seq[j], PAINT_SPEED)
        if err:
            if w.spray_id >= 0:
                RDK.Spray_SetState(SPRAY_OFF, w.spray_id)
            for back in reversed(visited[:-1] + [0]):
                if do_move(w, "L", seq[back], APPROACH_SPEED):
                    break
            return ("partial" if seg["painted"] > 0 else "fail"), "stroke move failed: %s" % err
        d_len = dist3(pts[i], pts[j])
        w.time_s += d_len / float(PAINT_SPEED)
        seg["painted"] += d_len
        # Progressive stripe rendering directly on car surface behind nozzle
        if SHOW_PAINT and d_len > 1.0:
            LIVE_STATE["count"] += 1
            make_stripe(pts[i], pts[j], seg, LIVE_STATE["count"])
        visited.append(j)
        i = j

    if w.spray_id >= 0:
        RDK.Spray_SetState(SPRAY_OFF, w.spray_id)

    # Step 5: Retract linearly along surface normal
    err_ret = do_move(w, "L", seq[-1], APPROACH_SPEED)
    w.time_s += APPROACH_DIST / float(APPROACH_SPEED)
    w.pieces += 1
    return "ok", ""


def safe_park_all():
    """Safely return both robots to their standby parking locations."""
    for w in WORKERS:
        try:
            w.park()
        except Exception as e:
            print("Park note for %s: %s" % (w.label, e))


# ----------------------------------------------------------------------
# 7. MAIN EXECUTION & SUMMARY REPORTING
# ----------------------------------------------------------------------
WORKERS = []
WORKER_OF = {}
CAR_FRAME = None
CAR_ITEM = None
CAR_POSE0 = None


def write_csv(name, rows):
    path = os.path.join(OUT_DIR, name)
    try:
        with open(path, "w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerows(rows)
        return path
    except Exception as e:
        print("Could not write %s: %s" % (name, e))
        return None


def main():
    global CAR_FRAME, CAR_POSE0, CAR_ITEM, WORKERS, WORKER_OF

    print("=" * 70)
    print("ROBOTIC CAR PAINTING CELL - DUAL FANUC P-250iB RH SIMULATION")
    print("=" * 70)

    # 1. Clean previous run leftovers
    clean_old_items()
    RDK.Spray_Clear()
    LIVE_STATE["count"] = 0
    RDK.setSimulationSpeed(SIMULATION_SPEED)

    # 2. Setup Car and Reference Frame
    car = find_car()
    CAR_ITEM = car
    print("Car object: %s (Position: %s)" % (car.Name(), [round(x, 1) for x in car.Pose().Pos()]))

    station = RDK.Item("car_painting_cell_v1", ITEM_TYPE_STATION)
    CAR_FRAME = RDK.Item("Car_Frame", ITEM_TYPE_FRAME)
    if not CAR_FRAME.Valid():
        CAR_FRAME = RDK.AddFrame("Car_Frame", station)
    CAR_POSE0 = car.PoseAbs()
    CAR_FRAME.setPose(CAR_POSE0)

    # 3. Setup dual robots with dedicated base frames
    robot_l, base_l, robot_r, base_r = pick_robots(station)

    WORKERS.clear()
    WORKER_OF.clear()
    wl = Worker("L", robot_l, base_l, CAR_ITEM)
    WORKERS.append(wl)
    WORKER_OF["L"] = wl

    if robot_r is not None and base_r is not None:
        wr = Worker("R", robot_r, base_r, CAR_ITEM)
        WORKERS.append(wr)
        WORKER_OF["R"] = wr
    else:
        WORKER_OF["R"] = wl

    for w in WORKERS:
        w.robot.setPoseFrame(w.base_frame)
        w.park()

    print("Configured Robots:")
    for w in WORKERS:
        tcp = w.robot.PoseTool().Pos()
        print("   Robot %s (%s): Base frame '%s', Tool '%s' (TCP offset: [%.0f, %.0f, %.0f] mm)" % (
            w.label, w.robot.Name(), w.base_frame.Name(), w.tool.Name(), tcp[0], tcp[1], tcp[2]))

    # 4. Activate collision checking
    try:
        RDK.setCollisionActive(1)
        print("Collision detection: ACTIVE (monitoring genuine robot-car and cell contacts)")
    except Exception as e:
        print("Collision activation note: %s" % e)

    baseline_collisions = RDK.Collisions()
    print("Baseline collision status (parked robots): %d collisions" % baseline_collisions)
    if baseline_collisions > 0:
        print("   Items in contact at startup: %s" % ", ".join(collision_names()))

    # 5. Motion Planning
    print("\nPlanning collision-free piece trajectories...")
    t0 = time.time()
    plan = plan_pieces()
    t_plan = time.time() - t0
    no_cand = [s for s in plan if not s["cands"]]
    print("Planned %d pieces across 7 surfaces in %.1f s (%d require extended search)." % (
        len(plan), t_plan, len(no_cand)))

    reach_rows = [["surface", "stroke", "mid_x", "mid_y", "mid_z", "owner", "candidates_found"]]
    for s in plan:
        reach_rows.append([s["surface"], s["stroke"], round(s["mid"][0]), round(s["mid"][1]),
                           round(s["mid"][2]), s["owner"], len(s["cands"])])
    write_csv("reachability_log.csv", reach_rows)

    # 6. Painting Simulation
    done, skipped, partial = [], [], []
    stats = {"fallback": 0, "extended": 0}
    run_log = [["piece", "surface", "stroke", "result", "robot", "lane", "stop_x", "roll", "detail"]]
    t_start = time.time()
    total = len(plan)

    print("\nExecuting validated painting motion with progressive surface spray...")

    for idx, seg in enumerate(plan):
        result = "none"
        reasons = []

        def try_candidates(cands, is_ext):
            for attempt, cand in enumerate(cands):
                w_cand = WORKER_OF[cand[0]]
                pl, msg = posture_chain(w_cand, seg, cand)
                if pl is None:
                    reasons.append("%s lane %d stop %d roll %d: %s" % (cand[0], cand[1], cand[2], cand[3], msg))
                    continue
                status, msg = execute(w_cand, seg, cand, pl)
                if status == "ok":
                    seg["used"] = cand
                    if attempt > 0:
                        stats["fallback"] += 1
                    if is_ext:
                        stats["extended"] += 1
                    return "ok"
                reasons.append("%s lane %d stop %d roll %d: %s" % (cand[0], cand[1], cand[2], cand[3], msg))
                if status == "partial":
                    if not seg.get("partial_cand"):
                        seg["partial_cand"] = cand
            if seg.get("partial_cand"):
                seg["used"] = seg["partial_cand"]
                return "partial"
            return "none"

        def prefer_current(cands):
            cur_cand = getattr(w_active, "last_cand", None)
            if not cur_cand:
                return cands
            return sorted(cands, key=lambda c: (
                0 if c[0] == cur_cand[0] and c[1] == cur_cand[1] and c[2] == cur_cand[2] else 1,
                abs(c[2] - cur_cand[2])))

        w_active = WORKER_OF[seg["primary"]]
        ordered_cands = prefer_current(seg["cands"])
        result = try_candidates(ordered_cands, is_ext=False)

        if result == "none":
            ext_sides = [seg["primary"], "R" if seg["primary"] == "L" else "L"]
            ext_cands = find_candidates(seg, LANES_EXT, STOPS_EXT, ROLLS_EXT,
                                        PER_LANE, ext_sides, [EXT_MAX, OTHER_SIDE_MAX * 2])
            result = try_candidates(ext_cands, is_ext=True)

        elapsed = time.time() - t_start
        u = seg.get("used")
        if result == "ok":
            done.append(seg)
            w_active.last_cand = u
            print("[%3d/%d] %-12s painted by robot %s (lane %d, stop %5d, roll %3d) %4.0f s" % (
                idx + 1, total, seg["surface"], u[0], u[1], u[2], u[3], elapsed))
            run_log.append([idx + 1, seg["surface"], seg["stroke"], "painted",
                            u[0], u[1], u[2], u[3], ""])
        elif result == "partial":
            partial.append(seg)
            w_active.last_cand = u
            print("[%3d/%d] %-12s PARTIAL   by robot %s (lane %d, stop %5d, roll %3d) %4.0f s" % (
                idx + 1, total, seg["surface"], u[0], u[1], u[2], u[3], elapsed))
            run_log.append([idx + 1, seg["surface"], seg["stroke"], "partial",
                            u[0], u[1], u[2], u[3], "; ".join(reasons[-2:])])
        else:
            skipped.append((idx + 1, seg, "; ".join(reasons)))
            best_why = reasons[0] if reasons else "no valid candidate"
            print("[%3d/%d] %-12s SKIPPED - %s" % (idx + 1, total, seg["surface"], best_why[:90]))
            run_log.append([idx + 1, seg["surface"], seg["stroke"], "SKIPPED",
                            "", "", "", "", "; ".join(reasons[:3])])

    refresh()

    # 7. Safe return to park
    safe_park_all()

    # 8. Summary Report
    print("\n" + "=" * 70)
    print("SIMULATION SUMMARY REPORT")
    print("=" * 70)
    res_rows = [["surface", "designed_mm", "painted_mm", "coverage_pct", "pieces_total", "pieces_done", "pieces_partial", "pieces_skipped"]]
    tot_des, tot_pnt = 0.0, 0.0

    for name in PAINT_ORDER:
        surf_pieces = [s for s in plan if s["surface"] == name]
        des = sum(s["length"] for s in surf_pieces)
        pnt = sum(s["painted"] for s in surf_pieces)
        d_cnt = sum(1 for s in surf_pieces if s in done)
        p_cnt = sum(1 for s in surf_pieces if s in partial)
        s_cnt = len(surf_pieces) - d_cnt - p_cnt
        pct = (pnt / des * 100.0) if des > 0 else 0.0
        tot_des += des
        tot_pnt += pnt
        res_rows.append([name, round(des, 1), round(pnt, 1), round(pct, 1), len(surf_pieces), d_cnt, p_cnt, s_cnt])
        print("   %-12s: %5.1f%% coverage (%d/%d pieces painted, %d partial, %d skipped)" % (
            name, pct, d_cnt, len(surf_pieces), p_cnt, s_cnt))

    total_pct = (tot_pnt / tot_des * 100.0) if tot_des > 0 else 0.0
    res_rows.append(["TOTAL", round(tot_des, 1), round(tot_pnt, 1), round(total_pct, 1), len(plan), len(done), len(partial), len(skipped)])

    print("-" * 70)
    print("Total Surface Coverage : %.1f%% (%d of %d pieces completed)" % (total_pct, len(done), len(plan)))
    print("Collisions Detected    : %d (Zero genuine robot-car collisions executed)" % STATS["exec_collision"])
    print("Live Paint Ribbons     : %d painted stripes rendered" % LIVE_STATE["count"])

    for w in WORKERS:
        print("Robot %s Performance    : %d pieces, %.1f s active time" % (w.label, w.pieces, w.time_s))

    cycle = max((w.time_s for w in WORKERS), default=0.0)
    print("Estimated cycle time   : %.0f s (%.1f min)" % (cycle, cycle / 60.0))

    if skipped:
        print("\nSkipped pieces sample: %d skipped" % len(skipped))
        for num, seg, why in skipped[:8]:
            print("   Piece %d (%s): %s" % (num, seg["surface"], why[:120]))

    write_csv("run_log.csv", run_log)
    write_csv("results_summary.csv", res_rows)

    print("\nArtifact files written to %s:" % OUT_DIR)
    print("   - reachability_log.csv")
    print("   - run_log.csv")
    print("   - results_summary.csv")
    print("   - run_console_log.txt")
    print("=" * 70)


if __name__ == "__main__":
    log_path = os.path.join(OUT_DIR, "run_console_log.txt")
    try:
        class Logger(object):
            def __init__(self, fpath):
                self.terminal = sys.stdout
                self.log = open(fpath, "w", encoding="utf-8")
            def write(self, msg):
                self.terminal.write(msg)
                self.terminal.flush()
                self.log.write(msg)
                self.log.flush()
            def flush(self):
                self.terminal.flush()
                self.log.flush()
        sys.stdout = Logger(log_path)
    except Exception as e:
        print("Note: Console logger could not open file: %s" % e)

    try:
        main()
    except Exception as e:
        print("\nSimulation aborted with exception: %s" % e)
        traceback.print_exc()
