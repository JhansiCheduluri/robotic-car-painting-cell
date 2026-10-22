"""
generate_paint_paths.py   (version 3 - dual robot, own collision pre-flight)
Robotic Car Painting Cell - paint simulation for RoboDK

HOW TO RUN
----------
1. Open RoboDK with your station (car "simple_car" + robot "Fanuc P-250iB RH").
2. Delete leftovers from older tries (the Brooks rail, old Paint_Stripe objects).
3. In a terminal:   cd C:\\projects\\robotic-car-painting-cell\\scripts
                    python generate_paint_paths.py

WHAT IT DOES
------------
* Uses TWO identical FANUC P-250iB RH paint robots, one on each side of the car,
  each riding on its own straight rail along the car. The car stays still.
  (The second robot is loaded from Simulation_Files\\Fanuc-P-250iB-RH.robot.
   If that fails, copy/paste the robot in the RoboDK tree and rename the copy
   "Robot_Right", or set USE_SECOND_ROBOT = False.)
* The left robot paints everything on the left half of the car, the right robot
  the right half (roof, hood, trunk, front, rear, sides). Zig-zag strokes,
  250 mm standoff, 40 % overlap, 400 mm/s.
* BEFORE every move the script checks the whole planned motion (approach,
  every 100 mm of the stroke, retract, the swing into position and the rail
  travel) with RoboDK's collision engine. Only contacts with the car or with
  the other robot count. A piece that is not safe is moved to another base
  position, lane or gun roll. If nothing is safe it is SKIPPED and reported.
* RoboDK's own collision stop stays ON while moves run. If it stops a move, that
  is counted as a collision event and reported (nothing is hidden).
* LIVE PAINTING: the stroke is driven through the same checked points, and a
  blue stripe (300 mm long) is added right after each spray move succeeds. Only
  sprayed length counts as painted; travel and approach never paint.
* A booth, conveyor and rails are drawn at the end (they clear the robots).
* At the end it probes the first failing postures (base moved away / gun pulled
  back) to show WHAT collides, and leaves the robot in that rejected posture
  as a display so RoboDK shows the contact in red.
* Writes: reachability_log.csv, run_log.csv, results_summary.csv and
  run_console_log.txt (the full console output) in the project folder.

All lengths are mm, angles degrees. Car frame: origin = car centre on the floor,
X forward, Y left, Z up.
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
    from robodk.robolink import *    # RoboDK API (new package layout)
    from robodk.robomath import *
except ImportError:
    from robolink import *           # older layout
    from robomath import *

# ----------------------------------------------------------------------
# 1. PARAMETERS
# ----------------------------------------------------------------------
ROBOT_NAME = "Fanuc P-250iB RH"      # the robot already in your station (left robot)
ROBOT2_NAME = "Robot_Right"          # name given to the second (right) robot
CAR_NAME = "simple_car"
USE_SECOND_ROBOT = True              # True = two robots (recommended)
BUILD_SCENERY = True                 # draw floor/walls/rails/conveyor after the run
SHOW_PAINT = True                    # draw blue paint stripes at all
LEAVE_COLLISION_POSTURE = True       # leave the robot in the first failing posture so you can see it
LIVE_PAINT = True                    # blue stripes appear WHILE the gun sprays (not after the run)
PAINT_GROUP = 3                      # checked points per live stripe (3 x 50 mm = 150 mm)
SHOW_TARGET_PATCHES = False          # yellow check patches on the surfaces (they hide the paint)
MAX_CONFIGS = 10                     # different arm configurations tried per base position

CAR_LENGTH, CAR_WIDTH, CAR_HEIGHT = 4800, 1900, 1600

STANDOFF = 250          # gun distance from the surface
FAN_WIDTH = 300         # spray width at the standoff
OVERLAP = 0.40          # 40 % overlap
STEP = FAN_WIDTH * (1 - OVERLAP)   # 180 mm between passes
PAINT_SPEED = 400       # mm/s while spraying
APPROACH_SPEED = 800    # mm/s approach / retract
APPROACH_DIST = 150     # mm

# Flat rectangles in the car frame.
#   origin: corner, u: stroke direction, v: step direction, normal: outward
SURFACES = {
    "Roof":  dict(origin=(-1000, -650, 1600), u=(1, 0, 0), v=(0, 1, 0),
                  len_u=1900, len_v=1300, normal=(0, 0, 1)),
    "Hood":  dict(origin=(900, -800, 1000), u=(1, 0, 0), v=(0, 1, 0),
                  len_u=1500, len_v=1600, normal=(0, 0, 1)),
    "Trunk": dict(origin=(-2400, -800, 1050), u=(1, 0, 0), v=(0, 1, 0),
                  len_u=1400, len_v=1600, normal=(0, 0, 1)),
    "Front": dict(origin=(2400, -800, 300), u=(0, 1, 0), v=(0, 0, 1),
                  len_u=1600, len_v=500, normal=(1, 0, 0)),
    "Rear":  dict(origin=(-2400, -800, 300), u=(0, 1, 0), v=(0, 0, 1),
                  len_u=1600, len_v=500, normal=(-1, 0, 0)),
    "Side_Right": dict(origin=(-2300, 950, 500), u=(1, 0, 0), v=(0, 0, 1),
                       len_u=4600, len_v=500, normal=(0, 1, 0)),
    "Side_Left": dict(origin=(-2300, -950, 500), u=(1, 0, 0), v=(0, 0, 1),
                      len_u=4600, len_v=500, normal=(0, -1, 0)),
}
PAINT_ORDER = ["Roof", "Hood", "Trunk", "Front", "Rear", "Side_Right", "Side_Left"]

# --- cell / planner settings ---
SIDE_SIGN = {"L": -1, "R": 1}        # L robot sits at negative Y, R at positive Y
SIDE_YAW = {"L": 90, "R": -90}       # each robot faces the car (its X axis points at the car)
LANES = (1800, 1600, 1400, 2000)     # base distance from the car centre line (first search)
LANES_EXT = (1500, 1700, 1900, 2100, 2200, 2400)   # extra lanes (second search)
STOPS = list(range(-3000, 3001, 300))              # rail stops along the car
STOPS_EXT = list(range(-3000, 3001, 150))
ROLLS = (0, 90, 180, 270)                          # gun rotation about its own axis
ROLLS_EXT = tuple(range(0, 360, 30))
SEGMENT_LEN = 600        # a stroke is cut into pieces of this length
CHECK_STEP = 50          # mm between collision-checked points along a stroke
JUMP_STEP = 45           # deg, max joint change between neighbouring checked points
JOINT_STEP_DEG = 3       # deg, resolution of the joint-move collision check
PER_LANE = 3             # candidate stops kept per lane (first search)
OTHER_SIDE_MAX = 3       # candidates borrowed from the other robot's side
EXT_MAX = 24             # candidates tried in the second search
TRACK_Y = 3300           # outer lane used when the base moves (mm from centre line)
TRACK_X = 3800           # single-robot mode only: base x when going round the car end
TRAVEL_STEP = 250        # mm per animation step of the base
JOINT_SPEED_EST = 120    # deg/s, only for the cycle-time estimate
RAIL_SPEED = 500         # mm/s, only for the cycle-time estimate
PAINT_COLOR = [0.05, 0.25, 0.95, 1.0]   # ONE colour, used by BOTH robots
PAINT_LIFT = 3           # mm the stripe floats above the surface

try:
    _HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:
    _HERE = os.path.join(os.path.expanduser("~"), "Documents")
OUT_DIR = os.path.abspath(os.path.join(_HERE, ".."))
SCENERY_PREFIXES = ("Paint_Stripe", "Booth_", "Rail_")


# ----------------------------------------------------------------------
# 2. GEOMETRY HELPERS (pure maths)
# ----------------------------------------------------------------------
def _add(a, b):
    return [a[i] + b[i] for i in range(3)]


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
    """Gun TCP pose: Z points INTO the surface, X follows the stroke."""
    z = _scale(normal, -1)
    x = stroke_dir
    y = _cross(z, x)
    return Mat([[x[0], y[0], z[0], position[0]],
                [x[1], y[1], z[1], position[1]],
                [x[2], y[2], z[2], position[2]],
                [0, 0, 0, 1]])


def base_pose(x, y, yaw):
    """Robot base pose in the car frame."""
    return transl(x, y, 0) * rotz(math.radians(yaw))


def raster_strokes(surf):
    """Zig-zag strokes: list of (start, end, direction), offset by STANDOFF."""
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
        strokes.append((p_start, p_end, direction))
    return strokes


def stroke_bands(surf):
    """(lo, hi) offsets along v, per stroke, of the area each stroke paints. Bands never overlap
    and never leave the surface, so the drawn stripes never touch each other."""
    n_strokes = max(1, int(math.ceil(surf["len_v"] / STEP)) + 1)
    out, prev_hi = [], 0.0
    for k in range(n_strokes):
        c = min(k * STEP, surf["len_v"])
        lo = max(c - STEP / 2.0, 0.0, prev_hi)
        hi = max(lo, min(c + STEP / 2.0, surf["len_v"]))
        out.append((lo - c, hi - c))
        prev_hi = max(prev_hi, hi)
    return out


def jl(j):
    """Joint Mat/list -> list of 6 floats (empty list if invalid)."""
    try:
        v = j.list() if hasattr(j, "list") else list(j)
    except Exception:
        return []
    v = [float(x) for x in v]
    return v[:6] if len(v) >= 6 else []


def jdist(s1, s2):
    return max(abs(p - q) for p, q in zip(s1, s2))


def box_tris(cx, cy, cz, sx, sy, sz):
    x0, x1 = cx - sx / 2.0, cx + sx / 2.0
    y0, y1 = cy - sy / 2.0, cy + sy / 2.0
    z0, z1 = cz - sz / 2.0, cz + sz / 2.0
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    pts = []
    for f in faces:
        a, b, c, d = [list(v[i]) for i in f]
        pts += [a, b, c, a, c, d]
    return pts



# ----------------------------------------------------------------------
# 2b. FIT THE SURFACE TABLE TO THE REAL CAR MESH (reads the STL file)
# ----------------------------------------------------------------------
def read_stl(path):
    with open(path, "rb") as f:
        data = f.read()
    tris = []
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if 84 + 50 * n == len(data):                        # binary STL
            for i in range(n):
                v = struct.unpack("<9f", data[96 + 50 * i:132 + 50 * i])
                tris.append(((v[0], v[1], v[2]), (v[3], v[4], v[5]), (v[6], v[7], v[8])))
            return tris
    text = data.decode("ascii", "ignore")                   # ASCII STL
    pts = [(float(a), float(b), float(c))
           for a, b, c in re.findall(r"vertex\s+(\S+)\s+(\S+)\s+(\S+)", text)]
    return [tuple(pts[i:i + 3]) for i in range(0, len(pts) - 2, 3)]


def find_stl(car):
    folder = os.path.join(OUT_DIR, "Simulation_Files")
    names = [car.Name() + ".stl", CAR_NAME + ".stl"]
    for nm in names:
        p = os.path.join(folder, nm)
        if os.path.exists(p):
            return p
    try:
        for fn in os.listdir(folder):
            if fn.lower().endswith(".stl") and "car" in fn.lower():
                return os.path.join(folder, fn)
    except Exception:
        pass
    return None


def fit_car_to_mesh(car):
    """Measure the car mesh. Returns None, or a dict with the car-frame offset (mm),
    a rotation about Z (deg), the surface table and the paint order."""
    path = find_stl(car)
    if not path:
        print("Car STL not found in Simulation_Files - using the built-in surface table.")
        return None
    try:
        tris = read_stl(path)
    except Exception as e:
        print("Could not read %s (%s) - using the built-in surface table." % (path, e))
        return None
    if not tris:
        return None
    pts = [v for t in tris for v in t]
    mn = [min(p[i] for p in pts) for i in range(3)]
    mx = [max(p[i] for p in pts) for i in range(3)]
    size = [mx[i] - mn[i] for i in range(3)]
    rot = 90.0 if size[1] > size[0] else 0.0
    cx, cy, cz = (mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0, mn[2]
    c = math.cos(math.radians(-rot))
    sn = math.sin(math.radians(-rot))

    def tf(p):                       # mesh coords -> car frame (centre on floor, X = long side)
        x, y = p[0] - cx, p[1] - cy
        return (c * x - sn * y, sn * x + c * y, p[2] - cz)

    L = max(size[0], size[1])
    W = min(size[0], size[1])
    H = size[2]
    print("Car mesh %s: %.0f x %.0f x %.0f mm (L x W x H); mesh origin is %.0f, %.0f, %.0f mm "
          "from the car centre on the floor." % (os.path.basename(path), L, W, H, -cx, -cy, -cz))

    # height map (top surface) on a 40 mm grid, car frame
    cell = 40.0
    nx, ny = int(L / cell) + 1, int(W / cell) + 1
    top = [[None] * ny for _ in range(nx)]
    for t in tris:
        a, b, d = tf(t[0]), tf(t[1]), tf(t[2])
        det = (b[1] - d[1]) * (a[0] - d[0]) + (d[0] - b[0]) * (a[1] - d[1])
        if abs(det) < 1e-6:
            continue
        i0 = max(0, int((min(a[0], b[0], d[0]) + L / 2.0) / cell) - 1)
        i1 = min(nx - 1, int((max(a[0], b[0], d[0]) + L / 2.0) / cell) + 1)
        j0 = max(0, int((min(a[1], b[1], d[1]) + W / 2.0) / cell) - 1)
        j1 = min(ny - 1, int((max(a[1], b[1], d[1]) + W / 2.0) / cell) + 1)
        for i in range(i0, i1 + 1):
            px = -L / 2.0 + (i + 0.5) * cell
            for j in range(j0, j1 + 1):
                py = -W / 2.0 + (j + 0.5) * cell
                l1 = ((b[1] - d[1]) * (px - d[0]) + (d[0] - b[0]) * (py - d[1])) / det
                l2 = ((d[1] - a[1]) * (px - d[0]) + (a[0] - d[0]) * (py - d[1])) / det
                l3 = 1.0 - l1 - l2
                if l1 < -1e-6 or l2 < -1e-6 or l3 < -1e-6:
                    continue
                z = l1 * a[2] + l2 * b[2] + l3 * d[2]
                if top[i][j] is None or z > top[i][j]:
                    top[i][j] = z

    # centre-line height profile -> plateaus
    prof = []
    for i in range(nx):
        zs = [top[i][j] for j in range(ny) if top[i][j] is not None and abs(-W / 2.0 + (j + 0.5) * cell) <= 0.25 * W]
        prof.append(max(zs) if zs else None)
    plateaus, cur = [], None
    for i in range(nx):
        z = prof[i]
        if z is None:
            cur = None
            continue
        if cur is not None and abs(z - cur["z"]) <= 40:
            cur["i1"] = i
        else:
            cur = dict(i0=i, i1=i, z=z)
            plateaus.append(cur)
    plateaus = [p for p in plateaus if (p["i1"] - p["i0"] + 1) * cell >= 500 and p["z"] > 300]
    if not plateaus:
        print("Could not find flat roof/hood/trunk areas in the mesh - using the built-in table.")
        return None
    roof = max(plateaus, key=lambda p: (round(p["z"] / 50), p["i1"] - p["i0"]))
    after = [p for p in plateaus if p["i0"] > roof["i1"]]
    before = [p for p in plateaus if p["i1"] < roof["i0"]]
    hood = max(after, key=lambda p: p["i1"] - p["i0"]) if after else None
    trunk = max(before, key=lambda p: p["i1"] - p["i0"]) if before else None

    MARGIN = 120.0
    surfaces, order = {}, []

    def plateau_surface(name, p):
        x0 = -L / 2.0 + p["i0"] * cell + MARGIN
        x1 = -L / 2.0 + (p["i1"] + 1) * cell - MARGIN
        ys = [-W / 2.0 + (j + 0.5) * cell
              for i in range(p["i0"], p["i1"] + 1) for j in range(ny)
              if top[i][j] is not None and abs(top[i][j] - p["z"]) <= 40]
        if x1 - x0 < 200 or not ys:
            return
        y0, y1 = min(ys) + MARGIN, max(ys) - MARGIN
        if y1 - y0 < 200:
            return
        surfaces[name] = dict(origin=(x0, y0, p["z"]), u=(1, 0, 0), v=(0, 1, 0),
                              len_u=x1 - x0, len_v=y1 - y0, normal=(0, 0, 1))
        order.append(name)

    plateau_surface("Roof", roof)
    if hood:
        plateau_surface("Hood", hood)
    if trunk:
        plateau_surface("Trunk", trunk)

    low = min(p["z"] for p in plateaus)
    hw = W / 2.0
    z0 = 500.0
    z1 = min(1100.0, 0.9 * low)
    if z1 - z0 < 150:
        z0 = 0.3 * low
    bz1 = min(800.0, 0.8 * low)
    surfaces["Front"] = dict(origin=(L / 2.0, -(hw - 150), 300), u=(0, 1, 0), v=(0, 0, 1),
                             len_u=2 * (hw - 150), len_v=bz1 - 300, normal=(1, 0, 0))
    surfaces["Rear"] = dict(origin=(-L / 2.0, -(hw - 150), 300), u=(0, 1, 0), v=(0, 0, 1),
                            len_u=2 * (hw - 150), len_v=bz1 - 300, normal=(-1, 0, 0))
    surfaces["Side_Right"] = dict(origin=(-L / 2.0 + 100, hw, z0), u=(1, 0, 0), v=(0, 0, 1),
                                  len_u=L - 200, len_v=z1 - z0, normal=(0, 1, 0))
    surfaces["Side_Left"] = dict(origin=(-L / 2.0 + 100, -hw, z0), u=(1, 0, 0), v=(0, 0, 1),
                                 len_u=L - 200, len_v=z1 - z0, normal=(0, -1, 0))
    order += ["Front", "Rear", "Side_Right", "Side_Left"]
    print("Fitted surfaces from the mesh:")
    for nm in order:
        sf = surfaces[nm]
        print("   %-10s origin (%5.0f, %5.0f, %5.0f)  length %4.0f x %4.0f" % (
            nm, sf["origin"][0], sf["origin"][1], sf["origin"][2], sf["len_u"], sf["len_v"]))
    return dict(offset=(cx, cy, cz), rot=rot, surfaces=surfaces, order=order)

# ----------------------------------------------------------------------
# 3. ROBODK SET-UP
# ----------------------------------------------------------------------
RDK = Robolink()


def find_car():
    item = RDK.Item(CAR_NAME, ITEM_TYPE_OBJECT)
    if item.Valid():
        return item
    names = [o.Name() for o in RDK.ItemList(ITEM_TYPE_OBJECT)]
    for o in RDK.ItemList(ITEM_TYPE_OBJECT):
        if "car" in o.Name().lower() and not o.Name().startswith(SCENERY_PREFIXES):
            return o
    raise Exception("Car object '%s' not found. Objects in the station: %s. "
                    "Set CAR_NAME at the top of the script." % (CAR_NAME, names))


def clean_old_items():
    for it in RDK.ItemList(ITEM_TYPE_OBJECT):
        if it.Name().startswith(SCENERY_PREFIXES):
            it.Delete()
    for it in RDK.ItemList(ITEM_TYPE_PROGRAM):
        if it.Name().startswith("Paint_"):
            it.Delete()
    for it in RDK.ItemList(ITEM_TYPE_TARGET):
        if re.match(r"^(Roof|Hood|Trunk|Front|Rear|Side_Right|Side_Left)_S\d\d_", it.Name()):
            it.Delete()


def ident(x):
    return getattr(x, "item", id(x))


def pick_robots():
    """Returns (left_robot, right_robot_or_None). Handles two robots with the same name."""
    robots = RDK.ItemList(ITEM_TYPE_ROBOT)
    if not robots:
        raise Exception("No robot found in the station.")
    left = RDK.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
    if not left.Valid():
        left = robots[0]
    if not USE_SECOND_ROBOT:
        return left, None
    others = [r for r in robots if ident(r) != ident(left)]
    if len(others) > 1:
        print("WARNING: %d extra robots in the station; using only the first extra one." % len(others))
    if others:
        others[0].setName(ROBOT2_NAME)
        return left, others[0]
    before = set(ident(r) for r in robots)
    paths = [os.path.join(OUT_DIR, "Simulation_Files", "Fanuc-P-250iB-RH.robot"),
             os.path.join(_HERE, "Fanuc-P-250iB-RH.robot")]
    try:
        lib = RDK.getParam("PATH_LIBRARY")
        if lib:
            paths.append(os.path.join(lib, "Fanuc", "Fanuc-P-250iB-RH.robot"))
    except Exception:
        pass
    for p in paths:
        if not os.path.exists(p):
            continue
        try:
            RDK.AddFile(os.path.abspath(p))
        except Exception as e:
            print("Could not load %s: %s" % (p, e))
            continue
        for x in RDK.ItemList(ITEM_TYPE_ROBOT):
            if ident(x) not in before:
                x.setName(ROBOT2_NAME)
                return left, x
    return left, None


class Worker(object):
    """One robot on its own rail. Base position is x, y, yaw in the car frame."""

    def __init__(self, label, robot):
        self.label = label
        self.robot = robot
        self.base_frame = robot.Parent()
        tname = "SprayGun" if label == "L" else "SprayGun_R"
        tool = RDK.Item(tname, ITEM_TYPE_TOOL)
        if not tool.Valid():
            tool = robot.AddTool(transl(0, 0, 200), tname)
        robot.setPoseTool(tool)
        self.tool = tool
        self.x, self.y, self.yaw = 0, 0, 0
        self.side = label
        self.time_s = 0.0
        self.pieces = 0

    def place(self, x, y, yaw, show=False):
        self.base_frame.setPose(CAR_POSE0 * base_pose(x, y, yaw))
        self.x, self.y, self.yaw = x, y, yaw
        self.side = "L" if y < 0 else "R"
        if show:
            refresh()

    def park(self):
        s = SIDE_SIGN[self.label]
        self.place(0, s * TRACK_Y, SIDE_YAW[self.label])
        try:
            self.robot.setJoints(jl(self.robot.JointsHome()))
        except Exception:
            pass


def refresh():
    try:
        RDK.Render()
    except Exception:
        pass


# ----------------------------------------------------------------------
# 4. COLLISION + INVERSE KINEMATICS HELPERS
# ----------------------------------------------------------------------
IGNORE_NAMES = set()        # items that were already touching at the start (not the car)
STATS = {"no_ik": 0, "jump": 0, "collision": 0, "blocked_travel": 0, "exec_collision": 0, "move_failed": 0}
HIT_ITEMS = {}
LAST_COLL = {}
EXEC_COLLISIONS = []      # collisions RoboDK itself reported while a move ran
LIVE_STATE = {"live": LIVE_PAINT, "deferred": [], "count": 0}


def collision_names():
    try:
        return sorted(set(i.Name() for i in RDK.CollisionItems()))
    except Exception:
        return None


ENV_ITEMS = []                 # objects the robots must never touch (the car, extra objects)
PAIR_API = {"ok": True}


def refresh_env_items():
    ENV_ITEMS[:] = [o for o in RDK.ItemList(ITEM_TYPE_OBJECT)
                    if not o.Name().startswith(SCENERY_PREFIXES) and o.Name() not in IGNORE_NAMES]


def hit():
    """List of REAL collisions as text like 'RobotA x simple_car' (empty list = safe).
    Pair by pair with RoboDK's Collision(item1, item2): robot-car, robot-other objects and
    robot-robot. Paint stripes, scenery and robot self-contact are not counted."""
    try:
        n = RDK.Collisions()
    except Exception:
        return []
    if n is None or n <= 0:
        return []
    if PAIR_API["ok"]:
        try:
            robots = [w.robot for w in WORKERS]
            found = []
            for r in robots:
                for e in ENV_ITEMS:
                    if RDK.Collision(r, e):
                        found.append("%s x %s" % (r.Name(), e.Name()))
            if len(robots) == 2 and RDK.Collision(robots[0], robots[1]):
                found.append("%s x %s" % (robots[0].Name(), robots[1].Name()))
            for f_ in found:
                HIT_ITEMS[f_] = HIT_ITEMS.get(f_, 0) + 1
            return found
        except Exception as e:
            PAIR_API["ok"] = False
            print("NOTE: RDK.Collision(item, item) is not available (%s); using the less precise "
                  "item-list check." % e)
    names = collision_names()
    if names is None:
        return ["unknown"]
    rel = [nm for nm in names if nm not in IGNORE_NAMES and not nm.startswith(SCENERY_PREFIXES)]
    robots = set(w.robot.Name() for w in WORKERS)
    env = [nm for nm in rel if nm not in robots and nm in [e.Name() for e in ENV_ITEMS]]
    if env and any(nm in robots for nm in rel):
        for nm in rel:
            HIT_ITEMS[nm] = HIT_ITEMS.get(nm, 0) + 1
        return rel
    return []


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
    """IK solution closest to joints `ref` (or RoboDK's default if ref is None)."""
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
    return ik_near(robot, pose, None) is not None


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


CONFIG_SEEDS = {}


def config_seeds(robot):
    """Starting joint vectors that cover the different arm configurations (elbow up/down,
    wrist flipped...). IK returns the solution closest to each seed."""
    key = ident(robot)
    if key in CONFIG_SEEDS:
        return CONFIG_SEEDS[key]
    seeds = []
    try:
        lo, hi = robot.JointLimits()
        lo, hi = jl(lo), jl(hi)
        if len(lo) == 6 and len(hi) == 6:
            q1 = [a_ + (b_ - a_) * 0.25 for a_, b_ in zip(lo, hi)]
            q3 = [a_ + (b_ - a_) * 0.75 for a_, b_ in zip(lo, hi)]
            mid = [(a_ + b_) / 2.0 for a_, b_ in zip(lo, hi)]
            for j2 in (q1[1], q3[1]):
                for j3 in (q1[2], q3[2]):
                    for j5 in (q1[4], q3[4]):
                        for j4 in (q1[3], q3[3]):
                            seeds.append([mid[0], j2, j3, j4, j5, mid[5]])
    except Exception:
        pass
    CONFIG_SEEDS[key] = seeds
    return seeds



def ik_self_test(w):
    """Put the gun on a known point above the roof via IK and compare the gun position
    reported by RoboDK with the intended position. Prints the error in mm."""
    try:
        name = PAINT_ORDER[0]
        sf = SURFACES[name]
        o, u, v, n = sf["origin"], sf["u"], sf["v"], sf["normal"]
        p = [o[i] + u[i] * sf["len_u"] / 2.0 + v[i] * sf["len_v"] / 2.0 + n[i] * STANDOFF for i in range(3)]
        pose_car = tool_pose(p, u, n)
        s_ = SIDE_SIGN[w.label]
        for lane in (1800, 1500, 2000):
            for rot in ROLLS:
                roll = rotz(math.radians(rot))
                base = base_pose(0, s_ * lane, SIDE_YAW[w.label])
                j = ik_near(w.robot, base.inv() * (pose_car * roll), None)
                if j:
                    w.place(0, s_ * lane, SIDE_YAW[w.label])
                    w.robot.setJoints(j)
                    got = w.tool.PoseAbs().Pos()
                    want = (CAR_POSE0 * (pose_car * roll)).Pos()
                    err = dist3(got, want)
                    print("IK self-test (%s robot): gun position error = %.1f mm %s" % (
                        w.label, err, "(OK)" if err < 5 else "<-- MISMATCH: IK/frames are off"))
                    w.park()
                    return err
        print("IK self-test (%s robot): no IK solution for the test point above the roof." % w.label)
    except Exception as e:
        print("IK self-test could not run: %s" % e)
    try:
        w.park()
    except Exception:
        pass
    return None

# ----------------------------------------------------------------------
# 5. PLANNING
# ----------------------------------------------------------------------
def build_pieces():
    pieces = []
    for name in PAINT_ORDER:
        surf = SURFACES[name]
        n = surf["normal"]
        bands = stroke_bands(surf)
        for sk, (p0, p1, d) in enumerate(raster_strokes(surf)):
            m = max(1, int(math.ceil(dist3(p0, p1) / SEGMENT_LEN)))
            for i in range(m):
                a = lerp(p0, p1, i / float(m))
                b = lerp(p0, p1, (i + 1) / float(m))
                mid = lerp(a, b, 0.5)
                owner = "L" if mid[1] <= 0 else "R"
                pieces.append(dict(surface=name, stroke=sk, a=a, b=b, d=d, n=n,
                                   length=dist3(a, b), mid=mid, owner=owner, painted=0.0,
                                   v=surf["v"], lo=bands[sk][0], hi=bands[sk][1]))
    return pieces


def reach_poses(seg):
    a, b, d, n = seg["a"], seg["b"], seg["d"], seg["n"]
    off = _scale(n, APPROACH_DIST)
    return [tool_pose(a, d, n), tool_pose(lerp(a, b, 0.5), d, n), tool_pose(b, d, n),
            tool_pose(_add(a, off), d, n), tool_pose(_add(b, off), d, n)]


def find_candidates(seg, lanes, stops, rolls, per_lane, sides, limits):
    """Base positions (side, lane, stop, roll) from which the piece, its approach and
    its retract are reachable. `limits` = max candidates per side."""
    poses = reach_poses(seg)
    mid_x = seg["mid"][0]
    order = sorted(stops, key=lambda s: abs(s - mid_x))
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
    sides_first = ["L", "R"]
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
                  key=lambda t: (sides_first.index(t[1]["primary"]), order_idx[t[1]["surface"]], t[0]))
    return [p for _, p in plan]


# ----------------------------------------------------------------------
# 6. SAFE MOTION (own pre-flight with RoboDK's collision engine)
# ----------------------------------------------------------------------
def posture_chain(w, seg, cand):
    """Return (plan, "") if the WHOLE motion (approach, every CHECK_STEP of the stroke, retract)
    is collision-free from this base position/roll, else (None, reason).
    plan = dict(seq=joints per checked point, pts=car-frame points, k0=stroke start, k1=stroke end)."""
    side, lane, bx, rot = cand
    robot = w.robot
    a, b, d, n = seg["a"], seg["b"], seg["d"], seg["n"]
    off = _scale(n, APPROACH_DIST)
    roll = rotz(math.radians(rot))
    y_in = SIDE_SIGN[side] * lane
    yaw = SIDE_YAW[side]
    inv = base_pose(bx, y_in, yaw).inv()
    m = max(1, int(math.ceil(seg["length"] / float(CHECK_STEP))))
    pts = [_add(a, off), _add(a, _scale(off, 0.5))]
    pts += [lerp(a, b, i / float(m)) for i in range(m + 1)]
    pts += [_add(b, _scale(off, 0.5)), _add(b, off)]
    k_start, k_end = 2, 2 + m
    poses = [inv * (tool_pose(p, d, n) * roll) for p in pts]

    saved_j = robot.Joints().list()
    saved_state = (w.x, w.y, w.yaw)
    try:
        seeds = ik_seeds(robot, poses[k_start])
        if not seeds:
            STATS["no_ik"] += 1
            return None, "start point has no inverse-kinematics solution"
        w.place(bx, y_in, yaw)
        reason = "no usable arm posture"
        tried = []
        for seed in seeds:
            s0 = ik_near(robot, poses[k_start], seed)
            if s0 is None or any(jdist(s0, t) < 3 for t in tried):
                continue
            if len(tried) >= MAX_CONFIGS:
                break
            tried.append(s0)
            seq = [None] * len(poses)
            seq[k_start] = s0
            ok = True
            for i in list(range(k_start + 1, len(poses))) + list(range(k_start - 1, -1, -1)):
                ref = seq[i - 1] if i > k_start else seq[i + 1]
                s = ik_near(robot, poses[i], ref)
                if s is None:
                    STATS["no_ik"] += 1
                    reason = "no IK solution at point %d of the move" % i
                    ok = False
                    break
                if jdist(s, ref) > JUMP_STEP:
                    STATS["jump"] += 1
                    reason = "joint jump %.0f deg at point %d" % (jdist(s, ref), i)
                    ok = False
                    break
                seq[i] = s
            if not ok:
                continue
            order = [k_start, k_end] + [i for i in range(len(poses)) if i not in (k_start, k_end)]
            bad = None
            for i in order:
                robot.setJoints(seq[i])
                names = hit()
                if names:
                    bad = (i, names)
                    break
            if bad:
                STATS["collision"] += 1
                LAST_COLL.update(worker=w, base=(bx, y_in, yaw), joints=list(seq[bad[0]]),
                                 surface=seg["surface"], names=bad[1], point=bad[0], cand=cand, seg=seg)
                reason = "collision with %s at point %d" % (", ".join(bad[1]), bad[0])
                continue
            return dict(seq=seq, pts=pts, k0=k_start, k1=k_end), ""
        return None, reason
    finally:
        robot.setJoints(saved_j)
        w.place(*saved_state)


def joint_path_clear(w, j0, j1):
    n = max(1, int(math.ceil(jdist(j0, j1) / float(JOINT_STEP_DEG))))
    for k in range(1, n + 1):
        t = k / float(n)
        w.robot.setJoints([p + (q - p) * t for p, q in zip(j0, j1)])
        names = hit()
        if names:
            return False, names
    return True, []


def walk(w, pts, show):
    """Move the base through (x, y, yaw) points, checking collisions at every step.
    Returns (ok, distance_mm, hit_names). On a hit the base steps back one step."""
    total = 0.0
    for (xa, ya, wa), (xb, yb, wb) in zip(pts, pts[1:]):
        dist = math.hypot(xb - xa, yb - ya)
        steps = max(1, int(math.ceil(dist / (TRAVEL_STEP if show else 400.0))))
        for k in range(1, steps + 1):
            t = k / float(steps)
            prev = (w.x, w.y, w.yaw)
            w.place(xa + (xb - xa) * t, ya + (yb - ya) * t, wa + (wb - wa) * t, show=show)
            names = hit()
            if names:
                w.place(*prev)
                return False, total, names
        total += dist
    return True, total, []


def route(w, side, lane, bx):
    s1 = SIDE_SIGN[side]
    y_in, yaw1 = s1 * lane, SIDE_YAW[side]
    p0 = (w.x, w.y, w.yaw)
    if w.side == side and w.x == bx and abs(w.y - y_in) < 1:
        return [], [], False
    if w.side == side:
        out_pts = [p0, (w.x, s1 * TRACK_Y, w.yaw), (bx, s1 * TRACK_Y, yaw1)]
    else:                       # single-robot mode: go round the nearer end of the car
        s0 = SIDE_SIGN[w.side]
        xe = TRACK_X if (w.x + bx) >= 0 else -TRACK_X
        out_pts = [p0, (w.x, s0 * TRACK_Y, w.yaw), (xe, s0 * TRACK_Y, w.yaw),
                   (xe, s1 * TRACK_Y, yaw1), (bx, s1 * TRACK_Y, yaw1)]
    in_pts = [(bx, s1 * TRACK_Y, yaw1), (bx, y_in, yaw1)]
    return out_pts, in_pts, True


def do_move(w, kind, target, speed):
    """Run one RoboDK move with RoboDK's OWN collision checking active.
    If RoboDK stops the robot, the collision is counted and logged, the robot is put back
    where it was, and an error text is returned. Returns None when the move succeeded."""
    saved = w.robot.Joints().list()
    try:
        w.robot.setSpeed(speed)
        if kind == "J":
            w.robot.MoveJ(target)
        else:
            w.robot.MoveL(target)
        return None
    except Exception as e:
        text = str(e).strip()
        msg = text.splitlines()[0] if text else type(e).__name__
        if "ollision" in text:
            STATS["exec_collision"] += 1
            try:
                RDK.Collisions()
                items = ", ".join(collision_names() or [])
            except Exception:
                items = "unknown"
            msg = "%s [items in contact: %s]" % (msg, items)
            EXEC_COLLISIONS.append(msg[:200])
        else:
            STATS["move_failed"] += 1
        try:
            w.robot.setJoints(saved)
        except Exception:
            pass
        return msg


def travel(w, side, lane, bx, c0, dry):
    """Bring base + arm to the approach posture c0. dry=True only tests it."""
    out_pts, in_pts, relocate = route(w, side, lane, bx)
    saved_j = w.robot.Joints().list()
    saved_state = (w.x, w.y, w.yaw)
    dist_total = 0.0
    try:
        if relocate:
            ok, dist, names = walk(w, out_pts, show=not dry)
            dist_total += dist
            if not ok:
                STATS["blocked_travel"] += 1
                return False, "base travel blocked by %s" % ", ".join(names), 0.0
        cur = jl(w.robot.Joints())
        if dry:
            ok, names = joint_path_clear(w, cur, c0)
            if not ok:
                STATS["blocked_travel"] += 1
                return False, "arm swing into position blocked by %s" % ", ".join(names), 0.0
            w.robot.setJoints(c0)
        else:
            w.time_s += jdist(cur, c0) / float(JOINT_SPEED_EST)
            err = do_move(w, "J", c0, APPROACH_SPEED)
            if err:
                return False, "move to approach failed: %s" % err, dist_total
        if relocate:
            ok, dist, names = walk(w, in_pts, show=not dry)
            dist_total += dist
            if not ok:
                STATS["blocked_travel"] += 1
                return False, "base travel in blocked by %s" % ", ".join(names), dist_total
        return True, "", dist_total
    finally:
        if dry:
            w.robot.setJoints(saved_j)
            w.place(*saved_state)


def execute(w, seg, cand, pl):
    """Run the validated motion. The stroke is driven through the SAME points that were
    collision-checked, and a blue stripe is added only after each part has really been sprayed.
    Returns ("ok" | "fail" | "partial", message)."""
    side, lane, bx, rot = cand
    seq, pts, k0, k1 = pl["seq"], pl["pts"], pl["k0"], pl["k1"]
    ok, msg, _ = travel(w, side, lane, bx, seq[0], dry=True)
    if not ok:
        return "fail", msg
    ok, msg, dist = travel(w, side, lane, bx, seq[0], dry=False)
    w.time_s += dist / float(RAIL_SPEED)
    if not ok:
        return "fail", msg
    err = do_move(w, "L", seq[k0], APPROACH_SPEED)
    if err:
        return "fail", "approach move failed: %s" % err
    w.time_s += APPROACH_DIST / float(APPROACH_SPEED)
    visited = [k0]
    i = k0
    while i < k1:
        j = min(i + PAINT_GROUP, k1)
        err = do_move(w, "L", seq[j], PAINT_SPEED)
        if err:
            # back out along the already validated points, then report a partial stroke
            for back in reversed(visited[:-1] + [0]):
                if do_move(w, "L", seq[back], APPROACH_SPEED):
                    break
            return ("partial" if seg["painted"] > 0 else "fail"), "stroke move failed: %s" % err
        length = dist3(pts[i], pts[j])
        w.time_s += length / float(PAINT_SPEED)
        paint_stripe(pts[i], pts[j], seg)          # paint appears only after a successful spray move
        seg["painted"] += length
        visited.append(j)
        i = j
    err = do_move(w, "L", seq[-1], APPROACH_SPEED)
    if err:
        return "ok", "stroke painted, retract move failed: %s" % err
    w.time_s += APPROACH_DIST / float(APPROACH_SPEED)
    w.pieces += 1
    return "ok", ""


def safe_park(w):
    """Drive the robot back to its parking place with collision-checked moves (no teleport)."""
    try:
        sgn = SIDE_SIGN[w.side]
        pts = [(w.x, w.y, w.yaw), (w.x, sgn * TRACK_Y, w.yaw), (0, sgn * TRACK_Y, SIDE_YAW[w.side])]
        ok, _, names = walk(w, pts, show=True)
        if not ok:
            print("   %s robot could not drive home: blocked by %s" % (w.label, ", ".join(names)))
            return
        home = jl(w.robot.JointsHome())
        ok, names = joint_path_clear(w, jl(w.robot.Joints()), home)
        if not ok:
            print("   %s robot could not move to its home posture: blocked by %s" % (w.label, ", ".join(names)))
            return
        err = do_move(w, "J", home, APPROACH_SPEED)
        if err:
            print("   %s robot home move failed: %s" % (w.label, err))
    except Exception as e:
        print("   %s robot park failed: %s" % (w.label, e))


# ----------------------------------------------------------------------
# 7. DRAWING (after the run only)
# ----------------------------------------------------------------------
def make_stripe(p, q, seg, number):
    """Blue strip between two gun points. It covers only this stroke's own band (no overlap with
    its neighbours) and is 1 mm shorter at both ends, so two stripes never touch each other.
    It lies PAINT_LIFT mm above the surface. Same colour for every robot."""
    n, v = seg["n"], seg["v"]
    length = dist3(p, q)
    if length < 4 or seg["hi"] - seg["lo"] < 4:
        return None
    u = [(q[i] - p[i]) / length for i in range(3)]
    p2 = _add(p, u)
    q2 = _add(q, _scale(u, -1))
    lift = [c * (PAINT_LIFT - STANDOFF) for c in n]
    lo, hi = seg["lo"] + 1.0, seg["hi"] - 1.0
    a0 = _add(_add(p2, lift), _scale(v, hi))
    a1 = _add(_add(p2, lift), _scale(v, lo))
    b0 = _add(_add(q2, lift), _scale(v, hi))
    b1 = _add(_add(q2, lift), _scale(v, lo))
    pts = [a0, a1, b1, a0, b1, b0, a0, b1, a1, a0, b0, b1]
    shape = RDK.AddShape(pts)
    shape.setName("Paint_Stripe_%04d" % number)
    shape.setColor(PAINT_COLOR)
    shape.setParent(CAR_FRAME)
    return shape


def stripe_collision_free(shape):
    """A stripe must never block a move. Switch off the stripe/car pair and any stripe/stripe pair.
    A stripe touching a ROBOT is never hidden. Returns False if the stripe still collides."""
    try:
        RDK.setCollisionActivePair(0, shape, CAR_ITEM)
        if RDK.Collisions() <= 0 or shape.Name() not in (collision_names() or []):
            return True
        for nm in (collision_names() or []):
            if nm.startswith("Paint_Stripe") and nm != shape.Name():
                RDK.setCollisionActivePair(0, shape, RDK.Item(nm))
        return not (RDK.Collisions() > 0 and shape.Name() in (collision_names() or []))
    except Exception:
        return False


def paint_stripe(p, q, seg):
    """Called right after a spray move succeeded."""
    if not SHOW_PAINT:
        return
    LIVE_STATE["count"] += 1
    if LIVE_STATE["live"]:
        try:
            shape = make_stripe(p, q, seg, LIVE_STATE["count"])
            if shape is None or stripe_collision_free(shape):
                return
            names = collision_names() or []
            shape.Delete()
            LIVE_STATE["live"] = False
            print("WARNING: a paint stripe collided with %s, so live painting is switched off; the "
                  "rest of the paint is drawn after the run." % ", ".join(names))
        except Exception as e:
            LIVE_STATE["live"] = False
            print("WARNING: live paint stripes failed (%s); the rest is drawn after the run." % e)
    LIVE_STATE["deferred"].append((p, q, seg, LIVE_STATE["count"]))


def probe_collision(dbg):
    """Find out WHAT collides for a rejected position: every arm configuration at the stroke
    start, the same posture with the base moved away, and the gun pulled back."""
    w, seg, cand, j0 = dbg["worker"], dbg["seg"], dbg["cand"], dbg["joints"]
    side, lane, bx, rot = cand
    robot = w.robot
    sgn, yaw = SIDE_SIGN[side], SIDE_YAW[side]
    saved_j = robot.Joints().list()
    saved_state = (w.x, w.y, w.yaw)
    a, n, d = seg["a"], seg["n"], seg["d"]
    print("\n--- Collision probe: piece %d (%s), robot %s, lane %d, stop %d, roll %d ---" % (
        dbg["piece"], seg["surface"], w.label, lane, bx, rot))
    print("   stroke start in car frame: (%.0f, %.0f, %.0f) mm, normal (%.0f, %.0f, %.0f)" % (
        a[0], a[1], a[2], n[0], n[1], n[2]))
    print("   robot base in car frame: x=%d y=%d yaw=%d; first colliding joints (deg): %s" % (
        bx, sgn * lane, yaw, [round(x, 1) for x in j0]))
    try:
        inv = base_pose(bx, sgn * lane, yaw).inv()
        roll = rotz(math.radians(rot))
        w.place(bx, sgn * lane, yaw)
        seeds = ik_seeds(robot, inv * (tool_pose(a, d, n) * roll))
        tried = []
        for seed in seeds:
            s0 = ik_near(robot, inv * (tool_pose(a, d, n) * roll), seed)
            if s0 is None or any(jdist(s0, t) < 3 for t in tried):
                continue
            tried.append(s0)
            robot.setJoints(s0)
            names = hit()
            print("   IK configuration %2d %s: %s" % (len(tried), [round(x) for x in s0],
                                                    ", ".join(names) if names else "CLEAR at the start point"))
        print("   (%d different arm configurations exist at the start point)" % len(tried))
        for delta in (0, 600, 1500, 2500):
            w.place(bx, sgn * (lane + delta), yaw)
            robot.setJoints(j0)
            names = hit()
            print("   first colliding posture, base %4d mm further from the car: %s" % (
                delta, ", ".join(names) if names else "clear"))
        w.place(bx, sgn * lane, yaw)
        for back in (0, 150, 300, 600, 1000):
            p = _add(a, _scale(n, back))
            j = ik_near(robot, inv * (tool_pose(p, d, n) * roll), j0)
            if j is None:
                print("   gun pulled back %4d mm from the start point: no IK solution" % back)
                continue
            robot.setJoints(j)
            names = hit()
            print("   gun pulled back %4d mm from the start point: %s" % (
                back, ", ".join(names) if names else "clear"))
    finally:
        robot.setJoints(saved_j)
        w.place(*saved_state)


def add_box(name, c, s, color):
    shape = RDK.AddShape(box_tris(c[0], c[1], c[2], s[0], s[1], s[2]))
    shape.setName(name)
    shape.setColor(color)
    shape.setParent(CAR_FRAME)


def add_target_patches():
    for name in PAINT_ORDER:
        sf = SURFACES[name]
        o, u, v, n = sf["origin"], sf["u"], sf["v"], sf["normal"]
        c = [o[i] + u[i] * sf["len_u"] / 2.0 + v[i] * sf["len_v"] / 2.0 + n[i] * 4 for i in range(3)]
        sz = [abs(u[i]) * sf["len_u"] + abs(v[i]) * sf["len_v"] + abs(n[i]) * 8 for i in range(3)]
        add_box("Booth_Target_" + name, c, sz, [1.0, 0.9, 0.1, 0.45])


def build_scenery():
    if SHOW_TARGET_PATCHES:
        add_target_patches()
    add_box("Booth_Floor", (0, 0, -45), (12000, 8000, 70), [0.55, 0.57, 0.6, 1])        # top at z=-10
    add_box("Booth_Conveyor", (0, 0, -10), (7600, 1500, 10), [0.15, 0.15, 0.17, 1])     # top at z=-5
    add_box("Booth_Wall_Left", (0, -4000, 1750), (12000, 60, 3500), [0.7, 0.8, 0.95, 0.12])
    add_box("Booth_Wall_Right", (0, 4000, 1750), (12000, 60, 3500), [0.7, 0.8, 0.95, 0.12])
    add_box("Rail_Left", (0, -1800, -20), (6800, 900, 30), [0.9, 0.75, 0.1, 1])
    if USE_SECOND_ROBOT_ACTIVE[0]:
        add_box("Rail_Right", (0, 1800, -20), (6800, 900, 30), [0.9, 0.75, 0.1, 1])


# ----------------------------------------------------------------------
# 8. MAIN RUN
# ----------------------------------------------------------------------
USE_SECOND_ROBOT_ACTIVE = [USE_SECOND_ROBOT]
DEBUG_POSTURE = {}
WORKERS = []
WORKER_OF = {}
CAR_FRAME = None
CAR_ITEM = None
DEBUG_LIST = []
CAR_POSE0 = None


def write_csv(name, rows):
    path = os.path.join(OUT_DIR, name)
    try:
        with open(path, "w", newline="") as f:
            csv.writer(f).writerows(rows)
        return path
    except Exception as e:
        print("Could not write %s: %s" % (name, e))
        return None


def main():
    global CAR_FRAME, CAR_POSE0, CAR_ITEM, SURFACES, PAINT_ORDER

    robot_l, robot_r = pick_robots()
    car = find_car()
    CAR_ITEM = car
    print("Using car object:", car.Name())
    clean_old_items()
    if RDK.Item("Brooks PreciseFlex Linear Rail 2000").Valid():
        print("WARNING: the Brooks rail is still in the station - delete it in RoboDK first.")

    CAR_FRAME = RDK.Item("Car_Frame", ITEM_TYPE_FRAME)
    if not CAR_FRAME.Valid():
        CAR_FRAME = RDK.AddFrame("Car_Frame")
    fit = fit_car_to_mesh(car)
    if fit:
        SURFACES, PAINT_ORDER = fit["surfaces"], fit["order"]
        cx, cy, cz = fit["offset"]
        CAR_POSE0 = car.PoseAbs() * transl(cx, cy, cz) * rotz(math.radians(fit["rot"]))
        if abs(cx) > 50 or abs(cy) > 50 or abs(cz) > 50 or fit["rot"]:
            print("NOTE: the car mesh origin is NOT at the car centre on the floor. The old surface "
                  "table was therefore in the wrong place - this is the most likely reason earlier "
                  "runs collided on every surface. The car frame has been moved to the real centre.")
    else:
        CAR_POSE0 = car.PoseAbs()
    CAR_FRAME.setPose(CAR_POSE0)

    # ---- robots ----
    wl = Worker("L", robot_l)
    WORKERS.append(wl)
    WORKER_OF["L"] = wl
    if USE_SECOND_ROBOT and robot_r is None:
        print("WARNING: could not load the second robot. Copy/paste the robot in the RoboDK "
              "tree, rename the copy '%s' and run again. Continuing with ONE robot." % ROBOT2_NAME)
        USE_SECOND_ROBOT_ACTIVE[0] = False
    if robot_r is not None:
        wr = Worker("R", robot_r)
        WORKERS.append(wr)
        WORKER_OF["R"] = wr
    else:
        WORKER_OF["R"] = wl
    for w in WORKERS:
        w.robot.setPoseFrame(w.base_frame)      # IK targets are relative to the robot base
        w.park()
    print("Robots: %s" % ", ".join(w.robot.Name() for w in WORKERS))
    if len(WORKERS) == 2 and WORKERS[0].robot.Name() == WORKERS[1].robot.Name():
        WORKERS[1].robot.setName(ROBOT2_NAME)

    # ---- collision baseline (things already touching before painting) ----
    try:
        RDK.setCollisionActive(1)
    except Exception as e:
        print("Collision checking not available (%s). The safety checks cannot run." % e)
    names = collision_names() or []
    robot_names = set(w.robot.Name() for w in WORKERS)
    base_items = [nm for nm in names if nm not in robot_names and not nm.startswith(SCENERY_PREFIXES)]
    if car.Name() in base_items:
        print("WARNING: a parked robot already touches the car - check the car position.")
        base_items.remove(car.Name())
    IGNORE_NAMES.update(base_items)
    if base_items:
        print("Note: these items already touch a parked robot: %s. They are excluded from the checks "
              "(delete them from the station if they are leftovers)." % ", ".join(base_items))
        for nm in base_items:
            for w in WORKERS:
                try:
                    RDK.setCollisionActivePair(0, w.robot, RDK.Item(nm))
                except Exception as e:
                    print("   could not switch off the pair %s / %s: %s" % (w.robot.Name(), nm, e))
    for w in WORKERS:
        try:
            tcp = w.robot.PoseTool().Pos()
            print("Robot %s: tool %s, TCP offset from flange = (%.0f, %.0f, %.0f) mm, base frame %s" % (
                w.robot.Name(), w.tool.Name(), tcp[0], tcp[1], tcp[2], w.base_frame.Name()))
        except Exception:
            pass

    refresh_env_items()
    print("Objects the robots must not touch: %s" % ", ".join(e.Name() for e in ENV_ITEMS))
    for w in WORKERS:
        ik_self_test(w)

    # ---- planning ----
    print("\nPlanning pieces (about a minute)...")
    t0 = time.time()
    plan = plan_pieces()
    no_cand = [s for s in plan if not s["cands"]]
    print("Planned %d pieces in %.0f s; %d have no base position in the first search "
          "(they get a wider search while painting)." % (len(plan), time.time() - t0, len(no_cand)))
    rows = [["surface", "stroke", "mid_x", "mid_y", "mid_z", "owner", "first_search_candidates"]]
    for s in plan:
        rows.append([s["surface"], s["stroke"], round(s["mid"][0]), round(s["mid"][1]),
                     round(s["mid"][2]), s["owner"], len(s["cands"])])
    write_csv("reachability_log.csv", rows)

    # ---- painting ----
    done, skipped, fails, partial = [], [], [], []
    stats = {"fallback": 0, "extended": 0}
    run_log = [["piece", "surface", "stroke", "result", "robot", "lane", "stop_x", "roll", "detail"]]
    t_start = time.time()
    total = len(plan)
    for idx, seg in enumerate(plan):
        painted = False
        result = "none"
        reasons = []
        tried = set()

        def try_list(cands, ext):
            for attempt, cand in enumerate(cands):
                if cand in tried:
                    continue
                tried.add(cand)
                w = WORKER_OF[cand[0]]
                plan_, why = posture_chain(w, seg, cand)
                if plan_ is None:
                    reasons.append("%s lane %d stop %d roll %d: %s" % (cand[0], cand[1], cand[2], cand[3], why))
                    continue
                status, msg = execute(w, seg, cand, plan_)
                if status == "ok":
                    seg["used"] = cand
                    if attempt > 0:
                        stats["fallback"] += 1
                    if ext:
                        stats["extended"] += 1
                    if msg:
                        reasons.append(msg)
                    return "ok"
                reasons.append("%s lane %d stop %d roll %d: %s" % (cand[0], cand[1], cand[2], cand[3], msg))
                if status == "partial":
                    seg["used"] = cand
                    return "partial"
            return "none"

        def prefer_current(cands):
            def key(c):
                w = WORKER_OF[c[0]]
                return 0 if (w.side == c[0] and w.x == c[2] and abs(abs(w.y) - c[1]) < 1) else 1
            return sorted(cands, key=key)

        try:
            result = try_list(prefer_current(seg["cands"]), False)
            if result == "none":
                owner = seg["owner"]
                other = "R" if owner == "L" else "L"
                ext = find_candidates(seg, LANES_EXT + LANES, STOPS_EXT, ROLLS_EXT, 4,
                                      [owner, other], [EXT_MAX, 6])
                result = try_list(ext, True)
            painted = (result == "ok")
            if result == "partial":
                partial.append(seg)
        except Exception as e:
            tb = traceback.format_exc()
            print("\n!!! SCRIPT ERROR in piece %d (%s): %s: %s\n%s" % (idx, seg["surface"], type(e).__name__, e, tb))
            reasons.append("script error: %s: %s" % (type(e).__name__, e))
            for w in WORKERS:
                try:
                    w.place(w.x, w.y, w.yaw)
                except Exception:
                    pass

        if painted:
            done.append(seg)
            u = seg["used"]
            run_log.append([idx, seg["surface"], seg["stroke"], "painted", u[0], u[1], u[2], u[3], ""])
            print("[%3d/%d] %-10s painted by robot %s (lane %d, stop %d, roll %d)  %.0f s" % (
                idx + 1, total, seg["surface"], u[0], u[1], u[2], u[3], time.time() - t_start))
        elif result == "partial":
            last = reasons[-1] if reasons else "stroke stopped"
            run_log.append([idx, seg["surface"], seg["stroke"], "PARTIAL", seg["used"][0], seg["used"][1],
                            seg["used"][2], seg["used"][3], last])
            print("[%3d/%d] %-10s PARTIAL (%.0f of %.0f mm painted) - %s  %.0f s" % (
                idx + 1, total, seg["surface"], seg["painted"], seg["length"], last[:80], time.time() - t_start))
        else:
            last = reasons[-1] if reasons else "no base position can reach this piece"
            skipped.append((idx, seg, last))
            if len(fails) < 12:
                fails.append((idx, seg["surface"], reasons[:3] or [last]))
            if len(DEBUG_LIST) < 2 and LAST_COLL and LAST_COLL.get("seg") is seg:
                DEBUG_LIST.append(dict(LAST_COLL, piece=idx))
            if len(skipped) <= 3:
                print("      why (first options tried):")
                for r_ in (reasons[:4] or [last]):
                    print("        -", r_[:150])
            run_log.append([idx, seg["surface"], seg["stroke"], "SKIPPED", "", "", "", "", last])
            print("[%3d/%d] %-10s SKIPPED - %s  %.0f s" % (idx + 1, total, seg["surface"], last[:100],
                                                           time.time() - t_start))

    # ---- drive robots home (collision-checked), then finish the display ----
    for w in WORKERS:
        safe_park(w)
    if LIVE_STATE["deferred"]:
        try:
            for p_, q_, sg_, num in LIVE_STATE["deferred"]:
                make_stripe(p_, q_, sg_, num)
            print("Drew %d paint stripes after the run (live painting was not possible)." % len(LIVE_STATE["deferred"]))
        except Exception as e:
            print("Could not draw the deferred paint stripes: %s" % e)
    for dbg in DEBUG_LIST:
        try:
            probe_collision(dbg)
        except Exception as e:
            print("Collision probe failed: %s" % e)
    if BUILD_SCENERY and not (LEAVE_COLLISION_POSTURE and DEBUG_LIST):
        try:
            build_scenery()
            print("Booth, conveyor and rails drawn.")
        except Exception as e:
            print("Could not draw the scenery: %s" % e)
    elif BUILD_SCENERY:
        print("Booth/conveyor/rails NOT drawn this time, to keep the failing-posture view clean. "
              "They are drawn automatically on a run without skipped pieces.")
    if LEAVE_COLLISION_POSTURE and DEBUG_LIST:
        try:
            dbg = DEBUG_LIST[0]
            dw = dbg["worker"]
            dw.place(*dbg["base"])
            dw.robot.setJoints(dbg["joints"])
            print("\nDIAGNOSTIC DISPLAY: the %s robot has been placed in the posture that collided for piece "
                  "%d (%s), touching %s. This is a display of a REJECTED posture, not a movement. Red parts "
                  "in RoboDK show the contact - please send a screenshot." % (
                      dw.label, dbg["piece"], dbg["surface"], ", ".join(dbg["names"])))
        except Exception as e:
            print("Could not show the failing posture: %s" % e)
    refresh()

    # ---- summary ----
    tot_len = dict((n_, sum(dist3(p0, p1) for p0, p1, d in raster_strokes(SURFACES[n_]))) for n_ in PAINT_ORDER)
    painted_len = dict((n_, 0.0) for n_ in PAINT_ORDER)
    by_robot = {"L": 0, "R": 0}
    piece_count = dict((n_, 0) for n_ in PAINT_ORDER)
    for s in plan:
        piece_count[s["surface"]] += 1
    piece_done = dict((n_, 0) for n_ in PAINT_ORDER)
    for s in plan:
        painted_len[s["surface"]] += s["painted"]
    for s in done:
        piece_done[s["surface"]] += 1
        by_robot[s["used"][0]] += 1

    print("\n=== Paint simulation summary ===")
    res = [["surface", "pieces", "pieces_painted", "coverage_percent"]]
    for n_ in PAINT_ORDER:
        cov = 100.0 * painted_len[n_] / tot_len[n_]
        print("%-11s %3d / %3d pieces fully painted  (%.0f %% of the stroke length actually sprayed)" % (
            n_, piece_done[n_], piece_count[n_], cov))
        res.append([n_, piece_count[n_], piece_done[n_], round(cov, 1)])
    all_cov = 100.0 * sum(painted_len.values()) / sum(tot_len.values())
    print("TOTAL: %.1f %% of all planned stroke length sprayed (%.0f of %.0f mm)" % (
        all_cov, sum(painted_len.values()), sum(tot_len.values())))
    res.append(["TOTAL", len(plan), len(done), round(all_cov, 1)])
    res.append(["collisions_reported_by_robodk_during_moves", STATS["exec_collision"], "", ""])
    print("Pieces painted: left robot %d, right robot %d" % (by_robot["L"], by_robot["R"]))
    times = [w.time_s for w in WORKERS]
    cycle = max(times) if len(WORKERS) > 1 else sum(times)
    print("Estimated cycle time: %.0f s (%.1f min) - robots work in parallel in the real cell; "
          "this is an estimate from path lengths and joint/rail speeds, not a RoboDK program time."
          % (cycle, cycle / 60.0))
    print("Pieces painted from a fall-back position: %d, from the wider search: %d" % (
        stats["fallback"], stats["extended"]))
    print("Rejected (no IK %d, joint jump %d, collision %d); blocked base/arm travel %d" % (
        STATS["no_ik"], STATS["jump"], STATS["collision"], STATS["blocked_travel"]))
    print("MEASURED: collisions reported by RoboDK during executed moves: %d; moves that failed for "
          "other reasons (IK/singularity/limits): %d; partial strokes: %d" % (
              STATS["exec_collision"], STATS["move_failed"], len(partial)))
    for t_ in EXEC_COLLISIONS[:5]:
        print("   RoboDK collision message:", t_)
    print("Options rejected by the pre-flight check never moved. Paint stripes drawn: %d (%s)." % (
        LIVE_STATE["count"], "live, during spraying" if LIVE_STATE["live"] else "after the run"))
    print("Validation level: stroke points checked every %d mm, joint moves every %d deg, with RoboDK's "
          "collision engine; reachable-only poses are NOT counted as painted." % (CHECK_STEP, JOINT_STEP_DEG))
    if HIT_ITEMS:
        print("Items that caused rejected options (most often first):")
        for nm, c in sorted(HIT_ITEMS.items(), key=lambda kv: -kv[1])[:8]:
            print("   %-28s %d" % (nm, c))
    if skipped:
        print("SKIPPED pieces (not painted): %d" % len(skipped))
        for idx, seg, why in skipped[:10]:
            print("   piece %d %s: %s" % (idx, seg["surface"], why[:110]))
        print("First failing options in detail:")
        for idx, surf, rs in fails[:6]:
            print("   piece %d %s" % (idx, surf))
            for r_ in rs:
                print("      -", r_[:140])
    write_csv("run_log.csv", run_log)
    write_csv("results_summary.csv", res)
    print("\nFiles written to %s: reachability_log.csv, run_log.csv, results_summary.csv" % OUT_DIR)


class _Tee(object):
    """Copy everything printed to run_console_log.txt (so you can send the full log)."""
    def __init__(self, path):
        self.f = open(path, "w")
        self.o = sys.stdout

    def write(self, t):
        self.o.write(t)
        try:
            self.f.write(t)
        except Exception:
            pass

    def flush(self):
        self.o.flush()
        try:
            self.f.flush()
        except Exception:
            pass


try:
    sys.stdout = _Tee(os.path.join(OUT_DIR, "run_console_log.txt"))
    main()
except Exception:
    print("\n!!! The script stopped with an error. Copy EVERYTHING below and send it:\n")
    traceback.print_exc()

