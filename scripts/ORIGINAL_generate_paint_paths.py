Created At: 2026-10-22T12:44:04+05:30
Completed At: 2026-10-22T12:44:05+05:30
File Path: `file:///c:/projects/robotic-car-painting-cell/scripts/generate_paint_paths.py`
Total Lines: 1522
Total Bytes: 63587
Showing lines 1 to 250
The following code has been modified to include a line number before every line, in the format: <line_number>: <original_line>. Please note that any changes targeting the original code should remove the line number, colon, and leading space.
1: """
2: generate_paint_paths.py   (version 3 - dual robot, own collision pre-flight)
3: Robotic Car Painting Cell - paint simulation for RoboDK
4: 
5: HOW TO RUN
6: ----------
7: 1. Open RoboDK with your station (car "simple_car" + robot "Fanuc P-250iB RH").
8: 2. Delete leftovers from older tries (the Brooks rail, old Paint_Stripe objects).
9: 3. In a terminal:   cd C:\\projects\\robotic-car-painting-cell\\scripts
10:                     python generate_paint_paths.py
11: 
12: WHAT IT DOES
13: ------------
14: * Uses TWO identical FANUC P-250iB RH paint robots, one on each side of the car,
15:   each riding on its own straight rail along the car. The car stays still.
16:   (The second robot is loaded from Simulation_Files\\Fanuc-P-250iB-RH.robot.
17:    If that fails, copy/paste the robot in the RoboDK tree and rename the copy
18:    "Robot_Right", or set USE_SECOND_ROBOT = False.)
19: * The left robot paints everything on the left half of the car, the right robot
20:   the right half (roof, hood, trunk, front, rear, sides). Zig-zag strokes,
21:   250 mm standoff, 40 % overlap, 400 mm/s.
22: * BEFORE every move the script checks the whole planned motion (approach,
23:   every 100 mm of the stroke, retract, the swing into position and the rail
24:   travel) with RoboDK's collision engine. Only contacts with the car or with
25:   the other robot count. A piece that is not safe is moved to another base
26:   position, lane or gun roll. If nothing is safe it is SKIPPED and reported.
27: * RoboDK's own collision stop stays ON while moves run. If it stops a move, that
28:   is counted as a collision event and reported (nothing is hidden).
29: * LIVE PAINTING: the stroke is driven through the same checked points, and a
30:   blue stripe (300 mm long) is added right after each spray move succeeds. Only
31:   sprayed length counts as painted; travel and approach never paint.
32: * A booth, conveyor and rails are drawn at the end (they clear the robots).
33: * At the end it probes the first failing postures (base moved away / gun pulled
34:   back) to show WHAT collides, and leaves the robot in that rejected posture
35:   as a display so RoboDK shows the contact in red.
36: * Writes: reachability_log.csv, run_log.csv, results_summary.csv and
37:   run_console_log.txt (the full console output) in the project folder.
38: 
39: All lengths are mm, angles degrees. Car frame: origin = car centre on the floor,
40: X forward, Y left, Z up.
41: """
42: 
43: import csv
44: import math
45: import os
46: import re
47: import struct
48: import sys
49: import time
50: import traceback
51: 
52: try:
53:     from robodk.robolink import *    # RoboDK API (new package layout)
54:     from robodk.robomath import *
55: except ImportError:
56:     from robolink import *           # older layout
57:     from robomath import *
58: 
59: # ----------------------------------------------------------------------
60: # 1. PARAMETERS
61: # ----------------------------------------------------------------------
62: ROBOT_NAME = "Fanuc P-250iB RH"      # the robot already in your station (left robot)
63: ROBOT2_NAME = "Robot_Right"          # name given to the second (right) robot
64: CAR_NAME = "simple_car"
65: USE_SECOND_ROBOT = True              # True = two robots (recommended)
66: BUILD_SCENERY = True                 # draw floor/walls/rails/conveyor after the run
67: SHOW_PAINT = True                    # draw blue paint stripes at all
68: LEAVE_COLLISION_POSTURE = True       # leave the robot in the first failing posture so you can see it
69: LIVE_PAINT = True                    # blue stripes appear WHILE the gun sprays (not after the run)
70: PAINT_GROUP = 3                      # checked points per live stripe (3 x 50 mm = 150 mm)
71: SHOW_TARGET_PATCHES = False          # yellow check patches on the surfaces (they hide the paint)
72: MAX_CONFIGS = 10                     # different arm configurations tried per base position
73: 
74: CAR_LENGTH, CAR_WIDTH, CAR_HEIGHT = 4800, 1900, 1600
75: 
76: STANDOFF = 250          # gun distance from the surface
77: FAN_WIDTH = 300         # spray width at the standoff
78: OVERLAP = 0.40          # 40 % overlap
79: STEP = FAN_WIDTH * (1 - OVERLAP)   # 180 mm between passes
80: PAINT_SPEED = 400       # mm/s while spraying
81: APPROACH_SPEED = 800    # mm/s approach / retract
82: APPROACH_DIST = 150     # mm
83: 
84: # Flat rectangles in the car frame.
85: #   origin: corner, u: stroke direction, v: step direction, normal: outward
86: SURFACES = {
87:     "Roof":  dict(origin=(-1000, -650, 1600), u=(1, 0, 0), v=(0, 1, 0),
88:                   len_u=1900, len_v=1300, normal=(0, 0, 1)),
89:     "Hood":  dict(origin=(900, -800, 1000), u=(1, 0, 0), v=(0, 1, 0),
90:                   len_u=1500, len_v=1600, normal=(0, 0, 1)),
91:     "Trunk": dict(origin=(-2400, -800, 1050), u=(1, 0, 0), v=(0, 1, 0),
92:                   len_u=1400, len_v=1600, normal=(0, 0, 1)),
93:     "Front": dict(origin=(2400, -800, 300), u=(0, 1, 0), v=(0, 0, 1),
94:                   len_u=1600, len_v=500, normal=(1, 0, 0)),
95:     "Rear":  dict(origin=(-2400, -800, 300), u=(0, 1, 0), v=(0, 0, 1),
96:                   len_u=1600, len_v=500, normal=(-1, 0, 0)),
97:     "Side_Right": dict(origin=(-2300, 950, 500), u=(1, 0, 0), v=(0, 0, 1),
98:                        len_u=4600, len_v=500, normal=(0, 1, 0)),
99:     "Side_Left": dict(origin=(-2300, -950, 500), u=(1, 0, 0), v=(0, 0, 1),
100:                       len_u=4600, len_v=500, normal=(0, -1, 0)),
101: }
102: PAINT_ORDER = ["Roof", "Hood", "Trunk", "Front", "Rear", "Side_Right", "Side_Left"]
103: 
104: # --- cell / planner settings ---
105: SIDE_SIGN = {"L": -1, "R": 1}        # L robot sits at negative Y, R at positive Y
106: SIDE_YAW = {"L": 90, "R": -90}       # each robot faces the car (its X axis points at the car)
107: LANES = (1800, 1600, 1400, 2000)     # base distance from the car centre line (first search)
108: LANES_EXT = (1500, 1700, 1900, 2100, 2200, 2400)   # extra lanes (second search)
109: STOPS = list(range(-3000, 3001, 300))              # rail stops along the car
110: STOPS_EXT = list(range(-3000, 3001, 150))
111: ROLLS = (0, 90, 180, 270)                          # gun rotation about its own axis
112: ROLLS_EXT = tuple(range(0, 360, 30))
113: SEGMENT_LEN = 600        # a stroke is cut into pieces of this length
114: CHECK_STEP = 50          # mm between collision-checked points along a stroke
115: JUMP_STEP = 45           # deg, max joint change between neighbouring checked points
116: JOINT_STEP_DEG = 3       # deg, resolution of the joint-move collision check
117: PER_LANE = 3             # candidate stops kept per lane (first search)
118: OTHER_SIDE_MAX = 3       # candidates borrowed from the other robot's side
119: EXT_MAX = 24             # candidates tried in the second search
120: TRACK_Y = 3300           # outer lane used when the base moves (mm from centre line)
121: TRACK_X = 3800           # single-robot mode only: base x when going round the car end
122: TRAVEL_STEP = 250        # mm per animation step of the base
123: JOINT_SPEED_EST = 120    # deg/s, only for the cycle-time estimate
124: RAIL_SPEED = 500         # mm/s, only for the cycle-time estimate
125: PAINT_COLOR = [0.05, 0.25, 0.95, 1.0]   # ONE colour, used by BOTH robots
126: PAINT_LIFT = 3           # mm the stripe floats above the surface
127: 
128: try:
129:     _HERE = os.path.dirname(os.path.abspath(__file__))
130: except NameError:
131:     _HERE = os.path.join(os.path.expanduser("~"), "Documents")
132: OUT_DIR = os.path.abspath(os.path.join(_HERE, ".."))
133: SCENERY_PREFIXES = ("Paint_Stripe", "Booth_", "Rail_")
134: 
135: 
136: # ----------------------------------------------------------------------
137: # 2. GEOMETRY HELPERS (pure maths)
138: # ----------------------------------------------------------------------
139: def _add(a, b):
140:     return [a[i] + b[i] for i in range(3)]
141: 
142: 
143: def _scale(a, s):
144:     return [a[i] * s for i in range(3)]
145: 
146: 
147: def _cross(a, b):
148:     return [a[1] * b[2] - a[2] * b[1],
149:             a[2] * b[0] - a[0] * b[2],
150:             a[0] * b[1] - a[1] * b[0]]
151: 
152: 
153: def lerp(a, b, t):
154:     return [a[i] + (b[i] - a[i]) * t for i in range(3)]
155: 
156: 
157: def dist3(a, b):
158:     return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))
159: 
160: 
161: def tool_pose(position, stroke_dir, normal):
162:     """Gun TCP pose: Z points INTO the surface, X follows the stroke."""
163:     z = _scale(normal, -1)
164:     x = stroke_dir
165:     y = _cross(z, x)
166:     return Mat([[x[0], y[0], z[0], position[0]],
167:                 [x[1], y[1], z[1], position[1]],
168:                 [x[2], y[2], z[2], position[2]],
169:                 [0, 0, 0, 1]])
170: 
171: 
172: def base_pose(x, y, yaw):
173:     """Robot base pose in the car frame."""
174:     return transl(x, y, 0) * rotz(math.radians(yaw))
175: 
176: 
177: def raster_strokes(surf):
178:     """Zig-zag strokes: list of (start, end, direction), offset by STANDOFF."""
179:     o, u, v, n = surf["origin"], surf["u"], surf["v"], surf["normal"]
180:     n_strokes = max(1, int(math.ceil(surf["len_v"] / STEP)) + 1)
181:     strokes = []
182:     for k in range(n_strokes):
183:         dv = min(k * STEP, surf["len_v"])
184:         base = _add(_add(o, _scale(v, dv)), _scale(n, STANDOFF))
185:         p_start = base
186:         p_end = _add(base, _scale(u, surf["len_u"]))
187:         direction = u
188:         if k % 2 == 1:
189:             p_start, p_end = p_end, p_start
190:             direction = _scale(u, -1)
191:         strokes.append((p_start, p_end, direction))
192:     return strokes
193: 
194: 
195: def stroke_bands(surf):
196:     """(lo, hi) offsets along v, per stroke, of the area each stroke paints. Bands never overlap
197:     and never leave the surface, so the drawn stripes never touch each other."""
198:     n_strokes = max(1, int(math.ceil(surf["len_v"] / STEP)) + 1)
199:     out, prev_hi = [], 0.0
200:     for k in range(n_strokes):
201:         c = min(k * STEP, surf["len_v"])
202:         lo = max(c - STEP / 2.0, 0.0, prev_hi)
203:         hi = max(lo, min(c + STEP / 2.0, surf["len_v"]))
204:         out.append((lo - c, hi - c))
205:         prev_hi = max(prev_hi, hi)
206:     return out
207: 
208: 
209: def jl(j):
210:     """Joint Mat/list -> list of 6 floats (empty list if invalid)."""
211:     try:
212:         v = j.list() if hasattr(j, "list") else list(j)
213:     except Exception:
214:         return []
215:     v = [float(x) for x in v]
216:     return v[:6] if len(v) >= 6 else []
217: 
218: 
219: def jdist(s1, s2):
220:     return max(abs(p - q) for p, q in zip(s1, s2))
221: 
222: 
223: def box_tris(cx, cy, cz, sx, sy, sz):
224:     x0, x1 = cx - sx / 2.0, cx + sx / 2.0
225:     y0, y1 = cy - sy / 2.0, cy + sy / 2.0
226:     z0, z1 = cz - sz / 2.0, cz + sz / 2.0
227:     v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
228:          (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
229:     faces = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
230:     pts = []
231:     for f in faces:
232:         a, b, c, d = [list(v[i]) for i in f]
233:         pts += [a, b, c, a, c, d]
234:     return pts
235: 
236: 
237: 
238: # ----------------------------------------------------------------------
239: # 2b. FIT THE SURFACE TABLE TO THE REAL CAR MESH (reads the STL file)
240: # ----------------------------------------------------------------------
241: def read_stl(path):
242:     with open(path, "rb") as f:
243:         data = f.read()
244:     tris = []
245:     if len(data) >= 84:
246:         n = struct.unpack("<I", data[80:84])[0]
247:         if 84 + 50 * n == len(data):                        # binary STL
248:             for i in range(n):
249:                 v = struct.unpack("<9f", data[96 + 50 * i:132 + 50 * i])
250:                 tris.append(((v[0], v[1], v[2]), (v[3], v[4], v[5]), (v[6], v[7], v[8])))
The above content does NOT show the entire file contents. If you need to view any lines of the file which were not shown to complete your task, call this tool again to view those lines.
