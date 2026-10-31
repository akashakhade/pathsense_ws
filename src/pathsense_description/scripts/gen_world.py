#!/usr/bin/env python3
"""
Generate a realistic, navigation-ready Gazebo (gz-sim / Harmonic) world of the
BVCOE Lavale college FIRST FLOOR from the base geometry world.

Inputs : base SDF (walls/stairs/pillars/amphitheatre traced from the floor-plan photo)
Outputs: college_first_floor.sdf      - furnished world
         college_rooms.yaml           - rooms, doors, nav goals (for the floor-plan graph)
         maps/college_first_floor.*   - Nav2 map with furniture (what the robot really sees)
         maps/college_first_floor_walls.* - walls-only map (= "floor-plan map" baseline)

Usage: python3 gen_world.py base.sdf out_dir
"""
import sys, os, re, math, json, random
import xml.etree.ElementTree as ET
import numpy as np
from scipy import ndimage

random.seed(7)
BASE, OUT = sys.argv[1], sys.argv[2]
os.makedirs(os.path.join(OUT, "maps"), exist_ok=True)

# ---------------------------------------------------------------- settings
WALL_H = 3.0          # floor-to-ceiling (college floors are ~3.0-3.3 m)
WALL_T = 0.15
DOOR_H = 2.10
SILL_H, HEAD_H = 0.90, 2.10
WIN_W, WIN_PITCH = 1.50, 3.0
MAIN_GATE_CLOSED = True     # glass gate at "Main Entry Gate No.1" (keeps robot inside)
MAP_RES = 0.05
MAP_ZMIN, MAP_ZMAX = 0.03, 1.50   # obstacle slice used for the Nav2 map

# ---------------------------------------------------------------- materials
MAT = {
    "wall":      (0.93, 0.91, 0.85), "skirting": (0.30, 0.28, 0.26),
    "frame":     (0.42, 0.27, 0.16), "door":     (0.55, 0.36, 0.20),
    "glass":     (0.62, 0.78, 0.88), "alu":      (0.75, 0.76, 0.78),
    "wood":      (0.66, 0.47, 0.28), "wood_dk":  (0.45, 0.30, 0.18),
    "laminate":  (0.85, 0.80, 0.70), "steel":    (0.55, 0.57, 0.60),
    "steel_lt":  (0.80, 0.81, 0.83), "black":    (0.08, 0.08, 0.09),
    "blue_seat": (0.18, 0.28, 0.55), "red_seat": (0.55, 0.15, 0.13),
    "grey_fab":  (0.35, 0.36, 0.38), "white":    (0.95, 0.95, 0.95),
    "ceramic":   (0.97, 0.97, 0.96), "board":    (0.12, 0.25, 0.18),
    "whiteboard":(0.97, 0.97, 0.97), "partition":(0.55, 0.62, 0.70),
    "lab_top":   (0.20, 0.22, 0.24), "cabinet":  (0.88, 0.86, 0.80),
    "green":     (0.20, 0.50, 0.25), "plant":    (0.18, 0.45, 0.18),
    "pot":       (0.60, 0.33, 0.20), "yellow":   (0.95, 0.78, 0.10),
    "stair":     (0.70, 0.68, 0.64), "pillar":   (0.86, 0.85, 0.82),
    "lift":      (0.62, 0.64, 0.68), "duct":     (0.74, 0.73, 0.70),
    "tier":      (0.78, 0.74, 0.68), "stage":    (0.60, 0.62, 0.66),
    "notice":    (0.55, 0.38, 0.22), "paper":    (0.96, 0.96, 0.90),
    "sofa":      (0.30, 0.22, 0.18), "carpet":   (0.45, 0.20, 0.20),
}
FLOOR = {  # floor finish per room type
    "corridor": (0.82, 0.80, 0.76), "classroom": (0.80, 0.78, 0.72),
    "lab": (0.74, 0.76, 0.78), "office": (0.84, 0.80, 0.72),
    "restroom": (0.70, 0.74, 0.76), "seminar": (0.50, 0.24, 0.22),
    "stair": (0.70, 0.68, 0.64),
}

# ---------------------------------------------------------------- room catalogue (FF-xxx legend on the plan)
ROOMS = {
 "101": ("Training & Placement Office", "office"),
 "102": ("Hard Water Laboratory", "lab_bench"),
 "103": ("Staff Room Computer", "staffroom"),
 "104": ("Class Room Computer", "classroom"),
 "105": ("Rest Room for Ladies (Accessible)", "restroom_acc"),
 "106": ("Rest Room for Ladies", "restroom"),
 "107": ("E&TC Department Lobby", "dept_lobby"),
 "107A": ("Mobile Comm. & Broadband Comm. Lab", "lab_elec"),
 "107B": ("Department Office E&TC", "office"),
 "107D": ("Audio Video & Microwave Engg. Lab", "lab_elec"),
 "107E": ("Analog Circuit Lab / Computer Class Room", "lab_elec"),
 "108": ("Computer Class Room", "lab_comp"),
 "109": ("HOD Cabin Computer", "cabin"),
 "110": ("Programming Laboratory", "lab_comp"),
 "111": ("Operating System Laboratory", "lab_comp"),
 "112": ("Gents Rest Room", "restroom"),
 "113": ("Computer Graphics Lab", "lab_comp"),
 "114": ("CAD Laboratory Civil", "lab_comp"),
 "115": ("Class Room Civil", "classroom"),
 "116": ("Engineering Mechanics / Transportation Engg. Lab", "lab_bench"),
 "117": ("Basic Civil / Surveying Labs", "lab_bench"),
 "118": ("Simulation Laboratory", "lab_comp"),
 "119": ("Ladies Rest Room", "restroom"),
 "120": ("Geotechnical / Engineering Geology Lab", "lab_bench"),
 "121": ("Class Room Civil", "classroom"),
 "122": ("Class Room Electronics", "classroom"),
 "123": ("Digital Circuit Laboratory", "lab_elec"),
 "124": ("Fire & Security Association of India", "office"),
 "125": ("Analog Communication Laboratory", "lab_elec"),
 "126": ("Gents Rest Room", "restroom"),
 "127": ("Rest Room for Gents (Accessible)", "restroom_acc"),
 "128": ("Class Room E&TC", "classroom"),
 "129": ("Class Room", "classroom"),
 "130": ("Seminar Hall", "seminar"),
 "131": ("Gents Rest Room for Staff", "restroom"),
 "132": ("Ladies Rest Room for Staff", "restroom"),
 "133": ("HOD Cabin Civil", "cabin"),
 "134": ("Department Office Civil", "office"),
 "135": ("Faculty Room", "staffroom"),
 "136": ("Faculty Room", "staffroom"),
 "137": ("Faculty Room", "staffroom"),
 "138": ("Students & Exam Section", "exam"),
 "139": ("Room 139", "office"),
 "Admin Office": ("Admin Office", "admin"),
 "Admin Annex": ("Admin Office (Annex)", "admin"),
 "Board Room": ("Board Room", "boardroom"),
 "Principal Cabin": ("Principal Cabin", "principal"),
}
for i in range(1, 7):
    ROOMS[f"Stair 0{i}"] = (f"Staircase No. {i}", "stair")

def floor_kind(t):
    return {"classroom": "classroom", "lab_bench": "lab", "lab_comp": "lab", "lab_elec": "lab",
            "restroom": "restroom", "restroom_acc": "restroom", "seminar": "seminar",
            "stair": "stair"}.get(t, "office")

# ---------------------------------------------------------------- parse base world
root = ET.parse(BASE).getroot()
link = root.find(".//model[@name='first_floor']/link")
boxes = []
for c in link.findall("collision"):
    p = [float(v) for v in c.find("pose").text.split()]
    s = [float(v) for v in c.find("geometry/box/size").text.split()]
    boxes.append(dict(name=c.get("name")[:-2], x=p[0], y=p[1], z=p[2], yaw=p[5], sx=s[0], sy=s[1], sz=s[2]))

def seg_of(b):
    """wall box -> (orient, coord, a, b, thickness)"""
    if abs(math.sin(b["yaw"])) < 0.5:
        return "H", b["y"], b["x"] - b["sx"] / 2, b["x"] + b["sx"] / 2, b["sy"]
    return "V", b["x"], b["y"] - b["sx"] / 2, b["y"] + b["sx"] / 2, b["sy"]

# rooms from w_<room>_<side><i>
room_sides = {}
for b in boxes:
    m = re.match(r"w_(.+)_([NSEW])\d$", b["name"])
    if m:
        room_sides.setdefault(m.group(1), {}).setdefault(m.group(2), []).append(seg_of(b))

rooms = {}
for rid, sides in room_sides.items():
    x1 = sides["W"][0][1]; x2 = sides["E"][0][1]; y1 = sides["S"][0][1]; y2 = sides["N"][0][1]
    rooms[rid] = dict(id=rid, rect=(min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)), doors=[])

# door openings = gaps between consecutive wall segments on one side of a room
doors = []
for rid, sides in room_sides.items():
    for side, segs in sides.items():
        segs = sorted(segs, key=lambda s: s[2])
        for s0, s1 in zip(segs, segs[1:]):
            g0, g1 = s0[3], s1[2]
            if 0.6 <= g1 - g0 <= 3.0:
                c = s0[1]; mid = (g0 + g1) / 2
                pos = (mid, c) if s0[0] == "H" else (c, mid)
                dup = next((d for d in doors if math.dist(d["pos"], pos) < 0.3), None)
                if dup:
                    dup["rooms"].append((rid, side)); continue
                doors.append(dict(orient=s0[0], c=c, a=g0, b=g1, pos=pos, rooms=[(rid, side)]))
# main entrance (gap in the west outline between outline_5 and outline_6)
o5 = next(b for b in boxes if b["name"] == "outline_5_0"); o6 = next(b for b in boxes if b["name"] == "outline_6_0")
ga, gb = o6["y"] + o6["sx"] / 2, o5["y"] - o5["sx"] / 2
MAIN_GATE = dict(orient="V", c=o5["x"], a=ga, b=gb, pos=(o5["x"], (ga + gb) / 2), rooms=[], main=True)
doors.append(MAIN_GATE)
for i, d in enumerate(doors):
    d["id"] = f"D{i+1:02d}"
    for rid, side in d["rooms"]:
        rooms[rid]["doors"].append(d)

# ---------------------------------------------------------------- SDF writer helpers
COLL = []   # every collision box: (x,y,yaw,sx,sy,z0,z1,layer)

def mat_xml(name, alpha=None):
    r, g, b = MAT.get(name, name) if isinstance(name, str) else name
    tr = f"<transparency>{1-alpha:.2f}</transparency>" if alpha is not None else ""
    return (f"<material><ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient><diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>"
            f"<specular>0.05 0.05 0.05 1</specular></material>", tr)

class Model:
    def __init__(self, name):
        self.name, self.parts, self.n = name, [], 0
    def box(self, x, y, z, sx, sy, sz, mat, yaw=0.0, collide=True, visual=True, alpha=None, layer="furn"):
        self.n += 1; k = f"b{self.n}"
        pose = f"<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>"
        geo = f"<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>"
        if collide:
            self.parts.append(f'<collision name="{k}_c">{pose}{geo}</collision>')
            COLL.append((x, y, yaw, sx, sy, z - sz / 2, z + sz / 2, layer))
        if visual:
            m, tr = mat_xml(mat, alpha)
            cs = "<cast_shadows>false</cast_shadows>" if alpha else ""
            self.parts.append(f'<visual name="{k}_v">{pose}{geo}{m}{tr}{cs}</visual>')
    def xml(self):
        if not self.parts: return ""
        return (f'<model name="{self.name}"><static>true</static><link name="link">'
                + "\n".join(self.parts) + "</link></model>")

MODELS = []

# ---------------------------------------------------------------- 1. walls (merged, doors cut, lintels, windows)
lines = {}
for b in boxes:
    if b["name"].startswith(("w_", "outline_")):
        o, c, a, bb, t = seg_of(b)
        lines.setdefault((o, round(c, 2)), []).append([a, bb])

def union(iv):
    iv = sorted(iv); out = []
    for a, b in iv:
        if out and a <= out[-1][1] + 0.02: out[-1][1] = max(out[-1][1], b)
        else: out.append([a, b])
    return out

def subtract(iv, a, b):
    out = []
    for s, e in iv:
        if e <= a or s >= b: out.append([s, e]); continue
        if s < a: out.append([s, a])
        if e > b: out.append([b, e])
    return [p for p in out if p[1] - p[0] > 0.01]

for k in lines:
    lines[k] = union(lines[k])
    for d in doors:
        if d["orient"] == k[0] and abs(d["c"] - k[1]) < 0.12:
            lines[k] = subtract(lines[k], d["a"], d["b"])

# raster helpers (shared with map)
X0, Y0, XW, YH = -10.0, -32.0, 85.0, 62.0
NX, NY = int(XW / MAP_RES), int(YH / MAP_RES)
JJ, II = np.mgrid[0:NY, 0:NX]
GX = X0 + (II + 0.5) * MAP_RES; GY = Y0 + (JJ + 0.5) * MAP_RES

def raster(coll, zmin=MAP_ZMIN, zmax=MAP_ZMAX, layers=None):
    occ = np.zeros((NY, NX), bool)
    for (x, y, yaw, sx, sy, z0, z1, layer) in coll:
        if z1 < zmin or z0 > zmax: continue
        if layers and layer not in layers: continue
        r = math.hypot(sx, sy) / 2 + 0.1
        i0, i1 = max(int((x - r - X0) / MAP_RES), 0), min(int((x + r - X0) / MAP_RES) + 1, NX)
        j0, j1 = max(int((y - r - Y0) / MAP_RES), 0), min(int((y + r - Y0) / MAP_RES) + 1, NY)
        if i0 >= i1 or j0 >= j1: continue
        dx, dy = GX[j0:j1, i0:i1] - x, GY[j0:j1, i0:i1] - y
        c, s = math.cos(yaw), math.sin(yaw)
        u, v = c * dx + s * dy, -s * dx + c * dy
        occ[j0:j1, i0:i1] |= (np.abs(u) <= sx / 2 + 1e-3) & (np.abs(v) <= sy / 2 + 1e-3)
    return occ

def cell(x, y): return int((y - Y0) / MAP_RES), int((x - X0) / MAP_RES)

# provisional raster of plain walls + fixed blocks, to find the outside of the building
tmp = []
for (o, c), iv in lines.items():
    for a, b in iv:
        if o == "H": tmp.append(((a + b) / 2, c, 0, b - a, WALL_T, 0, 3, "w"))
        else: tmp.append((c, (a + b) / 2, 0, WALL_T, b - a, 0, 3, "w"))
for b in boxes:
    if b["name"].startswith(("lift", "duct", "filler")):
        tmp.append((b["x"], b["y"], 0, b["sx"], b["sy"], 0, 3, "w"))
g = MAIN_GATE
tmp.append((g["c"], g["pos"][1], 0, WALL_T, g["b"] - g["a"], 0, 3, "w"))
wall_occ = raster(tmp)
lab, _ = ndimage.label(~wall_occ)
OUTSIDE = lab == lab[0, 0]

def is_out(x, y):
    j, i = cell(x, y)
    return 0 <= j < NY and 0 <= i < NX and OUTSIDE[j, i]
def is_wall(x, y):
    j, i = cell(x, y)
    return 0 <= j < NY and 0 <= i < NX and wall_occ[j, i]

walls = Model("walls")
def wall_piece(o, c, a, b, z0, z1, mat="wall", skirt=True, t=WALL_T):
    L = b - a
    if L < 0.01: return
    if o == "H": x, y, sx, sy = (a + b) / 2, c, L, t
    else: x, y, sx, sy = c, (a + b) / 2, t, L
    walls.box(x, y, (z0 + z1) / 2, sx, sy, z1 - z0, mat, layer="wall")
    if skirt and z0 == 0:
        for sgn in (-1, 1):
            if o == "H": walls.box(x, y + sgn * (t / 2 + 0.006), 0.05, L, 0.012, 0.10, "skirting", collide=False)
            else: walls.box(x + sgn * (t / 2 + 0.006), y, 0.05, 0.012, L, 0.10, "skirting", collide=False)

def window_ok(o, c, a, b):
    """window allowed if one side is outside, the other side is inside and no wall abuts it"""
    out_side = None
    for sgn in (-1, 1):
        pts = [((a + b) / 2, c + sgn * 0.45) if o == "H" else (c + sgn * 0.45, (a + b) / 2)]
        if all(is_out(*p) for p in pts): out_side = sgn
    if out_side is None: return False
    t = a - 0.25
    while t <= b + 0.25:
        p = (t, c - out_side * 0.25) if o == "H" else (c - out_side * 0.25, t)
        q = (t, c - out_side * 0.45) if o == "H" else (c - out_side * 0.45, t)
        if is_wall(*p) or is_out(*q): return False
        t += 0.05
    return True

N_WIN = 0
for (o, c), iv in lines.items():
    for a, b in iv:
        L = b - a
        wins = []
        if L >= WIN_W + 1.2:
            n = int((L - 1.0) // WIN_PITCH) + 1
            span = (n - 1) * WIN_PITCH
            for k in range(n):
                m = (a + b) / 2 - span / 2 + k * WIN_PITCH
                wa, wb = m - WIN_W / 2, m + WIN_W / 2
                if wa > a + 0.5 and wb < b - 0.5 and window_ok(o, c, wa, wb):
                    wins.append((wa, wb))
        cur = a
        for wa, wb in wins:
            wall_piece(o, c, cur, wa, 0, WALL_H)
            wall_piece(o, c, wa, wb, 0, SILL_H)
            wall_piece(o, c, wa, wb, HEAD_H, WALL_H, skirt=False)
            # glass + aluminium frame
            mx, my = ((wa + wb) / 2, c) if o == "H" else (c, (wa + wb) / 2)
            gx, gy = (wb - wa, 0.02) if o == "H" else (0.02, wb - wa)
            walls.box(mx, my, (SILL_H + HEAD_H) / 2, gx, gy, HEAD_H - SILL_H, "glass", alpha=0.35, layer="glass")
            fx, fy = (wb - wa, WALL_T + 0.02) if o == "H" else (WALL_T + 0.02, wb - wa)
            for zz in (SILL_H + 0.02, HEAD_H - 0.02, (SILL_H + HEAD_H) / 2):
                walls.box(mx, my, zz, fx, fy, 0.04, "alu", collide=False)
            mxx, myy = (0.04, WALL_T + 0.02) if o == "H" else (WALL_T + 0.02, 0.04)
            walls.box(mx, my, (SILL_H + HEAD_H) / 2, mxx, myy, HEAD_H - SILL_H, "alu", collide=False)
            cur = wb; N_WIN += 1
        wall_piece(o, c, cur, b, 0, WALL_H)

# lintels + door frames + door leaves
doorm = Model("doors")
for d in doors:
    o, c, a, b = d["orient"], d["c"], d["a"], d["b"]
    W = b - a; mid = (a + b) / 2
    top = DOOR_H if not d.get("main") else 2.4
    wall_piece(o, c, a, b, top, WALL_H, skirt=False)
    def P(t, n):  # along-wall t, normal offset n -> world
        return (t, c + n) if o == "H" else (c + n, t)
    yaw_wall = 0.0 if o == "H" else math.pi / 2
    # frames (visual)
    for t in (a + 0.03, b - 0.03):
        x, y = P(t, 0); doorm.box(x, y, top / 2, 0.06, WALL_T + 0.03, top, "frame", yaw=yaw_wall, collide=False)
    x, y = P(mid, 0); doorm.box(x, y, top + 0.03, W, WALL_T + 0.03, 0.06, "frame", yaw=yaw_wall, collide=False)
    if d.get("main"):
        if MAIN_GATE_CLOSED:   # closed glass double gate
            for t0, t1 in ((a + 0.06, mid), (mid, b - 0.06)):
                x, y = P((t0 + t1) / 2, 0)
                doorm.box(x, y, 1.2, t1 - t0 - 0.01, 0.03, 2.38, "glass", yaw=yaw_wall, alpha=0.4, layer="door")
                doorm.box(x, y, 1.2, t1 - t0 - 0.01, 0.05, 0.05, "steel", yaw=yaw_wall, collide=False)
        continue
    # leaf: opened 90 deg into the first room listed, hinged on the jamb nearer to a room corner
    rid, side = d["rooms"][0]
    rx1, ry1, rx2, ry2 = rooms[rid]["rect"]
    rc = ((rx1 + rx2) / 2, (ry1 + ry2) / 2)
    inward = 1 if ((rc[1] - c) if o == "H" else (rc[0] - c)) > 0 else -1
    lo, hi = (rx1, rx2) if o == "H" else (ry1, ry2)
    hinge = a if (a - lo) < (hi - b) else b
    lw = min(W - 0.08, 1.0)
    tsgn = 1 if hinge == a else -1
    x, y = P(hinge + tsgn * 0.06, inward * (WALL_T / 2 + lw / 2 + 0.01))
    doorm.box(x, y, 1.03, 0.04, lw, 2.04, "door", yaw=yaw_wall, layer="door")
    d["leaf"] = (x, y)
    d["inward"] = inward
MODELS += [walls, doorm]

# ---------------------------------------------------------------- 2. fixed building elements
fixed = Model("building_fixtures")
for b in boxes:
    n = b["name"]
    if n.startswith(("w_", "outline_")): continue
    if n.startswith(("lift", "duct", "filler")):
        fixed.box(b["x"], b["y"], WALL_H / 2, b["sx"], b["sy"], WALL_H, "lift" if n.startswith("lift") else "duct", layer="wall")
    elif n.startswith("pillar"):
        fixed.box(b["x"], b["y"], WALL_H / 2, 0.40, 0.40, WALL_H, "pillar", layer="wall")
    elif n.startswith(("step", "landing")):
        fixed.box(b["x"], b["y"], b["z"], b["sx"], b["sy"], b["sz"], "stair", yaw=b["yaw"], layer="stair")
        if n.startswith("step"):   # yellow anti-slip nosing on the leading edge
            st = re.match(r"step_(Stair\d+)_(\d)", n)
            nxt = next((q for q in boxes if q["name"] == f"step_{st.group(1)}_{int(st.group(2))+1}"), None) \
                  or next(q for q in boxes if q["name"] == f"landing_{st.group(1)}")
            if b["sx"] > b["sy"]:
                sgn = 1 if nxt["y"] < b["y"] else -1
                fixed.box(b["x"], b["y"] + sgn * (b["sy"] / 2 - 0.03), b["sz"] + 0.002, b["sx"], 0.05, 0.004, "yellow", collide=False)
            else:
                sgn = 1 if nxt["x"] < b["x"] else -1
                fixed.box(b["x"] + sgn * (b["sx"] / 2 - 0.03), b["y"], b["sz"] + 0.002, 0.05, b["sy"], 0.004, "yellow", collide=False)
    elif n.startswith("tier"):
        fixed.box(b["x"], b["y"], b["z"], b["sx"], b["sy"], b["sz"], "tier", yaw=b["yaw"], layer="stair")
    elif n == "stage":
        fixed.box(b["x"], b["y"], b["z"], b["sx"], b["sy"], b["sz"], "stage", layer="stair")
MODELS.append(fixed)

# lift landing doors (visual) on faces that open to reachable floor
for b in boxes:
    if not b["name"].startswith("lift"): continue
    for o, sgn in (("x", 1), ("x", -1), ("y", 1), ("y", -1)):
        if o == "x": px, py, L = b["x"] + sgn * (b["sx"] / 2 + 0.5), b["y"], b["sy"]
        else: px, py, L = b["x"], b["y"] + sgn * (b["sy"] / 2 + 0.5), b["sx"]
        if is_wall(px, py) or is_out(px, py) or L < 1.4: continue
        n = 2 if L > 3.4 else 1
        for k in range(n):
            off = (k - (n - 1) / 2) * (L / n)
            if o == "x":
                fixed.box(b["x"] + sgn * (b["sx"] / 2 + 0.01), b["y"] + off, 1.05, 0.02, 1.0, 2.1, "steel_lt", collide=False)
                fixed.box(b["x"] + sgn * (b["sx"] / 2 + 0.02), b["y"] + off, 1.05, 0.02, 0.02, 2.1, "black", collide=False)
            else:
                fixed.box(b["x"] + off, b["y"] + sgn * (b["sy"] / 2 + 0.01), 1.05, 1.0, 0.02, 2.1, "steel_lt", collide=False)
                fixed.box(b["x"] + off, b["y"] + sgn * (b["sy"] / 2 + 0.02), 1.05, 0.02, 0.02, 2.1, "black", collide=False)

# ---------------------------------------------------------------- 3. furniture library
class Item:
    """primitives in local frame: +y = front (the side a person uses / faces)"""
    def __init__(self, w, d):
        self.w, self.d, self.prims = w, d, []
    def b(self, x, y, z, sx, sy, sz, mat, collide=True, alpha=None):
        self.prims.append((x, y, z, sx, sy, sz, mat, collide, alpha)); return self

def desk_bench(w=1.5):
    it = Item(w, 0.9)
    it.b(0, 0.25, 0.735, w, 0.40, 0.03, "wood")
    it.b(0, 0.27, 0.60, w - 0.08, 0.32, 0.02, "steel", collide=False)
    it.b(0, -0.25, 0.44, w, 0.28, 0.03, "wood")
    it.b(0, -0.39, 0.62, w, 0.04, 0.30, "wood", collide=False)   # backrest
    for sx in (-1, 1):
        it.b(sx * (w / 2 - 0.03), 0.0, 0.37, 0.05, 0.86, 0.74, "steel")
    return it

def table(w, d, h=0.75, top="laminate", leg="steel"):
    it = Item(w, d)
    it.b(0, 0, h - 0.015, w, d, 0.03, top)
    for sx in (-1, 1):
        for sy in (-1, 1):
            it.b(sx * (w / 2 - 0.04), sy * (d / 2 - 0.04), (h - 0.03) / 2, 0.04, 0.04, h - 0.03, leg)
    it.b(0, -d / 2 + 0.03, h - 0.15, w - 0.1, 0.02, 0.2, leg, collide=False)  # modesty panel
    return it

def chair(seat="blue_seat"):
    it = Item(0.46, 0.46)
    it.b(0, 0, 0.45, 0.44, 0.44, 0.04, seat)
    it.b(0, -0.21, 0.72, 0.42, 0.03, 0.46, seat)
    for sx in (-1, 1):
        for sy in (-1, 1):
            it.b(sx * 0.19, sy * 0.19, 0.22, 0.03, 0.03, 0.44, "steel")
    return it

def office_chair():
    it = Item(0.62, 0.62)
    it.b(0, 0, 0.48, 0.48, 0.48, 0.07, "black")
    it.b(0, -0.24, 0.82, 0.46, 0.05, 0.60, "black")
    it.b(0, 0, 0.27, 0.06, 0.06, 0.40, "steel")
    it.b(0, 0, 0.05, 0.60, 0.06, 0.05, "black"); it.b(0, 0, 0.05, 0.06, 0.60, 0.05, "black")
    return it

def stool():
    it = Item(0.38, 0.38)
    it.b(0, 0, 0.63, 0.36, 0.36, 0.03, "wood")
    for sx in (-1, 1):
        for sy in (-1, 1):
            it.b(sx * 0.15, sy * 0.15, 0.31, 0.03, 0.03, 0.62, "steel")
    return it

def almirah(w=0.9):
    it = Item(w, 0.48)
    it.b(0, 0, 0.975, w, 0.48, 1.95, "steel")
    it.b(0, 0.245, 0.975, 0.01, 0.01, 1.9, "black", collide=False)
    it.b(0.05, 0.25, 1.0, 0.03, 0.02, 0.18, "steel_lt", collide=False)
    return it

def casework(w, d=0.6, h=0.9, top="lab_top"):
    it = Item(w, d)
    it.b(0, 0, (h - 0.03) / 2, w, d - 0.04, h - 0.03, "cabinet")
    it.b(0, 0.02, h - 0.015, w, d, 0.03, top)
    it.b(0, d / 2 - 0.02, 0.05, w, 0.01, 0.1, "black", collide=False)
    return it

def computer_on(it, x, y, h):
    it.b(x, y - 0.12, h + 0.21, 0.52, 0.04, 0.32, "black", collide=False)
    it.b(x, y - 0.14, h + 0.03, 0.18, 0.15, 0.06, "black", collide=False)
    it.b(x, y + 0.08, h + 0.01, 0.42, 0.14, 0.02, "grey_fab", collide=False)
    it.b(x + 0.28, y - 0.0, 0.23, 0.19, 0.42, 0.42, "black")   # CPU under desk -> real low obstacle

def equipment_on(it, x, y, h):
    w = random.choice([0.32, 0.4, 0.28]); hh = random.choice([0.16, 0.2, 0.12])
    it.b(x, y - 0.08, h + hh / 2, w, 0.3, hh, random.choice(["steel_lt", "black", "grey_fab"]), collide=False)

def sofa(w=1.8):
    it = Item(w, 0.85)
    it.b(0, 0.05, 0.22, w - 0.2, 0.75, 0.44, "sofa")
    it.b(0, -0.33, 0.45, w, 0.2, 0.9, "sofa")
    for sx in (-1, 1): it.b(sx * (w / 2 - 0.08), 0.0, 0.32, 0.16, 0.85, 0.64, "sofa")
    return it

def counter(w, d=0.6, h=1.05):
    it = Item(w, d)
    it.b(0, 0, h / 2, w, d, h, "wood_dk")
    it.b(0, 0.05, h + 0.015, w + 0.04, d + 0.1, 0.03, "laminate")
    return it

def wc_stall():
    it = Item(1.05, 1.5)
    for sx in (-1, 1): it.b(sx * 0.5, 0, 1.05, 0.04, 1.5, 1.8, "partition")
    it.b(0, 0.73, 1.92, 1.0, 0.04, 0.06, "alu", collide=False)
    it.b(0, -0.4, 0.2, 0.38, 0.6, 0.40, "ceramic")
    it.b(0, -0.68, 0.6, 0.45, 0.16, 0.4, "ceramic", collide=False)
    it.b(0.35, 0.33, 1.03, 0.04, 0.85, 2.0, "partition")   # stall door, left open
    return it

def vanity(w):
    it = Item(w, 0.55)
    it.b(0, 0, 0.42, w, 0.5, 0.84, "cabinet")
    it.b(0, 0.02, 0.855, w, 0.55, 0.03, "lab_top")
    n = max(1, int(w // 0.9))
    for k in range(n):
        x = (k - (n - 1) / 2) * (w / n)
        it.b(x, 0.0, 0.875, 0.42, 0.32, 0.02, "ceramic", collide=False)
        it.b(x, -0.22, 0.95, 0.04, 0.1, 0.16, "steel_lt", collide=False)
        it.b(x, -0.27, 1.55, 0.6, 0.01, 0.8, "glass", collide=False, alpha=0.8)
    return it

def podium():
    it = Item(0.6, 0.5); it.b(0, 0, 0.55, 0.6, 0.5, 1.1, "wood_dk"); return it

def water_cooler():
    it = Item(0.6, 0.5); it.b(0, 0, 0.67, 0.6, 0.5, 1.35, "steel_lt")
    it.b(0, 0.25, 0.95, 0.3, 0.02, 0.12, "black", collide=False); return it

def dustbin():
    it = Item(0.38, 0.38); it.b(0, 0, 0.32, 0.36, 0.36, 0.64, "green"); return it

def plant():
    it = Item(0.45, 0.45); it.b(0, 0, 0.22, 0.42, 0.42, 0.44, "pot")
    it.b(0, 0, 0.85, 0.55, 0.55, 0.85, "plant", collide=False); return it

def corridor_bench(w=1.8):
    it = Item(w, 0.45); it.b(0, 0, 0.44, w, 0.42, 0.04, "steel")
    it.b(0, -0.19, 0.65, w, 0.03, 0.4, "steel", collide=False)
    for sx in (-1, 1): it.b(sx * (w / 2 - 0.08), 0, 0.21, 0.05, 0.42, 0.42, "steel")
    return it

# ---------------------------------------------------------------- placement engine
class Placer:
    def __init__(self, rid, rect, clear, fixed_obs, model):
        self.rid, self.rect, self.clear, self.model = rid, rect, clear, model
        self.taken = list(fixed_obs)
        self.base = len(self.taken)
    @staticmethod
    def hit(a, b, pad=0.0):
        return not (a[2] + pad <= b[0] or b[2] + pad <= a[0] or a[3] + pad <= b[1] or b[3] + pad <= a[1])
    def fp(self, it, x, y, yaw):
        w, d = (it.w, it.d) if yaw % 180 == 0 else (it.d, it.w)
        return (x - w / 2, y - d / 2, x + w / 2, y + d / 2)
    def fits(self, it, x, y, yaw, pad=0.04):
        f = self.fp(it, x, y, yaw); r = self.rect
        if f[0] < r[0] - 1e-6 or f[1] < r[1] - 1e-6 or f[2] > r[2] + 1e-6 or f[3] > r[3] + 1e-6: return False
        if any(self.hit(f, c) for c in self.clear): return False
        return not any(self.hit(f, t, pad) for t in self.taken)
    def put(self, it, x, y, yaw, force=False):
        if not force and not self.fits(it, x, y, yaw): return False
        a = math.radians(yaw); c, s = math.cos(a), math.sin(a)
        for (lx, ly, z, sx, sy, sz, mat, col, alpha) in it.prims:
            self.model.box(x + c * lx - s * ly, y + s * lx + c * ly, z, sx, sy, sz, mat, yaw=a, collide=col, alpha=alpha)
        self.taken.append(self.fp(it, x, y, yaw))
        return True
    # wall-facing helpers: front of item points into the room
    def against(self, side, it, t, gap=0.0):
        x1, y1, x2, y2 = self.rect
        if side == "S": return t, y1 + it.d / 2 + gap, 0
        if side == "N": return t, y2 - it.d / 2 - gap, 180
        if side == "W": return x1 + it.d / 2 + gap, t, 270
        return x2 - it.d / 2 - gap, t, 90
    def along(self, side, maker, seat=None, seat_gap=0.55, step_extra=0.0, margin=0.1):
        x1, y1, x2, y2 = self.rect
        lo, hi = (x1, x2) if side in "NS" else (y1, y2)
        it = maker(); n = 0
        t = lo + it.w / 2 + margin
        while t + it.w / 2 <= hi - margin + 1e-6:
            x, y, yaw = self.against(side, it, t)
            if self.fits(it, x, y, yaw):
                self.put(it, x, y, yaw); n += 1
                if seat:
                    for sx_ in seat[1]:
                        a = math.radians(yaw); c, s = math.cos(a), math.sin(a)
                        lx, ly = sx_, it.d / 2 + seat_gap * 0.6 + 0.12
                        st = seat[0]()
                        self.put(st, x + c * lx - s * ly, y + s * lx + c * ly, (yaw + 180) % 360)
                t += it.w + step_extra
            else:
                t += 0.1
        return n

def facing_yaw(dx, dy):
    """yaw (deg, multiple of 90) so local +y points along (dx,dy)"""
    if abs(dx) > abs(dy): return 270 if dx > 0 else 90
    return 0 if dy > 0 else 180

# ---------------------------------------------------------------- room layouts
def side_len(rect, side):
    x1, y1, x2, y2 = rect
    return (x2 - x1) if side in "NS" else (y2 - y1)

def door_sides(R):
    return {s for d in R["doors"] for rid, s in d["rooms"] if rid == R["id"]}

def quiet_sides(R):
    """walls without a door, longest first"""
    ds = door_sides(R)
    return sorted([s for s in "NSEW" if s not in ds], key=lambda s: -side_len(R["inner"], s))

def front_side(R):
    """board/teacher end: the short end nearest the first door"""
    x1, y1, x2, y2 = R["inner"]
    d = R["doors"][0]["pos"] if R["doors"] else ((x1 + x2) / 2, (y1 + y2) / 2)
    if (x2 - x1) >= (y2 - y1):
        return "W" if abs(d[0] - x1) < abs(d[0] - x2) else "E"
    return "S" if abs(d[1] - y1) < abs(d[1] - y2) else "N"

def wall_board(P, side, w, kind="board"):
    x1, y1, x2, y2 = P.rect
    w = min(w, side_len(P.rect, side) - 1.0)
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    if side == "N": P.model.box(cx, y2 - 0.01, 1.5, w, 0.02, 1.2, kind, collide=False)
    if side == "S": P.model.box(cx, y1 + 0.01, 1.5, w, 0.02, 1.2, kind, collide=False)
    if side == "W": P.model.box(x1 + 0.01, cy, 1.5, 0.02, w, 1.2, kind, collide=False)
    if side == "E": P.model.box(x2 - 0.01, cy, 1.5, 0.02, w, 1.2, kind, collide=False)

def frame_axes(rect, front):
    """returns (fwd unit toward front wall, cross unit), front coordinate, back coordinate, cross range"""
    x1, y1, x2, y2 = rect
    if front == "N": return (0, 1), (1, 0), y2, y1, (x1, x2)
    if front == "S": return (0, -1), (1, 0), y1, y2, (x1, x2)
    if front == "E": return (1, 0), (0, 1), x2, x1, (y1, y2)
    return (-1, 0), (0, 1), x1, x2, (y1, y2)

def pt(front, f, c):
    """position given distance-from-front-wall f and cross coordinate c"""
    return {"N": lambda: (c, f), "S": lambda: (c, f), "E": lambda: (f, c), "W": lambda: (f, c)}[front]()

def layout_rows(P, R, front, unit_maker, row_pitch, front_gap, col_aisle, back_gap=0.6, behind=None):
    fwd, _, fc, bc, (c1, c2) = frame_axes(P.rect, front)
    yaw = facing_yaw(*fwd)
    it = unit_maker()
    cw = c2 - c1
    ncol = max(1, int((cw - 1.0 + col_aisle) // (it.w + col_aisle)))
    used = ncol * it.w + (ncol - 1) * col_aisle
    sgn = 1 if fc > bc else -1
    depth = abs(fc - bc)
    f = front_gap + it.d / 2
    while f + it.d / 2 <= depth - back_gap:
        for k in range(ncol):
            c = (c1 + c2) / 2 - used / 2 + it.w / 2 + k * (it.w + col_aisle)
            x, y = pt(front, fc - sgn * f, c)
            if P.put(unit_maker(), x, y, yaw) and behind:
                for off in behind[1]:
                    bx, by = pt(front, fc - sgn * (f + it.d / 2 + 0.38), c + off)
                    P.put(behind[0](), bx, by, yaw)
        f += row_pitch

def L_classroom(P, R):
    front = front_side(R)
    wall_board(P, front, 3.6)
    fwd, cross, fc, bc, (c1, c2) = frame_axes(P.rect, front)
    sgn = 1 if fc > bc else -1
    tx, ty = pt(front, fc - sgn * 1.1, c1 + (c2 - c1) * 0.72)
    yaw = facing_yaw(-fwd[0], -fwd[1])
    if P.put(table(1.2, 0.6, top="wood"), tx, ty, yaw):
        cx, cy = pt(front, fc - sgn * 0.45, c1 + (c2 - c1) * 0.72)
        P.put(chair("red_seat"), cx, cy, yaw)
    layout_rows(P, R, front, desk_bench, 1.0, 2.3, 0.8)
    for s in quiet_sides(R)[:1]:
        P.along(s, dustbin) if False else None

def L_lab_comp(P, R):
    front = front_side(R)
    wall_board(P, front, 2.4, "whiteboard")
    for s in "NSEW":
        if s == front: continue
        P.along(s, lambda: perim_desk(), seat=(office_chair, (-0.45, 0.45)), margin=0.05)
    # central double rows
    fwd, cross, fc, bc, (c1, c2) = frame_axes(P.rect, front)
    yaw = facing_yaw(*fwd)
    f = 2.2; sgn = 1 if fc > bc else -1
    depth = abs(fc - bc)
    while f < depth - 2.2:
        c = c1 + 2.4
        while c < c2 - 2.4:
            x, y = pt(front, fc - sgn * f, c)
            if P.put(lab_desk_pair(), x, y, yaw):
                for off in (-0.45, 0.45):
                    for dd, yw in ((-0.95, yaw), (0.95, (yaw + 180) % 360)):
                        bx, by = pt(front, fc - sgn * (f + dd), c + off)
                        P.put(office_chair(), bx, by, yw)
            c += 2.0
        f += 3.3

def perim_desk(w=1.8):
    it = table(w, 0.65, top="laminate")
    for x in (-0.45, 0.45): computer_on(it, x, 0.0, 0.75)
    return it

def lab_desk_pair(w=1.8):
    it = Item(w, 1.3)
    t = table(w, 1.3)
    it.prims += t.prims
    for x in (-0.45, 0.45):
        it.b(x, 0.15, 0.96, 0.52, 0.04, 0.32, "black", collide=False)
        it.b(x, -0.15, 0.96, 0.52, 0.04, 0.32, "black", collide=False)
        it.b(x + 0.3, 0.3, 0.23, 0.19, 0.42, 0.42, "black")
        it.b(x + 0.3, -0.3, 0.23, 0.19, 0.42, 0.42, "black")
    it.b(0, 0, 0.95, w, 0.02, 0.4, "grey_fab", collide=False)   # privacy divider
    return it

def L_lab_bench(P, R):
    qs = quiet_sides(R)
    for s in qs[:2]:
        def mk():
            it = casework(1.8)
            for x in (-0.5, 0.4): equipment_on(it, x, 0.1, 0.9)
            return it
        P.along(s, mk, margin=0.0)
    x1, y1, x2, y2 = P.rect
    long_x = (x2 - x1) >= (y2 - y1)
    # central islands
    iw, il = 1.2, 2.4
    if long_x:
        cy = (y1 + y2) / 2; x = x1 + 1.9 + il / 2
        while x + il / 2 < x2 - 1.6:
            isl = casework(il, iw, 0.9)
            for xx in (-0.7, 0.6): equipment_on(isl, xx, 0.1, 0.9)
            if P.put(isl, x, cy, 0):
                for xx in (-0.7, 0.0, 0.7):
                    P.put(stool(), x + xx, cy + 0.95, 0); P.put(stool(), x + xx, cy - 0.95, 0)
            x += il + 1.4
    else:
        cx = (x1 + x2) / 2; y = y1 + 1.9 + il / 2
        while y + il / 2 < y2 - 1.6:
            isl = casework(il, iw, 0.9)
            if P.put(isl, cx, y, 90):
                for yy in (-0.7, 0.0, 0.7):
                    P.put(stool(), cx + 0.95, y + yy, 0); P.put(stool(), cx - 0.95, y + yy, 0)
            y += il + 1.4
    for s in qs[2:3]:
        P.along(s, almirah)

def L_lab_elec(P, R):
    front = front_side(R)
    wall_board(P, front, 2.4, "whiteboard")
    def bench():
        it = table(1.8, 0.75, top="lab_top")
        for x in (-0.55, 0.0, 0.55): equipment_on(it, x, 0.0, 0.75)
        it.b(0, -0.36, 1.05, 1.8, 0.03, 0.55, "grey_fab", collide=False)   # instrument shelf back panel
        return it
    for s in "NSEW":
        if s != front:
            P.along(s, bench, seat=(stool, (-0.45, 0.45)), margin=0.05)
    # one central double-sided bench row if the room is wide enough
    fwd, cross, fc, bc, (c1, c2) = frame_axes(P.rect, front)
    sgn = 1 if fc > bc else -1
    yaw = facing_yaw(*fwd)
    f = 2.3
    while f < abs(fc - bc) - 2.3:
        c = (c1 + c2) / 2
        x, y = pt(front, fc - sgn * f, c)
        it = Item(1.8, 1.4); it.prims += table(1.8, 1.4, top="lab_top").prims
        for xx in (-0.5, 0.5): equipment_on(it, xx, 0.35, 0.75); equipment_on(it, xx, -0.2, 0.75)
        if P.put(it, x, y, yaw):
            for off in (-0.45, 0.45):
                for dd in (-1.05, 1.05):
                    bx, by = pt(front, fc - sgn * (f + dd), c + off)
                    P.put(stool(), bx, by, yaw)
        f += 3.4

def L_office(P, R, desks=True):
    qs = quiet_sides(R)
    if qs: P.along(qs[0], almirah, step_extra=0.05)
    x1, y1, x2, y2 = P.rect
    long_x = (x2 - x1) >= (y2 - y1)
    yaw = 0
    step_a, step_b = 2.4, 2.4
    xs = np.arange(x1 + 1.3, x2 - 1.0, step_a); ys = np.arange(y1 + 1.4, y2 - 1.0, step_b)
    for x in xs:
        for y in ys:
            t = table(1.4, 0.7, top="wood")
            computer_on(t, -0.3, 0.0, 0.75)
            if P.put(t, x, y, yaw):
                P.put(office_chair(), x, y - 0.75, 0)
                P.put(chair("grey_fab"), x - 0.3, y + 0.8, 180)
    if len(P.taken) - P.base < 3:   # small room: desks along the walls instead
        for s in qs[1:3] or "NS":
            P.along(s, lambda: table(1.4, 0.7, top="wood"), seat=(office_chair, (0.0,)), step_extra=0.3)

def L_staff(P, R):
    qs = quiet_sides(R)
    for s in qs[:2]:
        P.along(s, lambda: table(1.2, 0.65, top="wood"), seat=(chair, (0.0,)), step_extra=0.15)
    for s in qs[2:3]:
        P.along(s, almirah)

def L_cabin(P, R, big=False):
    x1, y1, x2, y2 = P.rect
    d = R["doors"][0]["pos"] if R["doors"] else (x1, y1)
    # desk on the half away from the door, user faces the door
    if (x2 - x1) >= (y2 - y1):
        far = x2 if abs(d[0] - x1) < abs(d[0] - x2) else x1
        dx = -1 if far == x2 else 1
        cx, cy = far + dx * 1.6, (y1 + y2) / 2
        yaw = facing_yaw(dx, 0)
        P.put(table(1.6, 0.8, top="wood_dk"), cx, cy, yaw)
        P.put(office_chair(), cx - dx * 0.75, cy, yaw)
        for off in (-0.45, 0.45):
            P.put(chair("grey_fab"), cx + dx * 0.85, cy + off, (yaw + 180) % 360)
    else:
        far = y2 if abs(d[1] - y1) < abs(d[1] - y2) else y1
        dy = -1 if far == y2 else 1
        cx, cy = (x1 + x2) / 2, far + dy * 1.6
        yaw = facing_yaw(0, dy)
        P.put(table(1.6, 0.8, top="wood_dk"), cx, cy, yaw)
        P.put(office_chair(), cx, cy - dy * 0.75, yaw)
        for off in (-0.45, 0.45):
            P.put(chair("grey_fab"), cx + off, cy + dy * 0.85, (yaw + 180) % 360)
    qs = quiet_sides(R)
    if qs: P.along(qs[0], almirah)
    if big and len(qs) > 1: P.along(qs[1], sofa)

def L_boardroom(P, R):
    x1, y1, x2, y2 = P.rect
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    long_x = (x2 - x1) >= (y2 - y1)
    L = (max(x2 - x1, y2 - y1)) - 2.6
    P.put(table(L, 1.3, top="wood_dk", leg="wood_dk"), cx, cy, 0 if long_x else 90)
    n = int(L // 0.75)
    for k in range(n):
        o = (k - (n - 1) / 2) * 0.75
        if long_x:
            P.put(office_chair(), cx + o, cy + 1.05, 180); P.put(office_chair(), cx + o, cy - 1.05, 0)
        else:
            P.put(office_chair(), cx + 1.05, cy + o, 90); P.put(office_chair(), cx - 1.05, cy + o, 270)
    qs = quiet_sides(R)
    if qs: wall_board(P, qs[-1], 2.0, "whiteboard")

def L_admin(P, R):
    L_office(P, R)

def L_exam(P, R):
    x1, y1, x2, y2 = P.rect
    ds = door_sides(R)
    s = next(iter(ds)) if ds else "S"
    # service counter 1.6 m in from the door wall, leaving 1.0 m passage
    if s in "NS":
        y = (y1 + 1.6) if s == "S" else (y2 - 1.6)
        P.put(counter((x2 - x1) - 1.3), x1 + ((x2 - x1) - 1.3) / 2 + 0.05, y, 0 if s == "S" else 180)
    else:
        x = (x1 + 1.6) if s == "W" else (x2 - 1.6)
        P.put(counter((y2 - y1) - 1.3), x, y1 + ((y2 - y1) - 1.3) / 2 + 0.05, 270 if s == "W" else 90)
    L_office(P, R)

def L_seminar(P, R):
    x1, y1, x2, y2 = P.rect
    # door is on S wall -> stage at N
    front = {"S": "N", "N": "S", "E": "W", "W": "E"}[next(iter(door_sides(R)), "S")]
    fwd, cross, fc, bc, (c1, c2) = frame_axes(P.rect, front)
    sgn = 1 if fc > bc else -1
    sw = (c2 - c1) - 1.6
    st = Item(sw, 2.2); st.b(0, 0, 0.225, sw, 2.2, 0.45, "wood_dk")
    sx, sy = pt(front, fc - sgn * 1.1, (c1 + c2) / 2)
    P.put(st, sx, sy, 0 if front in "NS" else 90)
    px, py = pt(front, fc - sgn * 1.2, c1 + (c2 - c1) * 0.78)
    pd = podium(); P.model.box(px, py, 0.45 + 0.55, 0.6, 0.5, 1.1, "wood", collide=True)
    wall_board(P, front, 4.0, "whiteboard")
    yaw = facing_yaw(*fwd)
    f = 3.6
    while f < abs(fc - bc) - 1.2:
        for c in np.arange(c1 + 0.8, c2 - 0.6, 0.55):
            if abs(c - (c1 + c2) / 2) < 0.65: continue
            x, y = pt(front, fc - sgn * f, c)
            P.put(chair("red_seat"), x, y, yaw)
        f += 0.95

def L_restroom(P, R, acc=False):
    qs = quiet_sides(R)
    if not qs: return
    s = qs[0]
    if acc:
        it = Item(0.7, 0.9); it.b(0, -0.1, 0.2, 0.4, 0.65, 0.4, "ceramic")
        it.b(0.42, -0.1, 0.75, 0.04, 0.7, 0.04, "steel_lt", collide=False)
        x1, y1, x2, y2 = P.rect
        t = ((x1 + x2) / 2) if s in "NS" else ((y1 + y2) / 2)
        x, y, yaw = P.against(s, it, t); P.put(it, x, y, yaw)
        if len(qs) > 1: P.along(qs[1], lambda: vanity(0.9))
        return
    P.along(s, wc_stall, margin=0.05)
    opp = {"N": "S", "S": "N", "E": "W", "W": "E"}[s]
    if opp in qs: P.along(opp, lambda: vanity(1.8), margin=0.2)
    elif len(qs) > 1: P.along(qs[1], lambda: vanity(1.8), margin=0.2)

def L_dept_lobby(P, R):
    qs = quiet_sides(R)
    if qs: P.along(qs[0], lambda: corridor_bench(1.8), step_extra=0.8)
    P.along(qs[0] if qs else "N", plant)

LAYOUT = {
    "classroom": L_classroom, "lab_comp": L_lab_comp, "lab_bench": L_lab_bench, "lab_elec": L_lab_elec,
    "office": L_office, "staffroom": L_staff, "cabin": L_cabin,
    "principal": lambda P, R: L_cabin(P, R, big=True), "boardroom": L_boardroom, "admin": L_admin,
    "exam": L_exam, "seminar": L_seminar, "restroom": L_restroom,
    "restroom_acc": lambda P, R: L_restroom(P, R, acc=True), "dept_lobby": L_dept_lobby,
    "stair": lambda P, R: None,
}

fixed_rects = []
for b in boxes:
    if b["name"].startswith(("lift", "duct", "filler", "pillar", "step", "landing")):
        fixed_rects.append((b["x"] - b["sx"] / 2, b["y"] - b["sy"] / 2, b["x"] + b["sx"] / 2, b["y"] + b["sy"] / 2))

floors = Model("floor_finishes")
furn_count = 0
for rid, R in rooms.items():
    name, typ = ROOMS.get(rid, (f"Room {rid}", "office"))
    R["name"], R["type"] = name, typ
    x1, y1, x2, y2 = R["rect"]
    h = WALL_T / 2
    R["inner"] = (x1 + h, y1 + h, x2 - h, y2 - h)
    ix1, iy1, ix2, iy2 = R["inner"]
    fc = FLOOR[floor_kind(typ)]
    if typ != "stair" and (ix2 - ix1) * (iy2 - iy1) > 0.5:
        floors.box((ix1 + ix2) / 2, (iy1 + iy2) / 2, 0.0015, ix2 - ix1, iy2 - iy1, 0.003, fc, collide=False)
    clear = []
    for d in R["doors"]:
        w = d["b"] - d["a"]
        dx, dy = d["pos"]
        if d["orient"] == "H":
            sgn = 1 if (iy1 + iy2) / 2 > dy else -1
            clear.append((d["a"] - 0.35, min(dy, dy + sgn * 1.6), d["b"] + 0.35, max(dy, dy + sgn * 1.6)))
        else:
            sgn = 1 if (ix1 + ix2) / 2 > dx else -1
            clear.append((min(dx, dx + sgn * 1.6), d["a"] - 0.35, max(dx, dx + sgn * 1.6), d["b"] + 0.35))
    R["clear"] = clear
    m = Model(f"furniture_{rid.replace(' ', '_').lower()}")
    P = Placer(rid, (ix1 + 0.02, iy1 + 0.02, ix2 - 0.02, iy2 - 0.02), clear, fixed_rects, m)
    LAYOUT[typ](P, R)
    furn_count += m.n
    MODELS.append(m)
MODELS.insert(0, floors)

# ---------------------------------------------------------------- 4. lobby + corridor furnishings
lobby = Model("furniture_lobby_corridors")
LP = Placer("lobby", (-10, -32, 75, 30), [(-2.5, -2.5, 2.5, 2.5)], fixed_rects, lobby)
for r in rooms.values():
    LP.taken.append(r["rect"]) if False else None
def lp(it, x, y, yaw):
    """corridor item: slide along the wall until it is >=1.4 m from every door"""
    for off in (0, 1.5, -1.5, 3.0, -3.0):
        px, py = (x + off, y) if yaw in (0, 180) else (x, y + off)
        if all(math.dist(d["pos"], (px, py)) >= 1.4 for d in doors) and not any(
                Placer.hit(LP.fp(it, px, py, yaw), t, 0.05) for t in LP.taken):
            return LP.put(it, px, py, yaw, force=True)
    print("  could not place corridor item at", (x, y)); return False
# entrance lobby: waiting sofas against the board-room / principal wall, plants, reception, notice board
lp(sofa(2.0), 5.6, -2.75, 0); lp(plant(), 7.3, -2.95, 0); lp(plant(), -4.25, 3.2, 0)
lp(table(1.2, 0.6, top='wood'), -3.3, -2.92, 180); lp(office_chair(), -3.3, -2.25, 180)   # security / visitor register desk
lobby.box(-0.3, 3.71, 1.5, 2.0, 0.02, 1.0, "notice", collide=False)
lobby.box(-0.3, 3.70, 1.5, 1.85, 0.01, 0.85, "paper", collide=False)
# water coolers, dustbins, corridor benches, notice boards
for (x, y, yaw) in [(16.0, 16.85, 180), (46.0, 16.85, 180), (56.5, -12.25, 0), (20.95, -12.35, 0), (17.7, 6.0, 90)]:
    lp(water_cooler(), x, y, yaw)
for (x, y) in [(23.0, 14.95), (34.5, 16.8), (52.0, 16.8), (47.0, 2.0), (47.0, -8.0), (30.0, -12.3), (44.0, -12.3), (18.35, -1.0), (16.0, -11.5)]:
    lp(dustbin(), x, y, 0)
for (x, y, yaw) in [(26.5, 14.45, 180), (34.5, 14.45, 180), (42.5, 14.45, 180), (26.5, -9.95, 0), (34.5, -9.95, 0), (42.5, -9.95, 0)]:
    lp(corridor_bench(1.8), x, y, yaw)
for (x, y) in [(25.0, 17.05), (39.0, 16.95), (56.0, -12.5)]:
    lobby.box(x, y + (0.04 if y > 0 else 0.0), 1.55, 1.6, 0.02, 1.0, "notice", collide=False)
MODELS.append(lobby)

# ---------------------------------------------------------------- 5. write SDF
world_hdr = f"""<?xml version="1.0"?>
<!--
  BVCOE Lavale - FIRST FLOOR (realistic, navigation-ready)
  Generated by gen_world.py from the floor-plan-traced base world.
  Frame: origin (0,0) = robot start in the ENTRANCE LOBBY, X = east (right on the plan), Y = north (up).
  Scale is estimated (0.1 m per plan pixel; the plan has no scale bar).
  Walls {WALL_H} m, doors 1.0-1.3 m (leaves open 90 deg), windows on exterior walls,
  furniture per room type from the FF-xxx legend. Main gate closed: {MAIN_GATE_CLOSED}.
  Rooms/doors/goals: config/college_rooms.yaml   Nav2 map: maps/college_first_floor.yaml
-->
<sdf version="1.9">
  <world name="college_first_floor">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.62 0.62 0.62 1</ambient>
      <background>0.80 0.86 0.93 1</background>
      <grid>false</grid>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.85 0.84 0.80 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.4 -0.9</direction>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.38 0.38 0.40 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.4 -0.3 -0.8</direction>
    </light>
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>{FLOOR['corridor'][0]} {FLOOR['corridor'][1]} {FLOOR['corridor'][2]} 1</ambient><diffuse>{FLOOR['corridor'][0]} {FLOOR['corridor'][1]} {FLOOR['corridor'][2]} 1</diffuse></material>
        </visual>
        <visual name="courtyard_paving">
          <pose>35.250 2.250 0.001 0 0 0</pose>
          <geometry><box><size>24.300 24.500 0.002</size></box></geometry>
          <material><ambient>0.70 0.72 0.66 1</ambient><diffuse>0.70 0.72 0.66 1</diffuse></material>
        </visual>
        <visual name="outside_terrain">
          <pose>30 -2 -0.001 0 0 0</pose>
          <geometry><box><size>120 90 0.001</size></box></geometry>
          <material><ambient>0.55 0.60 0.45 1</ambient><diffuse>0.55 0.60 0.45 1</diffuse></material>
        </visual>
      </link>
    </model>
"""
with open(os.path.join(OUT, "college_first_floor.sdf"), "w") as f:
    f.write(world_hdr)
    for m in MODELS:
        x = m.xml()
        if x: f.write("    " + x + "\n")
    f.write("  </world>\n</sdf>\n")

# ---------------------------------------------------------------- 6. maps
def write_map(occ, name, title):
    free_reach = None
    img = np.full(occ.shape, 205, np.uint8)
    inside = ~OUTSIDE
    img[inside & ~occ] = 254
    img[occ] = 0
    img = np.flipud(img)
    from PIL import Image
    Image.fromarray(img).save(os.path.join(OUT, "maps", name + ".pgm"))
    with open(os.path.join(OUT, "maps", name + ".yaml"), "w") as f:
        f.write(f"# {title}\nimage: {name}.pgm\nmode: trinary\nresolution: {MAP_RES}\n"
                f"origin: [{X0}, {Y0}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")

occ_full = raster(COLL)
occ_walls = raster(COLL, layers={"wall", "stair", "door", "glass"})
write_map(occ_full, "college_first_floor", "Ground-truth map incl. furniture (z 0.03-1.5 m slice) - use as Nav2 static map")
write_map(occ_walls, "college_first_floor_walls", "Walls/pillars/stairs only = what the floor plan shows (baseline for floor-plan vs SLAM comparison)")
np.save(os.path.join(OUT, "occ_full.npy"), occ_full)

# ---------------------------------------------------------------- 7. reachability + goals + rooms yaml
ROBOT_R = 0.30
infl = ndimage.binary_dilation(occ_full | OUTSIDE, iterations=int(math.ceil(ROBOT_R / MAP_RES)))
lab2, _ = ndimage.label(~infl)
start_lab = lab2[cell(0, 0)]
reach = lab2 == start_lab
dist = ndimage.distance_transform_edt(~(occ_full | OUTSIDE)) * MAP_RES

def nearest_reach(x, y, rect=None, prefer_clear=0.6):
    j, i = cell(x, y)
    ys, xs = np.nonzero(reach)
    wx, wy = X0 + (xs + 0.5) * MAP_RES, Y0 + (ys + 0.5) * MAP_RES
    mask = np.ones(len(xs), bool)
    if rect is not None:
        mask = (wx > rect[0]) & (wx < rect[2]) & (wy > rect[1]) & (wy < rect[3])
    if not mask.any(): return None
    wx, wy, xs2, ys2 = wx[mask], wy[mask], xs[mask], ys[mask]
    clr = dist[ys2, xs2]
    cost = np.hypot(wx - x, wy - y) - 2.0 * np.minimum(clr, prefer_clear)
    k = int(np.argmin(cost))
    return round(float(wx[k]), 2), round(float(wy[k]), 2)

def region_of(x, y):
    for rid, R in rooms.items():
        a = R["rect"]
        if a[0] < x < a[2] and a[1] < y < a[3]: return rid
    if -4.8 < x < 9.3 and -3.3 < y < 3.8: return "Entrance Lobby"
    if 24.5 < x < 44.5 and -9.7 < y < 14.2: return "Court Yard"
    if is_out(x, y): return "outside"
    return "Corridor"

report = []
yaml_lines = ["# Rooms, doors and navigation goals for the BVCOE first-floor world (generated by gen_world.py)",
              "# frame: map == world (origin = robot start in the Entrance Lobby). Units: metres, yaw in radians.",
              "frame_id: map", "robot_start: {x: 0.0, y: 0.0, yaw: 0.0}", "", "rooms:"]
for rid, R in sorted(rooms.items(), key=lambda kv: (len(kv[0]) > 4, kv[0])):
    x1, y1, x2, y2 = R["rect"]
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    g = nearest_reach(cx, cy, R["inner"])
    ok = g is not None
    report.append((rid, R["name"], ok))
    code = f"FF-{rid}" if re.match(r"^\d", rid) else rid
    yaml_lines += [f"  - id: \"{rid}\"", f"    code: \"{code}\"", f"    name: \"{R['name']}\"", f"    type: {R['type']}",
                   f"    center: [{cx:.2f}, {cy:.2f}]",
                   f"    bounds: [{x1:.2f}, {y1:.2f}, {x2:.2f}, {y2:.2f}]   # xmin, ymin, xmax, ymax (wall centrelines)",
                   f"    goal: {list(g) if g else 'null'}   # reachable, furniture-free point nearest the centre",
                   f"    reachable: {str(ok).lower()}",
                   f"    doors: [{', '.join(d['id'] for d in R['doors'])}]"]
yaml_lines += ["", "doors:"]
for d in doors:
    x, y = d["pos"]; w = d["b"] - d["a"]
    if d["orient"] == "H": p_plus, p_minus = (x, y + 1.0), (x, y - 1.0)
    else: p_plus, p_minus = (x + 1.0, y), (x - 1.0, y)
    A, B = region_of(*p_plus), region_of(*p_minus)
    yaw_plus = math.pi / 2 if d["orient"] == "H" else 0.0
    yaml_lines += [f"  - id: {d['id']}", f"    center: [{x:.2f}, {y:.2f}]", f"    width: {w:.2f}",
                   f"    wall: {'horizontal' if d['orient']=='H' else 'vertical'}",
                   f"    connects: [\"{A}\", \"{B}\"]",
                   f"    approach: {{\"{A}\": [{p_plus[0]:.2f}, {p_plus[1]:.2f}], \"{B}\": [{p_minus[0]:.2f}, {p_minus[1]:.2f}]}}"]
    if d.get("main"): yaml_lines.append(f"    main_entrance: true   # closed glass gate in the sim: {MAIN_GATE_CLOSED}")
with open(os.path.join(OUT, "college_rooms.yaml"), "w") as f:
    f.write("\n".join(yaml_lines) + "\n")

json.dump(dict(rooms={k: dict(rect=v["rect"], name=v["name"], type=v["type"]) for k, v in rooms.items()},
               doors=[dict(id=d["id"], pos=d["pos"]) for d in doors]), open(os.path.join(OUT, "debug.json"), "w"))
np.save(os.path.join(OUT, "reach.npy"), reach)
print(f"rooms {len(rooms)}  doors {len(doors)}  windows {N_WIN}  collision boxes {len(COLL)}  furniture prims {furn_count}")
print("unreachable rooms:", [(r, n) for r, n, ok in report if not ok])
