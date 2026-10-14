#!/usr/bin/env python3
"""
build_speedlimits.py — OpenStreetMap .osm.pbf  ->  compact speed-limit file for SignDetector.

Output format ("OSL2", big-endian, read by OsmLimits.kt in the app):
    int32 magic 0x4F534C32
    repeated (19 bytes):
        int32 lat1e6, int32 lon1e6, int32 lat2e6, int32 lon2e6,
        uint8 limit_forward, uint8 limit_backward   (0 = no limit known)
        uint8 flags:  bit0 = inside intravilan (built-up area)
                      bit1 = JUNCTION point record (lat1==lat2, lon1==lon2, limits 0)
                      bit2 = limit derived from rules (not an explicit OSM value)
                      bit3 = limit from a sign seen on the road (signs.mapszone.eu)

Junctions = nodes where >= 3 road branches of public roads meet (not service roads,
tracks or motorways). A speed restriction ends at the next junction.

Romania + intravilan polygons (optional 4th argument = folder with .gpkg or .shp files,
any CRS, e.g. Stereo70 / EPSG:3844):
  * roads are CUT at the intravilan boundary
  * inside:  explicit 60 or < 50 stays; anything else (missing, RO:urban/rural, 70+) -> 50
             (motorways and expressways are not changed)
  * outside: explicit numeric maxspeed stays; otherwise the national rule by category:
             motorway 130, expressway 120, European road (E..) 100,
             other national / county / communal road 90,
             dirt roads (track / unpaved) and residential/service: no limit
Other countries: explicit maxspeed (+ motorway default), as before.

Field signs (optional): with the environment variable SIGNS_EXPORT_KEY set, the signs
collected by the app are downloaded from https://signs.mapszone.eu/api/signs/export.
Trusted ones (set by hand, or seen >= 2 times and missed less often than seen) are
matched to the nearest road (<= 25 m, same driving direction); their limit applies from
the sign to the next junction / next sign / end of the road and wins over OSM and the
rules. An end-of-limit sign (0) only stops the previous one.

Usage:  python build_speedlimits.py romania-latest.osm.pbf RO osm_RO.bin [intravilan_dir]
"""
import glob
import math
import os
import re
import struct
import sys

import osmium

MIN_STEP_M = 30.0
MOTORWAY_DEFAULT = {"DE": None, "PL": 140, "BG": 140}   # others: 130
RURAL = {"DE": 100, "AT": 100, "CZ": 100, "SK": 100, "SI": 100, "HR": 100, "FR": 80, "ES": 80}

F_INTRA, F_JUNCTION, F_DERIVED, F_SIGN = 1, 2, 4, 8

# road classes whose crossings count as junctions
JUNCTION_HW = {"trunk", "primary", "secondary", "tertiary", "unclassified", "residential",
               "living_street", "trunk_link", "primary_link", "secondary_link", "tertiary_link"}
MOTORWAY_HW = {"motorway", "motorway_link"}
# roads that get a rule-based limit in Romania
RULE_HW = JUNCTION_HW | MOTORWAY_HW | {"track", "road"}
UNPAVED = {"unpaved", "dirt", "earth", "ground", "gravel", "grass", "mud", "sand",
           "fine_gravel", "compacted", "pebblestone", "rock", "woodchips"}


def parse(v, cc):
    """Same rules as OsmLimits.parse() in the app."""
    if not v:
        return None
    s = v.strip().lower()
    m = re.match(r"^(\d{1,3})\s*(mph)?", s)
    if m:
        n = int(m.group(1))
        return int(n * 1.609 + 0.5) if m.group(2) else n
    kind = s.split(":", 1)[1] if ":" in s else s
    if kind == "living_street":
        return 20
    if kind.startswith("urban"):
        return 50
    if kind.startswith("rural"):
        return RURAL.get(cc, 90)
    if kind.startswith("trunk") or kind.startswith("expressway"):
        return 100
    if kind.startswith("motorway"):
        return MOTORWAY_DEFAULT.get(cc, 130)
    if kind == "walk":
        return 5
    return None


def numeric(v):
    """Explicit numeric maxspeed (a real sign), or None for zone tags like RO:urban."""
    if not v:
        return None
    m = re.match(r"^\s*(\d{1,3})\s*(mph)?\s*$", v.lower())
    if not m:
        return None
    n = int(m.group(1))
    return int(n * 1.609 + 0.5) if m.group(2) else n


def meters(a1, o1, a2, o2):
    x = math.radians(o2 - o1) * math.cos(math.radians((a1 + a2) / 2))
    y = math.radians(a2 - a1)
    return math.hypot(x, y) * 6371000.0


# ---------------- intravilan polygons ----------------

class Intravilan:
    """Intravilan polygons from every .gpkg / .shp in [folder] (any CRS, e.g. Stereo70)."""
    def __init__(self, folder):
        import pyogrio
        import pyproj
        import shapely
        from shapely.strtree import STRtree
        self.shapely = shapely
        polys = []
        files = sorted(glob.glob(os.path.join(folder, "**", "*.gpkg"), recursive=True) +
                       glob.glob(os.path.join(folder, "**", "*.shp"), recursive=True))
        if not files:
            sys.exit(f"no .gpkg / .shp found in {folder}")
        for path in files:
            for layer in [l[0] for l in pyogrio.list_layers(path)]:
                info = pyogrio.read_info(path, layer=layer)
                if "polygon" not in str(info.get("geometry_type", "")).lower():
                    continue
                geoms = pyogrio.raw.read(path, layer=layer, columns=[])[2]
                crs = info.get("crs") or "EPSG:3844"
                # an ESRI-style Stereo70 WKT has no datum shift -> use the EPSG code instead
                epsg = pyproj.CRS(crs).to_epsg(min_confidence=20)
                if epsg:
                    crs = f"EPSG:{epsg}"
                tr = pyproj.Transformer.from_crs(crs, "EPSG:4326", always_xy=True)
                n0 = len(polys)
                for wkb in geoms:
                    if wkb is None:
                        continue
                    g = shapely.from_wkb(wkb)
                    if g.is_empty:
                        continue
                    g = shapely.transform(g, lambda xy: _tr(tr, xy))
                    if not g.is_valid:
                        g = g.buffer(0)
                    g = g.simplify(0.00001)            # ~1 m
                    if not g.is_empty:
                        polys.append(g)
                print(f"  {os.path.basename(path)}:{layer} ({crs}): {len(polys) - n0} polygons", flush=True)
        if not polys:
            sys.exit("no intravilan polygons loaded")
        self.polys = polys
        self.tree = STRtree(polys)
        for p in polys:
            shapely.prepare(p)

    def split(self, coords):
        """coords [(lon,lat)...] -> list of (inside: bool, [(lon,lat)...]) in driving order."""
        sh = self.shapely
        line = sh.LineString(coords)
        idx = self.tree.query(line, predicate="intersects")
        if len(idx) == 0:
            return [(False, coords)]
        for i in idx:
            if self.polys[i].contains(line):
                return [(True, coords)]
        u = sh.union_all([self.polys[i] for i in idx])
        out = []
        for inside, g in ((True, line.intersection(u)), (False, line.difference(u))):
            for part in _lines(g):
                c = list(part.coords)
                if len(c) < 2:
                    continue
                s0, s1 = line.project(sh.Point(c[0])), line.project(sh.Point(c[-1]))
                if s1 < s0:
                    c.reverse(); s0 = s1
                out.append((s0, inside, c))
        out.sort(key=lambda x: x[0])
        return [(ins, c) for _, ins, c in out] or [(False, coords)]


def _tr(tr, xy):
    import numpy as np
    x, y = tr.transform(xy[:, 0], xy[:, 1])
    return np.column_stack([x, y])


def _lines(g):
    t = g.geom_type
    if t == "LineString":
        return [g]
    if t in ("MultiLineString", "GeometryCollection"):
        r = []
        for p in g.geoms:
            r.extend(_lines(p))
        return r
    return []


# ---------------- Romania rules ----------------

def ro_category_default(t, hw):
    """Limit outside built-up areas by road category, None = no limit."""
    ref = (t.get("ref") or "").upper().replace(" ", "")
    iref = (t.get("int_ref") or "").upper()
    surface = (t.get("surface") or "").lower()
    if hw == "track" or surface in UNPAVED:
        return None
    if hw in MOTORWAY_HW or ref.startswith("A"):
        return 130
    if ref.startswith("DEX") or t.get("motorroad") == "yes" or t.get("expressway") == "yes":
        return 120
    if "E" in iref or re.search(r"(^|;)E\d", ref):
        return 100
    if hw in ("trunk", "primary", "secondary", "tertiary", "unclassified", "road") or hw.endswith("_link") \
            or ref.startswith(("DN", "DJ", "DC")):
        return 90
    return None          # residential, living_street outside built-up area -> unknown


def ro_is_fast(t, hw):
    ref = (t.get("ref") or "").upper().replace(" ", "")
    return hw in MOTORWAY_HW or ref.startswith(("A", "DEX")) or t.get("motorroad") == "yes"


def ro_inside(e):
    """Explicit value e inside intravilan -> limit."""
    if e is not None and (e == 60 or e < 50):
        return e
    return 50


# ---------------- field signs (signs.mapszone.eu) ----------------

SIGN_MATCH_M = 25.0
SIGN_CELL = 0.005          # grid for lookup, ~500 m


class FieldSigns:
    def __init__(self, signs):
        self.grid = {}
        for sg in signs:
            k = (int(math.floor(sg["lat"] / SIGN_CELL)), int(math.floor(sg["lon"] / SIGN_CELL)))
            self.grid.setdefault(k, []).append(sg)
        self.count = len(signs)
        self.used = 0

    @staticmethod
    def download():
        key = os.environ.get("SIGNS_EXPORT_KEY", "").strip()
        if not key:
            print("field signs: SIGNS_EXPORT_KEY not set -> skipped", flush=True)
            return None
        import json
        import urllib.request
        url = os.environ.get("SIGNS_URL", "https://signs.mapszone.eu/api/signs/export")
        try:
            req = urllib.request.Request(url, headers={"X-Export-Key": key,
                                                       "User-Agent": "OSM_RoadSpeedLimits-build/1.0"})
            with urllib.request.urlopen(req, timeout=60) as r:
                data = json.loads(r.read().decode("utf-8"))
        except Exception as e:          # the map is still built, just without them
            print(f"field signs: download failed ({e}) -> skipped", flush=True)
            return None
        ok = [sg for sg in data.get("signs", [])
              if sg.get("manual") or (sg.get("hits", 0) >= 2 and sg.get("misses", 0) < sg.get("hits", 0))]
        print(f"field signs: {len(data.get('signs', []))} downloaded, {len(ok)} trusted", flush=True)
        return FieldSigns(ok) if ok else None

    def near_box(self, la0, lo0, la1, lo1):
        pad = 0.0005
        out = []
        for y in range(int(math.floor((la0 - pad) / SIGN_CELL)), int(math.floor((la1 + pad) / SIGN_CELL)) + 1):
            for x in range(int(math.floor((lo0 - pad) / SIGN_CELL)), int(math.floor((lo1 + pad) / SIGN_CELL)) + 1):
                out.extend(self.grid.get((y, x), ()))
        return out


class Line:
    """A way in local metres: arc-length positions, projection, cutting."""
    def __init__(self, pts):
        self.pts = pts
        la0, lo0 = pts[0]
        self.kx = math.cos(math.radians(la0)) * 111320.0
        self.ky = 110540.0
        self.la0, self.lo0 = la0, lo0
        self.xy = [((lo - lo0) * self.kx, (la - la0) * self.ky) for la, lo in pts]
        self.cum = [0.0]
        for i in range(1, len(self.xy)):
            (x1, y1), (x2, y2) = self.xy[i - 1], self.xy[i]
            self.cum.append(self.cum[-1] + math.hypot(x2 - x1, y2 - y1))
        self.length = self.cum[-1]

    def project(self, la, lo):
        """-> (s along the line, distance m, bearing of the segment there)."""
        px, py = (lo - self.lo0) * self.kx, (la - self.la0) * self.ky
        best = (0.0, 1e18, 0.0)
        for i in range(1, len(self.xy)):
            (x1, y1), (x2, y2) = self.xy[i - 1], self.xy[i]
            vx, vy = x2 - x1, y2 - y1
            l2 = vx * vx + vy * vy
            t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((px - x1) * vx + (py - y1) * vy) / l2))
            d = math.hypot(x1 + t * vx - px, y1 + t * vy - py)
            if d < best[1]:
                best = (self.cum[i - 1] + t * math.sqrt(l2), d, (math.degrees(math.atan2(vx, vy)) + 360) % 360)
        return best

    def point(self, s):
        for i in range(1, len(self.cum)):
            if s <= self.cum[i] or i == len(self.cum) - 1:
                seg = self.cum[i] - self.cum[i - 1]
                t = 0.0 if seg == 0 else max(0.0, min(1.0, (s - self.cum[i - 1]) / seg))
                (a1, o1), (a2, o2) = self.pts[i - 1], self.pts[i]
                return (a1 + t * (a2 - a1), o1 + t * (o2 - o1))
        return self.pts[-1]

    def cut(self, a, b):
        out = [self.point(a)]
        for i, c in enumerate(self.cum):
            if a < c < b:
                out.append(self.pts[i])
        out.append(self.point(b))
        return out


def dir_diff(a, b):
    d = abs(a - b) % 360
    return 360 - d if d > 180 else d


# ---------------- passes ----------------

class DegreeHandler(osmium.SimpleHandler):
    """Pass 1: how many road branches meet at each node."""
    def __init__(self):
        super().__init__()
        self.deg = {}
        self.motor = set()

    def way(self, w):
        hw = w.tags.get("highway")
        if hw in MOTORWAY_HW:
            for n in w.nodes:
                self.motor.add(n.ref)
            return
        if hw not in JUNCTION_HW:
            return
        nodes = [n.ref for n in w.nodes]
        last = len(nodes) - 1
        d = self.deg
        for i, r in enumerate(nodes):
            d[r] = d.get(r, 0) + (1 if i in (0, last) else 2)


class Handler(osmium.SimpleHandler):
    def __init__(self, cc, out, junction_nodes, intra, signs=None):
        super().__init__()
        self.cc, self.out, self.jn, self.intra, self.signs = cc, out, junction_nodes, intra, signs
        self.ways = self.segs = self.junctions = self.motorway_defaults = 0
        self.inside_segs = self.derived_segs = self.sign_segs = 0
        self.done_j = set()

    def write(self, pts, fa, ba, flags):
        """pts [(lat,lon)...] thinned to >= 30 m, end points kept exactly."""
        if fa == 0 and ba == 0:
            return
        pa, po = pts[0]
        for i in range(1, len(pts)):
            la, lo = pts[i]
            if i < len(pts) - 1 and meters(pa, po, la, lo) < MIN_STEP_M:
                continue
            self.out.write(struct.pack(">iiiiBBB", round(pa * 1e6), round(po * 1e6),
                                       round(la * 1e6), round(lo * 1e6), fa, ba, flags))
            pa, po = la, lo
            self.segs += 1
            if flags & F_INTRA:
                self.inside_segs += 1
            if flags & F_DERIVED:
                self.derived_segs += 1
            if flags & F_SIGN:
                self.sign_segs += 1

    def finish(self, pts, refs, pieces, one_f, one_b):
        """pieces: [(coords, fwd, bwd, flags)] in driving order. Applies field signs, writes."""
        signs = []
        if self.signs is not None:
            la = [p[0] for p in pts]; lo = [p[1] for p in pts]
            cand = self.signs.near_box(min(la), min(lo), max(la), max(lo))
            if cand:
                line = Line(pts)
                for sg in cand:
                    s_, d, brg = line.project(sg["lat"], sg["lon"])
                    if d > SIGN_MATCH_M:
                        continue
                    dd = dir_diff(brg, sg["brg"])
                    if dd < 45 and not one_b:
                        signs.append((s_, "f", sg))
                    elif dd > 135 and not one_f:
                        signs.append((s_, "b", sg))
        if not signs:
            for c, fa, ba, fl in pieces:
                self.write(c, fa, ba, fl)
            return
        self.signs.used += len(signs)
        line = Line(pts)
        L = line.length
        # piece intervals on the original line
        iv = []
        for c, fa, ba, fl in pieces:
            a = line.project(*c[0])[0]; b = line.project(*c[-1])[0]
            iv.append((min(a, b), max(a, b), fa, ba, fl))
        jpos = [line.cum[i] for i, r in enumerate(refs) if r in self.jn]
        # one sign per place/direction: hand-set first, then the most seen
        signs.sort(key=lambda x: (x[1], x[0]))
        kept = []
        for sg in signs:
            if kept and kept[-1][1] == sg[1] and abs(kept[-1][0] - sg[0]) < 30:
                o = kept[-1][2]
                if (sg[2].get("manual"), sg[2].get("hits", 0)) > (o.get("manual"), o.get("hits", 0)):
                    kept[-1] = sg
                continue
            kept.append(sg)
        ovr = []      # (a, b, dir, value)
        for s_, dr, sg in kept:
            same = sorted(x[0] for x in kept if x[1] == dr)
            if dr == "f":
                ends = [p for p in jpos if p > s_ + 5] + [p for p in same if p > s_ + 1] + [L]
                a, b = s_, min(ends)
            else:
                ends = [p for p in jpos if p < s_ - 5] + [p for p in same if p < s_ - 1] + [0.0]
                a, b = max(ends), s_
            if sg["limit"] > 0 and b - a > 1:
                ovr.append((a, b, dr, int(sg["limit"])))
        cuts = sorted({0.0, L} | {x for i in iv for x in i[:2]} | {x for o in ovr for x in o[:2]})
        for a, b in zip(cuts, cuts[1:]):
            if b - a < 0.5:
                continue
            m = (a + b) / 2
            base = next((i for i in iv if i[0] - 0.01 <= m <= i[1] + 0.01), None)
            fa, ba, fl = (base[2], base[3], base[4]) if base else (0, 0, 0)
            for oa, ob, dr, v in ovr:
                if oa <= m <= ob:
                    if dr == "f": fa = min(v, 255)
                    else: ba = min(v, 255)
                    fl = (fl | F_SIGN) & ~F_DERIVED
            self.write(line.cut(a, b), fa, ba, fl)

    def way(self, w):
        t = w.tags
        hw = t.get("highway")
        if not hw:
            return
        pts, refs = [], []
        try:
            for n in w.nodes:
                if n.location.valid():
                    pts.append((n.location.lat, n.location.lon)); refs.append(n.ref)
        except osmium.InvalidLocationError:
            return
        if len(pts) < 2:
            return

        # junction point records (once per node)
        if hw in JUNCTION_HW:
            for r, (la, lo) in zip(refs, pts):
                if r in self.jn and r not in self.done_j:
                    self.done_j.add(r)
                    self.out.write(struct.pack(">iiiiBBB", round(la * 1e6), round(lo * 1e6),
                                               round(la * 1e6), round(lo * 1e6), 0, 0, F_JUNCTION))
                    self.junctions += 1

        ow = t.get("oneway")
        one_f = ow in ("yes", "true", "1") or hw == "motorway" or (t.get("junction") == "roundabout" and ow is None)
        one_b = ow == "-1"

        def directions(f, b):
            if one_f: b = None
            elif one_b: f = None
            return min(f or 0, 255), min(b or 0, 255)

        if self.intra is not None and hw in RULE_HW:
            self.finish(pts, refs, self.way_ro(t, hw, pts, directions), one_f, one_b)
            self.ways += 1
            return

        base = parse(t.get("maxspeed"), self.cc)
        f = parse(t.get("maxspeed:forward"), self.cc) or base
        b = parse(t.get("maxspeed:backward"), self.cc) or base
        derived = 0
        if f is None and b is None and hw == "motorway":
            f = b = MOTORWAY_DEFAULT.get(self.cc, 130)
            if f:
                self.motorway_defaults += 1; derived = F_DERIVED
        if f is None and b is None:
            if self.signs is None or hw not in RULE_HW:
                return
            f = b = 0              # no limit known; a field sign may still give one
        fa, ba = directions(f, b)
        self.finish(pts, refs, [(pts, fa, ba, derived)], one_f, one_b)
        self.ways += 1

    def way_ro(self, t, hw, pts, directions):
        ms = t.get("maxspeed")
        e_f = numeric(t.get("maxspeed:forward")) or numeric(ms)
        e_b = numeric(t.get("maxspeed:backward")) or numeric(ms)
        if hw == "living_street" or (ms or "").lower().endswith("living_street"):
            e_f = e_b = 20
        default = ro_category_default(t, hw)
        fast = ro_is_fast(t, hw)

        if fast:   # motorway / expressway: no intravilan rule
            f = e_f or default; b = e_b or default
            fa, ba = directions(f, b)
            return [(pts, fa, ba, 0 if (e_f or e_b) else F_DERIVED)]

        out = []
        pieces = self.intra.split([(lo, la) for la, lo in pts])
        for inside, c in pieces:
            p = [(la, lo) for lo, la in c]
            if inside:
                f, b = ro_inside(e_f), ro_inside(e_b)
                derived = 0 if (f == e_f and b == e_b) else F_DERIVED
                fa, ba = directions(f, b)
                out.append((p, fa, ba, F_INTRA | derived))
            else:
                f = e_f or default; b = e_b or default
                derived = 0 if (e_f or e_b) else F_DERIVED
                fa, ba = directions(f, b)
                out.append((p, fa, ba, derived))
        return out


def main():
    if len(sys.argv) not in (4, 5):
        sys.exit(__doc__)
    src, cc, dst = sys.argv[1], sys.argv[2].upper(), sys.argv[3]
    intra = None
    if len(sys.argv) == 5 and cc == "RO":
        print("loading intravilan polygons…", flush=True)
        intra = Intravilan(sys.argv[4])

    print("pass 1: junctions…", flush=True)
    d = DegreeHandler()
    d.apply_file(src)
    jn = {r for r, k in d.deg.items() if k >= 3 and r not in d.motor}
    del d
    print(f"  {len(jn)} junctions", flush=True)

    print("pass 2: roads…", flush=True)
    with open(dst, "wb") as out:
        out.write(struct.pack(">i", 0x4F534C32))
        h = Handler(cc, out, jn, intra, FieldSigns.download())
        h.apply_file(src, locations=True, idx="flex_mem")
    print(f"{cc}: {h.ways} roads, {h.segs} segments ({h.inside_segs} in intravilan, "
          f"{h.derived_segs} rule-based, {h.sign_segs} from field signs), {h.junctions} junctions, "
          f"{h.motorway_defaults} motorway defaults -> {dst}")
    if h.segs == 0:
        sys.exit("no segments — something is wrong")


if __name__ == "__main__":
    main()
