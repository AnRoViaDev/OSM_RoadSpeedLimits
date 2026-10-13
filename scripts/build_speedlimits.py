#!/usr/bin/env python3
"""
build_speedlimits.py — OpenStreetMap .osm.pbf  ->  compact speed-limit file for SignDetector.

Output format ("OSL1", big-endian, read by OsmLimits.kt in the app):
    int32 magic 0x4F534C31
    repeated:  int32 lat1e6, int32 lon1e6, int32 lat2e6, int32 lon2e6,
               uint8 limit_forward, uint8 limit_backward      (0 = unknown / not allowed)

Roads kept:
  * every way with highway=* and a usable maxspeed / maxspeed:forward / maxspeed:backward
  * motorways without maxspeed get the country's motorway default (e.g. RO 130)
Geometry is thinned to points >= 30 m apart.

Usage:  python build_speedlimits.py romania-latest.osm.pbf RO osm_RO.bin
"""
import math
import re
import struct
import sys

import osmium

MIN_STEP_M = 30.0
MOTORWAY_DEFAULT = {"DE": None, "PL": 140, "BG": 140}   # others: 130
RURAL = {"DE": 100, "AT": 100, "CZ": 100, "SK": 100, "SI": 100, "HR": 100, "FR": 80, "ES": 80}


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


def meters(a1, o1, a2, o2):
    x = math.radians(o2 - o1) * math.cos(math.radians((a1 + a2) / 2))
    y = math.radians(a2 - a1)
    return math.hypot(x, y) * 6371000.0


class Handler(osmium.SimpleHandler):
    def __init__(self, cc, out):
        super().__init__()
        self.cc, self.out = cc, out
        self.ways = self.segs = self.motorway_defaults = 0

    def way(self, w):
        t = w.tags
        hw = t.get("highway")
        if not hw:
            return
        base = parse(t.get("maxspeed"), self.cc)
        f = parse(t.get("maxspeed:forward"), self.cc) or base
        b = parse(t.get("maxspeed:backward"), self.cc) or base
        if f is None and b is None and hw == "motorway":
            f = b = MOTORWAY_DEFAULT.get(self.cc, 130)
            if f:
                self.motorway_defaults += 1
        if f is None and b is None:
            return
        ow = t.get("oneway")
        if ow in ("yes", "true", "1") or hw == "motorway" or (t.get("junction") == "roundabout" and ow is None):
            b = None
        elif ow == "-1":
            f = None
        if f is None and b is None:
            return
        pts = []
        try:
            for n in w.nodes:
                if n.location.valid():
                    pts.append((n.location.lat, n.location.lon))
        except osmium.InvalidLocationError:
            return
        if len(pts) < 2:
            return
        fa, ba = min(f or 0, 255), min(b or 0, 255)
        pa, po = pts[0]
        for i in range(1, len(pts)):
            la, lo = pts[i]
            if i < len(pts) - 1 and meters(pa, po, la, lo) < MIN_STEP_M:
                continue
            self.out.write(struct.pack(">iiiiBB", int(pa * 1e6), int(po * 1e6),
                                       int(la * 1e6), int(lo * 1e6), fa, ba))
            pa, po = la, lo
            self.segs += 1
        self.ways += 1


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    src, cc, dst = sys.argv[1], sys.argv[2].upper(), sys.argv[3]
    with open(dst, "wb") as out:
        out.write(struct.pack(">i", 0x4F534C31))
        h = Handler(cc, out)
        h.apply_file(src, locations=True, idx="flex_mem")
    print(f"{cc}: {h.ways} roads, {h.segs} segments "
          f"({h.motorway_defaults} motorways with default limit) -> {dst}")
    if h.segs == 0:
        sys.exit("no segments — something is wrong")


if __name__ == "__main__":
    main()
