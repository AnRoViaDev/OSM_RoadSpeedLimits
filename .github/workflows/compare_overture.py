#!/usr/bin/env python3
"""
compare_overture.py — compare the speed limits of Overture Maps (transportation/segment)
with our map file osm_RO.bin (OSL2), road class by road class.

    python compare_overture.py osm_RO.bin report.md diffs.csv [--segments ro_segments.parquet]

Without --segments the Romanian segments are downloaded from the latest Overture release
(public S3 bucket, no account) with DuckDB.

Report (markdown, also printed):
  per Overture road class: km of road, km with an Overture limit, km with an EXPLICIT
  limit in our file (OSM maxspeed or field sign, not a rule), km where both have one and
  agree / differ, and km where ONLY Overture has a real value (our file has a rule or nothing)
  -> that last number is what Overture would add.
diffs.csv: every place where Overture and our file disagree (for QGIS / the admin map).
"""
import argparse
import json
import math
import os
import struct
import subprocess
import sys
from collections import defaultdict

RO_BBOX = (20.2, 43.6, 29.8, 48.3)          # xmin, ymin, xmax, ymax
F_INTRA, F_JUNCTION, F_DERIVED, F_SIGN = 1, 2, 4, 8
GRID = 200.0
MAX_DIST_M = 15.0


# ---------------- our file ----------------

class OurMap:
    def __init__(self, path):
        d = open(path, "rb").read()
        magic = struct.unpack(">i", d[:4])[0]
        rs = {0x4F534C31: 18, 0x4F534C32: 19}[magic]
        n = (len(d) - 4) // rs
        self.recs = []
        self.cells = defaultdict(list)
        for i in range(n):
            o = 4 + i * rs
            a1, o1, a2, o2, f, b = struct.unpack(">iiiiBB", d[o:o + 18])
            fl = d[o + 18] if rs == 19 else 0
            if fl & F_JUNCTION:
                continue
            r = (a1 / 1e6, o1 / 1e6, a2 / 1e6, o2 / 1e6, f, b, fl)
            idx = len(self.recs)
            self.recs.append(r)
            y0, y1 = sorted((math.floor(r[0] * GRID), math.floor(r[2] * GRID)))
            x0, x1 = sorted((math.floor(r[1] * GRID), math.floor(r[3] * GRID)))
            if (y1 - y0 + 1) * (x1 - x0 + 1) > 400:
                continue
            for y in range(y0, y1 + 1):
                for x in range(x0, x1 + 1):
                    self.cells[(y, x)].append(idx)
        print(f"our map: {len(self.recs)} segments", flush=True)

    def at(self, lat, lon, brg):
        """-> (limit, flags) for this point + driving direction, or (0, 0)."""
        cy, cx = math.floor(lat * GRID), math.floor(lon * GRID)
        k = math.cos(math.radians(lat)) * 111320.0
        best, res = 1e9, (0, 0)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                for i in self.cells.get((cy + dy, cx + dx), ()):
                    a1, o1, a2, o2, f, b, fl = self.recs[i]
                    ax, ay = (o1 - lon) * k, (a1 - lat) * 110540.0
                    bx, by = (o2 - lon) * k, (a2 - lat) * 110540.0
                    vx, vy = bx - ax, by - ay
                    l2 = vx * vx + vy * vy
                    t = 0.0 if l2 == 0 else max(0.0, min(1.0, -(ax * vx + ay * vy) / l2))
                    d = math.hypot(ax + t * vx, ay + t * vy)
                    if d > MAX_DIST_M or d >= best:
                        continue
                    sb = (math.degrees(math.atan2(vx, vy)) + 360) % 360
                    diff = abs(sb - brg) % 360
                    diff = 360 - diff if diff > 180 else diff
                    v = f if diff < 50 else b if diff > 130 else 0
                    if v:
                        best, res = d, (v, fl)
        return res


# ---------------- Overture ----------------

def latest_release():
    out = subprocess.run(["aws", "s3", "ls", "--no-sign-request", "s3://overturemaps-us-west-2/release/"],
                         capture_output=True, text=True, check=True).stdout
    rel = sorted(l.split()[-1].strip("/") for l in out.splitlines() if l.strip().endswith("/"))
    return rel[-1]


def download_segments(path):
    import duckdb
    rel = latest_release()
    print(f"Overture release {rel}", flush=True)
    xmin, ymin, xmax, ymax = RO_BBOX
    con = duckdb.connect()
    for ext in ("httpfs", "spatial"):
        con.execute(f"INSTALL {ext}; LOAD {ext};")
    con.execute("SET s3_region='us-west-2';")
    con.execute(f"""
        COPY (
          SELECT id, class, ST_AsText(geometry) AS wkt, to_json(speed_limits) AS speed_limits
          FROM read_parquet('s3://overturemaps-us-west-2/release/{rel}/theme=transportation/type=segment/*',
                            hive_partitioning=1)
          WHERE subtype = 'road'
            AND bbox.xmin < {xmax} AND bbox.xmax > {xmin} AND bbox.ymin < {ymax} AND bbox.ymax > {ymin}
        ) TO '{path}' (FORMAT PARQUET)""")
    return rel


def parse_wkt(w):
    s = w[w.index("(") + 1:w.rindex(")")].replace("(", "").replace(")", "")
    pts = []
    for p in s.split(","):
        x, y = p.split()[:2]
        pts.append((float(y), float(x)))          # (lat, lon)
    return pts


def seg_len(pts):
    return sum(meters(*pts[i - 1], *pts[i]) for i in range(1, len(pts)))


def meters(a1, o1, a2, o2):
    x = math.radians(o2 - o1) * math.cos(math.radians((a1 + a2) / 2))
    return math.hypot(x, math.radians(a2 - a1)) * 6371000.0


def point_at(pts, frac):
    """(lat, lon, bearing) at a fraction of the line's length."""
    total = seg_len(pts)
    want, acc = total * frac, 0.0
    for i in range(1, len(pts)):
        l = meters(*pts[i - 1], *pts[i])
        if acc + l >= want or i == len(pts) - 1:
            t = 0 if l == 0 else (want - acc) / l
            (a1, o1), (a2, o2) = pts[i - 1], pts[i]
            brg = (math.degrees(math.atan2((o2 - o1) * math.cos(math.radians(a1)), a2 - a1)) + 360) % 360
            return a1 + t * (a2 - a1), o1 + t * (o2 - o1), brg
        acc += l
    return pts[-1][0], pts[-1][1], 0.0


def kmh(ms):
    if not ms or ms.get("value") is None:
        return None
    v = float(ms["value"])
    return round(v * 1.609) if (ms.get("unit") or "km/h").lower().startswith("mph") else round(v)


def limits_of(js):
    """Overture speed_limits -> [(start, end, heading or None, kmh)] (unconditional only)."""
    out = []
    for e in json.loads(js) if js else []:
        v = kmh(e.get("max_speed"))
        if not v:
            continue
        w = e.get("when") or {}
        if any(w.get(k) for k in ("during", "mode", "using", "recognized", "vehicle")):
            continue                               # time / vehicle specific -> skip
        rng = e.get("between") or [0.0, 1.0]
        out.append((float(rng[0]), float(rng[1]), w.get("heading"), v))
    return out


# ---------------- compare ----------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ours"); ap.add_argument("report"); ap.add_argument("diffs")
    ap.add_argument("--segments", default="")
    a = ap.parse_args()
    seg_file = a.segments or "ro_segments.parquet"
    rel = "local"
    if not a.segments:
        rel = download_segments(seg_file)
    import duckdb
    tab = [{"class": c, "wkt": w, "speed_limits": sl and str(sl)} for c, w, sl in
           duckdb.sql(f"SELECT class, wkt, speed_limits FROM read_parquet('{seg_file}')").fetchall()]
    print(f"Overture segments in the box: {len(tab)}", flush=True)
    ours = OurMap(a.ours)

    S = defaultdict(lambda: defaultdict(float))      # class -> metric -> km
    diffs = []
    for row in tab:
        pts = parse_wkt(row["wkt"])
        if len(pts) < 2:
            continue
        mid_lat = pts[len(pts) // 2][0]; mid_lon = pts[len(pts) // 2][1]
        if not (RO_BBOX[1] < mid_lat < RO_BBOX[3] and RO_BBOX[0] < mid_lon < RO_BBOX[2]):
            continue
        cls = row["class"] or "?"
        L = seg_len(pts) / 1000.0
        lims = limits_of(row["speed_limits"])
        st = S[cls]
        st["km"] += L
        # compare both directions at the middle of each limited stretch (or of the segment)
        samples = lims or [(0.0, 1.0, None, None)]
        for s0, s1, head, ov in samples:
            part = L * max(0.0, s1 - s0)
            la, lo, brg = point_at(pts, (s0 + s1) / 2)
            dirs = ["forward", "backward"] if head is None else [head]
            for d in dirs:
                w = part / len(dirs)
                b = brg if d == "forward" else (brg + 180) % 360
                our, fl = ours.at(la, lo, b)
                explicit = our and not (fl & F_DERIVED)
                if ov:
                    st["km_overture"] += w
                if explicit:
                    st["km_ours_explicit"] += w
                if ov and explicit:
                    if ov == our:
                        st["km_agree"] += w
                    else:
                        st["km_differ"] += w
                        diffs.append((la, lo, round(b), cls, ov, our, "explicit"))
                if ov and not explicit:
                    st["km_only_overture"] += w
                    if our and ov != our:
                        diffs.append((la, lo, round(b), cls, ov, our, "rule"))

    order = ["motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential",
             "living_street", "service", "track"]
    classes = sorted(S, key=lambda c: (order.index(c) if c in order else 99, c))
    lines = [f"# Overture ({rel}) vs our map — Romania", "",
             "km per direction of travel (a two-way road counts twice).", "",
             "| class | road km | Overture has limit | ours explicit | both agree | both differ | only Overture (ours = rule/none) |",
             "|---|---:|---:|---:|---:|---:|---:|"]
    tot = defaultdict(float)
    for c in classes:
        st = S[c]
        km2 = st["km"] * 2
        for k, v in st.items():
            tot[k] += v
        def pc(x): return f"{x:,.0f} ({100 * x / km2:.0f}%)" if km2 else "0"
        lines.append(f"| {c} | {st['km']:,.0f} | {pc(st['km_overture'])} | {pc(st['km_ours_explicit'])} | "
                     f"{st['km_agree']:,.0f} | {st['km_differ']:,.0f} | {st['km_only_overture']:,.0f} |")
    km2 = tot["km"] * 2
    lines.append(f"| **total** | **{tot['km']:,.0f}** | **{tot['km_overture']:,.0f}** | **{tot['km_ours_explicit']:,.0f}** | "
                 f"**{tot['km_agree']:,.0f}** | **{tot['km_differ']:,.0f}** | **{tot['km_only_overture']:,.0f}** |")
    agree_pct = 100 * tot["km_agree"] / max(1, tot["km_agree"] + tot["km_differ"])
    lines += ["", f"Where both have an explicit limit they agree on **{agree_pct:.1f}%** of the km.",
              f"Overture would add a real value on **{tot['km_only_overture']:,.0f} km** "
              f"({100 * tot['km_only_overture'] / max(1, km2):.1f}% of all direction-km) where we only have a rule or nothing.",
              f"Places where they disagree: {len(diffs)} (see diffs.csv)."]
    rep = "\n".join(lines)
    print(rep)
    open(a.report, "w").write(rep + "\n")
    with open(a.diffs, "w") as f:
        f.write("lat,lon,bearing,class,overture,ours,ours_kind\n")
        for d in diffs:
            f.write(f"{d[0]:.6f},{d[1]:.6f},{d[2]},{d[3]},{d[4]},{d[5]},{d[6]}\n")


if __name__ == "__main__":
    main()
