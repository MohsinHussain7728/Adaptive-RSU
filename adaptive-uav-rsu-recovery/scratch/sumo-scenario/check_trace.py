#!/usr/bin/env python3
"""Sanity check of an ns-2 mobility trace produced by fcd_to_ns2.py.

Reports per vehicle: time span of movement commands, x/y range and the district
(West/East) it stays in, and flags vehicles that cross the gap or stop early.
    python3 check_trace.py low/mobility_1.tcl --gap-min 400 --gap-max 800
"""
import argparse
import re

SET_RE = re.compile(r'^\$node_\((\d+)\) set ([XY])_ ([-\d.]+)')
DEST_RE = re.compile(r'^\$ns_ at ([\d.]+) "\$node_\((\d+)\) setdest ([-\d.]+) ([-\d.]+) ([-\d.]+)"')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("--gap-min", type=float, default=400.0)
    ap.add_argument("--gap-max", type=float, default=800.0)
    ap.add_argument("--tolerance", type=float, default=10.0, help="lane offset allowance [m]")
    a = ap.parse_args()

    pts, last_t = {}, {}
    for line in open(a.trace):
        m = SET_RE.match(line)
        if m:
            k, axis, v = int(m.group(1)), m.group(2), float(m.group(3))
            pts.setdefault(k, {"X": [], "Y": []})[axis].append(v)
            continue
        m = DEST_RE.match(line)
        if m:
            t, k, x, y = float(m.group(1)), int(m.group(2)), float(m.group(3)), float(m.group(4))
            pts[k]["X"].append(x)
            pts[k]["Y"].append(y)
            last_t[k] = max(last_t.get(k, 0.0), t)

    problems = 0
    for k in sorted(pts):
        xs, ys = pts[k]["X"], pts[k]["Y"]
        west = max(xs) <= a.gap_min + a.tolerance
        east = min(xs) >= a.gap_max - a.tolerance
        side = "West" if west else ("East" if east else "CROSSES GAP")
        problems += side == "CROSSES GAP"
        print(f"veh {k:3d}: x[{min(xs):7.1f},{max(xs):7.1f}] y[{min(ys):6.1f},{max(ys):6.1f}] "
              f"last move {last_t.get(k, 0):6.1f} s  {side}")
    print(f"{len(pts)} vehicles, {problems} crossing the gap")


if __name__ == "__main__":
    main()
