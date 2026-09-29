#!/usr/bin/env python3
"""Convert a SUMO FCD trace into an ns-2 mobility trace for ns-3's Ns2MobilityHelper.

Why not traceExporter.py? traceExporter assigns ns-2 node ids in order of first
appearance, which silently changes when insertion is delayed. Here SUMO vehicle
"veh<K>" always becomes ns-3 node K, so the traffic source/destination roles
(Vehicle 0 / Vehicle N-1) are stable across densities and seeds.

Motion model: between two FCD samples p(t_i) and p(t_{i+1}) the vehicle moves
on a straight line at constant speed ("setdest" at t_i).
"""
import argparse
import math
import re
import xml.etree.ElementTree as ET


def convert(fcd_path, tcl_path, num_vehicles, period=0.5):
    samples = {}  # node -> list of (t, x, y)
    veh_re = re.compile(r"^veh(\d+)$")
    for _, elem in ET.iterparse(fcd_path, events=("end",)):
        if elem.tag != "timestep":
            continue
        t = float(elem.get("time"))
        if abs(t / period - round(t / period)) < 1e-6:
            for v in elem.findall("vehicle"):
                m = veh_re.match(v.get("id"))
                if not m:
                    continue
                node = int(m.group(1))
                samples.setdefault(node, []).append((t, float(v.get("x")), float(v.get("y"))))
        elem.clear()

    missing = [k for k in range(num_vehicles) if k not in samples]
    if missing:
        raise SystemExit(f"FCD trace lacks vehicles {missing}; increase duration or check routes")

    lines = []
    for node in range(num_vehicles):
        t0, x0, y0 = samples[node][0]
        lines.append(f"$node_({node}) set X_ {x0:.2f}")
        lines.append(f"$node_({node}) set Y_ {y0:.2f}")
        lines.append(f"$node_({node}) set Z_ 0.0")
    events = []
    for node in range(num_vehicles):
        pts = samples[node]
        for (ta, xa, ya), (tb, xb, yb) in zip(pts, pts[1:]):
            dist = math.hypot(xb - xa, yb - ya)
            if dist < 1e-3:
                continue  # standing still; the previous movement has already ended
            speed = dist / (tb - ta)
            events.append((ta, node, f'$ns_ at {ta:.2f} "$node_({node}) setdest {xb:.2f} {yb:.2f} {speed:.3f}"'))
    events.sort(key=lambda e: (e[0], e[1]))
    lines.extend(e[2] for e in events)
    with open(tcl_path, "w", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    return {k: len(v) for k, v in samples.items()}


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("fcd")
    ap.add_argument("tcl")
    ap.add_argument("--vehicles", type=int, required=True)
    ap.add_argument("--period", type=float, default=0.5)
    a = ap.parse_args()
    convert(a.fcd, a.tcl, a.vehicles, a.period)
