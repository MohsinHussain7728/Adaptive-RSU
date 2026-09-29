#!/usr/bin/env python3
"""Reproducible SUMO 1.18 scenario generator for the UAV-assisted VANET recovery study.

Pipeline (all artefacts are written into this directory):

    road network  (network/city.nod.xml + city.edg.xml --netconvert--> city.net.xml)
        -> traffic (<scenario>/trips_<v>.trips.xml --duarouter--> routes_<v>.rou.xml)
        -> SUMO simulation (<scenario>/scenario_<v>.sumocfg --sumo--> fcd_<v>.xml)
        -> ns-2 mobility trace (<scenario>/mobility_<v>.tcl) for ns-3 Ns2MobilityHelper

Urban layout: two grid districts (West / East) separated by a river/park gap
without roads. Vehicles 0..ceil(N/2)-1 circulate in the West district
(Vehicle 0 = traffic source), the others in the East district (Vehicle N-1 =
traffic destination). RSUs: West RSUs, one bridge RSU in the gap, East RSUs
(same layout formula as env/scenario.cc).

Example:
    python3 generate_scenario.py --name medium --vehicles 40 --variants 1,2,3,4,5
"""
import argparse
import math
import os
import random
import shutil
import subprocess
import sys

from fcd_to_ns2 import convert

HERE = os.path.dirname(os.path.abspath(__file__))
NET_DIR = os.path.join(HERE, "network")


class Geometry:
    def __init__(self, block, blocks, gap):
        self.block, self.blocks, self.gap = block, blocks, gap
        self.width = block * blocks
        self.height = block * blocks
        self.east_min_x = self.width + gap
        self.east_max_x = 2 * self.width + gap
        self.gap_center_x = self.width + gap / 2.0

    def rsu_layout(self, num_rsus):
        """Mirror of uavrsu::ComputeRsuLayout (env/scenario.cc)."""
        h = self.height
        bridge = num_rsus // 2
        west, east = bridge, num_rsus - bridge - 1
        lerp = lambda a, b, t: (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        pos = []
        for i in range(west):
            t = 1.0 if west == 1 else i / (west - 1)
            pos.append(lerp((self.width / 2, h / 2), (self.width, h / 4), t))
        pos.append((self.gap_center_x, h / 2))
        for j in range(east):
            t = 0.0 if east == 1 else j / (east - 1)
            pos.append(lerp((self.east_min_x, 3 * h / 4), (self.east_min_x + self.width / 2, h / 2), t))
        return pos


def run(cmd, cwd):
    print("   $", " ".join(cmd))
    res = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if res.returncode != 0:
        sys.stderr.write(res.stdout + res.stderr)
        raise SystemExit(f"command failed: {' '.join(cmd)}")


def district_edges(prefix):
    return [e for e in DISTRICT_EDGES if e.startswith(prefix)]


DISTRICT_EDGES = []


def build_network(geo, speed_limit, num_rsus):
    """Road network + static additional files shared by all density scenarios."""
    os.makedirs(NET_DIR, exist_ok=True)
    nodes, edges = [], []
    for d, x_off in (("W", 0.0), ("E", geo.east_min_x)):
        for i in range(geo.blocks + 1):
            for j in range(geo.blocks + 1):
                nodes.append((f"{d}{i}_{j}", x_off + i * geo.block, j * geo.block))
        for i in range(geo.blocks + 1):
            for j in range(geo.blocks + 1):
                for di, dj in ((1, 0), (0, 1)):
                    ni, nj = i + di, j + dj
                    if ni > geo.blocks or nj > geo.blocks:
                        continue
                    a, b = f"{d}{i}_{j}", f"{d}{ni}_{nj}"
                    edges.append((f"{d}_{a}_{b}", a, b))
                    edges.append((f"{d}_{b}_{a}", b, a))

    with open(os.path.join(NET_DIR, "city.nod.xml"), "w", newline="\n") as f:
        f.write("<nodes>\n")
        for nid, x, y in nodes:
            f.write(f'    <node id="{nid}" x="{x:.2f}" y="{y:.2f}" type="priority"/>\n')
        f.write("</nodes>\n")
    with open(os.path.join(NET_DIR, "city.edg.xml"), "w", newline="\n") as f:
        f.write("<edges>\n")
        for eid, a, b in edges:
            f.write(f'    <edge id="{eid}" from="{a}" to="{b}" numLanes="1" speed="{speed_limit:.2f}"/>\n')
        f.write("</edges>\n")
    DISTRICT_EDGES[:] = [e[0] for e in edges]

    run(["netconvert", "--node-files", "city.nod.xml", "--edge-files", "city.edg.xml",
         "--output-file", "city.net.xml", "--offset.disable-normalization", "true",
         "--no-turnarounds", "false", "--junctions.corner-detail", "0"], NET_DIR)

    with open(os.path.join(NET_DIR, "rsus.add.xml"), "w", newline="\n") as f:
        f.write("<additional>\n")
        for k, (x, y) in enumerate(geo.rsu_layout(num_rsus)):
            role = "bridge" if k == num_rsus // 2 else "district"
            f.write(f'    <poi id="RSU{k}" type="RSU_{role}" color="0,200,0" layer="10" '
                    f'x="{x:.2f}" y="{y:.2f}" width="12" height="12"/>\n')
        f.write(f'    <poi id="UAV_HOME" type="UAV" color="0,0,255" layer="10" '
                f'x="{geo.width / 2:.2f}" y="{0.75 * geo.height:.2f}" width="12" height="12"/>\n')
        f.write("</additional>\n")
    with open(os.path.join(NET_DIR, "landmarks.poly.xml"), "w", newline="\n") as f:
        x0, x1 = geo.width + 20, geo.east_min_x - 20
        f.write("<additional>\n")
        f.write(f'    <poly id="river_park_gap" type="water" color="0,110,255" fill="1" layer="-1" '
                f'shape="{x0},-50 {x1},-50 {x1},{geo.height + 50} {x0},{geo.height + 50}"/>\n')
        f.write("</additional>\n")


def write_vtypes(path, max_speed):
    """Vehicle configuration file (passenger cars, Krauss car-following)."""
    with open(path, "w", newline="\n") as f:
        f.write("<additional>\n")
        f.write(f'    <vType id="car" vClass="passenger" accel="2.6" decel="4.5" sigma="0.5" length="4.5" '
                f'minGap="2.5" maxSpeed="{max_speed:.2f}" speedFactor="1.0" speedDev="0.1" color="1,0,0"/>\n')
        f.write("</additional>\n")


def write_trips(path, num_vehicles, geo, max_speed, duration, seed):
    """Every vehicle gets a long random tour (via edges) inside its own district."""
    rng = random.Random(seed)
    west_n = (num_vehicles + 1) // 2
    needed = max_speed * duration * 1.5
    vias_needed = max(4, int(math.ceil(needed / (1.5 * geo.block))))
    with open(path, "w", newline="\n") as f:
        f.write("<routes>\n")
        for k in range(num_vehicles):
            pool = district_edges("W_" if k < west_n else "E_")
            seq = [rng.choice(pool)]
            while len(seq) < vias_needed + 2:
                e = rng.choice(pool)
                if e != seq[-1]:
                    seq.append(e)
            f.write(f'    <trip id="veh{k}" type="car" depart="{0.1 * k:.1f}" from="{seq[0]}" to="{seq[-1]}" '
                    f'via="{" ".join(seq[1:-1])}" departPos="random" departLane="best" departSpeed="max"/>\n')
        f.write("</routes>\n")


def write_sumocfg(path, variant, duration, seed):
    with open(path, "w", newline="\n") as f:
        f.write(f"""<configuration>
    <input>
        <net-file value="../network/city.net.xml"/>
        <route-files value="routes_{variant}.rou.xml"/>
        <additional-files value="../network/rsus.add.xml,../network/landmarks.poly.xml"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="{duration}"/>
        <step-length value="0.1"/>
    </time>
    <processing>
        <time-to-teleport value="-1"/>
    </processing>
    <random_number>
        <seed value="{seed}"/>
    </random_number>
    <output>
        <fcd-output value="fcd_{variant}.xml"/>
    </output>
    <report>
        <no-step-log value="true"/>
        <no-warnings value="true"/>
    </report>
</configuration>
""")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", required=True, help="scenario directory name, e.g. low / medium / high")
    ap.add_argument("--vehicles", type=int, required=True)
    ap.add_argument("--max-speed", type=float, default=13.89, help="vehicle max speed [m/s]")
    ap.add_argument("--speed-limit", type=float, default=19.44,
                    help="road speed limit [m/s]; shared network, vehicle speed is capped by --max-speed")
    ap.add_argument("--duration", type=float, default=130.0, help="SUMO duration [s] (>= ns-3 simulationTime)")
    ap.add_argument("--variants", default="1,2,3,4,5", help="comma list of trace variants (SUMO seeds)")
    ap.add_argument("--block", type=float, default=200.0)
    ap.add_argument("--blocks", type=int, default=2)
    ap.add_argument("--gap", type=float, default=400.0)
    ap.add_argument("--rsus", type=int, default=5)
    ap.add_argument("--keep-fcd", action="store_true", help="keep the raw FCD XML files")
    a = ap.parse_args()

    for tool in ("netconvert", "duarouter", "sumo"):
        if shutil.which(tool) is None:
            raise SystemExit(f"{tool} not found; install SUMO 1.18 (see scripts/setup)")

    geo = Geometry(a.block, a.blocks, a.gap)
    print(f"[network] {geo.blocks}x{geo.blocks} blocks per district, gap {geo.gap} m")
    build_network(geo, a.speed_limit, a.rsus)

    out = os.path.join(HERE, a.name)
    os.makedirs(out, exist_ok=True)
    write_vtypes(os.path.join(out, "vtypes.add.xml"), a.max_speed)
    variants = [v for v in a.variants.split(",") if v]
    for v in variants:
        seed = int(v)
        print(f"[{a.name}] variant {v}: {a.vehicles} vehicles, max speed {a.max_speed} m/s")
        write_trips(os.path.join(out, f"trips_{v}.trips.xml"), a.vehicles, geo, a.max_speed, a.duration, seed)
        run(["duarouter", "--net-file", "../network/city.net.xml", "--route-files", f"trips_{v}.trips.xml",
             "--additional-files", "vtypes.add.xml", "--output-file", f"routes_{v}.rou.xml",
             "--seed", str(seed), "--no-warnings", "--no-step-log"], out)
        write_sumocfg(os.path.join(out, f"scenario_{v}.sumocfg"), v, a.duration, seed)
        run(["sumo", "-c", f"scenario_{v}.sumocfg"], out)
        counts = convert(os.path.join(out, f"fcd_{v}.xml"), os.path.join(out, f"mobility_{v}.tcl"), a.vehicles)
        print(f"   mobility_{v}.tcl: {len(counts)} vehicles, {sum(counts.values())} samples")
        if not a.keep_fcd:
            os.remove(os.path.join(out, f"fcd_{v}.xml"))
        for alt in ("routes_%s.rou.alt.xml" % v,):
            if os.path.exists(os.path.join(out, alt)):
                os.remove(os.path.join(out, alt))

    with open(os.path.join(out, "scenario_info.cfg"), "w", newline="\n") as f:
        f.write(f"# generated by generate_scenario.py\nname={a.name}\nnum_vehicles={a.vehicles}\n"
                f"west_vehicles={(a.vehicles + 1) // 2}\nmax_speed={a.max_speed}\nduration={a.duration}\n"
                f"block_length={a.block}\ndistrict_blocks={a.blocks}\ngap_width={a.gap}\n"
                f"num_rsus={a.rsus}\nvariants={a.variants}\n")
    with open(os.path.join(out, "rsus.csv"), "w", newline="\n") as f:
        f.write("id,x,y,z\n")
        for k, (x, y) in enumerate(geo.rsu_layout(a.rsus)):
            f.write(f"{k},{x:.2f},{y:.2f},0.0\n")
    print(f"[{a.name}] done -> {out}")


if __name__ == "__main__":
    main()
