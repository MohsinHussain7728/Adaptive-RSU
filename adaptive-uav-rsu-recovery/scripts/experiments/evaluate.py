#!/usr/bin/env python3
"""EVALUATION driver: baselines vs. trained RL agents on held-out mobility.

Strategies compared (all use the same scenario, failure and traffic):
    none          no UAV (radio off)                      -> lower bound
    static        UAV hovers at its start position
    reactive-rsu  greedy flight towards the failed RSU (non-RL)
    reactive-gap  greedy flight towards the measured connectivity gap (non-RL)
    random        random relative movements (RLAgent "random", no learning)
    rl-seed<k>    trained Q-learning model of training seed k, greedy (eval mode:
                  epsilon = evalEpsilon = 0, no Q-table updates)

Suites:
    main      densities low/medium/high, SUMO variant 5 (unseen in training),
              failureTime x run grid
    speed     medium_slow / medium_fast (vehicle speed sensitivity)
    location  medium density, failure of a non-bridge RSU (1 or 3)
    randstart medium density, random UAV start positions

Each run writes into its own directory results/eval/<suite>/<scenario>/<strategy>/<case>/
so runs can execute in parallel; analyze.py merges all CSVs.

    scripts/py.sh scripts/experiments/evaluate.py --suites main,speed,location,randstart --jobs 14
"""
import argparse
import glob
import os
import sys
from concurrent.futures import ThreadPoolExecutor

from common import REPO, run_sim


def strategies(train_dir):
    out = [("none", {"strategy": "none"}),
           ("static", {"strategy": "static"}),
           ("reactive-rsu", {"strategy": "reactive-rsu"}),
           ("reactive-gap", {"strategy": "reactive-gap"}),
           ("random", {"strategy": "rl", "rlAlgorithm": "random", "mode": "eval"})]
    for model in sorted(glob.glob(os.path.join(train_dir, "seed*", "uav_qtable.csv"))):
        seed = os.path.basename(os.path.dirname(model))
        out.append((f"rl-{seed}", {"strategy": "rl", "rlAlgorithm": "qlearning", "mode": "eval",
                                   "modelFile": model}))
    return out


def cases(suite, a):
    failures = [float(f) for f in a.failure_times.split(",")]
    runs = [int(r) for r in a.runs.split(",")]
    grid = [(f, r) for f in failures for r in runs]
    if suite == "main":
        return [(d, {"failedRsu": 2}, f, r) for d in ("low", "medium", "high") for f, r in grid]
    if suite == "speed":
        return [(d, {"failedRsu": 2}, f, r) for d in ("medium_slow", "medium_fast") for f, r in grid]
    if suite == "location":
        return [("medium", {"failedRsu": k}, f, r) for k in (1, 3) for f, r in grid]
    if suite == "randstart":
        return [("medium", {"failedRsu": 2, "randomUavStart": 1}, f, r) for f, r in grid]
    raise SystemExit(f"unknown suite {suite}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--suites", default="main")
    ap.add_argument("--strategies", default="", help="comma list (none,static,...,random,rl); default all")
    ap.add_argument("--failure-times", default="20,25,30")
    ap.add_argument("--runs", default="1,2")
    ap.add_argument("--variant", default="5")
    ap.add_argument("--simulation-time", type=float, default=125.0)
    ap.add_argument("--train-dir", default=os.path.join(REPO, "results", "train"))
    ap.add_argument("--output", default=os.path.join(REPO, "results", "eval"))
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    a = ap.parse_args()

    strats = strategies(a.train_dir)
    if a.strategies:
        wanted = a.strategies.split(",")
        strats = [s for s in strats if any(s[0] == w or (w == "rl" and s[0].startswith("rl-")) for w in wanted)]
    if not any(name.startswith("rl-") for name, _ in strats):
        print(f"warning: no trained models found in {a.train_dir}; evaluating baselines only")

    jobs = []
    for suite in a.suites.split(","):
        for density, extra, failure, run in cases(suite, a):
            for name, sargs in strats:
                case = f"f{extra['failedRsu']}_t{failure:g}_r{run}"
                out = os.path.join(a.output, suite, density, name, case)
                args = {"mobility": "sumo", "vehicleDensity": density, "traceVariants": a.variant,
                        "failureTime": failure, "simulationTime": a.simulation_time, "seed": 1, "run": run,
                        "netanim": 0, "flowmonXml": 0, "routingTables": 0, "verbose": 0,
                        "outputDir": out, "runLabel": f"{suite}:{name}"}
                args.update(extra)
                args.update(sargs)
                jobs.append((out, args))

    todo = [j for j in jobs if not os.path.exists(os.path.join(j[0], "recovery_results.csv"))]
    print(f"{len(jobs)} runs ({len(jobs) - len(todo)} already done), {a.jobs} parallel jobs", flush=True)

    failed = []

    def work(job):
        out, args = job
        rc, dt = run_sim(args, os.path.join(out, "run.log"))
        if rc != 0:
            failed.append(out)
        return dt

    done = 0
    with ThreadPoolExecutor(max_workers=a.jobs) as pool:
        for _ in pool.map(work, todo):
            done += 1
            if done % 20 == 0 or done == len(todo):
                print(f"  {done}/{len(todo)} runs finished", flush=True)
    if failed:
        print(f"{len(failed)} runs FAILED, e.g. {failed[0]}/run.log")
        sys.exit(1)
    print("evaluation finished")


if __name__ == "__main__":
    main()
