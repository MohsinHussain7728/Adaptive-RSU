#!/usr/bin/env python3
"""Q-learning (or any registered RL algorithm) TRAINING driver.

Each training seed owns one model file and is trained sequentially; seeds run
in parallel. Training is split into short blocks (one simulation process per
block, --trainingEpisodes=<block>) so that consecutive blocks can change the
mobility scenario (density, SUMO variant) and the failure time while the agent
keeps learning in the SAME model file:

    block k: density = densities[k % D], variant = variants[(k // D) % V],
             failureTime ~ U[failure_min, failure_max], UAV start ~ U(area)

The global episode index is passed with --episodeIndex so that the epsilon
decay continues across blocks. Output per seed: results/train/seed<k>/
(uav_qtable.csv, episode_results.csv, uav_actions.csv, ...).

    scripts/py.sh scripts/experiments/train.py --seeds 1,2,3 --episodes 450
"""
import argparse
import csv
import os
import random
import sys
from concurrent.futures import ProcessPoolExecutor

from common import REPO, run_sim


RESULT_CSVS = ("episode_results.csv", "performance_results.csv", "recovery_results.csv",
               "pdr_results.csv", "uav_actions.csv")


def trained_episodes(model_path):
    """episodes_trained from the Q-table header (0 if there is no model yet)."""
    if not os.path.exists(model_path):
        return 0
    with open(model_path) as f:
        header = f.readline().lstrip("# ").strip()
    fields = dict(kv.split("=", 1) for kv in header.split(",") if "=" in kv)
    return int(fields.get("episodes_trained", 0))


def drop_rows_after(out, last_run):
    """Removes CSV rows of runs > last_run (an interrupted block wrote them, the model does not include them)."""
    for name in RESULT_CSVS:
        path = os.path.join(out, name)
        if not os.path.exists(path):
            continue
        with open(path, newline="") as f:
            rows = list(csv.reader(f))
        if not rows:
            continue
        col = rows[0].index("run")
        kept = [rows[0]] + [r for r in rows[1:] if int(r[col]) <= last_run]
        with open(path, "w", newline="") as f:
            csv.writer(f, lineterminator="\n").writerows(kept)


def train_seed(seed, a):
    out = os.path.join(a.output, f"seed{seed}")
    os.makedirs(out, exist_ok=True)
    rng = random.Random(seed)
    densities = a.densities.split(",")
    variants = a.variants.split(",")
    blocks = (a.episodes + a.block - 1) // a.block
    elapsed = 0.0
    done = 0
    resume_at = 0
    if a.resume:
        resume_at = trained_episodes(os.path.join(out, "uav_qtable.csv"))
        drop_rows_after(out, 100000 * seed + resume_at)
        print(f"[seed {seed}] resuming after {resume_at} trained episodes", flush=True)
    for k in range(blocks):
        n = min(a.block, a.episodes - done)
        failure = round(rng.uniform(a.failure_min, a.failure_max), 1)
        if done + n <= resume_at:
            done += n
            continue
        args = {
            "mobility": "sumo",
            "vehicleDensity": densities[k % len(densities)],
            "traceVariants": variants[(k // len(densities)) % len(variants)],
            "strategy": "rl",
            "rlAlgorithm": a.algorithm,
            "mode": "train",
            "trainingEpisodes": n,
            "episodeIndex": done,
            "alpha": a.alpha,
            "gamma": a.gamma,
            "epsilon": a.epsilon,
            "epsilonMin": a.epsilon_min,
            "epsilonDecay": a.epsilon_decay,
            "randomUavStart": 1,
            "failureTime": failure,
            "simulationTime": failure + 90,
            "stopAfterEpisode": 0,
            "seed": seed,
            "run": 100000 * seed + done + 1,
            "saveEvery": 0,
            "netanim": 0,
            "flowmonXml": 0,
            "routingTables": 0,
            "verbose": 0,
            "outputDir": out,
            "runLabel": f"train_seed{seed}",
        }
        rc, dt = run_sim(args, os.path.join(out, "train.log"))
        if rc != 0:
            print(f"[seed {seed}] block {k} failed (rc={rc}); see {out}/train.log", flush=True)
            return seed, False
        done += n
        elapsed += dt
        if k % 10 == 0 or done == a.episodes:
            print(f"[seed {seed}] {done}/{a.episodes} episodes, {elapsed / 60:.1f} min", flush=True)
    return seed, True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", default="1,2,3")
    ap.add_argument("--episodes", type=int, default=450)
    ap.add_argument("--block", type=int, default=5, help="episodes per simulation process")
    ap.add_argument("--densities", default="low,medium,high")
    ap.add_argument("--variants", default="1,2,3,4", help="SUMO variants used for training (5 is held out)")
    ap.add_argument("--failure-min", type=float, default=10.0)
    ap.add_argument("--failure-max", type=float, default=35.0)
    ap.add_argument("--algorithm", default="qlearning")
    ap.add_argument("--alpha", type=float, default=0.1)
    ap.add_argument("--gamma", type=float, default=0.9)
    ap.add_argument("--epsilon", type=float, default=0.2)
    ap.add_argument("--epsilon-min", type=float, default=0.02)
    ap.add_argument("--epsilon-decay", type=float, default=0.995)
    ap.add_argument("--output", default=os.path.join(REPO, "results", "train"))
    ap.add_argument("--resume", action="store_true",
                    help="continue each seed from the episodes_trained stored in its uav_qtable.csv")
    a = ap.parse_args()

    seeds = [int(s) for s in a.seeds.split(",")]
    with ProcessPoolExecutor(max_workers=len(seeds)) as pool:
        results = list(pool.map(train_seed, seeds, [a] * len(seeds)))
    ok = all(r[1] for r in results)
    print("training finished" if ok else "training FAILED for some seeds")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
