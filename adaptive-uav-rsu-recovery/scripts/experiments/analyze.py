#!/usr/bin/env python3
"""Merges training/evaluation CSVs and produces summary tables and figures.

Inputs : results/train/seed*/episode_results.csv
         results/eval/**/{recovery,performance,pdr,uav_actions}_results.csv
Outputs: results/analysis/
         summary_recovery.csv    per suite/scenario/strategy (mean, std, n)
         summary_phases.csv      phase-based PDR/delay/throughput/overhead
         training_summary.csv    learning progress per seed
         summary.md              the tables above in Markdown
         fig_learning_curve.png, fig_recovery_rate.png, fig_recovery_time.png,
         fig_phase_pdr.png, fig_movement_cost.png, fig_overhead.png,
         fig_pdr_timeseries.png, fig_trajectories.png

    scripts/py.sh scripts/experiments/analyze.py
"""
import argparse
import glob
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from common import REPO  # noqa: E402

STRATEGY_ORDER = ["none", "static", "reactive-rsu", "reactive-gap", "random", "rl"]
DENSITY_ORDER = ["low", "medium", "high", "medium_slow", "medium_fast"]


def load(pattern):
    frames = []
    for path in glob.glob(pattern, recursive=True):
        try:
            df = pd.read_csv(path)
        except pd.errors.EmptyDataError:
            continue
        rel = os.path.relpath(path, os.path.join(REPO, "results", "eval")).split(os.sep)
        if len(rel) >= 5:
            df["suite"], df["density"], df["strategy_tag"], df["case"] = rel[0], rel[1], rel[2], rel[3]
        frames.append(df)
    return pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()


def family(tag):
    return "rl" if tag.startswith("rl-") else tag


def ordered(values, order):
    return [v for v in order if v in values] + sorted(v for v in values if v not in order)


def mean_std(df, keys, cols):
    g = df.groupby(keys)
    out = g[cols].agg(["mean", "std"])
    out.columns = [f"{c}_{s}" for c, s in out.columns]
    out["n"] = g.size()
    return out.reset_index()


def learning_curves(train_dir, out_dir, window):
    frames = []
    for path in sorted(glob.glob(os.path.join(train_dir, "seed*", "episode_results.csv"))):
        df = pd.read_csv(path)
        df = df[df["episode_in_run"] == 0].copy()  # one recovery episode per training simulation
        df["seed"] = os.path.basename(os.path.dirname(path))
        frames.append(df.sort_values("global_episode"))
    if not frames:
        return None
    tr = pd.concat(frames, ignore_index=True)
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    for seed, df in tr.groupby("seed"):
        x = df["global_episode"]
        axes[0].plot(x, df["recovered"].rolling(window, min_periods=1).mean(), label=seed)
        axes[1].plot(x, df["total_reward"].rolling(window, min_periods=1).mean(), label=seed)
        axes[2].plot(x, df["steps"].rolling(window, min_periods=1).mean(), label=seed)
    for ax, t in zip(axes, ["recovery rate", "episode return", "steps per episode"]):
        ax.set_title(f"Training: {t} (moving avg {window})")
        ax.set_xlabel("training episode")
        ax.grid(alpha=0.3)
        ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig_learning_curve.png"), dpi=150)
    plt.close(fig)

    rows = []
    for seed, df in tr.groupby("seed"):
        n = len(df)
        q = max(1, n // 5)
        first, last = df.head(q), df.tail(q)
        rows.append({"seed": seed, "episodes": n,
                     "recovery_rate_first20pct": first["recovered"].mean(),
                     "recovery_rate_last20pct": last["recovered"].mean(),
                     "mean_steps_first20pct": first["steps"].mean(),
                     "mean_steps_last20pct": last["steps"].mean(),
                     "return_first20pct": first["total_reward"].mean(),
                     "return_last20pct": last["total_reward"].mean()})
    summary = pd.DataFrame(rows)
    summary.to_csv(os.path.join(out_dir, "training_summary.csv"), index=False)
    return summary


def bar_plot(summary, metric, ylabel, path, suite="main"):
    s = summary[summary["suite"] == suite]
    if s.empty:
        return
    dens = ordered(s["density"].unique(), DENSITY_ORDER)
    strats = ordered(s["family"].unique(), STRATEGY_ORDER)
    x = np.arange(len(dens))
    w = 0.8 / len(strats)
    fig, ax = plt.subplots(figsize=(10, 4.5))
    for i, st in enumerate(strats):
        vals, errs = [], []
        for d in dens:
            r = s[(s["density"] == d) & (s["family"] == st)]
            vals.append(r[f"{metric}_mean"].iloc[0] if len(r) else np.nan)
            errs.append(r[f"{metric}_std"].iloc[0] if len(r) else 0)
        ax.bar(x + i * w - 0.4 + w / 2, vals, w, yerr=np.nan_to_num(errs), capsize=3, label=st)
    ax.set_xticks(x)
    ax.set_xticklabels(dens)
    ax.set_ylabel(ylabel)
    ax.grid(axis="y", alpha=0.3)
    ax.legend(ncol=3, fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def timeseries_plot(pdr, out_dir, density="medium", case="f2_t20_r1"):
    s = pdr[(pdr["suite"] == "main") & (pdr["density"] == density) & (pdr["case"] == case)]
    if s.empty:
        return
    fig, ax = plt.subplots(figsize=(11, 4.5))
    for tag in ordered(s["strategy_tag"].unique(), ["none", "static", "reactive-rsu", "reactive-gap", "random"]):
        d = s[s["strategy_tag"] == tag].sort_values("time")
        ax.plot(d["time"], d["window_pdr"].rolling(4, min_periods=1).mean(), label=tag, lw=1.3)
    ft = float(case.split("_t")[1].split("_")[0])
    ax.axvline(ft, color="k", ls="--", lw=1)
    ax.text(ft + 0.5, 1.02, "RSU failure", fontsize=8)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("windowed PDR (2 s smoothing)")
    ax.set_ylim(-0.02, 1.08)
    ax.set_title(f"PDR over time, {density} density, case {case}, SUMO variant 5")
    ax.grid(alpha=0.3)
    ax.legend(ncol=4, fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig_pdr_timeseries.png"), dpi=150)
    plt.close(fig)


def trajectory_plot(actions, out_dir, density="medium", case="f2_t20_r1"):
    s = actions[(actions["suite"] == "main") & (actions["density"] == density) & (actions["case"] == case)]
    if s.empty:
        return
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.add_patch(plt.Rectangle((0, 0), 400, 400, fc="#eef", ec="#99c", label="West district"))
    ax.add_patch(plt.Rectangle((800, 0), 400, 400, fc="#efe", ec="#9c9", label="East district"))
    ax.add_patch(plt.Rectangle((420, -50), 360, 500, fc="#cde6ff", ec="none", label="river / park gap"))
    rsus = [(200, 200), (400, 100), (600, 200), (800, 300), (1000, 200)]
    for k, (x, y) in enumerate(rsus):
        ax.plot(x, y, "s", color="k" if k == 2 else "g", ms=8)
        ax.text(x + 8, y + 8, f"RSU{k}" + (" (failed)" if k == 2 else ""), fontsize=8)
    for tag in ordered(s["strategy_tag"].unique(), ["reactive-rsu", "reactive-gap", "random"]):
        d = s[(s["strategy_tag"] == tag) & (s["episode_in_run"] == 0)].sort_values("step")
        if d.empty:
            continue
        xs = np.concatenate([[d["uav_x"].iloc[0]], d["target_x"].values])
        ys = np.concatenate([[d["uav_y"].iloc[0]], d["target_y"].values])
        ax.plot(xs, ys, "-o", ms=3, lw=1.2, label=f"{tag} ({len(d)} steps)")
    ax.set_xlim(-100, 1300)
    ax.set_ylim(-100, 500)
    ax.set_aspect("equal")
    ax.set_title(f"UAV trajectories during the first recovery episode ({density}, {case})")
    ax.legend(fontsize=7, loc="upper right", ncol=2)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig_trajectories.png"), dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results", default=os.path.join(REPO, "results"))
    ap.add_argument("--window", type=int, default=30)
    a = ap.parse_args()
    out = os.path.join(a.results, "analysis")
    os.makedirs(out, exist_ok=True)
    md = ["# Results summary\n"]

    tr = learning_curves(os.path.join(a.results, "train"), out, a.window)
    if tr is not None:
        md += ["## Training (first vs last 20% of episodes)\n", tr.to_markdown(index=False, floatfmt=".3f"), ""]

    ev = os.path.join(a.results, "eval")
    rec = load(os.path.join(ev, "**", "recovery_results.csv"))
    if rec.empty:
        print("no evaluation results found")
    else:
        rec["family"] = rec["strategy_tag"].map(family)
        rec["recovery_time_ok"] = rec["recovery_time_s"].where(rec["recovered"] == 1)
        rec["steps_ok"] = rec["recovery_steps"].where(rec["recovered"] == 1)
        cols = ["recovered", "recovery_time_ok", "steps_ok", "uav_distance_m", "uav_moves", "pdr_normal",
                "pdr_failure", "pdr_post_failure", "timely_pdr_post_failure", "pdr_stable",
                "uav_forwarded_after_failure", "re_recoveries"]
        by_tag = mean_std(rec, ["suite", "density", "strategy_tag"], cols)
        by_tag.to_csv(os.path.join(out, "summary_recovery_by_model.csv"), index=False)
        summary = mean_std(rec, ["suite", "density", "family"], cols)
        summary.to_csv(os.path.join(out, "summary_recovery.csv"), index=False)

        show = ["suite", "density", "family", "n", "recovered_mean", "recovery_time_ok_mean", "steps_ok_mean",
                "uav_distance_m_mean", "pdr_normal_mean", "pdr_failure_mean", "timely_pdr_post_failure_mean",
                "pdr_stable_mean"]
        tbl = summary[show].copy()
        tbl["_o"] = tbl["family"].map({s: i for i, s in enumerate(STRATEGY_ORDER)})
        tbl = tbl.sort_values(["suite", "density", "_o"]).drop(columns="_o")
        md += ["## Recovery (mean over cases; RL = mean over training seeds and cases)\n",
               "recovery time / steps are averaged over recovered runs only.\n",
               tbl.to_markdown(index=False, floatfmt=".3f"), ""]

        bar_plot(summary, "recovered", "recovery rate", os.path.join(out, "fig_recovery_rate.png"))
        bar_plot(summary, "recovery_time_ok", "recovery time after failure [s]",
                 os.path.join(out, "fig_recovery_time.png"))
        bar_plot(summary, "uav_distance_m", "UAV distance flown [m]", os.path.join(out, "fig_movement_cost.png"))
        bar_plot(summary, "timely_pdr_post_failure", "timely PDR after failure",
                 os.path.join(out, "fig_post_failure_pdr.png"))

        perf = load(os.path.join(ev, "**", "performance_results.csv"))
        perf["family"] = perf["strategy_tag"].map(family)
        pcols = ["timely_pdr", "pdr", "mean_delay_ms", "throughput_kbps", "normalized_routing_load", "aodv_packets"]
        phases = mean_std(perf, ["suite", "density", "family", "phase"], pcols)
        phases.to_csv(os.path.join(out, "summary_phases.csv"), index=False)
        main_ph = phases[(phases["suite"] == "main") & phases["phase"].isin(["pre_failure", "post_failure"])]
        md += ["## Phase metrics, main suite (pre_failure vs post_failure)\n",
               main_ph[["density", "family", "phase", "n", "timely_pdr_mean", "mean_delay_ms_mean",
                        "throughput_kbps_mean", "normalized_routing_load_mean"]].to_markdown(index=False,
                                                                                            floatfmt=".3f"), ""]

        post = phases[(phases["suite"] == "main") & (phases["phase"] == "post_failure")].copy()
        post = post.rename(columns={"normalized_routing_load_mean": "nrl_mean", "normalized_routing_load_std": "nrl_std"})
        bar_plot(post.assign(suite="main"), "nrl", "AODV packets per delivered packet (post failure)",
                 os.path.join(out, "fig_overhead.png"))

        phase_names = ["normal", "failure", "recovery", "stable"]
        s = phases[(phases["suite"] == "main") & phases["phase"].isin(phase_names)]
        if not s.empty:
            fig, axes = plt.subplots(1, len(DENSITY_ORDER[:3]), figsize=(15, 4), sharey=True)
            for ax, d in zip(axes, DENSITY_ORDER[:3]):
                sd = s[s["density"] == d]
                strats = ordered(sd["family"].unique(), STRATEGY_ORDER)
                x = np.arange(len(phase_names))
                w = 0.8 / max(1, len(strats))
                for i, st in enumerate(strats):
                    vals = [sd[(sd["family"] == st) & (sd["phase"] == p)]["timely_pdr_mean"].mean()
                            for p in phase_names]
                    ax.bar(x + i * w - 0.4 + w / 2, vals, w, label=st)
                ax.set_xticks(x)
                ax.set_xticklabels(phase_names)
                ax.set_title(f"{d} density")
                ax.grid(axis="y", alpha=0.3)
            axes[0].set_ylabel("timely PDR")
            axes[-1].legend(fontsize=7)
            fig.tight_layout()
            fig.savefig(os.path.join(out, "fig_phase_pdr.png"), dpi=150)
            plt.close(fig)

        pdr = load(os.path.join(ev, "**", "pdr_results.csv"))
        timeseries_plot(pdr, out)
        actions = load(os.path.join(ev, "**", "uav_actions.csv"))
        trajectory_plot(actions, out)

    with open(os.path.join(out, "summary.md"), "w") as f:
        f.write("\n".join(md) + "\n")
    print(f"analysis written to {out}")


if __name__ == "__main__":
    main()
