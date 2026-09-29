"""Shared helpers for the experiment drivers (run inside the ns3-uav-lab distro)."""
import os
import subprocess
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SIM_BIN = os.environ.get(
    "SIM_BIN",
    "/opt/ns3-uav-lab/ns-allinone-3.47/ns-3.47/build/scratch/uav-rsu-rl/ns3.47-uav-rsu-rl-optimized")


def run_sim(args, log_path=None):
    """Runs one simulation process from the repo root; returns (returncode, seconds)."""
    cmd = [SIM_BIN] + [f"--{k}={v}" for k, v in args.items()]
    t0 = time.time()
    if log_path:
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        with open(log_path, "a") as log:
            log.write("$ " + " ".join(cmd) + "\n")
            log.flush()
            rc = subprocess.run(cmd, cwd=REPO, stdout=log, stderr=subprocess.STDOUT).returncode
    else:
        rc = subprocess.run(cmd, cwd=REPO).returncode
    return rc, time.time() - t0
