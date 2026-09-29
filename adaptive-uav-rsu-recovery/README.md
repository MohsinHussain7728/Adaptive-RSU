# Adaptive UAV-Assisted VANET Recovery using Reinforcement Learning

ns-3.47 + SUMO 1.18 simulation of an urban IEEE 802.11p VANET in which a
Road-Side Unit (RSU) fails and a UAV relay restores end-to-end communication.
After the failure the UAV repeatedly **observes** the network, a **generic RL
agent** (Q-learning by default) chooses a **relative movement** (N/S/E/W/Stay
plus diagonals, 40 m per step), the UAV executes it, the environment measures
the new network state and computes a reward, and the agent learns — until the
windowed PDR recovers or `maxRecoverySteps` is reached.

There are **no predefined or candidate UAV positions** and no hard-coded final
location anywhere in the code: the UAV only ever moves relative to where it is,
and the place where it ends up is a consequence of the measured network state.

---

## 1. Toolchain (removable, lives in `C:\ns3-uav-lab`)

Everything (Ubuntu 24.04, g++/CMake/Ninja, SUMO 1.18.0, Python venv, ns-3.47)
is installed into a dedicated WSL2 distro whose virtual disk is stored in
`C:\ns3-uav-lab`. Nothing is installed into Windows itself.

```powershell
# install (PowerShell, from the repository root)
powershell -ExecutionPolicy Bypass -File .\scripts\setup\install_toolchain_windows.ps1 -Jobs 12
# remove everything again when done
powershell -ExecutionPolicy Bypass -File .\scripts\setup\remove_toolchain_windows.ps1
```

`scripts/setup/bootstrap_ubuntu.sh` (run automatically) installs the packages,
builds ns-3.47 (optimized; modules core, network, mobility, wifi, internet,
aodv, applications, flow-monitor, netanim) as the unprivileged user
`researcher`, and creates `/opt/ns3-uav-lab/venv` (numpy, pandas, matplotlib,
tabulate). The older manual WSL/DEFIANCE setup notes are kept in
[docs/ENVIRONMENT_SETUP_WSL.md](docs/ENVIRONMENT_SETUP_WSL.md).

All project scripts are bash scripts that run inside the distro. From Windows,
`scripts/wsl.ps1` runs them in the distro (quote comma lists in PowerShell):

```powershell
.\scripts\wsl.ps1 scripts/build.sh                 # copy scratch/uav-rsu-rl into ns-3 and build
.\scripts\wsl.ps1 scripts/test_rl.sh               # RL-layer unit test (no ns-3 involved)
.\scripts\wsl.ps1 scripts/generate_scenarios.sh    # regenerate all SUMO scenarios
.\scripts\wsl.ps1 scripts/run_sim.sh --strategy=reactive-gap --vehicleDensity=medium
.\scripts\wsl.ps1 scripts/py.sh scripts/experiments/train.py --seeds '1,2,3' --episodes 450
.\scripts\wsl.ps1 scripts/py.sh scripts/experiments/evaluate.py --suites 'main,speed,location,randstart'
.\scripts\wsl.ps1 scripts/py.sh scripts/experiments/analyze.py
```

---

## 2. Repository layout

```text
scratch/uav-rsu-rl/                 ns-3 scratch program (copied into $NS3_HOME/scratch by build.sh)
  uav-rsu-rl.cc                     entry point: scenario, radio, AODV, traffic, episode loop
  rl/                               GENERIC RL LAYER (no ns-3 includes)
    rl-agent.h/.cc                  RLAgent interface: SelectAction/Update/SaveModel/LoadModel/Reset
    qlearning-agent.h/.cc           Q-LEARNING IMPLEMENTATION (epsilon-greedy, CSV Q-table)
    random-agent.h/.cc              random-walk agent (lower-bound baseline)
    rl-agent-factory.h/.cc          name -> agent ("qlearning", "random"; add new algorithms here)
  env/                              ENVIRONMENT LAYER
    sim-config.*                    all parameters + command line
    scenario.*                      geometry, RSU layout, static debug layout, scenario_info.cfg
    rsu-manager.*                   RSUs, failure injection (PHY off), failure listeners
    uav-controller.*                relative actions, flight, operational-area clamping
    wifi-radio.h                    safe PHY switch-off (works around an ns-3.47 MAC crash)
    network-monitor.*               per-packet windows, forwarding counters, AODV overhead, drop diagnostics
    topology-observer.*             connectivity graph (components, UAV links, gap)
    state-encoder.*                 observation -> discrete state id + feature vector
    reward-function.*               reward computed from measured network state
    recovery-strategy.*             none / static / reactive-rsu / reactive-gap / RL strategy
    recovery-environment.*          the observe-decide-act-measure-learn loop, CSV logging
  experiment/                       CSV writer, NetAnim labels/colours
  tests/test_rl_agents.cpp          unit test of rl/ only
scratch/sumo-scenario/              reproducible SUMO pipeline
  generate_scenario.py              network -> trips -> duarouter -> sumo FCD -> ns-2 trace
  generate_all.sh                   low / medium / high / medium_slow / medium_fast, 5 variants each
  fcd_to_ns2.py, check_trace.py     FCD -> ns-2 conversion, trace sanity check
  network/                          city.nod/edg/net.xml, rsus.add.xml, landmarks.poly.xml
  <scenario>/                       vtypes.add.xml, trips_*.trips.xml, routes_*.rou.xml,
                                    scenario_*.sumocfg, mobility_*.tcl, scenario_info.cfg, rsus.csv
scripts/                            build/run helpers, setup/, experiments/ (train, evaluate, analyze)
legacy/baseline-scratch/            the original baseline ns-3 program (kept for reference)
```

---

## 3. Scenario

* **Road network** (SUMO): two 400 m × 400 m grid districts (2 × 2 blocks of
  200 m, one lane per direction) separated by a 400 m river/park gap without
  roads. West `x ∈ [0,400]`, East `x ∈ [800,1200]`, `y ∈ [0,400]`.
* **Vehicles**: the first ⌈N/2⌉ vehicles drive random tours in the West district,
  the rest in the East district. Vehicle 0 is the traffic source, vehicle N−1 the
  destination (UDP echo, 1024 B every 50 ms, port 9).
* **RSUs** (5): RSU0 (200,200), RSU1 (400,100), **RSU2 (600,200) = bridge over
  the gap**, RSU3 (800,300), RSU4 (1000,200). RSU2 fails at `failureTime`
  (PHY switched off). Because nothing else spans the gap, the failure genuinely
  partitions the network — this is verified, not assumed (see §6).
* **Radio**: 802.11p (10 MHz, OFDM 6 Mbit/s), 23 dBm, log-distance path loss
  (exponent 2) with a hard 250 m range. **AODV** routing.
* **ARP**: caches are pre-populated with PERMANENT entries (`--staticArp=1`).
  With dynamic ARP, ns-3 marks a neighbour DEAD for 100 s after three lost ARP
  requests, which black-holed the destination for whole runs on the busy shared
  channel. PERMANENT (not ns-3's auto-generated) entries are required because
  AODV maps MAC transmit failures to neighbours only through ALIVE/PERMANENT ARP
  entries; without that feedback a failed next hop is never declared broken.
* **UAV**: altitude 50 m, 20 m/s, starts at (200,300) (or `--uavX/--uavY`, or
  uniformly random with `--randomUavStart=1`), bounded to the operational area
  `[-100,1300] × [-100,500]`.
* **Densities**: low 20, medium 40, high 60 vehicles (50 km/h); medium_slow
  (30 km/h) and medium_fast (70 km/h) for speed sensitivity. Five SUMO seeds
  (mobility variants) per scenario. Variants 1–4 are used for training,
  variant 5 only for evaluation.

---

## 4. The RL design

### 4.1 Generic agent interface (`rl/rl-agent.h`)

```cpp
class RLAgent {
    virtual uint32_t SelectAction(const Observation& s) = 0;   // epsilon-greedy etc.
    virtual void     Update(const Transition& t) = 0;          // (s, a, r, s', terminal)
    virtual bool     SaveModel(const std::string& path) const = 0;
    virtual bool     LoadModel(const std::string& path) = 0;
    virtual void     Reset() = 0;
    virtual void     SetTrainingMode(bool training) = 0;       // eval: no updates, eval epsilon
    ...
};
```

`Observation` carries both a discrete state id (tabular methods) and a
real-valued feature vector (function-approximation methods such as DQN). The
simulation creates agents only via `CreateRLAgent(name, AgentConfig)` and never
touches Q-values. To add SARSA/Double-Q/DQN: implement `RLAgent`, register the
name in `rl/rl-agent-factory.cc`, run with `--rlAlgorithm=<name>`.

### 4.2 Q-learning (`rl/qlearning-agent.cc`)

* `Q(s,a) ← Q(s,a) + α [r + γ max_a' Q(s',a') − Q(s,a)]`, terminal:
  `Q(s,a) ← Q(s,a) + α [r − Q(s,a)]`. Timeout (maxRecoverySteps) is a
  truncation, not a terminal state, so it bootstraps.
* α = 0.1, γ = 0.9, ε = 0.2 (all CLI-configurable); optional per-episode decay
  `--epsilonDecay` down to `--epsilonMin`; greedy ties broken randomly.
* Q-table persisted as CSV (`uav_qtable.csv`): header with hyper-parameters,
  then per state the Q-values, visit counts and best action.

### 4.3 State (`env/state-encoder.cc`, environment side)

Relative, position-free encoding (243 states with the default 1 distance bin):

* **source side**: `LINKED` if the UAV has a link into the source's ground
  component, otherwise the direction (8 sectors) from the UAV to the nearest
  node of that component;
* **destination side**: same for the destination's component;
* **PDR level**: low (< 0.2) / medium (< `pdrThreshold`) / high.

The feature vector (11 values: cos/sin of the bearing and normalized distance
to the source and destination components, the two UAV link flags, the windowed
PDR and the normalized UAV position) is provided for non-tabular agents.

### 4.4 Actions (`env/uav-controller.cc`)

A0 N, A1 S, A2 E, A3 W, A4 Stay, A5 NE, A6 NW, A7 SE, A8 SW
(`--numActions=5` for the four directions + Stay). Step `--movementStep`
(default 40 m, 20–50 m sensible); moves are clamped to the operational area and
a boundary hit is penalized.

### 4.5 Reward (`env/reward-function.cc`, environment side)

`r = 2.0·ΔPDR + 0.5·PDR + 1.0·connectivity + 0.5·Δthroughput − 0.3·delay −
0.5·loss − 0.1·move − 0.3·unnecessary move − 0.5·boundary + shaping
(+5 success, −2 timeout)`; shaping is potential-based
(`γΦ(s') − Φ(s)`, Φ = −(max(0, d_src − R) + max(0, d_dst − R))/R with d the
UAV's distance to the source/destination component and R the link range;
Φ = 0 at the terminal state), which does not change the optimal policy.
Connectivity scores +1 when the observer sees an end-to-end path, otherwise
−0.4 + 0.3 per UAV link to the source/destination component. ΔPDR/Δthroughput
compare the measurement windows before and after the move. All weights are CLI
parameters (§4.8).

### 4.6 Recovery loop (`env/recovery-environment.cc`)

```
failure -> (detectionDelay) -> episode start
repeat every stepInterval (3 s):
    observe (windowed PDR/delay/throughput of the monitored flow, topology, AODV route)
    encode state -> strategy/agent selects relative action -> UAV flies (2 s)
    measure window [t-0.8-0.2, t-0.2) -> reward -> agent.Update (training only)
until windowed PDR >= pdrThreshold (0.8) or maxRecoverySteps (25)
```

After a recovery the loop keeps watching; if the PDR degrades again for
`degradationWindows` windows (vehicles moved), a new recovery episode starts.

### 4.7 Training vs evaluation

* `--mode=train`: exploration, Q-updates, model saved every `--saveEvery`
  episodes and at the end. `--trainingEpisodes=N` runs N independent ns-3
  simulations in one process with the agent kept in memory.
* `--mode=eval`: loads the model (required), ε = `--evalEpsilon` (0), no
  updates — the Q-table file is never written (verified by the unit test).

### 4.8 Command-line parameters (defaults; full list with `--PrintHelp`)

| group | parameters |
|---|---|
| scenario | `--mobility=sumo` (or `static` debug layout), `--vehicleDensity=low` (low/medium/high/medium_slow/medium_fast), `--traceVariants=1` (comma list, episode k uses variant k mod n), `--traceFile`, `--numVehicles=0` (all), `--numRsus=5`, `--sourceVehicle=0`, `--destinationVehicle=-1` (last) |
| failure | `--failedRsu=2` (−1 none), `--failureTime=20`, `--detectionDelay=1` |
| traffic | `--simulationTime=120`, `--trafficStart=2`, `--packetInterval=0.05`, `--packetSize=1024`, `--backgroundFlows=0`, `--backgroundRate=2`, `--stopAfterEpisode=-1` |
| radio / routing | `--txPowerDbm=23`, `--commRange=250`, `--pathLossExponent=2`, `--phyMode=OfdmRate6MbpsBW10MHz`, `--aodvHelloInterval=1`, `--aodvActiveRouteTimeout=3`, `--staticArp=1` |
| UAV | `--uavX --uavY` (home), `--uavZ=50`, `--randomUavStart=0`, `--uavSpeed=20`, `--movementStep=40`, `--numActions=9` (or 5), `--areaMinX/MaxX/MinY/MaxY` |
| recovery | `--strategy=rl` (none/static/reactive-rsu/reactive-gap/rl), `--mode=train` (or eval), `--maxRecoverySteps=25`, `--stepInterval=3`, `--measureWindow=0.8`, `--deliveryGrace=0.2`, `--pdrThreshold=0.8`, `--recoveryHoldSteps=1`, `--reRecovery=1`, `--degradationWindows=2`, `--linkMargin=0.95`, `--stateEncoder=connectivity`, `--stateDistanceBins=1` |
| RL | `--rlAlgorithm=qlearning` (or random), `--alpha=0.1`, `--gamma=0.9`, `--epsilon=0.2`, `--epsilonMin=0.02`, `--epsilonDecay=1`, `--evalEpsilon=0`, `--initQ=0`, `--trainingEpisodes=1`, `--episodeIndex=0`, `--modelFile` (default `<outputDir>/uav_qtable.csv`), `--saveEvery=10` |
| reward | `--wPdrImprovement=2 --wPdr=0.5` (level) `--wConnectivity=1 --wThroughput=0.5 --wDelay=0.3 --wLoss=0.5 --wMove=0.1 --wUnnecessaryMove=0.3 --wBoundary=0.5 --wShaping=1 --successBonus=5 --timeoutPenalty=2` |
| output | `--outputDir=results`, `--runLabel`, `--netanim=1`, `--animFile=uav-rsu-rl-animation.xml`, `--flowmonXml=1`, `--routingTables=1`, `--monitorInterval=0.5`, `--seed=1`, `--run=1`, `--verbose=1`, `--diagnostics=0` |

---

## 5. Outputs

| file | content |
|---|---|
| `uav_qtable.csv` | Q-table + visit counts + best action |
| `episode_results.csv` | one row per recovery episode (steps, recovered, reward, distance, explorations…) |
| `uav_actions.csv` | one row per UAV decision: state, action, decision type, positions, window metrics, links, AODV route, UAV forwarding, reward components |
| `pdr_results.csv` | 0.5 s time series: phase, UAV status, window PDR, delay, throughput, connectivity, UAV forwarding, AODV packets |
| `performance_results.csv` | phase metrics (normal / failure / recovery / stable, pre_failure, post_failure, overall): PDR, timely PDR, loss, throughput, mean/p95 delay, jitter, AODV packets/bytes, normalized routing load, UAV and failed-RSU forwarding |
| `recovery_results.csv` | one row per run: failure impact, recovery time, steps, distance, final position, UAV forwarding, source route after recovery |
| `flowmon_results.xml` | FlowMonitor (last episode) |
| `uav-rsu-rl-animation.xml` | NetAnim: "Vehicle i", "RSU 2 FAILED", "UAV: RECOVERING step k East", "UAV: RECOVERED" |
| `routing_tables/*.txt` | AODV tables before/after failure and after recovery |

Every CSV row starts with run-context columns (run id, strategy, algorithm,
scenario, trace variant, seed, run, failed RSU, episode) so files from many runs
can be concatenated.

---

## 6. Verification performed

* `scripts/test_rl.sh`: exact update rule, greedy optimality on a corridor MDP,
  evaluation leaves the Q-table byte-identical, save/load round trip, dimension
  mismatch rejected.
* `--diagnostics=1` prints per-node forwarding and IPv4/MAC/PHY drop counters.
  Before the failure the flow is relayed V0 → … → RSU0 → RSU1 → **RSU2** →
  RSU3 → RSU4 → … → V19 (RSU2 forwards every data packet). After the failure
  RSU2 forwards nothing, the source has no AODV route and the PDR is 0 for the
  no-UAV baseline. After a UAV recovery the UAV node forwards the flow and the
  source route is valid again (`route_next_hop_after_recovery`,
  `uav_forwarded_after_failure`, routing-table dumps).

---

* **ns-3.47 crash when a radio is switched off mid-broadcast.** For frames
  without acknowledgment `FrameExchangeManager::SendMpdu` schedules
  `TransmissionSucceeded()` without keeping the event; `WifiPhy::SetOffMode()`
  resets the MAC (`m_dcf = nullptr`) but the event still fires → segfault
  (hit once in ~1350 training episodes, when RSU2 was sending an AODV HELLO at
  the failure instant). `env/wifi-radio.h` defers the switch-off to the end of
  the ongoing transmission (≤ 1.5 ms). The RSU is marked failed and listeners
  are notified at the exact failure time.

---

## 7. Experiments

```powershell
# 1) training: 3 independent seeds x 450 episodes (blocks of 5 episodes that cycle
#    low/medium/high density and SUMO variants 1-4, failure time ~ U[10,35] s,
#    random UAV start, epsilon 0.2 -> 0.02 with decay 0.995/episode); ~75 min on 16 cores
.\scripts\wsl.ps1 scripts/py.sh scripts/experiments/train.py --seeds '1,2,3' --episodes 450
#    (add --resume to continue interrupted seeds from the episodes stored in uav_qtable.csv)
# 2) evaluation on the held-out SUMO variant 5: every strategy on the same cases
.\scripts\wsl.ps1 scripts/py.sh scripts/experiments/evaluate.py --suites 'main,speed,location,randstart'
# 3) tables and figures -> results/analysis/ (summary.md, summary_*.csv, fig_*.png)
.\scripts\wsl.ps1 scripts/py.sh scripts/experiments/analyze.py
```

Evaluation suites (simulation 125 s, failure times 20/25/30 s × runs 1/2 = 6
cases per scenario; RL = 3 trained models × 6 cases, greedy, no updates):

* **main**: RSU2 fails, low/medium/high density;
* **speed**: RSU2 fails, medium_slow (30 km/h) / medium_fast (70 km/h);
* **location**: RSU1 or RSU3 fails instead (medium);
* **randstart**: RSU2 fails, UAV starts at a random position (medium).

Strategies: `none` (UAV radio off), `static` (UAV hovers at its start),
`reactive-rsu` (greedy flight to the failed RSU's known location),
`reactive-gap` (greedy flight to the midpoint of the narrowest gap between the
source and destination components, using global knowledge of all node
positions), `random` (random relative actions), `rl-seed1..3` (Q-learning).

---

## 8. Results (held-out SUMO variant 5)

Recovery = windowed PDR ≥ 0.8 within 25 steps. Recovery time is measured from
the failure (it includes the 1 s detection delay) and averaged over the runs
that recovered. Timely PDR counts packets delivered within 200 ms
(`--deliveryGrace`), over the whole post-failure period. Stable PDR is the
timely PDR after recovery.

| scenario | strategy | recovered | recovery time [s] | UAV distance [m] | post-failure timely PDR | stable PDR |
|---|---|---|---|---|---|---|
| low | none / static / random | 0 / 6 | – | 0 / 0 / 1067 | 0.00 | – |
| low | reactive-rsu | 6 / 6 | 34.5 | 440 | **0.63** | **0.95** |
| low | reactive-gap | 6 / 6 | 41.0 | 553 | 0.38 | 0.72 |
| low | Q-learning | 18 / 18 | **34.5** | 593 | 0.47 | 0.84 |
| medium | none / static / random | 0 / 6 | – | 0 / 0 / 1067 | 0.00 | – |
| medium | reactive-rsu | 6 / 6 | 36.5 | 433 | **0.55** | **0.88** |
| medium | reactive-gap | 6 / 6 | 43.0 | 567 | 0.42 | 0.76 |
| medium | Q-learning | 18 / 18 | **34.8** | 591 | 0.45 | 0.81 |
| high | none / static / random | 0 / 6 | – | 0 / 0 / 1067 | 0.00 | – |
| high | reactive-rsu | 6 / 6 | 36.5 | 440 | 0.36 | **0.64** |
| high | reactive-gap | 6 / 6 | 37.5 | 487 | 0.28 | 0.59 |
| high | Q-learning | 18 / 18 | **36.3** | 580 | **0.39** | **0.64** |
| medium, random start | reactive-rsu | 6 / 6 | **34.5** | 440 | **0.55** | **0.86** |
| medium, random start | reactive-gap | 5 / 6 | 37.6 | 547 | 0.41 | 0.71 |
| medium, random start | Q-learning | 18 / 18 | 43.0 | 618 | 0.47 | 0.86 |
| medium_fast (70 km/h) | reactive-rsu / reactive-gap | 6 / 6, 6 / 6 | 33.0 / 39.0 | 440 / 473 | **0.59** / 0.54 | **0.91** / 0.89 |
| medium_fast (70 km/h) | Q-learning | 18 / 18 | **32.3** | 444 | 0.57 | 0.87 |
| medium_slow (30 km/h) | reactive-rsu / reactive-gap | 6 / 6, 6 / 6 | 34.0 / 38.5 | 440 / 553 | **0.49** / 0.44 | **0.81** / 0.79 |
| medium_slow (30 km/h) | Q-learning | 18 / 18 | **32.5** | 530 | 0.43 | 0.74 |

Training (moving average, first → last 20 % of 450 episodes): recovery rate
0.39 → 0.96, 0.31 → 0.99, 0.56 → 0.98; steps per episode ≈ 20 → 11
(`fig_learning_curve.png`).

**Interpretation**

* The failure of the bridge RSU is real and total: PDR drops from 0.96–0.99
  (0.76 at high density) to exactly 0 for `none`/`static`, the source has no
  AODV route, and it stays that way until the end of the simulation. A UAV that
  does not move (`static`) or moves randomly never restores communication.
* The learned Q-learning policies recover **every** evaluated bridge failure
  (108/108 runs, 3 independent models) on a mobility trace never seen in
  training, without knowing where the failed RSU was.
* Q-learning is **faster than the knowledge-free reactive-gap heuristic** in
  every scenario except random start (by 1–8 s), and **matches the
  reactive-rsu heuristic** (which is given the failed RSU's position). All
  competent strategies are close to the physical lower bound: the UAV must fly
  ≈ 400 m (≈ 10 steps of 40 m, 3 s each) from its home to the gap.
* Q-learning is **not uniformly better**. The reactive-rsu baseline has a
  higher post-failure and stable PDR in most scenarios and flies less: it
  parks right where the RSU was, whereas the RL UAV keeps making small
  corrective moves after recovery (larger distance, some re-recoveries). From
  random start positions RL is slower than both heuristics (43 s vs 34.5/37.6 s),
  and one of the three models (seed 1) is clearly weaker there (50.5 s,
  781 m). The coarse 243-state encoding and 450 training episodes are the
  likely limits.
* Non-bridge failures (location suite: RSU1 or RSU3) do **not** partition the
  network in this layout. Vehicles at the district edge are within range of
  RSU2, so even `none` keeps a PDR of ≈ 0.91 after the failure and all strategies
  are equivalent. The RL UAV still moves ≈ 124 m on average.
* In high density, runs with a UAV node have a lower *pre-failure* PDR (0.54)
  than `none` (0.76, no UAV radio). The difference is large and the variance is
  high (std 0.31 over 6 cases), so the UAV acting as an extra (sometimes poor)
  AODV relay on a congested channel is a likely cause. It affects all UAV
  strategies equally.

Detailed tables: `results/analysis/summary_recovery.csv`,
`summary_recovery_by_model.csv` (per model, with std), `summary_phases.csv`.
Figures: `fig_recovery_rate`, `fig_recovery_time`, `fig_movement_cost`,
`fig_post_failure_pdr`, `fig_overhead`, `fig_phase_pdr`, `fig_pdr_timeseries`,
`fig_trajectories`, `fig_learning_curve`.

### Demo run with NetAnim

```powershell
.\scripts\wsl.ps1 scripts/run_sim.sh --mobility=sumo --vehicleDensity=medium --traceVariants=5 `
  --failureTime=20 --simulationTime=125 --run=1 --strategy=rl --mode=eval `
  --modelFile=results/train/seed3/uav_qtable.csv --netanim=1 --flowmonXml=1 --routingTables=1 `
  --verbose=1 --diagnostics=1 --outputDir=results/demo/rl-seed3
```

Output in `results/demo/rl-seed3/`: the failure hits at 20 s, RSU2 relayed 718
packets before and 0 after it. The UAV recovers the flow after 31 s and 10 steps
(it first forwards data at 50.6 s), the source's next hop is `UAV` (4 hops), and
the UAV forwards 1513 packets. Open `uav-rsu-rl-animation.xml` in NetAnim.

### Visual inspection

Both viewers run inside the distro and open as normal Windows windows (WSLg):

```powershell
.\scripts\wsl.ps1 scripts/view_sumo.sh medium 5      # SUMO traffic of a scenario/variant in sumo-gui
.\scripts\wsl.ps1 scripts/view_netanim.sh            # NetAnim; open the printed .xml, press Play
```

The NetAnim viewer is built by `scripts/setup/build_netanim.sh` (Qt5, into
`/opt/ns3-uav-lab/netanim`, called by the installer).
