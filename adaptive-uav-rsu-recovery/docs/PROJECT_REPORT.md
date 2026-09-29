# Project Report: Adaptive UAV-Assisted VANET Recovery using Reinforcement Learning

This document records everything that was done to rebuild the project on this
machine: the toolchain, the code, the design decisions, the bugs that were found
and fixed, every group of simulation runs, and all results. The short user guide
is the repository [README](../README.md); this report is the complete record.

Figures referenced below live in `results/analysis/` (the `results/` folder is
git-ignored, so they exist locally only).

---

## 1. Summary

* A complete, reproducible ns-3.47 + SUMO 1.18.0 simulation was built from
  scratch in a removable toolchain under `C:\ns3-uav-lab`.
* After a Road-Side Unit (RSU) fails, a UAV repeatedly observes the network,
  a generic RL agent (Q-learning) picks a **relative** move (N/S/E/W/Stay +
  diagonals, 40 m), and this repeats until communication recovers or 25 steps
  pass. There are **no predefined or candidate UAV positions** anywhere.
* The RSU failure was verified to really break routing (source loses its AODV
  route, PDR drops to 0), and recovery was verified through actual packet
  forwarding by the UAV node.
* Three Q-learning agents were trained (3 seeds x 450 episodes, ~75 min) and
  evaluated against four baselines on a held-out SUMO mobility trace
  (384 evaluation runs in total).
* **Result:** Q-learning recovered **108/108** bridge-RSU failures. It is 1-8 s
  faster than the knowledge-free reactive heuristic and matches the heuristic
  that is given the failed RSU's position. It is **not** better in every
  metric: it flies more, has a lower post-failure PDR and higher delay than the
  best heuristic, and is slower from random start positions.
* Two real bugs were found and fixed along the way: an ARP/AODV interaction
  that silently black-holed traffic, and an ns-3.47 Wi-Fi MAC crash when a
  radio is switched off mid-broadcast.

---

## 2. Goal and constraints (from the specification)

| requirement | how it is met |
|---|---|
| No predefined/candidate UAV positions, no hard-coded final location | UAV only executes relative moves; the end position is the result of learning |
| Generic RL interface, Q-learning replaceable (SARSA, DQN, ...) | `rl/rl-agent.h` (`SelectAction/Update/SaveModel/LoadModel/Reset`), factory `CreateRLAgent(name)` |
| Simulation never touches Q-values; reward in environment layer; modular state encoder | `env/reward-function.*`, `env/state-encoder.*`; the strategy adapter only forwards observations/transitions |
| Q-learning defaults alpha=0.1, gamma=0.9, epsilon=0.2, configurable; terminal Q += alpha(r-Q); CSV Q-table | `rl/qlearning-agent.cc`, CLI `--alpha --gamma --epsilon`, `uav_qtable.csv` |
| Training and evaluation separate; evaluation epsilon~0, no updates | `--mode=train|eval`, `--evalEpsilon=0`; unit test checks the Q-table is untouched in eval |
| Verify the failure affects routing/forwarding/PDR/delay, and recovery via forwarding | Per-node forwarding counters, IPv4/MAC/PHY drop diagnostics, AODV routing table dumps, `uav_forwarded_after_failure` |
| Reproducible SUMO scenarios at low/medium/high density with varied speed, demand, failure location/time | `scratch/sumo-scenario/` generator, 5 scenarios x 5 variants; evaluation varies failure time and failed RSU |
| NetAnim output with labels | `uav-rsu-rl-animation.xml` ("Vehicle i", "RSU 2 FAILED", "UAV: RECOVERING step k", "UAV: RECOVERED") |
| Output files | `uav_qtable.csv`, `episode_results.csv`, `pdr_results.csv`, `recovery_results.csv`, `performance_results.csv`, `uav_actions.csv`, `flowmon_results.xml` |
| Phase-based metrics | normal / failure / recovery / stable / pre_failure / post_failure / overall |
| Baselines: no UAV, static UAV, reactive non-RL UAV, Q-learning; honest reporting | `none`, `static`, `reactive-rsu`, `reactive-gap`, plus `random`; results reported including where RL loses |
| Toolchain in one removable folder on C: | everything inside a WSL2 distro stored in `C:\ns3-uav-lab` |

---

## 3. Toolchain (installed, removable)

| item | details |
|---|---|
| Location | `C:\ns3-uav-lab` (WSL2 distro `ns3-uav-lab`, virtual disk in `C:\ns3-uav-lab\wsl`) |
| OS | Ubuntu 24.04 (imported from the official WSL root filesystem, not from the Store) |
| Default user | `researcher` (ns-3 refuses to build/run as root) |
| ns-3 | 3.47, optimized profile, modules core, network, mobility, wifi, internet, aodv, applications, flow-monitor, netanim; at `/opt/ns3-uav-lab/ns-allinone-3.47/ns-3.47` |
| SUMO | 1.18.0 from the Ubuntu package (`sumo`, `sumo-tools`, `sumo-gui`) |
| Python | venv `/opt/ns3-uav-lab/venv` with numpy, pandas, matplotlib, tabulate |
| NetAnim viewer | built from source with Qt5 into `/opt/ns3-uav-lab/netanim`, linked as `/usr/local/bin/netanim` |
| Extra | `gdb` (installed to debug the Wi-Fi crash) |
| GUI | sumo-gui and NetAnim open as Windows windows through WSLg |

Scripts:

* `scripts/setup/install_toolchain_windows.ps1` - imports the distro into
  `C:\ns3-uav-lab`, runs `bootstrap_ubuntu.sh` (packages, SUMO, venv, ns-3
  build) and `build_netanim.sh`.
* `scripts/setup/remove_toolchain_windows.ps1` - unregisters the distro and
  deletes `C:\ns3-uav-lab`. Nothing else was installed into Windows.
* `scripts/wsl.ps1` - runs any project bash script inside the distro from
  PowerShell (also strips Windows line endings from `.sh` files first).

---

## 4. Code structure

```text
scratch/uav-rsu-rl/               ns-3 program (copied into ns-3 by scripts/build.sh)
  uav-rsu-rl.cc                   entry point: nodes, radio, AODV, static ARP, traffic, episode loop
  CMakeLists.txt                  builds the simulator and the standalone RL unit test
  rl/                             GENERIC RL LAYER - no ns-3 includes
    rl-agent.h/.cc                RLAgent interface, Observation, Transition, AgentConfig
    qlearning-agent.h/.cc         Q-LEARNING IMPLEMENTATION (epsilon-greedy, decay, CSV save/load)
    random-agent.h/.cc            random-walk agent (lower-bound baseline)
    rl-agent-factory.h/.cc        "qlearning", "random" -> agent (new algorithms register here)
  env/                            ENVIRONMENT LAYER
    sim-config.h/.cc              every parameter + command line registration
    scenario.h/.cc                geometry, RSU layout, static debug layout, SUMO scenario_info.cfg
    rsu-manager.h/.cc             RSU nodes, failure injection, failure listeners
    uav-controller.h/.cc          relative actions, flight, operational-area clamping
    wifi-radio.h                  safe PHY switch-off (ns-3.47 crash workaround)
    network-monitor.h/.cc         per-packet tracking, windows, phases, forwarding, AODV overhead, drop diagnostics
    topology-observer.h/.cc       connectivity graph: components, UAV links, gaps, bearings
    env-snapshot.h                what one observation contains
    state-encoder.h/.cc           snapshot -> discrete state id + feature vector
    reward-function.h/.cc         reward computed from measured network state
    recovery-strategy.h/.cc       none / static / reactive-rsu / reactive-gap / RL adapter
    recovery-environment.h/.cc    observe -> decide -> act -> measure -> learn loop, all CSV logging
  experiment/
    results-writer.h/.cc          CSV writer with run-context columns
    animation-manager.h/.cc       NetAnim labels, colours, sizes, UAV status text
  tests/test_rl_agents.cpp        unit test of rl/ only (no ns-3)

scratch/sumo-scenario/            SUMO pipeline and generated scenarios
scripts/                          build/run/view helpers, setup/, experiments/
legacy/baseline-scratch/          the original baseline program (kept, not deleted)
docs/                             old WSL setup notes, this report
```

The layering keeps the RL algorithm completely separate from ns-3: the
`rl/` directory compiles and is unit-tested without any ns-3 library.

---

## 5. Scenario

* **Roads (SUMO):** two 400 m x 400 m grid districts (2 x 2 blocks of 200 m, one
  lane each way). West x in [0,400], East x in [800,1200], y in [0,400]. A
  400 m river/park gap without roads lies between them.
* **Vehicles:** the first half drive random tours inside the West district,
  the rest inside the East district (checked with `check_trace.py`: nobody
  leaves their district). Vehicle 0 is the source, the last vehicle the
  destination. Traffic: UDP echo, 1024 bytes every 50 ms (20 packets/s).
* **RSUs:** RSU0 (200,200), RSU1 (400,100), **RSU2 (600,200) - the only bridge
  over the gap**, RSU3 (800,300), RSU4 (1000,200). By default RSU2 fails.
* **Radio:** IEEE 802.11p, 10 MHz, 6 Mbit/s, 23 dBm, log-distance path loss
  (exponent 2) with a hard 250 m cut-off. Routing: AODV.
* **UAV:** 50 m altitude, 20 m/s, home (200,300) in the West district, or a
  random start. Operational area [-100,1300] x [-100,500].
* **Timing:** failure at 20-30 s (evaluation) or 10-35 s (training), 1 s
  detection delay, one decision every 3 s (2 s flight + settle + 0.8 s
  measurement window), maximum 25 decisions.

### SUMO scenarios generated

| scenario | vehicles | max speed | variants (SUMO seeds) |
|---|---|---|---|
| low | 20 | 50 km/h | 1-5 |
| medium | 40 | 50 km/h | 1-5 |
| high | 60 | 50 km/h | 1-5 |
| medium_slow | 40 | 30 km/h | 1-5 |
| medium_fast | 40 | 70 km/h | 1-5 |

Per scenario and variant: trips (`trips_v.trips.xml`) -> `duarouter` routes
(`routes_v.rou.xml`) -> SUMO run (`scenario_v.sumocfg`, 0.1 s step, 130 s) ->
FCD output -> ns-2 mobility trace (`mobility_v.tcl`) that ns-3 replays.
Variants 1-4 are used for training, **variant 5 only for evaluation**.

---

## 6. RL design

### State (243 discrete states)

Relative, position-free. For the source side and the destination side
separately: `LINKED` if the UAV has a radio link into that ground component,
otherwise the compass direction (8 sectors) from the UAV to the nearest node of
that component. Plus a PDR level: low (< 0.2), medium (< 0.8), high.
9 x 9 x 3 = 243 states. An 11-value feature vector (bearings, distances, link
flags, PDR, normalized position) is also produced for future function-
approximation agents such as DQN.

### Actions (9)

N, S, E, W, Stay, NE, NW, SE, SW; 40 m per step; moves are clamped to the
operational area (boundary hits are penalized).

### Reward (computed by the environment)

`2.0*dPDR + 0.5*PDR + 1.0*connectivity + 0.5*dThroughput - 0.3*delay
- 0.5*loss - 0.1*movement - 0.3*unnecessary move - 0.5*boundary
+ potential-based shaping (+5 on recovery, -2 on timeout)`.
Connectivity = +1 with an end-to-end path, otherwise -0.4 + 0.3 per UAV link
to the source/destination component. Shaping `gamma*Phi(s') - Phi(s)` with
Phi = -(gap to source component + gap to destination component)/range; this
form does not change the optimal policy. All weights are CLI parameters.

### Q-learning

`Q(s,a) += alpha [r + gamma max Q(s',.) - Q(s,a)]`; terminal (recovered):
`Q(s,a) += alpha [r - Q(s,a)]`; timeout is a truncation (bootstraps).
alpha = 0.1, gamma = 0.9, epsilon 0.2 decaying by 0.995 per episode to 0.02
during training, epsilon = 0 in evaluation.

### Recovery loop

```
RSU fails -> 1 s detection -> episode starts
every 3 s: observe (PDR window, topology, AODV route) -> encode state
           -> agent picks relative move -> UAV flies -> measure -> reward -> update (training only)
stop when windowed PDR >= 0.8 (recovered) or after 25 steps (timeout)
if the PDR later degrades for 2 windows (vehicles moved), a new episode starts
```

### Baselines

| name | behaviour | knowledge used |
|---|---|---|
| `none` | UAV radio off | - |
| `static` | UAV hovers at its start position | - |
| `reactive-rsu` | greedy flight to the failed RSU's position | location of the failed RSU |
| `reactive-gap` | greedy flight to the midpoint of the narrowest gap between the source and destination components | global positions of all nodes |
| `random` | random relative moves | - |
| `rl-seed1..3` | trained Q-learning policies, greedy | only the 243-state relative observation |

---

## 7. Development log: problems found and fixed

| problem | cause | fix |
|---|---|---|
| Destination never received packets although routes existed (PDR 0 before any failure) | ns-3 ARP marks a neighbour DEAD for 100 s after 3 unanswered requests; on the busy shared channel this silently black-holed the destination | Pre-populated ARP caches (`--staticArp=1`) |
| With ns-3 `NeighborCacheHelper`, the failed RSU1/RSU2 kept being used as next hop (~2000 frames lost) | Auto-generated ARP entries are not ALIVE/PERMANENT, so AODV never mapped MAC transmit failures to the dead neighbour | Entries are created as PERMANENT manually (`PopulatePermanentArp`); AODV now breaks the route correctly |
| Segfault in training seed 3 at episode ~387 | ns-3.47 bug: `FrameExchangeManager::SendMpdu` schedules `TransmissionSucceeded()` for no-ack frames without keeping the event; `WifiPhy::SetOffMode()` resets the MAC (`m_dcf = nullptr`) and the event then dereferences null. Happens when RSU2 is sending a broadcast (AODV HELLO) at the failure instant | `env/wifi-radio.h`: switch-off deferred to the end of the current transmission (<= 1.5 ms). Reproduced with gdb, fixed, verified; seed 3 resumed |
| UDP echo server trace callback mismatch | ns-3.47 `Rx` trace signature changed | Use `RxWithoutAddress` |
| `./ns3 build <target>` refused the scratch target | ns-3.47 only accepts runnable-program shortcuts | Build with `cmake --build cmake-cache --target ...` |
| Missing `uint32_t` | missing include | `#include <cstdint>` |
| Bash scripts failed with `$'\r'` errors | files saved with Windows line endings | `.gitattributes` (LF), `wsl.ps1` strips CR automatically, sources converted |
| PowerShell turned `--seeds 1,2,3` into three arguments | PowerShell array syntax | quote comma lists: `'1,2,3'` |
| Different speed limits per scenario but one shared road network | network speed limit was per scenario | fixed road limit 70 km/h; the vehicle type caps the actual speed |
| `to_markdown` failed in the analysis | missing `tabulate` | added to the venv and the bootstrap |
| Training interrupted by the crash | - | `train.py --resume` reads `episodes_trained` from the Q-table and drops CSV rows of the unfinished block |

A diagnostics mode (`--diagnostics=1`) was added during debugging. It prints
per-node forwarding counts, local deliveries, IPv4 drops by reason, Wi-Fi MAC
drops by reason and PHY receive drops by reason. The column
`first_uav_forward_time` was added because the geometric observer is
conservative (0.95 x 250 m, 3D distance) and can report "not linked" while the
radio link already works.

---

## 8. Verification runs (`results/verify/`)

| run | setup | key findings |
|---|---|---|
| `none` | static debug layout, 20 vehicles, no UAV, RSU2 fails at 20 s | before: PDR 0.956 (timely 0.883), delay 78 ms, RSU2 forwarded 661 packets; after: RSU2 forwards 0, source has no route (`-`), PDR 0 until the end |
| `gap` | same, reactive-gap UAV | recovered after 34 s / 11 steps / 440 m; UAV forwarded 2713 packets; source next hop V8 (8 hops); stable PDR 1.0, post-failure PDR 0.68 |
| `rl_train` | short Q-learning training, static layout | ~2 s per episode; Q-table CSV with 243 states and the action log written correctly |
| `sumo_low` | SUMO low density, variant 1, no UAV | normal timely PDR 0.93, RSU2 forwarded 693, then PDR 0 after the failure |
| `sumo_high` | SUMO high density, reactive-gap | normal timely PDR 0.92, recovered after 37 s / 12 steps, UAV forwarded 1885 packets, next hop V24 (6 hops), stable PDR 0.68 |
| `sumo_high_train` | timing check | high-density training episode ~11 s |
| `train_smoke/seed9` | 6-episode run of the training driver | driver, block logic and epsilon continuation work |

RL unit test (`scripts/test_rl.sh`): 11 checks pass (exact update rule,
terminal update, greedy optimality on a corridor task, evaluation leaves the
Q-table byte-identical, save/load round trip, dimension mismatch rejected).
Re-run after every change: `ALL RL TESTS PASSED`.

---

## 9. Training runs (`results/train/seed1..3/`)

| setting | value |
|---|---|
| seeds | 1, 2, 3 (independent agents, run in parallel) |
| episodes | 450 per seed = 1350 training simulations |
| blocks | 5 episodes per process; blocks cycle low -> medium -> high and SUMO variants 1-4 |
| failure time | uniform 10-35 s per block; simulation = failure + 90 s |
| UAV start | uniformly random in the operational area |
| epsilon | 0.2 x 0.995^episode, minimum 0.02 (final 0.021) |
| duration | seed 1: 72.8 min, seed 2: 74.7 min, seed 3: 60 min + 12 min after resume |

Seed 3 crashed at episode ~387 (the Wi-Fi bug above). After the fix it was
resumed from its saved model at episode 385; the partial rows of the crashed
block were removed, so the logs are consistent.

### Learning progress

| seed | recovery rate (first 20 % -> last 20 %) | steps per episode | episode return |
|---|---|---|---|
| 1 | 0.39 -> 0.96 | 20.1 -> 11.1 | -8.7 -> 3.5 |
| 2 | 0.31 -> 0.99 | 20.9 -> 12.4 | -9.8 -> 3.3 |
| 3 | 0.56 -> 0.98 | 18.1 -> 10.4 | -5.9 -> 5.2 |

![learning curve](../results/analysis/fig_learning_curve.png)

### What the agents learned

Only 31-35 of the 243 states were ever visited (5900-6900 Q-updates per seed).
The two dominant states and the learned action agree across all three seeds
and make physical sense:

| state | meaning | visits (s1/s2/s3) | learned action |
|---|---|---|---|
| 216 | UAV linked to the source side, destination side lies East, PDR low | 2749 / 2299 / 2247 | **E** (all seeds) |
| 132 | UAV linked to the destination side, source side lies West, PDR low | 2449 / 2461 / 2220 | **W** (all seeds) |
| 240 | UAV linked to both sides, PDR still low (AODV route not yet rebuilt) | 320 / 353 / 403 | N / Stay / Stay |
| 219 | linked to source, destination NE | 314 / 232 / 254 | N / NE / NE |
| 159 | linked to destination, source SW | 316 / 252 / 190 | SW / W / SW |

In words: "stay connected to the side you have and move toward the other
side; once both sides are reachable, wait for the route". No position was ever
given to the agent.

---

## 10. Evaluation runs (`results/eval/`)

All runs use the held-out SUMO variant 5, simulation time 125 s, failure times
20/25/30 s and two run numbers (6 cases per scenario). RL policies are greedy
(epsilon = 0) and never update or write their Q-table.

| suite | what varies | cases |
|---|---|---|
| main | RSU2 fails; low / medium / high density | 18 |
| speed | RSU2 fails; medium_slow (30 km/h) / medium_fast (70 km/h) | 12 |
| location | RSU1 or RSU3 fails instead (medium) | 12 |
| randstart | RSU2 fails; UAV starts at a random position (medium) | 6 |

| batch | runs | wall time |
|---|---|---|
| baselines (none, static, reactive-rsu, reactive-gap, random) | 240 | ~12 min (11 parallel jobs) |
| rl-seed1 + rl-seed2 | 96 | ~6 min |
| rl-seed3 | 48 | ~3 min |
| **total** | **384** | |

---

## 11. Results

Definitions: *recovered* = windowed PDR >= 0.8 within 25 steps. *Recovery
time* = seconds from the failure (includes 1 s detection) to recovery,
averaged over recovered runs. *Timely PDR* counts packets delivered within
200 ms. *Post-failure* = from the failure to the end of the simulation.
*Stable* = after recovery. RL values average 3 models x 6 cases.

### 11.1 Main suite (RSU2 fails)

| density | strategy | recovered | time [s] | distance [m] | post-failure timely PDR | stable PDR | post-failure delay [ms] |
|---|---|---|---|---|---|---|---|
| low | none | 0/6 | - | 0 | 0.00 | - | - |
| low | static | 0/6 | - | 0 | 0.00 | - | - |
| low | random | 0/6 | - | 1067 | 0.00 | - | - |
| low | reactive-rsu | 6/6 | 34.5 | 440 | **0.63** | **0.95** | 38 |
| low | reactive-gap | 6/6 | 41.0 | 553 | 0.38 | 0.72 | 58 |
| low | Q-learning | 18/18 | **34.5** | 593 | 0.47 | 0.84 | 96 |
| medium | none / static / random | 0/6 each | - | 0 / 0 / 1067 | 0.00 | - | - |
| medium | reactive-rsu | 6/6 | 36.5 | 433 | **0.55** | **0.88** | 29 |
| medium | reactive-gap | 6/6 | 43.0 | 567 | 0.42 | 0.76 | 23 |
| medium | Q-learning | 18/18 | **34.8** | 591 | 0.45 | 0.81 | 68 |
| high | none / static / random | 0/6 each | - | 0 / 0 / 1067 | 0.00 | - | - |
| high | reactive-rsu | 6/6 | 36.5 | 440 | 0.36 | **0.64** | 45 |
| high | reactive-gap | 6/6 | 37.5 | 487 | 0.28 | 0.59 | 57 |
| high | Q-learning | 18/18 | **36.3** | 580 | **0.39** | **0.64** | 73 |

Normal-phase (pre-failure) timely PDR: low 0.99, medium 0.96, high 0.76
(`none`) / 0.54 (runs with a UAV node).

![recovery time](../results/analysis/fig_recovery_time.png)

### 11.2 Speed and random-start suites (RSU2 fails, medium density)

| scenario | strategy | recovered | time [s] | distance [m] | post-failure timely PDR | stable PDR |
|---|---|---|---|---|---|---|
| 30 km/h | reactive-rsu | 6/6 | 34.0 | 440 | **0.49** | **0.81** |
| 30 km/h | reactive-gap | 6/6 | 38.5 | 553 | 0.44 | 0.79 |
| 30 km/h | Q-learning | 18/18 | **32.5** | 530 | 0.43 | 0.74 |
| 70 km/h | reactive-rsu | 6/6 | 33.0 | 440 | **0.59** | **0.91** |
| 70 km/h | reactive-gap | 6/6 | 39.0 | 473 | 0.54 | 0.89 |
| 70 km/h | Q-learning | 18/18 | **32.3** | 444 | 0.57 | 0.87 |
| random start | reactive-rsu | 6/6 | **34.5** | 440 | **0.55** | **0.86** |
| random start | reactive-gap | 5/6 | 37.6 | 547 | 0.41 | 0.71 |
| random start | Q-learning | 18/18 | 43.0 | 618 | 0.47 | 0.86 |

`none`, `static` and `random` recovered 0 of 6 in every one of these scenarios.

### 11.3 Location suite (RSU1 or RSU3 fails)

These failures do **not** partition the network: vehicles at the district
edge (x = 400 or 800) are within 200 m of RSU2, which keeps bridging. Even
`none` keeps a post-failure timely PDR of about 0.76 (failure-phase PDR 0.91)
and all strategies are equivalent (post-failure timely PDR 0.73-0.76). The UAV
strategies move although nothing needs repairing: random 240 m, reactive-rsu
210 m, Q-learning 124 m on average, reactive-gap 83 m.

### 11.4 Per-model results (Q-learning)

| suite | seed 1 | seed 2 | seed 3 |
|---|---|---|---|
| main low: time / distance | 34.5 s / 587 m | 35.0 s / 620 m | 34.0 s / 573 m |
| main medium | 34.5 s / 773 m | 34.0 s / 527 m | 36.0 s / 473 m |
| main high | 38.5 s / 647 m | 36.5 s / 627 m | 34.0 s / 467 m |
| random start | 50.5 s / 781 m | 43.5 s / 560 m | 35.0 s / 513 m |
| 70 km/h | 32.0 s / 493 m | 32.5 s / 440 m | 32.5 s / 400 m |
| 30 km/h | 33.5 s / 762 m | 32.0 s / 427 m | 32.0 s / 400 m |

All three models recover every case. Seed 3 is the most efficient; seed 1
flies noticeably more and is the slowest from random starts.

### 11.5 Routing overhead (main suite, normalized routing load = AODV packets per delivered data packet)

| density | pre-failure | post-failure reactive-rsu | reactive-gap | Q-learning |
|---|---|---|---|---|
| low | 1.7 | 3.7 | 6.2 | 4.7 |
| medium | 4.5 | 11.4 | 13.4 | 14.7 |
| high | 17.5 | 53.9 | 71.6 | 51.2 |

Recovery multiplies the AODV overhead (route breaks, RREQ floods while the UAV
moves); strategies that settle faster and move less after recovery cause less.

---

## 12. Interpretation (honest assessment)

1. **The failure is real.** When the bridge RSU fails, the network splits into
   two components: RSU2 forwards nothing, the source has no AODV route, and the
   PDR stays at exactly 0 for `none` and `static` until the end.
2. **Q-learning works and generalizes.** 108/108 recoveries on a mobility
   trace never seen in training, from both the fixed home position and random
   starts, at all densities and speeds, without knowing where the failed RSU
   was.
3. **It is as fast as the best heuristic, faster than the fair one.** RL
   matches `reactive-rsu` (which is told the failed RSU's position) and beats
   `reactive-gap` by 1-8 s. All competent strategies are close to the physical
   limit: the UAV must fly about 400 m (about 10 steps of 3 s) to reach the
   gap.
4. **It is not uniformly better.**
   * `reactive-rsu` has a higher post-failure and stable PDR in most
     scenarios and flies less. It parks where the RSU was, while the RL UAV
     keeps making small corrective moves after recovery, which briefly breaks
     routes (more distance, more re-recoveries, higher post-failure delay:
     68-96 ms vs 23-58 ms).
   * From random start positions RL is slower (43 s vs 34.5 / 37.6 s), and one
     model (seed 1) is clearly weaker there.
   * It moves during non-partitioning failures where no action is needed.
5. **Likely limits:** the coarse 243-state encoding (only direction, not
   distance, and no notion of "hold position"), only ~33 visited states, and
   450 training episodes. A reward term for holding a working relay position,
   distance bins (`--stateDistanceBins=2`) or a function-approximation agent
   using the existing feature vector are the natural next steps.
6. **High-density side effect:** runs with a UAV node have a lower
   *pre-failure* PDR (0.54) than `none` (0.76). The variance is high (std 0.31
   over 6 cases); the UAV acting as an extra, sometimes poor AODV relay on a
   congested channel is a likely cause. It affects all UAV strategies equally.

---

## 13. Demo run and visual outputs

Command (RL seed 3, medium density, variant 5, failure at 20 s):

```powershell
.\scripts\wsl.ps1 scripts/run_sim.sh --mobility=sumo --vehicleDensity=medium --traceVariants=5 --failureTime=20 --simulationTime=125 --run=1 --strategy=rl --mode=eval --modelFile=results/train/seed3/uav_qtable.csv --netanim=1 --flowmonXml=1 --routingTables=1 --verbose=1 --diagnostics=1 --outputDir=results/demo/rl-seed3
```

Outcome (`results/demo/rl-seed3/`):

| metric | value |
|---|---|
| normal phase | timely PDR 0.99, delay 10 ms; RSU2 forwarded 718 packets |
| failure at 20 s | RSU2 forwards 0; PDR 0 |
| recovery | 31 s after the failure, 10 steps, UAV first forwards data at 50.6 s |
| after recovery | source next hop = UAV (4 hops); UAV forwarded 1513 packets; stable timely PDR 0.80 |
| files | `uav-rsu-rl-animation.xml` (1.2 MB), `flowmon_results.xml` (1.0 MB), routing tables before/after failure and after recovery, all CSVs |

![PDR over time](../results/analysis/fig_pdr_timeseries.png)

![UAV trajectories](../results/analysis/fig_trajectories.png)

Viewers:

```powershell
.\scripts\wsl.ps1 scripts/view_sumo.sh medium 5      # SUMO traffic (sumo-gui)
.\scripts\wsl.ps1 scripts/view_netanim.sh            # NetAnim: open the printed .xml, press Play
```

Other figures in `results/analysis/`: `fig_recovery_rate.png`,
`fig_movement_cost.png`, `fig_post_failure_pdr.png`, `fig_overhead.png`,
`fig_phase_pdr.png`.

---

## 14. Output files

| file | content |
|---|---|
| `uav_qtable.csv` | header with hyper-parameters and `episodes_trained`, then per state: 9 Q-values, 9 visit counts, best action, total visits |
| `state_encoding.csv` | human-readable meaning of every state id |
| `episode_results.csv` | one row per recovery episode |
| `uav_actions.csv` | one row per UAV decision (state, action, explore/exploit, positions, window metrics, links, AODV next hop, reward components) |
| `pdr_results.csv` | 0.5 s time series (phase, PDR, delay, throughput, connectivity, UAV forwarding, AODV packets) |
| `performance_results.csv` | per phase: PDR, timely PDR, loss, throughput, mean/p95 delay, jitter, AODV packets/bytes, normalized routing load, forwarding |
| `recovery_results.csv` | one row per run: failure impact, recovery time/steps/distance, final position, UAV forwarding, route after recovery |
| `flowmon_results.xml` | ns-3 FlowMonitor |
| `uav-rsu-rl-animation.xml` | NetAnim animation |
| `routing_tables/*.txt` | AODV routing tables of all nodes before failure, after failure, after recovery |

Analysis outputs (`results/analysis/`): `summary.md`, `summary_recovery.csv`,
`summary_recovery_by_model.csv` (with standard deviations),
`summary_phases.csv`, `training_summary.csv`, and the figures.

---

## 15. How to reproduce

Windows (toolchain already installed), from the project folder:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\scripts\wsl.ps1 scripts/build.sh
.\scripts\wsl.ps1 scripts/test_rl.sh
.\scripts\wsl.ps1 scripts/generate_scenarios.sh                        # optional, files exist
.\scripts\wsl.ps1 scripts/py.sh scripts/experiments/train.py --seeds '1,2,3' --episodes 450
.\scripts\wsl.ps1 scripts/py.sh scripts/experiments/evaluate.py --suites 'main,speed,location,randstart' --jobs 12
.\scripts\wsl.ps1 scripts/py.sh scripts/experiments/analyze.py
```

Move the existing `results/` folder aside first to start from scratch
(otherwise training continues from the saved Q-tables and evaluation skips
finished runs).

Native Ubuntu 24.04: replace `.\scripts\wsl.ps1 scripts/X.sh` with
`bash scripts/X.sh`; one-time setup with
`sudo bash scripts/setup/bootstrap_ubuntu.sh 8`,
`sudo bash scripts/setup/build_netanim.sh 8` and
`sudo chown -R "$USER:$USER" /opt/ns3-uav-lab`.

Remove the toolchain from Windows:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\setup\remove_toolchain_windows.ps1
```

---

## 16. Files added or changed in the repository

| path | status |
|---|---|
| `scratch/uav-rsu-rl/**` | new layered simulator (rl/, env/, experiment/, tests/) |
| `scratch/sumo-scenario/generate_scenario.py`, `generate_all.sh`, `check_trace.py` | new SUMO pipeline; `fcd_to_ns2.py` reused |
| `scratch/sumo-scenario/network/`, `low/ medium/ high/ medium_slow/ medium_fast/` | generated scenarios |
| `scripts/*.sh`, `scripts/wsl.ps1` | build, run, test, view, Python wrapper |
| `scripts/setup/*` | toolchain install/remove, Ubuntu bootstrap, NetAnim build |
| `scripts/experiments/*.py` | training, evaluation, analysis drivers |
| `legacy/baseline-scratch/` | original baseline program moved here (preserved) |
| `docs/ENVIRONMENT_SETUP_WSL.md` | previous README (setup notes) moved here |
| `docs/PROJECT_REPORT.md` | this report |
| `README.md` | new user guide |
| `.gitignore` | updated: `results/`, animations, FlowMonitor XML, SUMO FCD files and `__pycache__/` ignored |
| `.gitattributes` | enforces LF line endings for `.sh/.py/.cc/.h/.tcl/.xml/.csv` (CRLF for `.ps1`) |

Nothing has been committed. Note: in the outer repository `E:\Adaptive-RSU`
this folder is registered as a submodule, but it has no `.git` directory of
its own, so git currently does not track the changes listed above.
