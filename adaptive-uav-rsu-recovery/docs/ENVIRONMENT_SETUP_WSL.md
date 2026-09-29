# NS-3 + SUMO + DEFIANCE Research Environment Setup on Windows

This document describes the complete setup of a research simulation environment on **Windows 11 using WSL2 + Ubuntu**, with:

- WSL2
- Ubuntu 24.04
- GCC / G++
- CMake
- Ninja
- Python 3
- Git
- NS-3.47
- Eclipse SUMO
- TraCI
- libsumo
- ns-3-ai
- DEFIANCE
- pybind11
- Protocol Buffers
- Python dependencies used by DEFIANCE/SUMO

The recommended architecture is:

```text
Windows 11
    │
    └── WSL2
          │
          └── Ubuntu 24.04
                │
                ├── NS-3.47
                │
                ├── SUMO
                │    ├── sumo
                │    ├── sumo-gui
                │    ├── TraCI
                │    └── libsumo
                │
                ├── ns-3-ai
                │
                └── DEFIANCE
                     └── SUMO integration
```

---

# 1. Install WSL2

Open **PowerShell as Administrator** on Windows.

Check WSL:

```powershell
wsl --status
```

List available distributions:

```powershell
wsl --list --online
```

Install Ubuntu:

```powershell
wsl --install -d Ubuntu-24.04
```

Restart Windows if requested.

After installation, launch Ubuntu from the Start Menu or run:

```powershell
wsl -d Ubuntu-24.04
```

---

# 2. Verify WSL2

From PowerShell:

```powershell
wsl --list --verbose
```

Expected:

```text
NAME            STATE           VERSION
Ubuntu-24.04    Running         2
```

If Ubuntu is stopped, start it with:

```powershell
wsl -d Ubuntu-24.04
```

---

# 3. Important WSL Memory Information

WSL2 uses memory dynamically.

Check memory from Ubuntu:

```bash
free -h
```

Check CPU cores:

```bash
nproc
```

For example:

```text
Mem:    3.7Gi
Swap:   1.0Gi

nproc
8
```

Closing Ubuntu does not permanently reserve that RAM. WSL releases/reclaims resources when the VM shuts down.

To explicitly shut down WSL from **PowerShell**:

```powershell
wsl --shutdown
```

Then Windows can reclaim the memory used by WSL.

---

# 4. Update Ubuntu

Inside Ubuntu:

```bash
sudo apt update
sudo apt upgrade -y
```

---

# 5. Install Basic Development Tools

Install GCC/G++, CMake, Ninja, Git, Python and related development tools:

```bash
sudo apt install -y \
    build-essential \
    g++ \
    gcc \
    cmake \
    ninja-build \
    git \
    python3 \
    python3-dev \
    python3-full \
    python3-pip \
    python3-venv \
    wget \
    curl \
    unzip \
    tar \
    bzip2
```

Verify:

```bash
g++ --version
```

```bash
cmake --version
```

```bash
python3 --version
```

```bash
git --version
```

```bash
ninja --version
```

---

# 6. Create Research Workspace

Create a dedicated directory:

```bash
mkdir -p ~/research
cd ~/research
```

Verify:

```bash
pwd
```

Expected:

```text
/home/<username>/research
```

All research simulation software can be kept inside this directory.

---

# 7. Install Python Command Alias

Some older scripts use:

```bash
python
```

instead of:

```bash
python3
```

Ubuntu may only provide `python3`.

Install the compatibility package:

```bash
sudo apt install -y python-is-python3
```

Verify:

```bash
python --version
```

```bash
which python
```

Expected:

```text
Python 3.12.x
/usr/bin/python
```

---

# 8. Install pybind11

DEFIANCE requires pybind11.

Install the Ubuntu development package:

```bash
sudo apt install -y pybind11-dev
```

Verify the CMake package:

```bash
cmake --find-package \
    -DNAME=pybind11 \
    -DCOMPILER_ID=GNU \
    -DLANGUAGE=CXX \
    -DMODE=EXIST
```

A message containing:

```text
pybind11 found.
```

confirms that CMake can locate it.

---

# 9. Install Protocol Buffers

DEFIANCE depends on ns-3-ai, which requires Protocol Buffers.

Install:

```bash
sudo apt update
sudo apt install -y \
    libprotobuf-dev \
    protobuf-compiler
```

Verify:

```bash
protoc --version
```

Example:

```text
libprotoc 3.21.12
```

The exact version may differ.

---

# 10. Download NS-3.47

Go to the research directory:

```bash
cd ~/research
```

Download the NS-3.47 all-in-one archive:

```bash
wget https://www.nsnam.org/releases/ns-allinone-3.47.tar.bz2
```

Extract:

```bash
tar -xjf ns-allinone-3.47.tar.bz2
```

Enter the NS-3 directory:

```bash
cd ~/research/ns-allinone-3.47/ns-3.47
```

Verify:

```bash
ls
```

You should see files/directories including:

```text
CMakeLists.txt
LICENSE
README.md
src
contrib
examples
scratch
ns3
```

---

# 11. Configure NS-3

From:

```text
~/research/ns-allinone-3.47/ns-3.47
```

run:

```bash
./ns3 configure --enable-examples --enable-tests
```

A successful configuration should show:

```text
Examples                      : ON
Tests                         : ON
```

and:

```text
Configuring done
Generating done
```

---

# 12. Build NS-3

Build NS-3:

```bash
./ns3 build
```

This can take considerable time because NS-3 contains many modules.

On a machine with limited RAM, avoid unnecessary parallelism.

You can explicitly control Ninja jobs:

```bash
ninja -C cmake-cache -j3
```

For example:

```bash
./ns3 build
```

may eventually execute something similar to:

```text
cmake --build ... -j 3
```

If Ubuntu/WSL becomes unstable during a large build, reduce the number of jobs.

---

# 13. Verify NS-3

Check the NS-3 command:

```bash
./ns3 --help
```

Note:

```bash
./ns3 --version
```

is **not** a valid option for this NS-3 version.

---

# 14. Install SUMO

Install SUMO and its tools:

```bash
sudo apt update
sudo apt install -y sumo sumo-tools sumo-doc
```

Verify:

```bash
sumo --version
```

Expected format:

```text
Eclipse SUMO sumo Version 1.18.0
```

Check SUMO GUI:

```bash
which sumo
```

```bash
which sumo-gui
```

Expected:

```text
/usr/bin/sumo
/usr/bin/sumo-gui
```

---

# 15. Set SUMO_HOME

Find SUMO:

```bash
which sumo
```

If SUMO is installed through Ubuntu packages, its tools are normally located under:

```text
/usr/share/sumo
```

Set:

```bash
export SUMO_HOME=/usr/share/sumo
```

Verify:

```bash
echo $SUMO_HOME
```

To make it permanent:

```bash
echo 'export SUMO_HOME=/usr/share/sumo' >> ~/.bashrc
source ~/.bashrc
```

Verify:

```bash
echo $SUMO_HOME
```

---

# 16. Verify SUMO Python Tools

Test:

```bash
python3 -c "import traci; print('TraCI import successful')"
```

Check where TraCI is installed:

```bash
python3 -c "import traci; print(traci.__file__)"
```

Expected location is similar to:

```text
/usr/lib/python3/dist-packages/traci/__init__.py
```

Do not use:

```python
traci.__version__
```

because the installed TraCI package does not necessarily expose a `__version__` attribute.

---

# 17. Verify libsumo

Run:

```bash
python3 -c "import libsumo; print('libsumo import successful')"
```

Expected:

```text
libsumo import successful
```

---

# 18. Test SUMO + TraCI

Create a test directory:

```bash
mkdir -p ~/research/sumo-test
cd ~/research/sumo-test
```

Create the required SUMO network and route files.

For example, verify the generated network:

```bash
grep '<edge' grid.net.xml | head -30
```

Then create/run a SUMO configuration:

```bash
sumo -c test.sumocfg
```

A successful simulation should produce output similar to:

```text
Step #100.00 (... vehicles TOT 1 ...)
```

---

# 19. Test TraCI Python Script

Create:

```bash
nano traci_test.py
```

Use:

```python
import traci

sumo_binary = "sumo"

sumo_cmd = [
    sumo_binary,
    "-c",
    "test.sumocfg"
]

traci.start(sumo_cmd)

print("SUMO started successfully")

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()

    vehicle_ids = traci.vehicle.getIDList()

    if vehicle_ids:
        vehicle_id = vehicle_ids[0]
        position = traci.vehicle.getPosition(vehicle_id)
        speed = traci.vehicle.getSpeed(vehicle_id)

        print(
            f"Vehicle: {vehicle_id}, "
            f"Position: {position}, "
            f"Speed: {speed:.2f} m/s"
        )

print("SUMO simulation finished")

traci.close()
```

Run:

```bash
python3 traci_test.py
```

Expected:

```text
SUMO started successfully
...
SUMO simulation finished
```

---

# 20. DEFIANCE Dependencies

DEFIANCE is located inside:

```text
contrib/defiance
```

It depends on:

```text
ns-3
ns-3-ai
pybind11
Protobuf
Python
SUMO
SUMO Python tools
```

Check DEFIANCE:

```bash
ls -la contrib/defiance
```

---

# 21. Make DEFIANCE Importable from Python

DEFIANCE is an ns-3 contributed module, so its Python package may not automatically be on `PYTHONPATH`.

Set:

```bash
export PYTHONPATH="$HOME/research/ns-allinone-3.47/ns-3.47/contrib:$PYTHONPATH"
```

Verify:

```bash
python3 -c "import defiance; print('defiance import successful')"
```

Expected:

```text
defiance import successful
```

Check location:

```bash
python3 -c "import defiance; print(defiance.__file__)"
```

Expected:

```text
/home/<username>/research/ns-allinone-3.47/ns-3.47/contrib/defiance/__init__.py
```

Make it permanent:

```bash
echo 'export PYTHONPATH="$HOME/research/ns-allinone-3.47/ns-3.47/contrib:$PYTHONPATH"' >> ~/.bashrc
source ~/.bashrc
```

---

# 22. Verify DEFIANCE + SUMO Python Dependencies

Run:

```bash
python3 -c "import libsumo, osmBuild, osmGet, randomTrips, geopy, defiance; print('DEFIANCE SUMO dependencies OK')"
```

Expected:

```text
DEFIANCE SUMO dependencies OK
```

If `geopy` is missing, install it using a virtual environment rather than modifying Ubuntu's system Python.

Create a research Python environment:

```bash
cd ~/research
python3 -m venv research-venv
source research-venv/bin/activate
```

Then:

```bash
pip install geopy
```

If required, install other Python-only dependencies inside this environment.

Leave the environment with:

```bash
deactivate
```

---

# 23. Verify DEFIANCE CMake Configuration

From:

```bash
cd ~/research/ns-allinone-3.47/ns-3.47
```

run:

```bash
./ns3 configure --enable-examples --enable-tests
```

The important output should contain:

```text
Found pybind11
```

and:

```text
Found Protobuf
```

Most importantly:

```text
Modules configured to be built:
...
ai
...
defiance
...
```

DEFIANCE should **not** appear under:

```text
Modules that cannot be built
```

---

# 24. Verify DEFIANCE Examples

Run:

```bash
ninja -C cmake-cache -t targets | grep -i "defiance-"
```

You should see targets such as:

```text
defiance-addition-example
defiance-agent-agent-communication
defiance-ai-demo
defiance-handover
defiance-sumo-test
defiance-sumo-topology-creator
```

The important target for SUMO testing is:

```text
defiance-sumo-test
```

---

# 25. Build DEFIANCE SUMO Example

Instead of rebuilding the entire NS-3 project:

```bash
./ns3 build defiance-sumo-test
```

However, depending on the current NS-3 configuration, this may cause Ninja to build additional pending dependencies.

If you only need to verify whether the executable exists:

```bash
find build -type f \
    \( -name "*defiance-sumo-test*" -o -name "*sumo-example*" \) \
    | head -20
```

The executable should look similar to:

```text
build/contrib/defiance/examples/ns3.47-defiance-sumo-test-default
```

---

# 26. Set NS3_HOME

The DEFIANCE SUMO example uses `NS3_HOME`.

Set:

```bash
export NS3_HOME="$HOME/research/ns-allinone-3.47/ns-3.47"
```

Verify:

```bash
echo $NS3_HOME
```

Expected:

```text
/home/<username>/research/ns-allinone-3.47/ns-3.47
```

Make it permanent:

```bash
echo 'export NS3_HOME="$HOME/research/ns-allinone-3.47/ns-3.47"' >> ~/.bashrc
source ~/.bashrc
```

---

# 27. Run DEFIANCE SUMO Example

From the NS-3 root:

```bash
cd ~/research/ns-allinone-3.47/ns-3.47
```

You can run:

```bash
./ns3 run defiance-sumo-test
```

Alternatively, if the executable already exists:

```bash
./build/contrib/defiance/examples/ns3.47-defiance-sumo-test-default
```

The direct executable method can be useful because `./ns3 run` may trigger additional builds.

---

# 28. DEFIANCE SUMO Example Files

The example is located at:

```text
contrib/defiance/examples/sumo-test/
```

Check:

```bash
ls -la contrib/defiance/examples/sumo-test
```

Important files:

```text
README.md
create_network.py
generate_trace.py
sumo-example.cc
```

The C++ example:

```text
contrib/defiance/examples/sumo-test/sumo-example.cc
```

uses the Python scripts to generate the SUMO network and mobility traces.

---

# 29. DEFIANCE SUMO CMake Target

The target is defined in:

```text
contrib/defiance/examples/CMakeLists.txt
```

The relevant definition is:

```cmake
build_lib_example(
        NAME defiance-sumo-test
        SOURCE_FILES sumo-test/sumo-example.cc
        LIBRARIES_TO_LINK
                ${libapplications}
                ${libdefiance}
                ${libinternet}
                ${libmobility}
                ${libnetanim}
)
```

Therefore the correct NS-3 target is:

```text
defiance-sumo-test
```

---

# 30. OSM / SUMO Network Generation

The DEFIANCE SUMO example may download an OpenStreetMap area and convert it into a SUMO network.

The general workflow is:

```text
OpenStreetMap
      ↓
OSM XML
      ↓
SUMO netconvert
      ↓
network.net.xml
      ↓
SUMO traffic generation
      ↓
mobility traces
      ↓
NS-3
```

Generated files may be placed under:

```text
contrib/defiance/scenarios/examples/sumo-test/
```

Check:

```bash
ls -la contrib/defiance/scenarios/examples/sumo-test/
```

and:

```bash
ls -la contrib/defiance/scenarios/examples/sumo-test/config_files/
```

---

# 31. Common OSM Error

If you see:

```text
406 Not Acceptable
Error: Could not open osm-file 'map_bbox.osm.xml'.
Error: No nodes loaded.
```

then the OSM download failed.

The later error:

```text
FileNotFoundError:
.../config_files/network.net.xml
```

is usually a consequence of the failed OSM download.

Do not assume that `network.net.xml` itself is the original problem.

Inspect:

```bash
sed -n '1,130p' contrib/defiance/examples/sumo-test/create_network.py
```

to determine the OSM service/endpoint being used by the example.

---

# 32. Common Python Error

If you get:

```text
sh: 1: python: not found
```

install:

```bash
sudo apt install -y python-is-python3
```

Verify:

```bash
python --version
```

---

# 33. Common PEP 668 Error

If:

```bash
python3 -m pip install --user <package>
```

returns:

```text
error: externally-managed-environment
```

do **not** use:

```bash
--break-system-packages
```

unless you specifically understand the consequences.

Instead create a virtual environment:

```bash
python3 -m venv ~/research/research-venv
```

Activate:

```bash
source ~/research/research-venv/bin/activate
```

Install packages:

```bash
pip install <package>
```

Deactivate:

```bash
deactivate
```

---

# 34. Common pybind11 Error

If CMake says:

```text
Skipping contrib/defiance: pybind11 not found
```

install:

```bash
sudo apt install -y pybind11-dev
```

Verify:

```bash
cmake --find-package \
    -DNAME=pybind11 \
    -DCOMPILER_ID=GNU \
    -DLANGUAGE=CXX \
    -DMODE=EXIST
```

Then reconfigure:

```bash
./ns3 configure --enable-examples --enable-tests
```

---

# 35. Common Protobuf Error

If CMake says:

```text
Skipping contrib/ai: Protobuf not found
```

or:

```text
Skipping contrib/defiance: Protobuf not found
```

install:

```bash
sudo apt install -y libprotobuf-dev protobuf-compiler
```

Verify:

```bash
protoc --version
```

Then:

```bash
./ns3 configure --enable-examples --enable-tests
```

---

# 36. Common DEFIANCE Target Missing Error

If:

```bash
./ns3 run defiance-sumo-test
```

returns:

```text
Couldn't find the specified program: defiance-sumo-test
```

check:

```bash
ninja -C cmake-cache -t targets | grep -i "defiance-"
```

If nothing appears, check the NS-3 configure output.

DEFIANCE must appear under:

```text
Modules configured to be built
```

If it appears under:

```text
Modules that cannot be built
```

look for the dependency immediately above the DEFIANCE skip message.

---

# 37. Common `absl/time/time.h` Error

You may encounter:

```text
fatal error: absl/time/time.h: No such file or directory
```

from:

```text
contrib/ai/examples/multi-bss/
```

This is an `ns3-ai` example dependency and is not necessarily required for the DEFIANCE SUMO experiment.

If the specific research experiment does not use that AI example, do not immediately install unrelated dependencies. First determine whether the required executable has already been generated:

```bash
find build -type f \
    \( -name "*defiance-sumo-test*" -o -name "*sumo-example*" \) \
    | head -20
```

---

# 38. Useful Navigation Commands

Go to research directory:

```bash
cd ~/research
```

Go to NS-3:

```bash
cd ~/research/ns-allinone-3.47/ns-3.47
```

Go to DEFIANCE:

```bash
cd ~/research/ns-allinone-3.47/ns-3.47/contrib/defiance
```

Go to DEFIANCE SUMO example:

```bash
cd ~/research/ns-allinone-3.47/ns-3.47/contrib/defiance/examples/sumo-test
```

Return to NS-3 root:

```bash
cd ~/research/ns-allinone-3.47/ns-3.47
```

---

# 39. Useful Verification Commands

## WSL

From PowerShell:

```powershell
wsl --list --verbose
```

## Ubuntu

```bash
lsb_release -a
```

## Compiler

```bash
g++ --version
```

## CMake

```bash
cmake --version
```

## Python

```bash
python3 --version
python --version
```

## Git

```bash
git --version
```

## SUMO

```bash
sumo --version
sumo-gui --version
```

## SUMO location

```bash
which sumo
which sumo-gui
```

## TraCI

```bash
python3 -c "import traci; print('TraCI OK')"
```

## libsumo

```bash
python3 -c "import libsumo; print('libsumo OK')"
```

## DEFIANCE

```bash
python3 -c "import defiance; print('DEFIANCE Python OK')"
```

## pybind11

```bash
python3 -c "import pybind11; print(pybind11.get_include())"
```

## Protobuf

```bash
protoc --version
```

## NS-3

```bash
./ns3 --help
```

---

# 40. Recommended Environment Variables

Add these to `~/.bashrc`:

```bash
export NS3_HOME="$HOME/research/ns-allinone-3.47/ns-3.47"
export SUMO_HOME="/usr/share/sumo"
export PYTHONPATH="$NS3_HOME/contrib:$PYTHONPATH"
```

Apply:

```bash
source ~/.bashrc
```

Verify:

```bash
echo $NS3_HOME
echo $SUMO_HOME
echo $PYTHONPATH
```

---

# 41. Complete Environment Verification

Run:

```bash
echo "===== WSL / Linux ====="
uname -a

echo "===== Compiler ====="
g++ --version | head -1

echo "===== CMake ====="
cmake --version | head -1

echo "===== Python ====="
python3 --version

echo "===== Git ====="
git --version

echo "===== Protobuf ====="
protoc --version

echo "===== SUMO ====="
sumo --version | head -1

echo "===== SUMO GUI ====="
which sumo-gui

echo "===== TraCI ====="
python3 -c "import traci; print('TraCI: OK')"

echo "===== libsumo ====="
python3 -c "import libsumo; print('libsumo: OK')"

echo "===== DEFIANCE ====="
python3 -c "import defiance; print('DEFIANCE: OK')"

echo "===== pybind11 ====="
python3 -c "import pybind11; print('pybind11:', pybind11.__version__)"

echo "===== NS-3 ====="
echo "$NS3_HOME"
```

---

# 42. Quick Start After Everything Is Installed

Once the environment has been installed, future sessions only need:

```bash
cd ~/research/ns-allinone-3.47/ns-3.47
```

Check WSL environment:

```bash
echo $NS3_HOME
echo $SUMO_HOME
```

Check SUMO:

```bash
sumo --version
```

Check Python:

```bash
python3 --version
```

Check DEFIANCE:

```bash
python3 -c "import defiance; print('DEFIANCE OK')"
```

Check the target:

```bash
ninja -C cmake-cache -t targets | grep -i "defiance-sumo"
```

Build if necessary:

```bash
./ns3 build defiance-sumo-test
```

Run:

```bash
./ns3 run defiance-sumo-test
```

Or, if already built:

```bash
./build/contrib/defiance/examples/ns3.47-defiance-sumo-test-default
```

---

# 43. Recommended Project Directory Structure

Keep research code separate from installed software:

```text
~/research/
│
├── ns-allinone-3.47/
│   └── ns-3.47/
│       ├── src/
│       ├── contrib/
│       │   ├── ai/
│       │   └── defiance/
│       ├── scratch/
│       └── build/
│
├── sumo-test/
│   ├── test.sumocfg
│   ├── grid.net.xml
│   ├── ...
│   └── traci_test.py
│
└── research-venv/
```

For your actual research experiments, it is recommended to use:

```text
ns-3.47/scratch/
```

for custom NS-3 simulation programs.

---

# 44. Important Notes

### Do not install NS-3 directly into Windows

The recommended setup is:

```text
Windows → WSL2 → Ubuntu → NS-3
```

rather than attempting to build the complete NS-3 + DEFIANCE + SUMO environment natively with Windows toolchains.

### Do not mix Windows Python with WSL Python

Inside Ubuntu, use:

```bash
python3
```

and:

```bash
/usr/bin/python3
```

Do not use a Windows Python installation for NS-3/SUMO dependencies.

### Keep the project inside the Linux filesystem

Prefer:

```text
/home/mohsin/research/
```

rather than:

```text
/mnt/c/...
```

for compilation-heavy projects.

This generally gives much better filesystem performance under WSL.

### Large NS-3 builds can consume significant resources

Check:

```bash
free -h
```

and:

```bash
nproc
```

If the system becomes unstable, reduce Ninja parallelism:

```bash
ninja -C cmake-cache -j2
```

or:

```bash
ninja -C cmake-cache -j3
```

---

# 45. Current Known Working Stack

The environment established during this setup was approximately:

```text
OS:
    Windows 11
    WSL2

Linux:
    Ubuntu 24.04

Compiler:
    GCC/G++ 13.3.0

CMake:
    3.28.3

Python:
    3.12.3

Git:
    2.43.0

NS-3:
    3.47

SUMO:
    1.18.0

pybind11:
    2.11.1

Protobuf:
    3.21.12

DEFIANCE:
    configured through ns-3.47 contrib/

ns-3-ai:
    configured through contrib/ai

TraCI:
    Python package available

libsumo:
    Python module available
```

---

# 46. Final Setup Checklist

Before starting research experiments, verify all of these:

```text
[ ] WSL2 installed
[ ] Ubuntu 24.04 installed
[ ] GCC/G++ installed
[ ] CMake installed
[ ] Ninja installed
[ ] Git installed
[ ] Python 3 installed
[ ] python command available
[ ] pybind11-dev installed
[ ] Protobuf installed
[ ] protoc working
[ ] NS-3.47 downloaded
[ ] NS-3.47 configured
[ ] NS-3.47 built
[ ] SUMO installed
[ ] sumo command working
[ ] sumo-gui command working
[ ] SUMO_HOME configured
[ ] TraCI import working
[ ] libsumo import working
[ ] ns-3-ai configured
[ ] DEFIANCE configured
[ ] NS3_HOME configured
[ ] PYTHONPATH configured
[ ] defiance Python import working
[ ] defiance-sumo-test target available
[ ] DEFIANCE SUMO executable built
```

Once these are complete, the environment is ready for developing and running the NS-3 + SUMO + DEFIANCE research simulations.
