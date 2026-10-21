# Development setup

## Core library (any platform)

Requirements: CMake 3.22 or newer, Ninja, a C++17 compiler.

On Windows, install "Desktop development with C++" from the Visual Studio Installer (it includes MSVC, CMake, and Ninja) and run the commands below from a "x64 Native Tools Command Prompt for VS" or a Developer PowerShell. Sanitizer presets (`asan-ubsan`, `tsan`) use GCC/Clang flags; run them under WSL or Docker.

```sh
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Other presets: `release`, `asan-ubsan`, `tsan`. Build output goes to `build/<preset>/`.

The sanitizer presets build in Debug and run tens of times slower than release. They define `SWARM_SANITIZED`, and the few heaviest tests (the formation swarm tests) use a smaller size there through `tests/support/test_scale.hpp`. The assertions are the same; the full sizes run in the release jobs.

## Checks before pushing

```sh
bash scripts/check_core_invariants.sh
clang-format --dry-run -Werror $(git ls-files '*.cpp' '*.hpp')
```

## Network code

The tests that use sockets bind to 127.0.0.1 on ports the operating system chooses, so they need no setup and parallel runs do not collide. They do need the loopback interface, which every desktop and CI runner has. On Windows, a firewall prompt for a test executable can be dismissed; loopback traffic does not leave the machine.

To look at the traffic on Linux while a test runs, `ss -ulpn` and `ss -tlpn` list the UDP and TCP sockets, and `tcpdump -i lo -X 'udp or tcp'` shows the packets. Sockets, wall-clock time, and the `net/` library are allowed in `net/` and the apps and are forbidden in `core/`; `scripts/check_core_invariants.sh` enforces it.

## Unreal plugin

Requirements: Unreal Engine 5.4 and Visual Studio 2022 with the "Game development with C++" workload.

1. Run `scripts/link_ue.ps1` once. It creates two directory junctions (both ignored by git).
2. Open `UnrealHost/UnrealHost.uproject`. Accept the prompt to build the modules.

The plugin compiles `core/format` as sources. Keep that directory free of Eigen, yaml-cpp, and wall-clock calls, and make sure it builds as C++20 (CI has a job for this).

The plugin module is built without unity builds (`bUseUnity = false` in `SwarmSim.Build.cs`). The `core/format` sources are written to compile one at a time, as CMake builds them, and use the same names in anonymous namespaces; merged into one translation unit they collide.

To build from a command line without opening the editor (adjust the engine path; 4 parallel jobs keeps the load low):

```
F:\UE_5.4\Engine\Build\BatchFiles\Build.bat UnrealHostEditor Win64 Development -Project=F:\vscode\Drone\UnrealHost\UnrealHost.uproject -WaitMutex -MaxParallelActions=4
```

Asset packs for the viewer are installed into `UnrealHost/Content` with the editor's Fab window (Window, Fab, Add to Project). Fab's licence does not allow redistributing the raw files, so everything under `UnrealHost/Content` except the placeholder is ignored by git, and so is `UnrealHost/Config`. Which packs the author used is listed in the viewer's documentation, not in the repository. A pack's assets can be inspected without opening the editor, with a Python script run by `UnrealEditor-Cmd.exe UnrealHost.uproject -run=pythonscript -script=FILE -nullrhi -unattended -EnablePlugins=PythonScriptPlugin`.

The plugin's automation tests run without a window and without a GPU:

```
F:\UE_5.4\Engine\Binaries\Win64\UnrealEditor-Cmd.exe F:\vscode\Drone\UnrealHost\UnrealHost.uproject -ExecCmds="Automation RunTests SwarmSim; Quit" -unattended -nullrhi -nosplash -stdout -log -DisablePlugins=Fab
```

`-DisablePlugins=Fab` is needed because Unreal's Fab plugin crashes in a windowless editor when it restores its browser tab. The tests that use an asset pack report "skipped" when the pack is not installed. They are not part of CI, which cannot build Unreal.

The terrain is built from a scene file, which `swarm_sim export-scene LOG --out FILE` writes. In the editor's Python, `unreal.SwarmSceneLibrary.build_terrain_from_scene_file(world, path)` builds it.

The Unreal build is not covered by CI. Check it by hand before tagging a release. It was last built with Unreal Engine 5.4.4, Visual Studio 2022 (MSVC 14.38), and the Windows 10.0.26100 SDK, with no errors.

## Python analysis

```sh
python -m venv .venv
.venv/Scripts/activate        # Windows; use .venv/bin/activate elsewhere
pip install -r analysis/requirements.txt
```

Run the tests from the `analysis` directory:

```sh
cd analysis
python -m pytest -q
```

Run an experiment and summarize it (build the `swarm_batch` tool first):

```sh
build/release/apps/swarm_batch experiments/ablation.yaml --out results/ablation.csv --jobs 4
cd analysis
python -m swarmsim_analysis summarize ../results/ablation.csv --by variant,loss \
    --metrics formation_rmse_settled_m,min_separation_m --out ../results/ablation_summary.csv
```

The package versions are pinned in `analysis/requirements.txt`; CI installs the same ones.

Figures from the same table:

```sh
python -m swarmsim_analysis figures ../results/ablation.csv --out ../results/figures \
    --where faults=one
```

To play a run in a level, the plugin is linked into the level's project and the game is started from the command line. In a Windows shell that does not rewrite paths (not Git Bash, which turns `/Game/...` into a file path):

```
UnrealEditor.exe PROJECT.uproject /Game/Levels/MAP -game -windowed -EnablePlugins=SwarmSim -SwarmLog=run.log -SwarmAnchor=X,Y -SwarmScale=6 -SwarmCamera=orbit
```

Add `-SwarmShotDir=DIR -SwarmShotDelay=30 -SwarmShotCount=2` to save renders of the game and quit. The options are listed in `docs/MODELS.md`, under the Unreal viewer: playback.
