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

## Checks before pushing

```sh
bash scripts/check_core_invariants.sh
clang-format --dry-run -Werror $(git ls-files '*.cpp' '*.hpp')
```

## Unreal plugin

Requirements: Unreal Engine 5.4 and Visual Studio 2022 with the "Game development with C++" workload.

1. Run `scripts/link_ue.ps1` once. It creates two directory junctions (both ignored by git).
2. Open `UnrealHost/UnrealHost.uproject`. Accept the prompt to build the modules.

The plugin compiles `core/format` as sources. Keep that directory free of Eigen, yaml-cpp, and wall-clock calls, and make sure it builds as C++20 (CI has a job for this).

The Unreal build is not covered by CI. Check it by hand before tagging a release.

## Python analysis

```sh
python -m venv .venv
.venv/Scripts/activate        # Windows; use .venv/bin/activate elsewhere
pip install -r analysis/requirements.txt
```
