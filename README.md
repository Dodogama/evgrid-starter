# evgrid-starter

A minimal, dependency-free C++20 project that already does the thing your work
project does — badly, on purpose. Radar detections in, log-odds evidence grid out.
Every scenario in the roadmap below is an upgrade to this codebase.

## Toolchain (CachyOS / Arch)

```bash
sudo pacman -S --needed base-devel cmake ninja gdb clang lldb
# base-devel brings gcc; clang brings clangd + clang-format + clang-tidy
```

VS Code extensions:

| Extension | Publisher | Why |
|---|---|---|
| clangd | LLVM | Real IntelliSense. Backed by the actual compiler frontend. |
| CMake Tools | Microsoft | Configure/build/debug from the status bar. |
| CodeLLDB | Vadim Chugunov | Better debugger UX than MI-mode gdb. |

**Do not also install Microsoft's "C/C++" (ms-vscode.cpptools) IntelliSense.**
It fights clangd. If it's installed for the debugger, set
`"C_Cpp.intelliSenseEngine": "disabled"` in settings.

`.vscode/settings.json`:

```json
{
  "clangd.arguments": ["--compile-commands-dir=build/debug", "--header-insertion=never"],
  "cmake.configureOnOpen": true,
  "editor.formatOnSave": true,
  "[cpp]": { "editor.defaultFormatter": "llvm-vs-code-extensions.vscode-clangd" }
}
```

## Build and run

```bash
cmake --preset debug     # configures into build/debug, writes compile_commands.json
cmake --build --preset debug
./build/debug/evgrid_demo
```

The debug preset builds with ASan + UBSan. Leave them on while you're learning —
they turn silent undefined behavior into a loud stack trace, which is the single
biggest thing standing between you and productive C++.

Release build: `cmake --preset release && cmake --build --preset release`.

## What's in here

```
include/eg/grid.hpp   GridSpec, Cell, LogOddsGrid   — the data structure
include/eg/radar.hpp  Pose2D, Detection, SensorModel — the sensor abstraction
src/grid.cpp          indexing, world<->cell, log-odds math
src/radar.cpp         inverse sensor model + ray marching
src/main.cpp          toy world simulator, 20-scan run, ASCII renderer
```

## Roadmap

1. Read and break the starter — indexing, references, const, RAII
2. Grid ownership & lifetime — copy/move, rule of zero, `std::span` views
3. Templates & `Eigen` — swap hand-rolled math for a real linear algebra library
4. Dead reckoning + pose error — watch the map smear when odometry drifts
5. Scan matching / registration — ICP, then NDT, then compare
6. Dempster-Shafer — replace log-odds with (m_occ, m_free, m_unknown)
7. Doppler-based dynamic/static separation — use range rate to reject movers
8. Performance — profile, cache layout, `std::execution`, then a thread pool
9. Testing & CI — GoogleTest, deterministic seeds, regression on grid checksums
10. Interfaces — clean the API up so a second sensor type drops in without edits

## Notes on MSVC parity

Your work build will likely be MSVC. Things that compile on GCC and fail there:

- Missing `#include`s that GCC pulls in transitively (`<cstdint>`, `<algorithm>`)
- `M_PI` — not defined by MSVC without `_USE_MATH_DEFINES`. Use `std::numbers::pi`
  (C++20) in anything you'll port.
- VLAs (`int arr[n]`) — a GCC extension, not standard C++. Never use them.
- Two-phase name lookup in templates: MSVC is more permissive by default.
  `/permissive-` is already set in CMakeLists.txt for this reason.

Occasionally build with `clang++` too. Three compilers disagreeing with you is
cheaper than one compiler disagreeing with you at integration time.
