# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

This project uses [PROS](https://pros.cs.purdue.edu/) (a VEX robotics C++ framework). Build via the PROS CLI:

```bash
pros build          # compile the project
pros upload         # upload to the robot brain
pros build-upload   # build then upload in one step
pros make           # alternative to pros build (uses Makefile)
```

There are no unit tests. Validation is done by running on hardware.

## Architecture Overview

This is a fork of **LemLib v0.5.6** extended with a custom team library called **`pushback`** for VRC Push Back (2025-2026).

### Two-layer structure

**Layer 1 — LemLib** (`include/lemlib/`, `src/lemlib/`)
The upstream autonomous motion library. Provides:
- `lemlib::Chassis` — the central object for all motion (PID, Pure Pursuit, Boomerang, odometry)
- `lemlib::Pose` — (x, y, theta) in inches/degrees using a field-centered coordinate system (±70 inches)
- `lemlib::TrackingWheel`, `lemlib::OdomSensors` — odometry configuration
- `lemlib::ExpoDriveCurve` — joystick expo curves for driver control

**Layer 2 — pushback** (`include/pushback/`, `src/pushback/`)
The team-specific hardware abstraction built on top of LemLib:

- **`Robot`** (`Robot.hpp/cpp`) — centralizes all hardware into one object. Passed by reference to subsystems. Key methods: `ram()`, `jiggle()`, `reset_x()`/`reset_y()` (wall-based odometry correction using distance sensors), `safe_to_reset_x()`/`safe_to_reset_y()` (guards for when reset is valid).
- **`Intake`** (`Intake.hpp/cpp`) — 3-stage intake control. `runIntake()` must be called every opcontrol loop tick. `color_sort()` and `anti_jam()` are also designed to run every loop tick.
- **`Piston`** (`Piston.hpp/cpp`) — wraps `pros::ADIDigitalOut` with toggle support. `register_controller()` must be called in `initialize()` before `toggleFire()` works in opcontrol.

### Entry point: `src/main.cpp`

All hardware is declared globally at the top of `main.cpp`. The global `robot` object owns references to everything. Autonomous routines are plain functions defined in the same file and one is called from `autonomous()`.

The active autonomous is selected by uncommenting the desired function call inside `autonomous()`.

### Wall-reset odometry

During autonomous, a background task (`reset_robot`) continuously calls `robot.safe_to_reset_x/y()` and `robot.reset_x/y()` to correct odometry drift using two side-facing distance sensors. The field coordinate system is ±70 inches from center, so `reset_x = 70 - sensor_distance`. Resets are gated by angle, proximity to walls, and field obstacle zones (matchloader areas) to avoid bad readings.

### Coordinate system

Field center is (0, 0). Positive X is toward the red matchloader wall. Positive Y is toward the top of the field. Angles in LemLib are in degrees, where 0° = facing positive Y (up).

### Pure Pursuit paths

Path files live in `static/` as `.txt` files and are loaded with the `ASSET()` macro. They are followed using `chassis.follow(asset, lookahead, timeout, forward, async)`.

### Path visualizer

`visualizers/vis.py` reads `visualizers/logs.txt` (serial output from `logData()` background task, format: `x,y,theta` at 50ms intervals) and renders the robot path on a field image. Run it from the `visualizers/` directory:

```bash
cd visualizers
python vis.py
```

Old logs are auto-archived to `visualizers/oldLogs/` on each run (deduplication by file content).

## Key Conventions

- All autonomous routines are blocking by default; pass `async = true` as the last argument to LemLib motion calls for non-blocking, then call `chassis.waitUntilDone()` when you need to sync.
- `robot.ram(speed, ms)` replaces `chassis.arcade` + `pros::delay` for simple timed drives.
- `moveStraight(length, timeout, params)` in `main.cpp` computes a target point from current pose and heading — use this instead of raw `moveToPoint` for relative straight movements.
- `color_sort` and `wall_reset_enabled` are global booleans that gate background task behavior; set them at the top of each autonomous routine as needed.
