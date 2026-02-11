# Tashan Tactile Sensor (TS-F-A) — Function & Capability Breakdown

This document summarizes the functions and capabilities implemented in this repository for the **Tashan TS-F-A tactile sensor** Isaac Sim extension.

## 1) Sensor Outputs (what data the model provides)

The simulated TS-F-A outputs **11 channels**:

1. Proximity
2. Normal force
3. Tangential force
4. Tangential direction (0–359°)
5–11. Seven raw capacitance channels

These channels are documented as the core feature set in the main project README.

## 2) Runtime User Workflow in Isaac Sim

The extension UI exposes a 3-step interaction model:

- **LOAD**: creates a new stage, adds lighting, loads the TS-F-A USD asset, spawns falling test cards, initializes world scene objects, and enables scenario controls.
- **RESET**: tears down and re-initializes sensor/scenario state so simulation can be re-run from initial conditions.
- **RUN / STOP**:
  - RUN starts timeline playback and per-physics-step sensor sampling.
  - STOP pauses the timeline and saves a force/proximity trend plot image.

This behavior is implemented through template callbacks in `extension.py` + `ui_builder.py` and scenario logic in `scenario.py`.

## 3) Core Extension and UI Functions

### Extension lifecycle (`extension.py`)

- `on_startup(ext_id)`: creates docking window, toolbar/menu action, UIBuilder instance, and stage/timeline/physx interfaces.
- `on_shutdown()`: removes menu actions/items, clears window, runs UI cleanup.
- `_on_window(visible)`: subscribes to stage and timeline events when opened, unsubscribes and cleans up when closed.
- `_build_ui()`: builds UI and docks panel near Viewport.
- `_menu_callback()`: toggles extension window and notifies UIBuilder.
- `_on_timeline_event(event)`: hooks physics-step callback on PLAY, clears it on STOP, forwards events to UIBuilder.
- `_on_physics_step(step)`: forwards each physics step to UIBuilder.
- `_on_stage_event(event)`: resets on OPENED/CLOSED stage events, forwards event.

### UI controls and scene orchestration (`ui_builder.py`)

- `build_ui()`: builds two frames:
  - **World Controls**: LOAD + RESET
  - **Run Scenario**: RUN/STOP state button
- `_setup_scene()`: creates new stage, adds light, and initializes world instance.
- `_setup_scenario()`: prepares the TS scenario backend, adds ground plane and sensor objects to scene.
- `_on_post_reset_btn()`: invokes scenario teardown and re-enables run controls.
- `_update_scenario(step)`: per-physics-step call into scenario backend.
- `_on_run_scenario_a_text()`: timeline play.
- `_on_run_scenario_b_text()`: timeline pause + chart rendering.
- `_reset_extension()`, `_reset_ui()`: clean full extension state after stage reset/open.

## 4) Sensor/Scenario Backend Capabilities

### Base abstraction (`ScenarioTemplate`)

- Declares scenario lifecycle hooks:
  - `setup_scenario`
  - `teardown_scenario`
  - `update_scenario`

### TS-F-A example implementation (`ExampleScenario`)

- `__init__()`:
  - initializes runtime state,
  - allocates per-frame data buffers,
  - calls `_load_register_sensor()` to load native sensor callback module.

- `setup_scenario(...)`:
  - calls `_set_up_sensor_and_scene()` to add TS-F-A and test cards,
  - marks scenario running,
  - sets viewport camera,
  - initializes Rerun stream (`rr.init`) for live scalar telemetry.

- `teardown_scenario()`:
  - re-initializes scene,
  - clears buffers and running state.

- `update_scenario(step, step_ind)`:
  - fetches frame data by invoking native callback `TSsensor(self.tactile, self.range)`,
  - buffers first 100 samples,
  - updates elapsed simulation time,
  - publishes live values to Rerun.

- `draw_data()`:
  - generates/saves `sensor_data/module.png` from buffered data,
  - plots proximity, normal force, tangential force versus sample index.

- `_load_register_sensor()`:
  - detects Isaac Sim version,
  - inserts version-specific binary path (`ts_sensor_lib/isaac-sim-<version>`) into `sys.path`,
  - imports `register_sensor.TSsensor` from compiled module,
  - logs success/failure.

- `_update_rerun_visualization()`:
  - logs scalar streams `force_normal`, `force_tangential`, `proximity` to Rerun.

- `_set_up_sensor_and_scene()`:
  - adds TS-F-A USD at `/World/Tip`,
  - applies transform,
  - creates articulation object,
  - spawns 4 thin dynamic cuboids (cards) above sensor to free-fall,
  - configures range sensor path `/World/Tip/pad_4/LightBeam_Sensor`,
  - configures tactile rigid-prim expression `/World/Tip/pad_[1-7]` with contact filtering and max contact count.

## 5) Native Sensor Library Integration

The tactile callback itself is provided as versioned shared objects:

- `ts_sensor_lib/isaac-sim-4.5.0/register_sensor.so`
- `ts_sensor_lib/isaac-sim-5.0.0/register_sensor.so`

At runtime, `ExampleScenario` dynamically selects the folder matching the app version and imports `TSsensor` from `register_sensor`.

## 6) Practical Capabilities Demonstrated in This Repo

- Product-like tactile model simulation aligned to TS-F-A hardware concept.
- Combined **proximity + contact-force + raw-capacitance** representation.
- Event-driven integration with Isaac Sim timeline/stage lifecycle.
- Runtime one-click scene setup and replay via extension UI.
- Physics-step synchronous sensor data acquisition.
- Lightweight online telemetry visualization (Rerun scalars).
- Offline post-run plotting for quick validation/regression checks.
- Reference recipe for embedding TS-F-A in custom robot scenes (`TSsensor(self.tactile, self.range)`).

