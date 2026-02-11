# Tashan TS-F-A Tactile Sensor — Complete Technical Documentation

> Comprehensive reference for integrating the TS-F-A into robotic simulation
> scenarios. Covers every capability and limitation extractable from the
> repository source code, native binary symbols, USD asset, and vendor
> documentation.

---

## Table of Contents

1. [Sensor Overview](#1-sensor-overview)
2. [Physical Layer — Hardware & Sensing Principles](#2-physical-layer--hardware--sensing-principles)
   - 2.1 [Taxel Array Geometry](#21-taxel-array-geometry)
   - 2.2 [Capacitive Sensing Layer](#22-capacitive-sensing-layer)
   - 2.3 [Force Sensing](#23-force-sensing)
   - 2.4 [Proximity Sensing (LightBeam)](#24-proximity-sensing-lightbeam)
   - 2.5 [Observed Force Ranges (from simulation output)](#25-observed-force-ranges-from-simulation-output)
   - 2.6 [Physical Dimensions & Mounting](#26-physical-dimensions--mounting)
3. [Output Data Specification](#3-output-data-specification)
   - 3.1 [11-Channel Feature Vector](#31-11-channel-feature-vector)
   - 3.2 [Derived Quantities](#32-derived-quantities)
4. [Native Sensor Library — Internal Architecture](#4-native-sensor-library--internal-architecture)
   - 4.1 [Exported Functions](#41-exported-functions)
   - 4.2 [Internal Processing Pipeline](#42-internal-processing-pipeline)
   - 4.3 [Noise Model](#43-noise-model)
   - 4.4 [Contact Force Matrix Computation](#44-contact-force-matrix-computation)
   - 4.5 [Friction Data Processing](#45-friction-data-processing)
   - 4.6 [Capacitance Computation](#46-capacitance-computation)
5. [Simulation Integration Layer](#5-simulation-integration-layer)
   - 5.1 [Isaac Sim Extension Architecture](#51-isaac-sim-extension-architecture)
   - 5.2 [Physics Engine Configuration](#52-physics-engine-configuration)
   - 5.3 [USD Scene Hierarchy](#53-usd-scene-hierarchy)
   - 5.4 [Contact Detection Configuration](#54-contact-detection-configuration)
   - 5.5 [Supported Isaac Sim Versions](#55-supported-isaac-sim-versions)
6. [Software API Reference](#6-software-api-reference)
   - 6.1 [TSsensor() — Primary Interface](#61-tssensor--primary-interface)
   - 6.2 [ScenarioTemplate Base Class](#62-scenariotemplate-base-class)
   - 6.3 [ExampleScenario (Free-Fall)](#63-examplescenario-free-fall)
   - 6.4 [ExampleScenario2 (Manual Drag)](#64-examplescenario2-manual-drag)
   - 6.5 [Extension Lifecycle](#65-extension-lifecycle)
   - 6.6 [UI Builder Controls](#66-ui-builder-controls)
7. [Visualization & Telemetry](#7-visualization--telemetry)
   - 7.1 [Rerun Real-Time Streaming](#71-rerun-real-time-streaming)
   - 7.2 [Matplotlib Offline Plots](#72-matplotlib-offline-plots)
8. [Existing Simulation Scenarios](#8-existing-simulation-scenarios)
   - 8.1 [Scenario 1 — Free-Fall Impact Test](#81-scenario-1--free-fall-impact-test)
   - 8.2 [Scenario 2 — Manual Drag Interaction](#82-scenario-2--manual-drag-interaction)
9. [Integration Guide — Adding TS-F-A to Custom Robot Scenarios](#9-integration-guide--adding-ts-f-a-to-custom-robot-scenarios)
10. [Known Limitations & Constraints](#10-known-limitations--constraints)
11. [Dependencies & System Requirements](#11-dependencies--system-requirements)
12. [Repository File Map](#12-repository-file-map)

---

## 1. Sensor Overview

The **Tashan TS-F-A** is a universal tactile sensor module designed for robotic
fingertip applications. Developed by Tashan Technology (他山科技), it is
described as **China's first tactile simulation model based on a real commercial
product**. The simulation model is built as an NVIDIA Isaac Sim extension and
provides a physics-accurate digital twin of the physical sensor hardware.

**Key characteristics:**

| Property | Value |
|---|---|
| Sensor model | TS-F-A (Universal Tactile Sensor) |
| Vendor | Tashan Technology (他山科技) |
| Simulation platform | NVIDIA Isaac Sim 4.5.0 / 5.0.0 |
| Physics engine | NVIDIA PhysX (via Isaac Sim) |
| Output dimensionality | 11 channels per frame |
| Sensing modalities | Proximity, Normal Force, Tangential Force, Tangential Direction, 7-ch Capacitance |
| Taxel count | 7 independent tactile pads |
| Sampling rate (sim) | 60 Hz (synchronized with physics timestep) |
| 3D asset format | USD (Pixar Universal Scene Description), crate v0.8.0 |
| Native library | Compiled Cython → shared object (`register_sensor.so`) |
| License | Apache License 2.0 |
| Extension version | 1.0.1 |

---

## 2. Physical Layer — Hardware & Sensing Principles

### 2.1 Taxel Array Geometry

The TS-F-A sensor surface is divided into **7 independent tactile pads**
(taxels), each acting as a discrete contact-sensing element.

```
USD prim paths:
  /World/Tip/pad_1
  /World/Tip/pad_2
  /World/Tip/pad_3
  /World/Tip/pad_4   ← also hosts the LightBeam proximity sensor
  /World/Tip/pad_5
  /World/Tip/pad_6
  /World/Tip/pad_7
```

**Pad arrangement (from simulation screenshot):** The 7 pads are visible on the
green sensing surface of the simulated module. The layout shows a central pad
surrounded by peripheral pads, forming a roughly rectangular sensing area. The
pads are separated by thin dividing lines, creating distinct contact zones.

**Sensing surface geometry (from test object positioning):**

| Parameter | Value | Source |
|---|---|---|
| Approximate sensing area footprint | ~40 mm x 20 mm (X x Y) | Inferred from plate positions at +/-20mm X, +/-10mm Y |
| Pad count | 7 | USD hierarchy + code `pad_[1-7]` |
| Prim expression for all pads | `/World/Tip/pad_[1-7]` | `scenario.py:166` |
| Contact capacity per pad (scenario 1) | 5 contacts | `max_contact_count=7*5` at `scenario.py:169` |
| Contact capacity per pad (scenario 2) | 8 contacts | `max_contact_count=7*8` at `scenario2.py:190` |

**Important limitation:** Individual taxel spatial resolution (mm per taxel) and
exact pitch/spacing between pads are encoded in the binary USD asset
(`TS-F-A.usd`, crate format) and are not directly readable from the repository
source code. The 3D geometry is compiled into the 3.0 MB USD file.

### 2.2 Capacitive Sensing Layer

Each of the 7 pads provides a **raw capacitance channel**, giving 7 independent
capacitance readings per frame.

| Parameter | Value | Source |
|---|---|---|
| Number of capacitance channels | 7 (one per pad) | Output channels 4–10 |
| Output indices | `sensorFrameData[4]` through `sensorFrameData[10]` | `scenario2.py:152-153` |
| Output unit | Raw capacitance (unit not specified in code) | — |
| Capacitance range (min/max) | **Not exposed** — encoded in native binary | `register_sensor.so` |
| Capacitance resolution | **Not exposed** — encoded in native binary | `register_sensor.so` |

**How capacitance is computed:** The native library symbol `capacitance` confirms
this is computed internally. Based on the symbol table, the computation likely
uses the `get_contact_force_matrix` and `normal_forces_7` data (7 per-pad normal
forces) to derive capacitance values through the `Equation` function, which
presumably maps physical contact deformation to capacitance change.

**Limitation:** The actual capacitance unit (pF, fF, or arbitrary), absolute
range, and resolution are opaque — they are baked into the compiled
`register_sensor.so` binary and the `Equation` transfer function. No calibration
constants or conversion factors are exposed in the Python API.

### 2.3 Force Sensing

The sensor measures contact forces decomposed into normal and tangential
(shear) components.

| Parameter | Value | Source |
|---|---|---|
| Normal force output | Channel 1 — scalar in Newtons (N) | `scenario.py:94`, plot Y-axis label |
| Tangential force output | Channel 2 — scalar magnitude in Newtons (N) | `scenario.py:95` |
| Tangential direction | Channel 3 — degrees, 0–359 | README: "fingertip direction as 0" |
| Direction reference | 0 = fingertip direction | `README.md:15` |
| Direction resolution | Continuous float (computed via `atan2`) | Binary symbol `atan2` |
| Per-pad normal forces | 7 individual normal forces (internal) | Binary symbol `normal_forces_7` |
| Aggregate friction force | Summed friction across pads (internal) | Binary symbol `friction_force_aggregate` |
| Per-finger friction | Per-finger friction force (internal) | Binary symbol `friction_force_finger` |

**Force computation pipeline (reconstructed from binary symbols):**

```
PhysX contact data
    ↓
get_contact_force_matrix()     → raw contact force matrix (all pads)
get_friction_data()            → friction points, forces, counts
    ↓
normal_forces_7                → per-pad normal forces (7 values)
friction_force_aggregate       → total friction across all pads
friction_force_finger          → finger-level friction
    ↓
tangential_force               → magnitude of shear force (via linalg.norm)
direction                      → angle of shear force (via atan2)
    ↓
Equation()                     → transfer function mapping to sensor output
    ↓
Generate_std() + noise_scale   → adds simulated sensor noise
    ↓
[proximity, normal, tangential, direction, cap1..cap7]
```

**Friction data decomposition (from binary symbols):**

| Internal variable | Description |
|---|---|
| `friction_local` | Friction forces in local (sensor) frame |
| `friction_world` | Friction forces in world frame |
| `friction_points` | Contact point positions |
| `friction_count` | Number of friction contacts |
| `friction_pair_contacts_count` | Contacts per pair |
| `friction_pair_contacts_start_indices` | Index offsets per pair |
| `friction_start_idx` | Start index for friction data |
| `pair_forces` | Per-pair contact forces |

### 2.4 Proximity Sensing (LightBeam)

The sensor includes a **light-beam based proximity sensor** located on pad 4.

| Parameter | Value | Source |
|---|---|---|
| Sensor type | LightBeam range sensor | `scenario.py:164` |
| USD prim path | `/World/Tip/pad_4/LightBeam_Sensor` | `scenario.py:164` |
| Isaac Sim API | `isaacsim.sensors.physx._range_sensor` | `scenario.py:10` |
| Interface | `acquire_lightbeam_sensor_interface` | Binary symbol |
| Data retrieval | `get_linear_depth_data` | Binary symbol |
| Output channel | Channel 0 (proximity) | `sensorFrameData[0]` |
| Output unit | Scalar (likely normalized or meters) | — |

The proximity sensor uses a simulated light beam to measure distance to the
nearest object above the sensor surface. The native library acquires the
`lightbeam_sensor_interface` and calls `get_linear_depth_data` to read depth
values, which are then mapped to the proximity output channel.

### 2.5 Observed Force Ranges (from simulation output)

The sample output plot (`sensor_data/module.png`) from the free-fall card
scenario provides empirical force ranges:

| Measurement | Observed Value | Condition |
|---|---|---|
| Peak normal force | ~3.8 N | 20g card free-fall impact from ~5mm height |
| Peak tangential force | ~1.1 N | Same impact event |
| Steady-state proximity | ~0.1 (constant baseline) | After cards settle on surface |
| Resting normal force | ~0.0 N | After impact dissipation |
| Impact duration | ~3–4 timesteps (~50–67 ms) | From first contact to settling |
| Secondary impact peak | ~0.6 N normal | Bounce from second card |

**Limitation:** These are observed values from a single test configuration (4
cards, 20g each, free-fall from 5mm). The absolute force measurement range
(minimum detectable force, saturation force) is not documented and is
determined by the PhysX contact solver combined with the native `Equation`
transfer function.

### 2.6 Physical Dimensions & Mounting

| Parameter | Value | Source |
|---|---|---|
| Sensor form factor | Fingertip module (rounded rectangular) | Hardware photo `ts-f-a_real.png` |
| Housing material (physical) | Dark polymer/rubber casing | Hardware photo |
| Mounting | Through-hole at base for mechanical attachment | Hardware photo |
| Sim mount position | `/World/Tip` at translation [0, 0, 0.05] m | `scenario.py:149` |
| Articulation root | `/World/Tip/root_joint` | `scenario.py:151` |
| Sensor elevation in sim | 50 mm above world origin | Z-translate = 0.05 |
| Sensing surface height | ~101.5 mm above origin (50mm mount + ~51.5mm body) | Plate Z at 0.1015 |
| USD asset file | `assets/TS-F-A.usd` (3.0 MB, PXR-USDC crate v0.8.0) | Binary USD |

**Physical sensor (from image analysis):** The TS-F-A has a compact fingertip
form factor with a rounded rectangular sensing surface on top and a cylindrical
mounting hole at the base. The simulation model accurately reproduces this
geometry with 7 green-colored contact pads visible on the sensing surface.

---

## 3. Output Data Specification

### 3.1 11-Channel Feature Vector

Every physics step, the `TSsensor()` function returns an **11-element array**:

| Index | Channel | Unit | Range | Description |
|---|---|---|---|---|
| 0 | Proximity | scalar | continuous | Distance to nearest object above pad 4 (LightBeam sensor) |
| 1 | Normal force | N | continuous | Aggregate perpendicular contact force across all 7 pads |
| 2 | Tangential force | N | continuous | Aggregate shear/friction force magnitude across all pads |
| 3 | Tangential direction | degrees | 0–359 | Angle of shear force vector; 0 = fingertip direction |
| 4 | Capacitance ch 1 | raw | continuous | Raw capacitance reading from pad 1 |
| 5 | Capacitance ch 2 | raw | continuous | Raw capacitance reading from pad 2 |
| 6 | Capacitance ch 3 | raw | continuous | Raw capacitance reading from pad 3 |
| 7 | Capacitance ch 4 | raw | continuous | Raw capacitance reading from pad 4 |
| 8 | Capacitance ch 5 | raw | continuous | Raw capacitance reading from pad 5 |
| 9 | Capacitance ch 6 | raw | continuous | Raw capacitance reading from pad 6 |
| 10 | Capacitance ch 7 | raw | continuous | Raw capacitance reading from pad 7 |

### 3.2 Derived Quantities

Scenario 2 computes additional derived values from the raw output
(`scenario2.py:144-149`):

| Quantity | Formula | Description |
|---|---|---|
| Shear X | `tangential_force * cos(direction_rad)` | X-component of shear vector |
| Shear Y | `tangential_force * sin(direction_rad)` | Y-component of shear vector |
| `direction_rad` | `np.deg2rad(sensorFrameData[3])` | Direction converted to radians |

---

## 4. Native Sensor Library — Internal Architecture

The core sensor computation is implemented in a compiled Cython module
distributed as a platform-specific shared object.

### 4.1 Exported Functions

| Function | Signature | Purpose |
|---|---|---|
| `TSsensor` | `TSsensor(tactile_prim, range_sensor_path) → list[11]` | Main sensor callback — returns 11-channel data |
| `Equation` | Internal (not documented) | Transfer function mapping raw physics data to calibrated sensor output |
| `Generate_std` | Internal (not documented) | Generates standard deviation values for noise injection |

### 4.2 Internal Processing Pipeline

Reconstructed from binary symbol analysis of `register_sensor.so`:

```
1. CONTACT DATA ACQUISITION
   ├── tactile_prim.get_contact_force_matrix()   → raw contact forces
   ├── tactile_prim.get_friction_data()           → friction forces & points
   └── acquire_lightbeam_sensor_interface()
       └── get_linear_depth_data()                → proximity depth

2. PER-PAD FORCE DECOMPOSITION
   ├── normal_forces_7                            → 7 individual pad normals
   ├── friction_forces                            → per-contact friction
   ├── friction_local / friction_world            → frame-transformed friction
   ├── friction_force_aggregate                   → total friction force
   └── friction_force_finger                      → finger-level friction

3. FORCE VECTOR COMPUTATION
   ├── normal = sum/mean of normal_forces_7       → aggregate normal (N)
   ├── tangential_force = linalg.norm(friction)   → shear magnitude (N)
   └── direction = atan2(fy, fx)                  → shear angle (deg)

4. CAPACITANCE MAPPING
   └── capacitance = Equation(normal_forces_7)    → 7 raw capacitance values

5. NOISE INJECTION
   ├── noise = Generate_std(noise_scale)          → simulated measurement noise
   └── output += noise                            → noisy sensor readings

6. OUTPUT ASSEMBLY
   └── [proximity, normal, tangential, direction, cap1..cap7]
```

### 4.3 Noise Model

The native library includes a **stochastic noise model** for realistic sensor
simulation:

| Symbol | Purpose |
|---|---|
| `noise` | Noise values applied to output |
| `noise_scale` | Scale factor controlling noise amplitude |
| `Generate_std` | Function generating noise standard deviation |
| `random` | NumPy random number generation |
| `std` | Standard deviation computation |

This means the sensor output is **not deterministic** — repeated simulations
with identical physics states may produce slightly different readings due to
injected noise. This is important for:
- Reinforcement learning (provides exploration via sensor noise)
- Robustness testing (algorithms must handle noisy inputs)
- Sim-to-real transfer (noise characteristics can approximate real hardware)

**Limitation:** The `noise_scale` parameter is not configurable from the Python
API. It is hardcoded inside the compiled binary. Users cannot adjust noise
amplitude without recompiling the native module.

### 4.4 Contact Force Matrix Computation

The library uses Isaac Sim's PhysX contact reporting API:

```python
# Internal to register_sensor.so:
contact_force_matrix = tactile_prim.get_contact_force_matrix()
# Returns: shape (num_shapes, num_filters, 3) tensor of contact forces
#   num_shapes = 7 (one per pad)
#   num_filters = number of filtered contact objects
#   3 = force XYZ components
```

Key internal variables:
- `num_shapes`: Number of collision shapes (7 pads)
- `num_filters`: Number of contact filter objects
- `pair_forces`: Per-pair contact forces between pads and objects

### 4.5 Friction Data Processing

Friction (tangential) forces are processed through a multi-step pipeline:

1. **Raw friction retrieval:** `get_friction_data()` returns friction contact
   points, forces, and indexing data
2. **Frame transformation:** Forces are available in both `friction_local`
   (sensor frame) and `friction_world` (world frame) coordinates
3. **Aggregation:** `friction_force_aggregate` sums all friction contributions
4. **Pose compensation:** `get_local_poses()` with quaternion transforms
   (`Quatf`, `Transform`, `GetInverse`) converts forces between reference
   frames

### 4.6 Capacitance Computation

The `Equation` function converts physical contact data to capacitance values.
Based on the internal symbol references:

- Inputs: Per-pad normal forces (`normal_forces_7`), possibly friction data
- Processing: Mathematical equation (likely polynomial or empirical curve fit)
- Output: 7 raw capacitance values
- The `scale` parameter may control sensitivity mapping

**Limitation:** The exact mathematical form of `Equation()` is unknown — it is
compiled into the shared object. This means users cannot:
- Modify the force-to-capacitance mapping
- Adjust sensitivity characteristics
- Apply custom calibration curves

---

## 5. Simulation Integration Layer

### 5.1 Isaac Sim Extension Architecture

```
┌──────────────────────────────────────────────────┐
│                  Isaac Sim Runtime                │
│               (NVIDIA Omniverse Kit)              │
├──────────────────────────────────────────────────┤
│                                                  │
│  ┌──────────────────────────────────────────┐    │
│  │        Extension (omni.ext.IExt)         │    │
│  │          extension.py:46                 │    │
│  │                                          │    │
│  │  ┌─────────────┐  ┌──────────────────┐   │    │
│  │  │ UIBuilder   │  │ Timeline Events  │   │    │
│  │  │ ui_builder  │  │ Physics Steps    │   │    │
│  │  │  .py:30     │  │ Stage Events     │   │    │
│  │  └──────┬──────┘  └────────┬─────────┘   │    │
│  │         │                  │              │    │
│  │  ┌──────▼──────────────────▼──────────┐   │    │
│  │  │   ExampleScenario / Scenario2      │   │    │
│  │  │   scenario.py:51 / scenario2.py:39 │   │    │
│  │  └──────────────┬─────────────────────┘   │    │
│  │                 │                         │    │
│  │  ┌──────────────▼─────────────────────┐   │    │
│  │  │   register_sensor.so (Cython)      │   │    │
│  │  │   TSsensor() → 11-ch output        │   │    │
│  │  └──────────────┬─────────────────────┘   │    │
│  │                 │                         │    │
│  └─────────────────┼─────────────────────────┘    │
│                    │                              │
│  ┌─────────────────▼────────────────────────┐     │
│  │        PhysX Contact Solver              │     │
│  │   RigidPrim contact detection            │     │
│  │   LightBeam range sensor                 │     │
│  └──────────────────────────────────────────┘     │
└──────────────────────────────────────────────────┘
```

### 5.2 Physics Engine Configuration

| Parameter | Value | Source |
|---|---|---|
| Physics engine | NVIDIA PhysX | Via `omni.physx` |
| Physics timestep (dt) | 1/60.0 s = 16.67 ms | `ui_builder.py:107` |
| Rendering timestep | 1/60.0 s = 16.67 ms | `ui_builder.py:107` |
| Simulation frequency | 60 Hz | Derived from dt |
| Physics-render sync | Locked 1:1 | Both set to same dt |
| Ground plane | Default Isaac Sim ground | `ui_builder.py:182` |
| Lighting | SphereLight, radius=2, intensity=100000 | `ui_builder.py:145-147` |
| Light position | [6.5, 0, 12] meters | `ui_builder.py:147` |

### 5.3 USD Scene Hierarchy

```
/World
├── Tip/                                (TS-F-A sensor root)
│   ├── root_joint                      (SingleArticulation anchor)
│   ├── pad_1                           (tactile pad — RigidPrim)
│   ├── pad_2                           (tactile pad — RigidPrim)
│   ├── pad_3                           (tactile pad — RigidPrim)
│   ├── pad_4/                          (tactile pad — RigidPrim)
│   │   └── LightBeam_Sensor           (proximity range sensor)
│   ├── pad_5                           (tactile pad — RigidPrim)
│   ├── pad_6                           (tactile pad — RigidPrim)
│   └── pad_7                           (tactile pad — RigidPrim)
├── SphereLight                         (scene illumination)
├── defaultGroundPlane                  (physics ground)
└── [test objects]                      (scenario-dependent)
    ├── Cube1..Cube4                    (scenario 1: falling cards)
    └── Plate1..Plate4                  (scenario 2: drag plates)
```

### 5.4 Contact Detection Configuration

**Scenario 1 (Free-fall):**
```python
RigidPrim(
    prim_paths_expr="/World/Tip/pad_[1-7]",   # 7 contact pads
    name="finger_tactile",
    contact_filter_prim_paths_expr=["/World/Cube1"],  # filtered objects
    max_contact_count=7*5,                     # 35 max contacts total
)
```

**Scenario 2 (Manual drag):**
```python
RigidPrim(
    prim_paths_expr="/World/Tip/pad_[1-7]",
    name="finger_tactile",
    contact_filter_prim_paths_expr=[
        "/World/Plate1", "/World/Plate2",
        "/World/Plate3", "/World/Plate4"
    ],
    max_contact_count=7*8,                     # 56 max contacts total
)
```

**Contact filter behavior:** Only contacts between the 7 pads and the
explicitly listed filter objects are detected. All other collisions are ignored
by the sensor. This is critical for custom scenarios — you must add your
contact objects to the filter list.

### 5.5 Supported Isaac Sim Versions

| Version | Library Path | Status |
|---|---|---|
| Isaac Sim 4.5.0 | `ts_sensor_lib/isaac-sim-4.5.0/register_sensor.so` | Supported |
| Isaac Sim 5.0.0 | `ts_sensor_lib/isaac-sim-5.0.0/register_sensor.so` | Supported (primary) |
| Other versions | — | Not supported (prints warning) |

Version detection is automatic at runtime via `APP.get_app().get_app_version()`.

---

## 6. Software API Reference

### 6.1 TSsensor() — Primary Interface

```python
from register_sensor import TSsensor

sensor_data = TSsensor(tactile_prim, range_sensor_path)
```

**Parameters:**

| Parameter | Type | Description |
|---|---|---|
| `tactile_prim` | `RigidPrim` | Rigid prim with `prim_paths_expr="/World/Tip/pad_[1-7]"` and contact filtering configured |
| `range_sensor_path` | `str` | Path to LightBeam sensor, e.g. `"/World/Tip/pad_4/LightBeam_Sensor"` |

**Returns:** `list` of 11 `float` values (see [Section 3.1](#31-11-channel-feature-vector))

**Call frequency:** Once per physics step (60 Hz)

**Thread safety:** Must be called from the physics step callback thread

### 6.2 ScenarioTemplate Base Class

Defined in both `scenario.py:26` and `scenario2.py:25`:

```python
class ScenarioTemplate:
    def setup_scenario(self):    ...  # Initialize sensor and scene
    def teardown_scenario(self): ...  # Clean up state
    def update_scenario(self):   ...  # Per-physics-step update
```

### 6.3 ExampleScenario (Free-Fall)

**File:** `scenario.py:51`

| Method | Description |
|---|---|
| `__init__()` | Allocates buffers, loads native library |
| `setup_scenario(articulation, object_prim)` | Loads USD, spawns cards, sets camera, starts Rerun |
| `teardown_scenario()` | Re-initializes scene, clears buffers |
| `update_scenario(step, step_ind)` | Calls TSsensor(), buffers first 100 samples, logs to Rerun |
| `draw_data()` | Saves matplotlib plot to `sensor_data/module.png` |

**Data buffer:** First 100 frames (~1.67 seconds at 60 Hz)

### 6.4 ExampleScenario2 (Manual Drag)

**File:** `scenario2.py:39`

Same interface as ExampleScenario with these differences:

| Difference | Scenario 1 | Scenario 2 |
|---|---|---|
| Test objects | 4 falling cards | 4 draggable plates |
| Buffer size | 100 frames | 600 frames (10 seconds) |
| Rerun channels | 3 (proximity, normal, tangential) | 13 (+ direction, shear X/Y, 7 capacitance) |
| Plot type | Single panel (force) | Dual panel (force + capacitance) |
| Interaction | Passive (free-fall) | Active (manual drag in GUI) |
| Output data type | Python list | NumPy array (`dtype=float`) |
| Rerun app name | `tashan_standard_demo` | `tashan_manual_force_demo` |

### 6.5 Extension Lifecycle

**File:** `extension.py:46` — `Extension(omni.ext.IExt)`

| Event | Callback | Action |
|---|---|---|
| Extension loaded | `on_startup(ext_id)` | Creates window, registers menu, initializes PhysX/timeline interfaces |
| Extension unloaded | `on_shutdown()` | Removes menu items, clears window, garbage collects |
| Window opened | `_on_window(True)` | Subscribes to stage/timeline events, builds UI |
| Window closed | `_on_window(False)` | Unsubscribes events, cleans up UI |
| Timeline PLAY | `_on_timeline_event` | Subscribes to physics step events |
| Timeline STOP | `_on_timeline_event` | Unsubscribes physics step events |
| Physics step | `_on_physics_step(step)` | Forwards to UIBuilder |
| Stage opened/closed | `_on_stage_event` | Resets extension state |

### 6.6 UI Builder Controls

**File:** `ui_builder.py:30`

| Button | Label | Callback | Action |
|---|---|---|---|
| LoadButton | LOAD | `_setup_scene` + `_setup_scenario` | Creates stage, loads sensor, spawns objects |
| ResetButton | RESET | `_on_post_reset_btn` | Tears down and re-initializes scenario |
| StateButton | RUN / STOP | `_on_run_scenario_a_text` / `_on_run_scenario_b_text` | Play timeline / Pause + generate plot |

---

## 7. Visualization & Telemetry

### 7.1 Rerun Real-Time Streaming

| Parameter | Value |
|---|---|
| Library | `rerun-sdk==0.18.2` (exact version required) |
| App name (scenario 1) | `tashan_standard_demo` |
| App name (scenario 2) | `tashan_manual_force_demo` |
| Spawn mode | `rr.init(..., spawn=True)` — auto-launches Rerun Viewer |

**Scenario 1 — Rerun scalar streams (3 channels):**

| Stream path | Source channel |
|---|---|
| `sensors/force_normal` | `sensorFrameData[1]` |
| `sensors/force_tangential` | `sensorFrameData[2]` |
| `sensors/proximity` | `sensorFrameData[0]` |

**Scenario 2 — Rerun scalar streams (13 channels):**

| Stream path | Source |
|---|---|
| `sensors/proximity` | `sensorFrameData[0]` |
| `sensors/force_normal` | `sensorFrameData[1]` |
| `sensors/force_tangential` | `sensorFrameData[2]` |
| `sensors/tangential_direction_deg` | `sensorFrameData[3]` |
| `sensors/shear/x` | Derived: `tangential * cos(direction)` |
| `sensors/shear/y` | Derived: `tangential * sin(direction)` |
| `sensors/capacitance/ch_1` | `sensorFrameData[4]` |
| `sensors/capacitance/ch_2` | `sensorFrameData[5]` |
| `sensors/capacitance/ch_3` | `sensorFrameData[6]` |
| `sensors/capacitance/ch_4` | `sensorFrameData[7]` |
| `sensors/capacitance/ch_5` | `sensorFrameData[8]` |
| `sensors/capacitance/ch_6` | `sensorFrameData[9]` |
| `sensors/capacitance/ch_7` | `sensorFrameData[10]` |

### 7.2 Matplotlib Offline Plots

**Scenario 1 output:** `sensor_data/module.png`
- Single panel: Normal force, Tangential force, Proximity vs. time step index
- X-axis: "Time (ms)" — note: actually sample index, not true milliseconds
- Y-axis: "Force (N)"
- Data: First 100 buffered frames

**Scenario 2 output:** `sensor_data/module_scenario2.png`
- Upper panel: Normal force, Tangential force, Proximity vs. time step
- Lower panel: 7 capacitance channels (ch_1 through ch_7) vs. time step
- X-axis: "Time step"
- Y-axis upper: "Force / Proximity"
- Y-axis lower: "Capacitance (raw)"
- Data: First 600 buffered frames

---

## 8. Existing Simulation Scenarios

### 8.1 Scenario 1 — Free-Fall Impact Test

**Purpose:** Validate sensor response to impact events by dropping thin cards
onto the sensor surface.

**Test objects — 4 DynamicCuboid cards:**

| Property | Value |
|---|---|
| Count | 4 |
| Prim paths | `/World/Cube1` through `/World/Cube4` |
| Dimensions (scale) | [42.8 mm, 27.0 mm, 0.1 mm] |
| Mass | 20 g (0.02 kg) each |
| Color | White [1.0, 1.0, 1.0] |
| Initial positions | [0, 10mm, 105mm + i*5mm] for i=0..3 |
| Vertical spacing | 5 mm between cards |
| Fall height above sensor | ~4.5 mm (first card), ~19.5 mm (last card) |

**Camera view:**
- Eye: [-0.2, 0, 0.2] meters
- Target: [0.0, 0.0, 0.05] meters

### 8.2 Scenario 2 — Manual Drag Interaction

**Purpose:** Test shear/friction force response by manually dragging contact
plates across the sensor surface in the Isaac Sim viewport.

**Test objects — 4 DynamicCuboid plates:**

| Property | Value |
|---|---|
| Count | 4 |
| Prim paths | `/World/Plate1` through `/World/Plate4` |
| Dimensions (scale) | [18.0 mm, 18.0 mm, 3.0 mm] |
| Mass | 10 g (0.01 kg) each |
| Colors | Varying blue shades [0.9, 0.9-i*0.1, 1.0] |
| Positions | [+/-20mm, +/-10mm, 101.5mm] — arranged in 2x2 grid |

**Camera view:**
- Eye: [-0.16, 0, 0.16] meters
- Target: [0.0, 0.0, 0.055] meters

---

## 9. Integration Guide — Adding TS-F-A to Custom Robot Scenarios

To embed the TS-F-A sensor in your own robotic simulation:

### Step 1: Import the USD asset

```python
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import UsdGeom, Gf

usd_path = "<path_to>/ts.sensor.tactile/assets/TS-F-A.usd"
prim = add_reference_to_stage(usd_path=usd_path, prim_path="/World/Tip")

# Position the sensor
xform = UsdGeom.Xformable(prim)
xform.ClearXformOpOrder()
translate_op = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
translate_op.Set(Gf.Vec3d(x, y, z))  # your desired position
```

### Step 2: Configure the tactile prim and range sensor

```python
from isaacsim.core.prims import RigidPrim

# Define which objects the sensor should detect contact with
self.range = "/World/Tip/pad_4/LightBeam_Sensor"
self.tactile = RigidPrim(
    prim_paths_expr="/World/Tip/pad_[1-7]",
    name="finger_tactile",
    contact_filter_prim_paths_expr=["/World/YourObject1", "/World/YourObject2"],
    max_contact_count=7 * N,  # N = max contacts per pad you expect
)
```

### Step 3: Read sensor data each physics step

```python
from register_sensor import TSsensor

def on_physics_step(self, step):
    data = TSsensor(self.tactile, self.range)
    proximity     = data[0]
    normal_force  = data[1]   # Newtons
    tangential    = data[2]   # Newtons
    direction     = data[3]   # degrees, 0-359
    capacitance   = data[4:11]  # 7 raw values
```

### Step 4: Add to World scene

```python
from isaacsim.core.prims import SingleArticulation

articulation = SingleArticulation("/World/Tip/root_joint")
world.scene.add(articulation)
world.scene.add(self.tactile)
```

**Critical notes for custom integration:**
- The `contact_filter_prim_paths_expr` list MUST include all objects you want
  the sensor to detect. Objects not in this list will be invisible to the
  tactile sensor even if they physically collide.
- `max_contact_count` must be large enough. If contacts exceed this limit,
  some will be silently dropped.
- The native library (`register_sensor.so`) must be in `sys.path` before
  importing `TSsensor`.

---

## 10. Known Limitations & Constraints

### Hardware/Physical Layer Limitations

| Limitation | Detail | Impact |
|---|---|---|
| **Fixed taxel count** | 7 pads only — cannot add/remove pads | Spatial resolution is fixed |
| **No configurable sensitivity** | `Equation` transfer function is compiled | Cannot tune force-to-capacitance mapping |
| **No configurable noise** | `noise_scale` is hardcoded in binary | Cannot adjust noise for different sim-to-real scenarios |
| **Unknown absolute ranges** | Min/max force, capacitance range not documented | Cannot validate against physical sensor datasheet |
| **Unknown capacitance units** | Raw values without unit specification | Cannot directly compare to real hardware readings |
| **No temperature modeling** | No thermal effects on capacitance/force | Unrealistic for temperature-sensitive applications |
| **No hysteresis modeling** | No mention of loading/unloading asymmetry | May not match real elastomer behavior |
| **No creep/drift modeling** | No time-dependent deformation effects | Long-duration force applications may be unrealistic |
| **Proximity sensor on pad 4 only** | Single proximity point, not an array | Limited spatial proximity resolution |

### Simulation Layer Limitations

| Limitation | Detail | Impact |
|---|---|---|
| **Fixed 60 Hz sampling** | Physics dt locked to 1/60 s | Cannot increase temporal resolution |
| **PhysX-dependent accuracy** | Contact forces are PhysX approximations | May differ from real-world contact mechanics |
| **No deformable body support** | Sensor pads are rigid prims | No soft-body deformation of sensing surface |
| **No multi-sensor support** | Single sensor instance assumed | Difficult to simulate multi-finger hands |
| **Binary USD asset** | Sensor geometry not human-editable | Cannot modify pad layout without re-exporting USD |
| **Two Isaac Sim versions only** | Only 4.5.0 and 5.0.0 supported | Future Isaac Sim versions need new binary builds |
| **No ROS integration** | No ROS topics/services | Must add custom ROS bridge for ROS-based pipelines |
| **No RL environment wrapper** | No Gym/Gymnasium environment | Must build custom RL wrapper around the extension |
| **Rerun version pinned** | Exactly `rerun-sdk==0.18.2` required | Version conflicts possible with other tools |
| **Linux only** | Built for x86-64 Linux (ELF binaries) | No Windows or macOS support |

### Software API Limitations

| Limitation | Detail | Impact |
|---|---|---|
| **No batch sensor queries** | One `TSsensor()` call per step | Cannot query subsets of pads independently |
| **No per-pad force output** | Only aggregate normal/tangential exposed | Internal `normal_forces_7` not accessible from Python |
| **No raw contact point data** | Contact positions not exposed | Cannot determine where on a pad contact occurs |
| **No friction coefficient access** | Friction computed internally | Cannot inspect or override friction parameters |
| **Contact filter must be explicit** | Objects must be listed by prim path | Cannot use wildcard patterns for filter objects |
| **Buffer size hardcoded** | 100 or 600 frames depending on scenario | Must modify source to change buffer size |
| **No callback/event API** | Polling only (per-step query) | Cannot register threshold-based callbacks |
| **Single scenario at a time** | UIBuilder imports only `ExampleScenario` | Must modify `ui_builder.py:27` to switch scenarios |

### Data Quality Limitations

| Limitation | Detail | Impact |
|---|---|---|
| **Plot X-axis mislabeled** | Scenario 1 labels axis "Time (ms)" but plots sample index | Misleading temporal scale |
| **No signal-to-noise ratio spec** | SNR unknown | Cannot assess measurement quality |
| **No frequency response data** | Bandwidth/cutoff unknown | Cannot determine if 60 Hz captures all dynamics |
| **No calibration procedure** | No documented calibration workflow | Raw values may drift between sessions |
| **Contact filter in scenario 1** | Only filters on `/World/Cube1` (not Cube2-4) | May miss contacts from other cubes |

---

## 11. Dependencies & System Requirements

### System Requirements

| Requirement | Specification |
|---|---|
| Operating system | Ubuntu 22.04 (verified) |
| Architecture | x86-64 (ELF 64-bit shared objects) |
| GPU | NVIDIA RTX series (Isaac Sim requirement) |
| Isaac Sim | 4.5.0 or 5.0.0 |
| NVIDIA Driver | Per Isaac Sim requirements |

### Python Dependencies

| Package | Version | Purpose |
|---|---|---|
| `numpy` | (Isaac Sim bundled) | Array operations, math |
| `matplotlib` | (Isaac Sim bundled) | Offline plot generation |
| `rerun-sdk` | **0.18.2** (exact) | Real-time telemetry visualization |

### Omniverse / Isaac Sim Dependencies

| Package | Purpose |
|---|---|
| `omni.kit.uiapp` | Kit UI application framework |
| `omni.ext` | Extension base class |
| `omni.kit.commands` | Kit command system |
| `omni.kit.actions.core` | Action registry |
| `omni.kit.menu.utils` | Menu management |
| `omni.physx` | PhysX physics interface |
| `omni.timeline` | Timeline play/pause/stop control |
| `omni.ui` | UI widgets |
| `omni.usd` | USD context and stage management |
| `isaacsim.core.api` | Core simulation API (World, objects) |
| `isaacsim.core.prims` | Prim wrappers (RigidPrim, SingleArticulation, XFormPrim) |
| `isaacsim.core.utils` | Stage, viewport, prim utilities |
| `isaacsim.sensors.physx` | PhysX-based sensors (_range_sensor) |
| `isaacsim.gui.components` | UI element wrappers (ScrollingWindow, StateButton, etc.) |
| `isaacsim.examples.extension` | Example connectors (LoadButton, ResetButton) |
| `isaacsim.storage.native` | Asset path resolution |
| `pxr` (Pixar USD) | UsdGeom, UsdLux, Gf, Sdf |

---

## 12. Repository File Map

```
Tashan-Isaac-Sim/
├── .gitignore                              Git ignore rules
├── LICENSE                                 Apache License 2.0
├── README.md                               English user manual
├── README_zh.md                            Chinese (中文) user manual
├── TASHAN_TACTILE_SENSOR_FUNCTIONS.md      Function capability breakdown
├── TASHAN_TS-F-A_FULL_DOCUMENTATION.md     This document
│
└── ts.sensor.tactile/                      Isaac Sim extension package
    ├── config/
    │   └── extension.toml                  Extension metadata (v1.0.1)
    │
    ├── data/
    │   ├── icon.png                        Extension toolbar icon
    │   ├── preview.png                     Promotional preview image
    │   ├── ts-f-a_real.png                 Hardware photograph
    │   ├── ts-f-a_sim.png                  Simulation screenshot
    │   └── ts_tactile_extension.png        UI screenshot
    │
    ├── docs/
    │   ├── README.md                       Extension usage (minimal)
    │   └── CHANGELOG.md                    Version history
    │
    ├── assets/
    │   └── TS-F-A.usd                      3D sensor model (3.0 MB, USDC v0.8.0)
    │
    ├── sensor_data/
    │   └── module.png                      Generated output plot
    │
    └── ts_tactile_extension_python/        Python extension source
        ├── __init__.py                     Package init (imports Extension)
        ├── extension.py                    Extension lifecycle (161 lines)
        ├── global_variables.py             Config constants (12 lines)
        ├── ui_builder.py                   UI controls & orchestration (249 lines)
        ├── scenario.py                     Free-fall test scenario (171 lines)
        ├── scenario2.py                    Manual drag scenario (192 lines)
        ├── README.md                       Template documentation
        │
        └── ts_sensor_lib/                  Native sensor binaries
            ├── isaac-sim-4.5.0/
            │   └── register_sensor.so      Compiled sensor module (130 KB)
            └── isaac-sim-5.0.0/
                └── register_sensor.so      Compiled sensor module (125 KB)
```

---

## Appendix A: Binary Symbol Table (register_sensor.so — Isaac Sim 5.0.0)

Complete list of domain-relevant symbols extracted from the compiled native
library, providing insight into internal processing:

**Exported Python functions:**
- `TSsensor` — main sensor callback
- `Equation` — transfer function (force → capacitance mapping)
- `Generate_std` — noise standard deviation generator

**Internal data variables:**
- `capacitance`, `normal`, `normal_forces`, `normal_forces_7`
- `tangential_force`, `friction_forces`, `pair_forces`
- `friction_force_aggregate`, `friction_force_finger`
- `friction_local`, `friction_world`, `friction_points`
- `friction_count`, `friction_start_idx`
- `friction_pair_contacts_count`, `friction_pair_contacts_start_indices`
- `noise`, `noise_scale`, `scale`
- `Proximity`, `touchSensor`, `contact_view`
- `direction`, `orientations`, `quat`
- `num_filters`, `num_shapes`

**External API calls:**
- `get_contact_force_matrix` (PhysX contact data)
- `get_friction_data` (PhysX friction data)
- `get_linear_depth_data` (LightBeam range sensor)
- `get_local_poses` (prim pose retrieval)
- `acquire_lightbeam_sensor_interface` (sensor interface acquisition)

**Math/NumPy operations:**
- `atan2`, `linalg`, `norm`, `mean`, `std`, `sum`, `zeros`, `reshape`
- `random` (noise generation)

**USD/Pixar types:**
- `Gf`, `Vec3f`, `Quatf`, `Transform`, `GetInverse`

---

## Appendix B: Version History

| Version | Date | Changes |
|---|---|---|
| 0.1.0 | 2025-06-18 | Initial version of TS tactile extension |
| 1.0.1 | 2025-01-21 | Updated extension description and test settings |

---

*Document generated from deep analysis of the Tashan-Isaac-Sim repository,
including source code, binary symbol extraction, USD asset inspection, and
output data analysis.*
