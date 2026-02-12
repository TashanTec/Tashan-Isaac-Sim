# Simple Stack with Tashan Sensors

This extension demonstrates a cube stacking task using the Franka Emika Panda robot equipped with Tashan TS-F-A tactile sensors on the gripper fingers.

## Features

- **Franka Robot**: Uses the Franka Emika Panda manipulator for manipulation tasks
- **Tashan Tactile Sensors**: Both gripper fingers are equipped with TS-F-A tactile sensors
- **Real-time Visualization**: Sensor data is streamed to Rerun for real-time monitoring
- **Stacking Task**: Demonstrates picking up a cube and stacking it on top of another cube

## Sensor Data

The Tashan TS-F-A sensors provide 11 channels of data per finger:
- **Channel 0**: Proximity sensing
- **Channel 1**: Normal force (N)
- **Channel 2**: Tangential force (N)
- **Channel 3**: Tangential force direction (0-359°)
- **Channels 4-10**: 7 raw capacitance channels

## Usage

1. Open Isaac Sim
2. Load the extension from the Examples browser under "Manipulation" category
3. Click "Load" to set up the scene
4. Click "Reset" to initialize the robot and cubes
5. Click "Start Stacking" to begin the stacking task
6. Monitor sensor data in the Rerun viewer window

## Requirements

- Isaac Sim 4.5.0 or 5.0.0
- Tashan Tactile Sensor extension (ts.sensor.tactile)
- Rerun SDK (install with: `pip install rerun-sdk==0.18.2`)

## Code Structure

- `simple_stack.py`: Main scenario implementation with sensor integration
- `extension.py`: Extension registration and UI handling
- `global_variables.py`: Extension metadata

## References

- [Franka Robot Documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/assets/usd_assets_robots.html)
- [Tashan Tactile Sensors](../../../TASHAN_TACTILE_SENSOR_FUNCTIONS.md)
