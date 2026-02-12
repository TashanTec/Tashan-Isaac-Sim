# SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This example demonstrates stacking two cubes using a Franka robot equipped with
Tashan tactile sensors on the gripper fingers, with real-time Rerun visualization.
"""

import numpy as np
import os
import sys
import carb
import rerun as rr

from isaacsim.core import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.prims import RigidPrim, XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from isaacsim.manipulators import SingleManipulator
from isaacsim.manipulators.grippers import ParallelGripper
from pxr import UsdGeom, Gf
import omni.kit.app as APP


class SimpleStack:
    """
    Simple stacking task with Franka robot equipped with Tashan tactile sensors.
    """

    def __init__(self):
        self._world = None
        self._franka = None
        self._gripper = None
        self._cube1 = None
        self._cube2 = None
        self._target_position = None

        # Tashan sensor components
        self._left_finger_sensor = None
        self._right_finger_sensor = None
        self._left_range_sensor = None
        self._right_range_sensor = None
        self._sensor_data_left = []
        self._sensor_data_right = []
        self._sensor_buffer = []

        # Task state
        self._task_running = False
        self._current_step = 0
        self._stacking_phase = "idle"  # idle, approach_cube1, grasp_cube1, lift_cube1, move_to_stack, place, release

        # Load Tashan sensor library
        self._load_tashan_sensor()

    def _load_tashan_sensor(self):
        """Load the Tashan sensor native library."""
        try:
            version = APP.get_app().get_app_version()
            print(f"Isaac Sim Version: {version}")

            # Path to the Tashan sensor library (assuming it's in the ts.sensor.tactile extension)
            tashan_extension_path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                "..",
                "ts.sensor.tactile",
                "ts_tactile_extension_python"
            )

            ts_lib_path = os.path.join(tashan_extension_path, "ts_sensor_lib", f"isaac-sim-{version}")

            if ts_lib_path not in sys.path:
                sys.path.insert(0, ts_lib_path)

            from register_sensor import TSsensor
            print("TS sensor callback registered successfully")

        except Exception as e:
            print(f"Warning: Failed to load Tashan sensor library: {e}")
            print("Sensors will not be available.")

    def setup(self):
        """Set up the world, robot, objects, and sensors."""
        self._world = World.instance()

        # Add ground plane
        self._world.scene.add_default_ground_plane()

        # Load Franka robot
        self._setup_franka_robot()

        # Create cubes for stacking
        self._setup_cubes()

        # Attach Tashan sensors to gripper fingers
        self._setup_tashan_sensors()

        # Set camera view
        set_camera_view(eye=[1.5, 1.5, 1.0], target=[0.0, 0.0, 0.3])

        # Initialize Rerun visualization
        rr.init("simple_stack_with_tashan_sensors", spawn=True)

        print("SimpleStack setup complete!")

    def _setup_franka_robot(self):
        """Load and configure the Franka robot."""
        # Get path to Franka robot USD
        assets_root_path = get_assets_root_path()
        franka_usd_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"

        robot_prim_path = "/World/Franka"

        # Add robot to stage
        add_reference_to_stage(usd_path=franka_usd_path, prim_path=robot_prim_path)

        # Create manipulator (Franka with gripper)
        self._franka = self._world.scene.add(
            SingleManipulator(
                prim_path=robot_prim_path,
                name="franka",
                end_effector_prim_name="panda_hand",
                gripper=ParallelGripper(
                    end_effector_prim_path=f"{robot_prim_path}/panda_hand",
                    joint_prim_names=["panda_finger_joint1", "panda_finger_joint2"],
                    joint_opened_positions=[0.04, 0.04],
                    joint_closed_positions=[0.0, 0.0],
                    action_deltas=[0.01, 0.01]
                )
            )
        )

        # Set initial robot position
        self._franka.set_world_pose(position=np.array([0.0, 0.0, 0.0]))
        self._gripper = self._franka.gripper

        print(f"Franka robot loaded from: {franka_usd_path}")

    def _setup_cubes(self):
        """Create two cubes for the stacking task."""
        # Cube 1 - to be picked up
        self._cube1 = self._world.scene.add(
            DynamicCuboid(
                prim_path="/World/Cube1",
                name="cube1",
                position=np.array([0.5, 0.0, 0.05]),
                scale=np.array([0.05, 0.05, 0.05]),
                color=np.array([1.0, 0.0, 0.0]),  # Red
                mass=0.1,
            )
        )

        # Cube 2 - target cube (already placed)
        self._cube2 = self._world.scene.add(
            DynamicCuboid(
                prim_path="/World/Cube2",
                name="cube2",
                position=np.array([0.3, 0.3, 0.05]),
                scale=np.array([0.05, 0.05, 0.05]),
                color=np.array([0.0, 0.0, 1.0]),  # Blue
                mass=0.1,
            )
        )

        # Target position for stacking (on top of cube2)
        self._target_position = np.array([0.3, 0.3, 0.15])  # 0.1m above cube2

        print("Cubes created for stacking task")

    def _setup_tashan_sensors(self):
        """Attach Tashan tactile sensors to the Franka gripper fingers."""
        robot_prim_path = "/World/Franka"

        # Get path to Tashan sensor USD asset
        tashan_extension_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "..",
            "ts.sensor.tactile",
            "assets",
            "TS-F-A.usd"
        )

        if not os.path.exists(tashan_extension_path):
            print(f"Warning: Tashan sensor USD not found at {tashan_extension_path}")
            print("Sensors will not be attached.")
            return

        # Attach sensor to left finger
        left_finger_sensor_path = f"{robot_prim_path}/panda_hand/panda_leftfinger/TashanSensor_Left"
        try:
            prim_left = add_reference_to_stage(
                usd_path=tashan_extension_path,
                prim_path=left_finger_sensor_path
            )

            # Position sensor on left finger tip
            xform_left = UsdGeom.Xformable(prim_left)
            xform_left.ClearXformOpOrder()
            translate_op = xform_left.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
            translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.042))  # At finger tip

            # Rotate sensor to face inward
            rotate_op = xform_left.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble)
            rotate_op.Set(Gf.Vec3d(0, 0, 0))  # Adjust rotation as needed

        except Exception as e:
            print(f"Warning: Could not attach left finger sensor: {e}")

        # Attach sensor to right finger
        right_finger_sensor_path = f"{robot_prim_path}/panda_hand/panda_rightfinger/TashanSensor_Right"
        try:
            prim_right = add_reference_to_stage(
                usd_path=tashan_extension_path,
                prim_path=right_finger_sensor_path
            )

            # Position sensor on right finger tip
            xform_right = UsdGeom.Xformable(prim_right)
            xform_right.ClearXformOpOrder()
            translate_op = xform_right.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
            translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.042))  # At finger tip

            # Rotate sensor to face inward
            rotate_op = xform_right.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble)
            rotate_op.Set(Gf.Vec3d(0, 0, 180))  # Mirror rotation

        except Exception as e:
            print(f"Warning: Could not attach right finger sensor: {e}")

        # Define tactile sensor rigid prims for contact detection
        try:
            # Left finger tactile pads
            self._left_finger_sensor = RigidPrim(
                prim_paths_expr=f"{left_finger_sensor_path}/pad_[1-7]",
                name="left_finger_tactile",
                contact_filter_prim_paths_expr=["/World/Cube1", "/World/Cube2"],
                max_contact_count=7 * 10,
            )
            self._world.scene.add(self._left_finger_sensor)

            # Right finger tactile pads
            self._right_finger_sensor = RigidPrim(
                prim_paths_expr=f"{right_finger_sensor_path}/pad_[1-7]",
                name="right_finger_tactile",
                contact_filter_prim_paths_expr=["/World/Cube1", "/World/Cube2"],
                max_contact_count=7 * 10,
            )
            self._world.scene.add(self._right_finger_sensor)

            # Range sensors (proximity)
            self._left_range_sensor = f"{left_finger_sensor_path}/pad_4/LightBeam_Sensor"
            self._right_range_sensor = f"{right_finger_sensor_path}/pad_4/LightBeam_Sensor"

            print("Tashan sensors attached to gripper fingers!")

        except Exception as e:
            print(f"Warning: Could not configure tactile sensors: {e}")

    def reset(self):
        """Reset the task to initial state."""
        # Reset cubes to initial positions
        if self._cube1:
            self._cube1.set_world_pose(position=np.array([0.5, 0.0, 0.05]))
            self._cube1.set_linear_velocity(np.array([0, 0, 0]))
            self._cube1.set_angular_velocity(np.array([0, 0, 0]))

        if self._cube2:
            self._cube2.set_world_pose(position=np.array([0.3, 0.3, 0.05]))
            self._cube2.set_linear_velocity(np.array([0, 0, 0]))
            self._cube2.set_angular_velocity(np.array([0, 0, 0]))

        # Reset robot to home position
        if self._franka:
            self._franka.set_joint_positions(
                positions=np.array([0.0, -1.0, 0.0, -2.2, 0.0, 2.4, 0.8])
            )

        # Open gripper
        if self._gripper:
            self._gripper.open()

        # Reset task state
        self._current_step = 0
        self._stacking_phase = "idle"
        self._sensor_buffer = []

        print("Task reset!")

    async def _on_stacking_event_async(self):
        """Execute the stacking task asynchronously."""
        self._task_running = True
        self._stacking_phase = "approach_cube1"
        print("Starting stacking task...")

    def update(self, step: float):
        """Update task state and read sensor data."""
        if not self._task_running:
            return

        self._current_step += 1

        # Read Tashan sensor data
        self._read_sensor_data()

        # Update Rerun visualization
        self._update_rerun_visualization()

        # Execute stacking logic based on current phase
        self._execute_stacking_logic()

    def _read_sensor_data(self):
        """Read data from Tashan sensors on both fingers."""
        try:
            from register_sensor import TSsensor

            if self._left_finger_sensor and self._left_range_sensor:
                self._sensor_data_left = TSsensor(self._left_finger_sensor, self._left_range_sensor)

            if self._right_finger_sensor and self._right_range_sensor:
                self._sensor_data_right = TSsensor(self._right_finger_sensor, self._right_range_sensor)

        except Exception as e:
            # Sensor reading failed - continue without sensor data
            pass

    def _update_rerun_visualization(self):
        """Update Rerun with sensor data and task state."""
        # Log task state
        rr.log("task/phase", rr.TextLog(self._stacking_phase))
        rr.log("task/step", rr.Scalar(self._current_step))

        # Log left finger sensor data
        if len(self._sensor_data_left) >= 11:
            rr.log("sensors/left_finger/proximity", rr.Scalar(self._sensor_data_left[0]))
            rr.log("sensors/left_finger/force_normal", rr.Scalar(self._sensor_data_left[1]))
            rr.log("sensors/left_finger/force_tangential", rr.Scalar(self._sensor_data_left[2]))
            rr.log("sensors/left_finger/force_direction", rr.Scalar(self._sensor_data_left[3]))

            # Log capacitance channels
            for i in range(7):
                rr.log(f"sensors/left_finger/capacitance/ch_{i+1}",
                      rr.Scalar(self._sensor_data_left[4 + i]))

        # Log right finger sensor data
        if len(self._sensor_data_right) >= 11:
            rr.log("sensors/right_finger/proximity", rr.Scalar(self._sensor_data_right[0]))
            rr.log("sensors/right_finger/force_normal", rr.Scalar(self._sensor_data_right[1]))
            rr.log("sensors/right_finger/force_tangential", rr.Scalar(self._sensor_data_right[2]))
            rr.log("sensors/right_finger/force_direction", rr.Scalar(self._sensor_data_right[3]))

            # Log capacitance channels
            for i in range(7):
                rr.log(f"sensors/right_finger/capacitance/ch_{i+1}",
                      rr.Scalar(self._sensor_data_right[4 + i]))

        # Log gripper state
        if self._gripper:
            gripper_positions = self._gripper.get_joint_positions()
            rr.log("gripper/position", rr.Scalar(gripper_positions[0] if len(gripper_positions) > 0 else 0))

    def _execute_stacking_logic(self):
        """Simple stacking logic - this is a placeholder for actual motion planning."""
        # This is a simplified version - in a real implementation, you would use
        # motion planning and more sophisticated control

        if self._stacking_phase == "approach_cube1":
            # Move end effector above cube1
            # This is a placeholder - implement actual motion planning
            pass

        elif self._stacking_phase == "grasp_cube1":
            # Close gripper to grasp cube1
            if self._gripper:
                self._gripper.close()
            self._stacking_phase = "lift_cube1"

        elif self._stacking_phase == "lift_cube1":
            # Lift cube1
            # Placeholder for motion
            pass

        elif self._stacking_phase == "move_to_stack":
            # Move to position above cube2
            # Placeholder for motion
            pass

        elif self._stacking_phase == "place":
            # Lower cube1 onto cube2
            # Placeholder for motion
            pass

        elif self._stacking_phase == "release":
            # Open gripper to release cube1
            if self._gripper:
                self._gripper.open()
            self._task_running = False
            print("Stacking complete!")
