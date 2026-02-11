# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim.core.prims import RigidPrim
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.prims import SingleArticulation

from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import UsdGeom, Gf
import omni.kit.app as APP
import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import rerun as rr


class ScenarioTemplate:
    def __init__(self):
        pass

    def setup_scenario(self):
        pass

    def teardown_scenario(self):
        pass

    def update_scenario(self):
        pass


class ExampleScenario2(ScenarioTemplate):
    """
    Manual interaction scenario.

    Compared with `scenario.py`, this variant places contact plates directly on top of the
    tactile surface so users can drag them over the pads in Isaac Sim and inspect shear response.

    Rerun logging includes:
      - proximity / normal / tangential / tangential direction
      - all 7 raw capacitance channels
      - tangential vector components reconstructed from magnitude + direction
    """

    def __init__(self):
        self._articulation = None
        self._running_scenario = False
        self._time = 0.0

        self.sensorFrameData = []
        self.sensorBuffer = []
        self._load_register_sensor()

    def setup_scenario(self, articulation, object_prim):
        self._set_up_sensor_and_scene()

        self._running_scenario = True
        set_camera_view(eye=[-0.16, 0, 0.16], target=[0.00, 0.00, 0.055])
        rr.init("tashan_manual_force_demo", spawn=True)

    def teardown_scenario(self):
        self._set_up_sensor_and_scene()
        self._time = 0.0
        self._articulation = None
        self._running_scenario = False
        self.sensorBuffer = []

    def update_scenario(self, step: float, step_ind: int):
        from register_sensor import TSsensor

        self.sensorFrameData = np.array(TSsensor(self.tactile, self.range), dtype=float)
        if step_ind <= 600:
            self.sensorBuffer.append(self.sensorFrameData.copy())

        self._time += step
        self._update_rerun_visualization()

    def draw_data(self):
        if len(self.sensorBuffer) == 0:
            return

        current_path = os.path.dirname(os.path.realpath(__file__))
        path = os.path.join(current_path, "../sensor_data/module_scenario2.png")

        frame_data = np.array(self.sensorBuffer)
        x = np.arange(len(frame_data))

        fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

        axes[0].plot(x, frame_data[:, 1], label="normal", linestyle="-")
        axes[0].plot(x, frame_data[:, 2], label="tangential", linestyle="--")
        axes[0].plot(x, frame_data[:, 0], label="proximity", linestyle=":")
        axes[0].set_ylabel("Force / Proximity")
        axes[0].legend(loc="upper right")

        for i in range(7):
            axes[1].plot(x, frame_data[:, 4 + i], label=f"cap_{i + 1}")
        axes[1].set_xlabel("Time step")
        axes[1].set_ylabel("Capacitance (raw)")
        axes[1].legend(loc="upper right", ncol=4)

        plt.suptitle("TS-F-A tactile + capacitance feedback (manual drag scenario)")
        plt.tight_layout()
        plt.savefig(path)
        print(f"Sensor data saved to {path}")

    def _load_register_sensor(self):
        try:
            version = APP.get_app().get_app_version()
            print("Isaac Sim Version:", version)
            current_dir = os.path.dirname(os.path.abspath(__file__))
            if version == "4.5.0":
                pass
            elif version == "5.0.0":
                pass
            else:
                print("TaShan sensor not supported on version")

            ts_lib_path = os.path.join(current_dir, "ts_sensor_lib", "isaac-sim-" + version)
            if ts_lib_path not in sys.path:
                sys.path.insert(0, ts_lib_path)

            from register_sensor import TSsensor

            print("TS sensor callback registered successfully via TSensor module")
        except Exception as e:
            print(f"Failed to initialize TS sensor callback: {e}")
            return False

    def _update_rerun_visualization(self):
        # Base channels
        rr.log("sensors/proximity", rr.Scalar(self.sensorFrameData[0]))
        rr.log("sensors/force_normal", rr.Scalar(self.sensorFrameData[1]))
        rr.log("sensors/force_tangential", rr.Scalar(self.sensorFrameData[2]))
        rr.log("sensors/tangential_direction_deg", rr.Scalar(self.sensorFrameData[3]))

        # Shear decomposition from polar representation
        angle_rad = np.deg2rad(self.sensorFrameData[3])
        shear_x = self.sensorFrameData[2] * np.cos(angle_rad)
        shear_y = self.sensorFrameData[2] * np.sin(angle_rad)
        rr.log("sensors/shear/x", rr.Scalar(shear_x))
        rr.log("sensors/shear/y", rr.Scalar(shear_y))

        # Raw capacitance channels (5-11 in TS output list)
        for i in range(7):
            rr.log(f"sensors/capacitance/ch_{i + 1}", rr.Scalar(self.sensorFrameData[4 + i]))

    def _set_up_sensor_and_scene(self):
        robot_prim_path = "/World/Tip"
        usd_path = os.path.join(os.path.dirname(__file__), "../assets/TS-F-A.usd")
        prim = add_reference_to_stage(usd_path=usd_path, prim_path=robot_prim_path)

        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        translate_op = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        translate_op.Set(Gf.Vec3d(0, 0, 0.05))

        self._articulation = SingleArticulation("/World/Tip/root_joint")
        print(f"Loading robot from {usd_path}")

        plate_positions = [
            np.array([-0.02, -0.01, 0.1015]),
            np.array([0.02, -0.01, 0.1015]),
            np.array([-0.02, 0.01, 0.1015]),
            np.array([0.02, 0.01, 0.1015]),
        ]

        for i, plate_position in enumerate(plate_positions):
            DynamicCuboid(
                prim_path=f"/World/Plate{i + 1}",
                name=f"plate{i + 1}",
                position=plate_position,
                scale=np.array([0.018, 0.018, 0.003]),
                color=np.array([0.9, 0.9 - (i * 0.1), 1.0]),
                mass=0.01,
            )

        self.range = "/World/Tip/pad_4/LightBeam_Sensor"
        self.tactile = RigidPrim(
            prim_paths_expr="/World/Tip/pad_[1-7]",
            name="finger_tactile",
            contact_filter_prim_paths_expr=["/World/Plate1", "/World/Plate2", "/World/Plate3", "/World/Plate4"],
            max_contact_count=7 * 8,
        )
