# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim.sensors.physx import _range_sensor
from isaacsim.core.prims import RigidPrim
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.objects.cuboid import FixedCuboid
from isaacsim.core.prims import SingleArticulation, XFormPrim

from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdGeom, Gf
import omni.kit.app as APP
import numpy as np
import matplotlib.pyplot as plt
import os, sys
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


"""
Scenario 3: controlled load-increment test.

- Place TS-F-A in a tight open-top box to keep plates constrained laterally.
- Add 10 plates with 1g each.
- Activate one plate at a time in a loop so load increases step-by-step.
- Log measured and expected load to Rerun for force-per-gram validation.
"""


class ExampleScenario(ScenarioTemplate):
    def __init__(self):
        self._articulation = None
        self._running_scenario = False
        self._time = 0.0

        self.sensorFrameData = []
        self.sensorBuffer = []

        self.plates = []
        self.active_plate_count = 0
        self.load_interval_steps = 60
        self.plate_mass_kg = 0.001  # 1 g per plate
        self.g = 9.81

        self._load_register_sensor()

    def setup_scenario(self, articulation, object_prim):
        self._set_up_sensor_and_scene()

        self._running_scenario = True
        set_camera_view(eye=[-0.18, 0, 0.18], target=[0.00, 0.00, 0.06])
        rr.init("tashan_incremental_load_demo", spawn=True)

    def teardown_scenario(self):
        self._set_up_sensor_and_scene()
        self._time = 0.0
        self._articulation = None
        self._running_scenario = False
        self.sensorBuffer = []
        self.active_plate_count = 0

    def update_scenario(self, step: float, step_ind: int):
        from register_sensor import TSsensor

        self._activate_plates_by_step(step_ind)

        self.sensorFrameData = TSsensor(self.tactile, self.range)
        self.sensorBuffer.append(
            [
                float(self.sensorFrameData[1]),
                float(self._expected_total_force()),
                float(self.active_plate_count),
            ]
        )

        self._time += step
        self._update_rerun_visualization()

    def draw_data(self):
        if len(self.sensorBuffer) == 0:
            return

        current_path = os.path.dirname(os.path.realpath(__file__))
        path = os.path.join(current_path, "../sensor_data/module_scenario3.png")

        data = np.array(self.sensorBuffer)
        x = np.arange(len(data))

        plt.figure(figsize=(12, 8))
        plt.plot(x, data[:, 0], label="Measured normal force (N)", linestyle="-")
        plt.plot(x, data[:, 1], label="Expected total force (N)", linestyle="--")
        plt.plot(x, data[:, 2], label="Active plate count", linestyle=":")
        plt.title("Scenario3: Incremental load validation")
        plt.xlabel("Time step")
        plt.ylabel("Value")
        plt.legend()
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

            try:
                from register_sensor import TSsensor
                print(f"TS sensor callback registered successfully via TSensor module")

            except ImportError as e:
                print(f"Failed to import TSensor module from {ts_lib_path}: {e}")

        except Exception as e:
            print(f"Failed to initialize TS sensor callback: {e}")
            return False

    def _expected_total_force(self):
        return self.active_plate_count * self.plate_mass_kg * self.g

    def _activate_plates_by_step(self, step_ind):
        if self.active_plate_count >= len(self.plates):
            return

        if step_ind > 0 and (step_ind % self.load_interval_steps == 0):
            plate = self.plates[self.active_plate_count]
            drop_height = 0.108 + 0.0015 * self.active_plate_count
            plate.set_world_pose(position=np.array([0.0, 0.0, drop_height]))
            self.active_plate_count += 1

    def _update_rerun_visualization(self):
        rr.log(f"sensors/force_normal", rr.Scalar(self.sensorFrameData[1]))
        rr.log(f"sensors/force_tangential", rr.Scalar(self.sensorFrameData[2]))
        rr.log(f"sensors/proximity", rr.Scalar(self.sensorFrameData[0]))

        if len(self.sensorFrameData) >= 11:
            for i in range(7):
                rr.log(f"sensors/capacitance/ch_{i + 1}", rr.Scalar(self.sensorFrameData[4 + i]))

        rr.log("load/active_plate_count", rr.Scalar(self.active_plate_count))
        rr.log("load/expected_total_force_n", rr.Scalar(self._expected_total_force()))

        if self.active_plate_count > 0:
            force_per_gram = float(self.sensorFrameData[1]) / (self.active_plate_count * 1.0)
            rr.log("load/measured_force_per_gram_n", rr.Scalar(force_per_gram))

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

        # Tight open-top box around sensing region to keep plates from sliding/falling away.
        wall_half_height = 0.018
        wall_thickness = 0.002
        wall_span = 0.028
        wall_z = 0.101 + wall_half_height

        FixedCuboid(
            prim_path="/World/LoadBoxFloor",
            name="load_box_floor",
            position=np.array([0.0, 0.0, 0.1005]),
            scale=np.array([0.058, 0.058, 0.001]),
            color=np.array([0.3, 0.3, 0.3]),
        )
        FixedCuboid(
            prim_path="/World/LoadBoxWallPosX",
            name="load_box_wall_pos_x",
            position=np.array([wall_span, 0.0, wall_z]),
            scale=np.array([wall_thickness, 0.058, wall_half_height * 2]),
            color=np.array([0.4, 0.4, 0.4]),
        )
        FixedCuboid(
            prim_path="/World/LoadBoxWallNegX",
            name="load_box_wall_neg_x",
            position=np.array([-wall_span, 0.0, wall_z]),
            scale=np.array([wall_thickness, 0.058, wall_half_height * 2]),
            color=np.array([0.4, 0.4, 0.4]),
        )
        FixedCuboid(
            prim_path="/World/LoadBoxWallPosY",
            name="load_box_wall_pos_y",
            position=np.array([0.0, wall_span, wall_z]),
            scale=np.array([0.058, wall_thickness, wall_half_height * 2]),
            color=np.array([0.4, 0.4, 0.4]),
        )
        FixedCuboid(
            prim_path="/World/LoadBoxWallNegY",
            name="load_box_wall_neg_y",
            position=np.array([0.0, -wall_span, wall_z]),
            scale=np.array([0.058, wall_thickness, wall_half_height * 2]),
            color=np.array([0.4, 0.4, 0.4]),
        )

        self.plates = []
        self.active_plate_count = 0

        for i in range(10):
            plate = DynamicCuboid(
                prim_path=f"/World/LoadPlate{i + 1}",
                name=f"load_plate_{i + 1}",
                # Start parked away from the center; drop in sequentially from update loop.
                position=np.array([0.08, 0.0, 0.13 + i * 0.002]),
                scale=np.array([0.016, 0.016, 0.002]),
                color=np.array([0.8, 0.8 - i * 0.05, 1.0]),
                mass=self.plate_mass_kg,
            )
            self.plates.append(plate)

        self.range = "/World/Tip/pad_4/LightBeam_Sensor"
        self.tactile = RigidPrim(
            prim_paths_expr="/World/Tip/pad_[1-7]",
            name="finger_tactile",
            contact_filter_prim_paths_expr=[f"/World/LoadPlate{i + 1}" for i in range(10)],
            max_contact_count=7 * 20,
        )
