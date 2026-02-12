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
This scenario tests the universal tactile sensor of Tashan, recording the dynamic changes in proximity,
normal force, and tangential force output of the sensor during the free fall of a small card to the sensor;

The particular framework under which this scenario operates should not be taken as a direct
recomendation to the user about how to structure their code.  In the simple example put together
in this template, this particular structure served to improve code readability and separate
the logic that runs the example from the UI design.
"""


class ExampleScenario(ScenarioTemplate):
    def __init__(self):
        self._articulation = None
        self._running_scenario = False
        self._time = 0.0

        # add buffer
        self.sensorFrameData = []
        self.sensorBuffer = []
        # add tashan sensor lib
        self._load_register_sensor()

    def setup_scenario(self, articulation, object_prim):
        self._set_up_sensor_and_scene()

        self._running_scenario = True
        set_camera_view(eye=[-0.2, 0, 0.2], target=[0.00, 0.00, 0.05])     # set camera view
        rr.init("tashan_standard_demo", spawn=True)

    def teardown_scenario(self):
        self._set_up_sensor_and_scene()
        self._time = 0.0
        self._articulation = None
        self._running_scenario = False
        self.sensorBuffer = []

    def update_scenario(self, step: float, step_ind: int):
        from register_sensor import TSsensor
        self.sensorFrameData = TSsensor(self.tactile, self.range)
        if step_ind <= 100:
            self.sensorBuffer.append(self.sensorFrameData)
        self._time += step
        # print(("current time step: ", step_ind, self.sensorFrameData))
        self._update_rerun_visualization()

    def draw_data(self):
        current_path = os.path.dirname(os.path.realpath(__file__))
        path = os.path.join(current_path, "../sensor_data/module.png")

        plt.figure(figsize=(12,8))
        Fc_data = np.array(self.sensorBuffer)
        x = [i for i in range(len(Fc_data))]

        plt.plot(x, Fc_data[:,1], label="normal",     linestyle="-")
        plt.plot(x, Fc_data[:,2], label="tangential", linestyle="--")
        plt.plot(x, Fc_data[:,0], label="Proximity",  linestyle="-")

        # 添加标题和标签
        plt.title('Module Tactile Feedback')
        plt.xlabel('Time (ms)')
        plt.ylabel('Force (N)')
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

            ts_lib_path = os.path.join(current_dir, "ts_sensor_lib", "isaac-sim-"+ version)
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

    def _update_rerun_visualization(self):
        rr.log(f"sensors/force_normal", rr.Scalar(self.sensorFrameData[1]))
        rr.log(f"sensors/force_tangential", rr.Scalar(self.sensorFrameData[2]))
        rr.log(f"sensors/proximity", rr.Scalar(self.sensorFrameData[0]))

        # Additional capacitive channels from TS output (indices 4..10)
        if len(self.sensorFrameData) >= 11:
            for i in range(7):
                rr.log(f"sensors/capacitance/ch_{i + 1}", rr.Scalar(self.sensorFrameData[4 + i]))

    def _set_up_sensor_and_scene(self):
        robot_prim_path = "/World/Tip"
        ## update here, this could be from ISAAC_ASSETS_PATH later
        ## path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"
        usd_path = os.path.join(os.path.dirname(__file__), "../assets/TS-F-A.usd")
        prim = add_reference_to_stage(usd_path=usd_path, prim_path=robot_prim_path)
        # Add transformation operation
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()   # Clear existing transformation operations
        translate_op = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        translate_op.Set(Gf.Vec3d(0, 0, 0.05))

        self._articulation = SingleArticulation("/World/Tip/root_joint")
        print(f"Loading robot from {usd_path}")

        for i in range(4):
            DynamicCuboid(
                prim_path=f"/World/Cube{i+1}",
                name=f"card{i}",
                position=np.array([0, 0.01, 0.105 + i * 0.005]),
                scale=np.array([0.0428, 0.027, 0.0001]),
                color=np.array([1.0, 1.0, 1.0]),
                mass=0.02,
            )

        self.range = "/World/Tip/pad_4/LightBeam_Sensor"
        self.tactile = RigidPrim(
            prim_paths_expr="/World/Tip/pad_[1-7]",
            name="finger_tactile",
            contact_filter_prim_paths_expr=["/World/Cube1"],
            max_contact_count= 7*5,
        )
