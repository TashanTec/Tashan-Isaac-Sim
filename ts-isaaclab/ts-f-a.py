# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Launch Isaac Sim Simulator first."""

import argparse
import os, sys

from isaaclab.app import AppLauncher
import omni.kit.app as APP

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on adding sensors on a robot.")
parser.add_argument("--num_envs", type=int, default=2, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.utils import configclass


##
# Pre-defined configs
##

def load_register_sensor():
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

        ts_lib_path = os.path.join(current_dir, "ts_sensor_lib", "isaacsim-"+ version)
        if ts_lib_path not in sys.path:
            sys.path.insert(0, ts_lib_path)

    except Exception as e:
        print(f"Failed to initialize TS sensor callback: {e}")
        return False

@configclass
class SensorsSceneCfg(InteractiveSceneCfg):
    """Design the scene with sensors on the robot."""

    # ground plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # robot
    robot = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0, 0, 0.1)),
        spawn=UsdFileCfg(
            usd_path= os.path.join(os.path.dirname(__file__), "assets/TS-F-A.usd"),
            activate_contact_sensors = True,
            scale=(10.0, 10.0, 10.0))
        )

    # Rigid Object
    cube = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.CuboidCfg(
            size=(0.428, 0.27, 0.001),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0),metallic=0.2),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0, 0, 0.5)),
    )

    # sensors
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/pad_[1-7]",
        update_period=0.0,
        history_length=6,
        debug_vis=False
    )



def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene, origins: torch.Tensor):
    """Run the simulator."""
    from register_sensor import TSsensor

    # Define simulation stepping
    cube_object = scene["cube"]
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    # Simulate physics
    while simulation_app.is_running():
        # Reset
        if count % 500 == 0:
            # reset counter
            count = 0
            root_state = cube_object.data.default_root_state.clone()
            root_state[:, :3] += origins
            cube_object.write_root_pose_to_sim(root_state[:, :7])

            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting robot state...")

        # perform step
        sim.step()
        # update sim-time
        sim_time += sim_dt
        count += 1
        # update buffers
        scene.update(sim_dt)

        # print information from the sensors
        print("-------------------------------")
        print(scene["contact_forces"])
        print("Received max contact force of: ", torch.sum(scene["contact_forces"].data.net_forces_w, dim=1))

        data = TSsensor(scene["contact_forces"].data, scene["robot"].prim_paths)
        print(data)


def main():
    """Main function."""
    load_register_sensor()

    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view(eye=(3.5, 3.5, 3.5), target=(0.0, 0.0, 0.0))
    # design scene
    scene_cfg = SensorsSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    origin = torch.tensor([[1.0, 0.0, 0.5], [-1, 0.1, 0.5]], device=sim.device)
    run_simulator(sim, scene, origin)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
