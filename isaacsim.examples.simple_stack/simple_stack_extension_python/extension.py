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

import asyncio
import os

import omni.ext
import omni.ui as ui
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.examples.interactive.base_sample import BaseSampleUITemplate
from .simple_stack import SimpleStack
from isaacsim.gui.components.ui_utils import btn_builder


class SimpleStackExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self.example_name = "Simple Stack"
        self.category = "Manipulation"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Simple Stack with Tashan Sensors",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/tutorial_core_adding_manipulator.html",
            "overview": "This Example shows how to stack two cubes using Franka robot equipped with Tashan tactile sensors in Isaac Sim.\n\nThe gripper fingers are equipped with Tashan TS-F-A tactile sensors that provide real-time feedback during the grasping and stacking operations. Sensor data is visualized using Rerun.\n\nPress the 'Open in IDE' button to view the source code.",
            "sample": SimpleStack(),
        }

        ui_handle = SimpleStackUI(**ui_kwargs)

        get_browser_instance().register_example(
            name=self.example_name,
            execute_entrypoint=ui_handle.build_window,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

        return

    def on_shutdown(self):
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
        return


class SimpleStackUI(BaseSampleUITemplate):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def build_extra_frames(self):
        """Build additional UI frames for task control."""
        extra_stacks = self.get_extra_frames_handle()
        self.task_ui_elements = {}

        with extra_stacks:
            with ui.CollapsableFrame(
                title="Task Control",
                width=ui.Fraction(0.33),
                height=0,
                visible=True,
                collapsed=False,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):
                self.build_task_controls_ui()

            with ui.CollapsableFrame(
                title="Sensor Information",
                width=ui.Fraction(0.33),
                height=0,
                visible=True,
                collapsed=False,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):
                self.build_sensor_info_ui()

    def _on_stacking_button_event(self):
        """Handle stacking button click."""
        asyncio.ensure_future(self.sample._on_stacking_event_async())
        self.task_ui_elements["Start Stacking"].enabled = False
        return

    def post_reset_button_event(self):
        """Enable buttons after reset."""
        self.task_ui_elements["Start Stacking"].enabled = True
        return

    def post_load_button_event(self):
        """Enable buttons after load."""
        self.task_ui_elements["Start Stacking"].enabled = True
        return

    def post_clear_button_event(self):
        """Disable buttons after clear."""
        self.task_ui_elements["Start Stacking"].enabled = False
        return

    def build_task_controls_ui(self):
        """Build the task control UI."""
        with ui.VStack(spacing=5):
            dict = {
                "label": "Start Stacking",
                "type": "button",
                "text": "Start Stacking",
                "tooltip": "Start the cube stacking task",
                "on_clicked_fn": self._on_stacking_button_event,
            }

            self.task_ui_elements["Start Stacking"] = btn_builder(**dict)
            self.task_ui_elements["Start Stacking"].enabled = False

    def build_sensor_info_ui(self):
        """Build sensor information display."""
        with ui.VStack(spacing=5):
            ui.Label(
                "Tashan TS-F-A Tactile Sensors",
                height=20,
                word_wrap=True,
            )

            ui.Spacer(height=5)

            ui.Label(
                "Sensors attached to both gripper fingers provide:",
                height=20,
                word_wrap=True,
            )

            ui.Spacer(height=5)

            with ui.VStack(spacing=2):
                ui.Label("• Proximity sensing", height=15)
                ui.Label("• Normal force detection", height=15)
                ui.Label("• Tangential force detection", height=15)
                ui.Label("• 7-channel capacitance data", height=15)

            ui.Spacer(height=10)

            ui.Label(
                "Data is visualized in real-time using Rerun.",
                height=20,
                word_wrap=True,
            )
