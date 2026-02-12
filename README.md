# Tashan Technology Tactile Simulation Platform User Manual

[English](README.md) | [中文](README_zh.md)


## Introduction
Welcome to the Tashan Technology Tactile Simulation Platform! Developed based on Isaac Sim, this platform provides researchers and developers with an efficient, high-precision robotic tactile simulation environment to advance research and innovation in robotic tactile perception technology. Our model is China's first tactile simulation model based on real products, playing a significant role in promoting the development of embodied intelligent robots.<br>
![real module](ts.sensor.tactile/data/ts-f-a_real.png)


## Feature Overview
- Universal Tactile Sensor TS-F-A, outputting 11 dimensional feature channels:
    - Proximity sensing [1];

    - Tactile sensing [2~4]: Normal force, Tangential force, Tangential force direction (0-359°, with fingertip direction as 0°);

    - Raw capacitance values [5~11]: 7-channel raw capacitance data.


## Workstation Setup
Before using this platform, please ensure your system meets the [Isaac Sim Requirements](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/requirements.html).
- Project based on Isaac Sim 5.0.0: [Download Isaac Sim 5.0.0](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/download.html);

- Workstation installation: [Workstation Installation](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_workstation.html);
    ```bash
    # Install visualization dependencies
    cd <isaacsim>
    ./python.sh -m pip install rerun-sdk==0.18.2
    ```

- Download asset packs: [Local Assets Packs](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_faq.html#isaac-sim-setup-assets-content-pack) for local and offline use.


## User Guide
Before using the TS TACTILE extension, we recommend familiarizing yourself with Isaac Sim documentation on [Extension Template Generator](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/utilities/extension_template_generator.html) to understand how to create and use extensions.

1. Enable TS TACTILE Extension<br>
    First clone this project to download tactile model assets and extension file:
    ```bash
    cd <your_workspace>
    git clone git@github.com:TashanTec/Tashan-Isaac-Sim.git
    ```

    - Enable Extension Terminal Command
    ```bash
    cd <isaacsim>
    ./isaac-sim.sh --ext-folder <your_workspace>/Tashan-Isaac-Sim/ --enable ts.sensor.tactile
    ```

    - Enable Extension Isaac Sim GUI<br>
    In the top menu bar, click ​Window → Extensions​ to open Extension Manager;
    Click the menu button (three horizontal lines), select ​Settings,then create a new directory in the right-hand list and add the <your_workspace>/Tashan-Isaac-Sim folder path.
    Search for ​​"TS TACTILE EXTENSION"​​ in the search bar, click the ​ENABLED​ button to activate;
    To auto-load the extension on simulator startup, toggle the ​AUTOLOAD​ option.
    ![Extension](ts.sensor.tactile/data/ts_tactile_extension.png)<br>
    Both methods have been verified on Ubuntu 22.04.


2. Test TS TACTILE Extension:
    - LOAD: Click to load the TS-F-A tactile sensor module and cards, and simultaneously launch the data visualization window;

    - RESET: Click to restore to initial state

    - RUN: Click to start real-time output of TS-F-A tactile sensor data to the terminal. The cards will free-fall onto the tactile sensor module

    - STOP: Click to pause simulation and generate a time-series plot of sensor tactile data to verify model accuracy and stability

3. Run the Simple Stack Example:
    ```bash
    cd <isaacsim>
    ./isaac-sim.sh --ext-folder <your_workspace>/Tashan-Isaac-Sim/ --enable isaacsim.examples.simple_stack
    ```

    This example demonstrates:
    - Franka robot with Tashan sensors on gripper fingers
    - Cube stacking task with tactile feedback
    - Real-time Rerun visualization of 11-channel sensor data from both fingers

    See the [Simple Stack README](isaacsim.examples.simple_stack/docs/README.md) for detailed usage instructions.


## Application Scenarios & Examples
1. Tactile Perception Research: Researchers can develop tactile perception algorithms for signal processing, feature extraction, and object recognition

2. Robotic Manipulation Simulation: Simulate dexterous hand tactile perception and manipulation processes to develop and optimize robotic operation strategies

3. Multimodal Sensory Fusion: Integrate tactile perception with vision, audio, and other sensory modalities for comprehensive environmental perception and object understanding

4. **Simple Stack with Tashan Sensors**: A practical example demonstrating cube stacking using a Franka Emika Panda robot equipped with Tashan TS-F-A tactile sensors on both gripper fingers. This example shows:
   - Integration of Tashan sensors with standard robotic manipulators
   - Real-time sensor data visualization using Rerun
   - Practical grasping and manipulation with tactile feedback
   - See [Simple Stack Documentation](isaacsim.examples.simple_stack/docs/README.md) for details


## Frequently Asked Questions
1. Q: Scene loading takes too long or fails during simulation startup.<br>
A: Primarily caused by network issues. We recommend downloading asset packs locally and configuring them as local assets, which can accelerate loading speeds by 10-100x compared to cloud loading.

2. Q: How to add tactile sensors to my own model?<br>
A: First import the tactile sensor module's USD file into your model, define the self.tactile and self.range variables in the ts.sensor.tactile/ts_tactile_extension_python/scenario.py file, then call the TSsensor(self.tactile, self.range) function.

3. Q: After enabling the extension and clicking LOAD, a "RuntimeError: Failed to find Rerun Viewer executable in PATH." error appears<br>
A: This indicates incomplete visualization dependencies. Activate your conda environment and run pip install rerun-sdk==0.18.2, then restart the application.


## Contribution & Feedback
We welcome global partners to participate in training and optimizing this platform by contributing technical expertise and innovative ideas. If you encounter issues or have suggestions, please contact us through:
GitHub Repository: Submit issues at https://github.com/TashanTec/Tashan-Isaac-Sim.git.<br>
Email: Contact us at zhangrun@tashantec.com