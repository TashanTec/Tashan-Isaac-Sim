# 他山科技触觉模拟仿真平台使用手册

[中文](README.md)

## 简介
欢迎使用他山科技触觉模拟仿真平台！本平台基于 Isaac Sim 开发，旨在为研究人员和开发者提供一个高效、精准的机器人触觉模拟环境，助力机器人触觉感知技术的研究与创新。我们的模型是国内首个基于真实产品的触觉模拟仿真模型，对推动具身智能机器人的发展具有重要意义。<br>


## 功能概述
- 通用触觉传感器 TS-F-A，输出11维特征通道：
    - 接近觉[1]；

    - 触觉[2~4]: 法向力、切向力、切向力方向（以指尖方向为0, 顺时针 0-359度）；

    - 原始电容值[5~11]: 7个压力通道原始电容值。


## 环境安装
在开始使用本平台之前，请检查您的系统是否满足 [Isaac Sim Requirements](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/requirements.html)。
- 项目基于isaac sim 4.5.0版本，下载 [Isaac Sim 4.5.0](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_workstation.html)；
- isaaclab 安装 [Isaac Lab Installation](https://docs.robotsfan.com/isaaclab/source/setup/installation/binaries_installation.html)


## 使用说明

```bash
# 克隆本项目
cd <your_workspace>
git clone git@github.com:TashanTec/Tashan-Isaac-Sim.git
git checkout tashan-isaac-lab

# 激活Isaaclab python 环境
conda activate env_isaaclab
python ts-isaaclab/ts-f-a.py
```
