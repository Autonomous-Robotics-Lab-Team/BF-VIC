# Barrier Function-Based Variable Impedance Controller

## 项目说明 | Project Description

本项目基于 **Ubuntu 20.04 + ROS Noetic**，依赖官方 `franka_ros`，对其中的 `franka_example_controllers` 进行了定制修改，用于实现自定义控制逻辑。  

This project is based on **Ubuntu 20.04 + ROS Noetic**, and depends on the official `franka_ros`. 
The `franka_example_controllers` package is customized to implement user-defined control logic.

---

## 1. 系统环境 | System Environment

- Ubuntu 20.04  
- ROS Noetic  
- Properly installed and configured `franka_ros`  
- Franka Emika Panda robot supported  

---

## 2. 编译说明 | Build Instructions

本项目 **不作为独立 ROS package 编译**，而是需要直接替换 `franka_ros` 中的示例控制器目录。  

This project is **not built as an independent ROS package**.  
Instead, it replaces the example controllers inside `franka_ros`.

### 依赖仓库 | Dependencies

- [`franka_ros`](https://github.com/frankaemika/franka_ros)
- [`libfranka`](https://github.com/frankaemika/libfranka)
- [`orbbec_sdk`](https://github.com/orbbec/OrbbecSDK_ROS1.git)

请确保 `franka_ros` 能够正常编译和运行。  

Make sure that `franka_ros` can be built and runs correctly.

假设你的工作空间为 `catkin_ws`，替换其中的 `franka_example_controllers`：  

Assuming your workspace is `catkin_ws`, replace `franka_example_controllers` as follows:

```bash
cp -r franka_example_controllers ~/catkin_ws/src/franka_ros/
catkin_make
```

## 3. 运行说明 | Run Instructions

```
# 启动相机 | Start camera
roslaunch orbbec_camera gemini2.launch

# 启动控制器 | Start controller
roslaunch franka_example_controllers nocontact_impedance_controller.launch robot_ip:=xx.xx.xx.xx
```

