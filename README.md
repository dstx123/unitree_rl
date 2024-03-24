# Unitree_RL

[中文文档](README_CN.md)

Quadruped robot reinforcement learning deployment method based on the unitree_guide state machine. The model trained through Isaac gym is successfully simulated in gazebo and deployed on the real robot. By modifying `CMakeLists.txt`, the switch between simulation and deployment can be achieved.

Considering the file size, only `go1_description` is reserved in `src/unitree_ros/robots`. If you need to use other robots, please add the urdf file yourself.

A trained weight file for Unitree Go1 walking on flat ground is provided.

If this repository is helpful to you, please help click stars, thank you.

## Prepare

Download libtorch: libtorch-cxx11-abi-shared-with-deps-1.10.0+cu113.zip

```
https://download.pytorch.org/libtorch/cu113/libtorch-cxx11-abi-shared-with-deps-1.10.0%2Bcu113.zip
```

Remember the folder path after decompression: `/yourpath/libtorch`

Download the repository:

```
git clone https://github.com/dstx123/unitree_rl.git
```

## Compile

### libtorch path

Modify the libtorch path in the file `unitree_guide/unitree_guide/CMakeLists.txt`:

```
set(Torch_DIR /yourpath/libtorch/share/cmake/Torch)
```

### Compile the project

Run in the directory where `src` is located

``` 
catkin_make
```

### Load custom model

If you need to load a model you trained yourself, you may need to make the following adjustments to the code:

#### Weight model

Modify the model path in the file `unitree_guide/unitree_guide/CMakeLists.txt`:

```cmake
set(BODY_MODEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/model/body.jit")
set(ADAPT_MODEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/model/adapt.jit")
add_definitions(-DBODY_MODEL_PATH="${BODY_MODEL_PATH}")
add_definitions(-DADAPT_MODEL_PATH="${ADAPT_MODEL_PATH}")
```

Here you can customize and add your own models.

Modify the corresponding model loading method in `unitree_guide\unitree_guide\src\FSM\State_RL.cpp`. Please pay attention to the function `void State_RL::_loadPolicy()` and `void State_RL::_action_compute()`

### Input and output variables

Please check the order of input and output variables and related parameters. Mainly seen in `unitree_guide\unitree_guide\src\FSM\State_RL.cpp` and `C:\Users\user\Desktop\unitree_rl\src\unitree_guide\unitree_guide\include\FSM\State_RL.h`

## Running

### simulation

start gazebo:

```
source devel/setup.bash
roslaunch unitree_guide gazeboSim.launch
```

Create a new terminal and run the controller:

```
source devel/setup.bash
sudo ./devel/lib/unitree_guide/junior_ctrl
```

In the terminal, you can use the 'w' 's' 'a' 'd' keys to control the robot's x and y-axis speed, and the 'j' 'l' keys to control the robot's rotation.

### Physical deployment

Make the following changes in the file  `unitree_guide/unitree_guide/CMakeLists.txt`:

```
set(SIMULATION OFF)         # Use Gazebo or not, ON or OFF
set(REAL_ROBOT ON)          # Link real robot or not, ON or OFF
```

Recompile:
```
catkin_make
```

Connect the computer and the robot through a network cable. Please make sure you can ping the robot. Please refer to Unitree repository for details.

Start the robot, and after the robot stands up, run this in the terminal:

```
sudo ./devel/lib/unitree_guide/junior_ctrl
```

此时，狗会坐下，进入passive状态。按`L2+A`，狗站起来。然后按'start'，此时，狗进入强化学习状态机，通过左摇杆控制机器人的x和y轴速度，通过右摇杆控制机器人旋转。

At this time, the robot will sit down and enter the passive state. Press `L2+A`, the robot will stands up. Then press 'start'. At this time, the robot enters the reinforcement learning state machine, controlling the robot's x- and y-axis speeds through the left joystick, and controlling the robot's rotation through the right joystick.

## Acknowledgments

Thanks to Unitree official for providing the `rl_ws` project.
