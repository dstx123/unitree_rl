# Unitree_RL

[English document](README.md)

基于unitree_guide状态机的四足机器人强化学习部署方案。将通过Isaac gym训练的模型成功在gazebo中实现仿真，并完成真机部署。通过修改`CMakeLists.txt`即可实现仿真与部署的切换。

考虑到文件大小，仅保留了go1的urdf文件，如需使用其他机器人，请在`src/unitree_ros/robots`中自行添加。

提供了一个训练好的宇树Go1平地行走的权重文件。

如果该项目对你有帮助，请帮忙点个star，谢谢。

## 准备

下载libtorch：libtorch-cxx11-abi-shared-with-deps-1.10.0+cu113.zip

```
https://download.pytorch.org/libtorch/cu113/libtorch-cxx11-abi-shared-with-deps-1.10.0%2Bcu113.zip
```

解压后记住文件夹路径`/yourpath/libtorch`

下载该仓库：

```
git clone https://github.com/dstx123/unitree_rl.git
```

## 编译

### libtorch路径

在文件`unitree_guide/unitree_guide/CMakeLists.txt`中修改libtorch路径：

```
set(Torch_DIR /yourpath/libtorch/share/cmake/Torch)
```

### 编译项目

在src所在的目录运行

``` 
catkin_make
```

### 加载自定义模型

如果你需要加载自己训练的模型，你可能需要对代码做以下调整：

#### 权重模型

在文件`unitree_guide/unitree_guide/CMakeLists.txt`中修改模型路径：

```cmake
set(BODY_MODEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/model/body.jit")
set(ADAPT_MODEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/model/adapt.jit")
add_definitions(-DBODY_MODEL_PATH="${BODY_MODEL_PATH}")
add_definitions(-DADAPT_MODEL_PATH="${ADAPT_MODEL_PATH}")
```

这里可以自定义添加你自己的模型。

在`unitree_guide\unitree_guide\src\FSM\State_RL.cpp`中修改相应的模型加载方式：`void State_RL::_loadPolicy()`和`void State_RL::_action_compute()`

### 输入输出

请检查输入输出变量的顺序及相关参数。主要见`unitree_guide\unitree_guide\src\FSM\State_RL.cpp`和`C:\Users\user\Desktop\unitree_rl\src\unitree_guide\unitree_guide\include\FSM\State_RL.h`

## 运行

### 仿真

启动gazebo

```
source devel/setup.bash
roslaunch unitree_guide gazeboSim.launch
```

新建一个终端，运行控制器

```
source devel/setup.bash
sudo ./devel/lib/unitree_guide/junior_ctrl
```

在终端中可以通过'w' 's' 'a' 'd'键控制机器人x和y轴速度，通过'j' 'l'键控制机器人旋转。

### 实物部署

在文件`unitree_guide/unitree_guide/CMakeLists.txt`中做如下修改：

```
set(SIMULATION OFF)         # Use Gazebo or not, ON or OFF
set(REAL_ROBOT ON)          # Link real robot or not, ON or OFF
```

重新编译：

```
catkin_make
```

通过网线连接电脑和狗，请确保可以ping通，详细参考宇树仓库。

启动狗，待狗站起来后，在电脑上运行：

```
sudo ./devel/lib/unitree_guide/junior_ctrl
```

此时，狗会坐下，进入passive状态。按`L2+A`，狗站起来。然后按'start'，此时，狗进入强化学习状态机，通过左摇杆控制机器人的x和y轴速度，通过右摇杆控制机器人旋转。

## 致谢

感谢宇树官方提供的rl_ws项目。
