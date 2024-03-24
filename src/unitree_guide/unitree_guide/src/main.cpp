/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include <unistd.h>  // 进程控制
#include <csignal>  // 信号处理
#include <sched.h>  // 调度策略

#include "control/ControlFrame.h"  // 控制框架
#include "control/CtrlComponents.h"  // 控制组件
#include "Gait/WaveGenerator.h"  // 步态生成器
#include "control/BalanceCtrl.h"  // 平衡控制

// 条件编译
#ifdef COMPILE_WITH_REAL_ROBOT
#include "interface/IOSDK.h"
#endif // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_ROS
#include "interface/KeyBoard.h"
#include "interface/IOROS.h"
#endif // COMPILE_WITH_ROS

bool running = true;  // 全局变量

// over watch the ctrl+c command
void ShutDown(int sig)  // 用于捕获ctrl+c信号并执行命令
{
    std::cout << "stop the controller" << std::endl;
    running = false;
}

// 尝试将进程调度策略设置为实时的先入先出（FIFO）策略，以确保关键任务的响应时间和执行顺序
void setProcessScheduler()  // 实时调度设置
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc, char **argv)
{
    /* set real-time process */
    setProcessScheduler();  // 设置进程的实时调度
    /* set the print format */
    std::cout << std::fixed << std::setprecision(3);

#ifdef RUN_ROS  // 初始化ROS系统
    ros::init(argc, argv, "unitree_gazebo_servo");
#endif // RUN_ROS

    IOInterface *ioInter;
    CtrlPlatform ctrlPlat;

#ifdef COMPILE_WITH_SIMULATION  // 创建仿真平台实例
    ioInter = new IOROS();
    ctrlPlat = CtrlPlatform::GAZEBO;
#endif // COMPILE_WITH_SIMULATION

#ifdef COMPILE_WITH_REAL_ROBOT  // 创建真实机器人实例
#ifdef ROBOT_TYPE_Go2
    unitree::robot::ChannelFactory::Instance()->Init(0,argv[1]);
#endif
    ioInter = new IOSDK();

    ctrlPlat = CtrlPlatform::REALROBOT;
#endif // COMPILE_WITH_REAL_ROBOT

    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);  // 创建控制组件实例，设置其成员变量
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.002; // run at 500hz
    ctrlComp->running = &running;

#ifdef ROBOT_TYPE_A1
    ctrlComp->robotModel = new A1Robot();
#endif
#ifdef ROBOT_TYPE_Go1
    ctrlComp->robotModel = new Go1Robot();
#endif
#ifdef ROBOT_TYPE_Go2
    ctrlComp->robotModel = new Go2Robot();
#endif
    ctrlComp->waveGen = new WaveGenerator(0.35, 0.5, Vec4(0, 0.5, 0.5, 0)); // Trot
    // ctrlComp->waveGen = new WaveGenerator(1.1, 0.75, Vec4(0, 0.25, 0.5, 0.75));  //Crawl, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.6, Vec4(0, 0.5, 0.5, 0));  //Walking Trot, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.35, Vec4(0, 0.5, 0.5, 0));  //Running Trot, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.7, Vec4(0, 0, 0, 0));  //Pronk, only for sim

    ctrlComp->geneObj();  // 创建估计器和平衡控制器

    ControlFrame ctrlFrame(ctrlComp);  // 创建控制框架实例

    signal(SIGINT, ShutDown);  // 设置一个信号处理函数。当ctrl+c被捕获时，会请求中断程序，并调用shutdown函数。

    while (running)  // 主控制循环
    {
        ctrlFrame.run();
    }

    delete ctrlComp;
    return 0;
}
