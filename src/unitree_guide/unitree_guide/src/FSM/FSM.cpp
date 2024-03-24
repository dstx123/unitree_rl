/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSM.h"
#include <iostream>  // 用于状态转换时输出日志信息

FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){
    
    // 初始化各种状态，并赋值给状态列表变量中对应的成员
    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.fixedDown = new State_FixedDown(_ctrlComp);
    _stateList.rl = new State_RL(_ctrlComp);
    initialize();  // 调用该函数设置初始状态和模式
}

FSM::~FSM(){  // 析构函数
    _stateList.deletePtr();
}

void FSM::initialize(){
    _currentState = _stateList.passive;  // FSM初始状态设置为passive
    _currentState -> enter();  // 调用passive的enter
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;  // FSM模式设置为NORMAL
}

void FSM::run(){
    _startTime = getSystemTime();  // 记录开始执行的系统时间
    _ctrlComp->sendRecv();  // 发送控制命令并接收机器人当前的状态
    _ctrlComp->runWaveGen();  // 步态波形生成
    _ctrlComp->estimator->run();  // 运行状态估计器
    if(!checkSafty()){  // 安全性检查，倾角过大则进入passive状态
        _ctrlComp->ioInter->setPassive();
    }

    if(_mode == FSMMode::NORMAL){  // 正常状态下
        _currentState->run();  // 执行当前状态行为
        _nextStateName = _currentState->checkChange();  // 检查是否需要切换到另一个状态
        if(_nextStateName != _currentState->_stateName){  // 如果状态不一样
            _mode = FSMMode::CHANGE;  // 更新为change模式
            _nextState = getNextState(_nextStateName); // 获取下一个状态的实例
            std::cout << "Switched from " << _currentState->_stateNameString
                      << " to " << _nextState->_stateNameString << std::endl;  // 打印状态切换
        }
    }
    else if(_mode == FSMMode::CHANGE){  // 如果是change模式
        _currentState->exit();  // 执行当前状态的退出
        _currentState = _nextState;  // 切换到下一个状态
        _currentState->enter();  // 进入下一个状态
        _mode = FSMMode::NORMAL; // 切换回normal
        _currentState->run();  // 执行行为
    }

    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));  // 绝对等待，保证控制周期
}

FSMState* FSM::getNextState(FSMStateName stateName){  // 获取下一个状态函数
    switch (stateName)
    {
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDSTAND:
        return _stateList.fixedStand;
        break;
    case FSMStateName::FIXEDDOWN:
        return _stateList.fixedDown;
        break;
    case FSMStateName::RL:
        return _stateList.rl;
    default:
        return _stateList.invalid;
        break;
    }
}

bool FSM::checkSafty(){  // 安全检查函数，检查机器人与地面的角度是否在安全范围内，如果不安全则返回false
    // The angle with z axis less than 60 degree
    if(_ctrlComp->lowState->getRotMat()(2,2) < 0.5 ){  // 获取旋转矩阵第三行第三列的值，即z轴旋转角的余弦值，大于0.5即超过60度
        return false;
    }else{
        return true;
    }
}