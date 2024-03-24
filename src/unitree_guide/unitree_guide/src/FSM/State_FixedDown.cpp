/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include "FSM/State_FixedDown.h"

State_FixedDown::State_FixedDown(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::FIXEDDOWN, "fixed down")
{

}

void State_FixedDown::enter()
{
    for (int i = 0; i < 4; i++)
    {
        if (_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO)  // 若是gazebo平台
        {
            _lowCmd->setSimStanceGain(i);  // 设置电机参数
        }
        else if (_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT)  // 若是真狗
        {
            _lowCmd->setRealStanceGain(i);  // 设置电机参数
        }
        _lowCmd->setZeroDq(i);  // 设置关节速度为0
        _lowCmd->setZeroTau(i);  // 设置电机力矩为0
    }

    for (int i = 0; i < 12; i++)
    {
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;  // 设置当前关节角度
        _startPos[i] = _lowState->motorState[i].q;  // 设置进入状态机时的初始关节角度
    }
    _ctrlComp->setAllStance(); // 设置步态相位生成状态为STANCE_ALL
}

void State_FixedDown::run()
{

    _percent_1 += (float)1 / _duration_1;
    _percent_1 = _percent_1 > 1 ? 1 : _percent_1;
    if (_percent_1 < 1)
    {
        for (int j = 0; j < 12; j++)
        {
            _lowCmd->motorCmd[j].q = (1 - _percent_1) * _startPos[j] + _percent_1 * _targetPos_3[j];
            _lowCmd->motorCmd[j].dq = 0;
            _lowCmd->motorCmd[j].Kp = 60;
            _lowCmd->motorCmd[j].Kd = 5;
            _lowCmd->motorCmd[j].tau = 0;
        }
    
    }
}

void State_FixedDown::exit()
{
    _percent_1 = 0;    
}

FSMStateName State_FixedDown::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    
    if(_percent_1>=1)
        return FSMStateName::PASSIVE;
    else
        return FSMStateName::FIXEDDOWN;
}