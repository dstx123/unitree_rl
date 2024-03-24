/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "control/ControlFrame.h"

ControlFrame::ControlFrame(CtrlComponents *ctrlComp):_ctrlComp(ctrlComp){
    _FSMController = new FSM(_ctrlComp);
}  // 定义构造函数，将ctrlComp通过构造函数初始化给成员变量_ctrlComp

void ControlFrame::run(){
    _FSMController->run();
}