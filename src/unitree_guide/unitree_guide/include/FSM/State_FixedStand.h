/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FIXEDSTAND_H
#define FIXEDSTAND_H

#include "FSM/FSMState.h"

class State_FixedStand : public FSMState{
public:
    State_FixedStand(CtrlComponents *ctrlComp);
    ~State_FixedStand(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    // float _targetPos_1[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
    //                           0.0, 1.36, -2.65, 0.0, 1.36, -2.65};
    // float _targetPos_2[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
    //                          0.0, 0.67, -1.3, 0.0, 0.67, -1.3};  // 两个target pos的效果比较好，先收拢腿，再站起来。这样有利于再摔倒后复位。
    
    // float _targetPos_2[12] = {0.0, 0.3, -0.7,
    //                           0.0, 0.3, -0.7,
    //                           0.0, 0.3, -0.7,
    //                           0.0, 0.3, -0.7};  // wtw的初始站位
    float _targetPos_1[12] = {-0.1, 1.36, -2.65, 
                              0.1, 1.36, -2.65,
                              -0.1, 1.36, -2.65,
                              0.1, 1.36, -2.65};  // 髋关节打开一点
    float _targetPos_2[12] = {-0.1, 0.8, -1.5, // FR
                              0.1, 0.8, -1.5, //FL
                              -0.1, 1.0, -1.5, // RR
                              0.1, 1.0, -1.5 // RL
                              };  // 默认关节角度
    
    float _startPos[12];
    float _duration_1 = 500;   // steps
    float _duration_2 = 500; // B2
    float _percent_1 = 0;    //%
    float _percent_2 = 0;    //%
};

#endif  // FIXEDSTAND_H