/**********************************************************************
 RL simulation and deploy
***********************************************************************/
#ifndef RL_H
#define RL_H

#include "FSM/FSMState.h"
#include <torch/torch.h>
#include <torch/script.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstring> // 对于memcpy
#include <vector>
#include <algorithm>

class State_RL : public FSMState
{
public:
    State_RL(CtrlComponents *ctrlComp);
    ~State_RL() {}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    float _targetPos_rl[12];  // 下发给电机的目标关节角度
    float _last_targetPos_rl[12];  // 上一次下发给电机的目标关节角度
    const float _default_dof_pos[12] = {-0.1, 0.8, -1.5, // FR
                                        0.1, 0.8, -1.5, //FL
                                        -0.1, 1.0, -1.5, // RR
                                        0.1, 1.0, -1.5 // RL
                                        };  // TODO: 这样设置和初始站立的不一样，对电机影响大吗？
    // 运动默认关节角度，宇树默认：FR、FL、RR、RL；（模型输入默认：FL、FR、RL、RR）

    const float _duration_1 = 700;
    float _percent_1 = 0;

    // 多线程
    std::thread _inferenceThread; // 推理线程
    std::mutex _mutex;  // 互斥锁
    std::atomic<bool> _threadRunning {true};  // 线程循环启动标志位
    std::atomic<bool> _inferenceReady {false};  // 推理完成标志位
    void _inferenceLoop();  // 模型推理循环函数，在一个独立的线程中执行
    long long _start_RL_Time;  // 记录线程启动时间
    const double _RL_dt = 0.02; // RL控制频率50hz

    // RL推理
    void _loadPolicy();
    void _observations_compute();
    void _obs_buffer_update();
    void _action_compute();

    torch::jit::script::Module _body_module;
    torch::jit::script::Module _adapt_module;
    torch::jit::script::Module _privileged_module;
    torch::jit::script::Module _scandots_module;

    // 可能需要调整的参数
    const float Kp = 20.0;   //宇树原版60.0;  改成模型对应
    const float Kd = 0.5;   //5.0;
    const int _num_obs = 45;  // 观测数量
    const int _num_obs_history = 10;  // 历史观测时间步
    const float clip_observations = 100.0;  // 观测限幅
    const float clip_actions = 100.0;  // 动作限幅
    const float action_scale = 0.25;  // 动作缩放
    const float hip_scale_reduction = 0.5;  // 臀关节进一步缩放

    const float scale_lin_vel = 2.0;
    const float scale_ang_vel = 0.25;
    torch::Tensor scale_commands = torch::tensor({scale_lin_vel, scale_lin_vel, scale_ang_vel});
    float scale_dof_pos = 1.0;
    float scale_dof_vel = 0.05;

    //, torch::dtype(torch::kDouble)
    torch::Tensor _obs_buffer_tensor = torch::zeros({1, _num_obs_history*_num_obs});  // 观测缓存
    float _joint_q[12];  // RL推理出的关节角度
    torch::Device device = torch::kCUDA;  // 如需调用GPU

    // 
    Estimator *_est;  // 状态估计器
    QuadrupedRobot *_robModel;  // 机器人模型
    VecInt4 *_contact;  // 接触力，先用着，大概率有问题TODO：
    Vec3 _vCmdBody;   // 身体线速度
    double _dYawCmd;  // z轴角速度
    double _dYawCmdPast; // 上一个z轴角速度
    Vec2 _vxLim, _vyLim, _wyawLim;  // 速度限幅
    RotMat _B2G_RotMat, _G2B_RotMat;  // 旋转矩阵

    torch::Tensor _action = torch::zeros({1, 12});  // 动作
    torch::Tensor _observation = torch::zeros({1, _num_obs});  // 观测
    torch::Tensor _gravity_vec; // 重力矩阵


    void _getUserCmd();  // 解析用户输入命令，更新_vCmdBody和_dYawCmd
    void _init_buffers();  // 参数初始化
    torch::Tensor QuatRotateInverse(torch::Tensor q, torch::Tensor v);
    const int dof_mapping[13] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
};

#endif // RL_H