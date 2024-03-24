/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include "FSM/State_RL.h"

#include <fstream>
void saveArrayToFile(const float actions[], size_t size, const std::string& filename) {
    std::ofstream file(filename, std::ios::app);
    if (file.is_open()) {
        for (size_t i = 0; i < size; ++i) {
            file << actions[i];
            if (i < size - 1) file << " ";
        }
        file << "\n";
        file.close();
    } else {
        std::cerr << "Failed to open file: " << filename << std::endl;
    }
}

State_RL::State_RL(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::RL, "rl"), 
    _est(ctrlComp->estimator), _contact(ctrlComp->contact), 
    _robModel(ctrlComp->robotModel){
        this->_vxLim = _robModel->getRobVelLimitX();  // 速度限幅
        this->_vyLim = _robModel->getRobVelLimitY();
        this->_wyawLim = _robModel->getRobVelLimitYaw();
}

void State_RL::_init_buffers()
{
    this->_dYawCmdPast=0.0;

    this->_action = torch::zeros({1, 12});
    this->_observation = torch::zeros({1, this->_num_obs});
    this->_gravity_vec = torch::tensor({{0.0, 0.0, -1.0}});
    this->_obs_buffer_tensor = torch::zeros({1, this->_num_obs_history*this->_num_obs});
}

void State_RL::_loadPolicy()  // 加载JIT模型
{
    this->_body_module = torch::jit::load(BODY_MODEL_PATH);
    this->_adapt_module = torch::jit::load(ADAPT_MODEL_PATH);
}

torch::Tensor State_RL::QuatRotateInverse(torch::Tensor q, torch::Tensor v)
{
    c10::IntArrayRef shape = q.sizes();
    torch::Tensor q_w = q.index({torch::indexing::Slice(), -1});
    torch::Tensor q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(0, 3)});
    torch::Tensor a = v * (2.0 * torch::pow(q_w, 2) - 1.0).unsqueeze(-1);
    torch::Tensor b = torch::cross(q_vec, v, /*dim=*/-1) * q_w.unsqueeze(-1) * 2.0;
    torch::Tensor c = q_vec * torch::bmm(q_vec.view({shape[0], 1, 3}), v.view({shape[0], 3, 1})).squeeze(-1) * 2.0;
    return a - b + c;
}

void State_RL::_observations_compute()
{
    // 获取当前的观测信息

    torch::Tensor base_quat = torch::tensor({{_lowState->imu.quaternion[1], _lowState->imu.quaternion[2], _lowState->imu.quaternion[3], _lowState->imu.quaternion[0]}});

    // 重力观测
    torch::Tensor projected_gravity = QuatRotateInverse(base_quat, this->_gravity_vec);

    // 命令观测
    _userValue = _lowState->userValue;  // 获取用户输入
    _getUserCmd();  // 解析用户输入命令，更新速度命令
    torch::Tensor commands = torch::cat({torch::tensor({_vCmdBody(0)}), torch::tensor({_vCmdBody(1)}), torch::tensor({_dYawCmd})}, -1);

    // 相对关节角度
    torch::Tensor dof_pos_tensor = torch::tensor({_lowState->motorState[3].q-this->_default_dof_pos[3], _lowState->motorState[4].q-this->_default_dof_pos[4], _lowState->motorState[5].q-this->_default_dof_pos[5],
                                                  _lowState->motorState[0].q-this->_default_dof_pos[0], _lowState->motorState[1].q-this->_default_dof_pos[1], _lowState->motorState[2].q-this->_default_dof_pos[2],
                                                  _lowState->motorState[9].q-this->_default_dof_pos[9], _lowState->motorState[10].q-this->_default_dof_pos[10], _lowState->motorState[11].q-this->_default_dof_pos[11],
                                                  _lowState->motorState[6].q-this->_default_dof_pos[6], _lowState->motorState[7].q-this->_default_dof_pos[7], _lowState->motorState[8].q-this->_default_dof_pos[8]});
    float dof_pos_temp[12] = {_lowState->motorState[3].q, _lowState->motorState[4].q, _lowState->motorState[5].q,
                              _lowState->motorState[0].q, _lowState->motorState[1].q, _lowState->motorState[2].q,
                              _lowState->motorState[9].q, _lowState->motorState[10].q, _lowState->motorState[11].q,
                              _lowState->motorState[6].q, _lowState->motorState[7].q, _lowState->motorState[8].q};
    //saveArrayToFile(dof_pos_temp, 12, "obs_values_cpp.txt");

    // 关节角速度
    torch::Tensor dof_vel_tensor = torch::tensor({_lowState->motorState[3].dq, _lowState->motorState[4].dq, _lowState->motorState[5].dq,
                                                  _lowState->motorState[0].dq, _lowState->motorState[1].dq, _lowState->motorState[2].dq,
                                                  _lowState->motorState[9].dq, _lowState->motorState[10].dq, _lowState->motorState[11].dq,
                                                  _lowState->motorState[6].dq, _lowState->motorState[7].dq, _lowState->motorState[8].dq});

    // 角速度
    torch::Tensor body_ang_vel = torch::tensor({{_lowState->imu.gyroscope[0], _lowState->imu.gyroscope[1], _lowState->imu.gyroscope[2]}});
    //torch::Tensor body_ang_vel = QuatRotateInverse(base_quat, ang_vel);

    // // 是否触地，这个好像不太对
    // torch::Tensor contact_states = torch::tensor({(*_contact)(1),(*_contact)(0),(*_contact)(3),(*_contact)(2)});

    this->_observation = torch::cat({body_ang_vel.view({1, -1}) * scale_ang_vel,
                                projected_gravity.view({1, -1}),
                                commands.view({1, -1}) * scale_commands.view({1, -1}),
                                dof_pos_tensor.view({1, -1}) * scale_dof_pos,
                                dof_vel_tensor.view({1, -1}) * scale_dof_vel,
                                _action.view({1, -1})}, -1);
    //std::cout<<"obs shape: "<<_observation.sizes()<<std::endl;
    this->_observation = torch::clamp(this->_observation, -clip_observations, clip_observations);
}

void State_RL::_getUserCmd(){
    /* Movement */
    _vCmdBody(0) =  invNormalize(_userValue.ly, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = -invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9*_dYawCmdPast + (1-0.9) * _dYawCmd;
    _dYawCmdPast = _dYawCmd;
}

void State_RL::_obs_buffer_update()
{
    // 更新输入的历史观测buffer
    // 每次更新时，移除最旧的61个元素，并追加新的观测数据
    _observations_compute(); // 获取新观测
    this->_obs_buffer_tensor = torch::cat({this->_obs_buffer_tensor.narrow(1, this->_num_obs, (this->_num_obs_history*this->_num_obs-this->_num_obs)), this->_observation.view({1, this->_num_obs})}, -1);
}

void State_RL::_action_compute()
{
    // 以50hz调用rl_policy来生成关节角度
    torch::Tensor latent = _adapt_module.forward({this->_obs_buffer_tensor}).toTensor();
    this->_action = _body_module.forward({torch::cat({_observation, latent}, -1)}).toTensor();
    this->_action = torch::clamp(this->_action, -clip_actions, clip_actions);

    torch::Tensor actions_scaled = this->_action * this->action_scale;
    int indices[] = {0, 3, 6, 9};
    for (int i : indices)
        actions_scaled[0][i] *= this->hip_scale_reduction; // 模型顺序

    for(int i=0; i<12; i++)
    {
        this->_joint_q[i] = actions_scaled[0][this->dof_mapping[i]].item<double>();
        this->_joint_q[i] += this->_default_dof_pos[i];
    }

    //saveArrayToFile(this->_joint_q, 12, "action_values_cpp.txt");
}

void State_RL::enter()
{
    for (int i = 0; i < 12; i++)
    {
        _lowCmd->motorCmd[i].mode = 10;
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;  // 设置当前关节角度
        _lowCmd->motorCmd[i].dq = 0;
        _lowCmd->motorCmd[i].Kp = this->Kp;
        _lowCmd->motorCmd[i].Kd = this->Kd;
        _lowCmd->motorCmd[i].tau = 0;

        // 设置进入状态机时的初始关节角度为目标角度
        this->_targetPos_rl[i] = this->_default_dof_pos[i];  // 初始pd调整至默认关节位置
        this->_last_targetPos_rl[i] = _lowState->motorState[i].q;
        this->_joint_q[i] = this->_default_dof_pos[i];
    }

    _init_buffers(); // 初始化参数
    _loadPolicy();  // 刚进入RL状态时先加载模型

    // 初始化_obs_buffer_tensor，并获取满足历史时间步长度的观测buffer
    this->_obs_buffer_tensor = torch::zeros({1, this->_num_obs_history*_num_obs});
    _action = torch::zeros({1, 12});
    for (int i = 0; i < this->_num_obs_history; ++i) {
        _obs_buffer_update(); // 更新初始的观测buffer
    }

    // 启动线程
    _threadRunning = true;  // 线程循环启动
    _inferenceReady = false; // 推理完成标志位
    if (!_inferenceThread.joinable()) {  // joinable检查线程是否在执行中，执行中会返回true。
        _inferenceThread = std::thread(&State_RL::_inferenceLoop, this);  // 创建线程
    }
}

void State_RL::_inferenceLoop() {  // 模型推理线程
    // std::lock_guard是C++11引入的一个RAII风格的互斥锁包装器
    // 当std::lock_guard对象被创建时，会自动尝试锁定给定的互斥锁_mutex，并在std::lock_guard对象的生命周期结束时自动解锁该互斥锁
    while (_threadRunning) // 进入线程循环
    {
        _start_RL_Time = getSystemTime();  // 记录开始执行的系统时间
        _obs_buffer_update(); // 更新一次观测buffer

        // long long obs_Time = getSystemTime();
        // std::cout << "obs compute time: " << obs_Time-_start_RL_Time << std::endl;
        
        {  // 作用域界定，用于控制下面第一行的生命周期，锁会在之后由析构函数释放
            std::lock_guard<std::mutex> lock(_mutex);
            _action_compute(); // 执行模型推理
        }

        // long long action_Time = getSystemTime();
        // std::cout << "action compute time: " << action_Time-_start_RL_Time << std::endl;

        _inferenceReady = true;
        this->_percent_1 = 0;
        absoluteWait(_start_RL_Time, (long long)(this->_RL_dt * 1000000));  // 绝对等待，保证控制周期，即线程频率50hz
    }
}

void State_RL::run()
{
    // 以500hz实时发送关节角度
    // 互斥锁防止数据访问冲突
    if (_inferenceReady) {
        std::lock_guard<std::mutex> lock(_mutex);
        // 使用推理结果
        memcpy(this->_targetPos_rl, this->_joint_q, sizeof(this->_joint_q));
        _inferenceReady = false;
    }

    this->_percent_1 += (float)1 / this->_duration_1;
    this->_percent_1 = this->_percent_1 > 1 ? 1 : this->_percent_1;
    for (int j = 0; j < 12; j++)  // 跟上一次的位置平滑滤波
    {
        _lowCmd->motorCmd[j].mode = 10;
        _lowCmd->motorCmd[j].q = (1 - this->_percent_1) * this->_last_targetPos_rl[j] + this->_percent_1 * this->_targetPos_rl[j];
        _lowCmd->motorCmd[j].dq = 0;
        _lowCmd->motorCmd[j].Kp = this->Kp;  // TODO: 记得改一下值
        _lowCmd->motorCmd[j].Kd = this->Kd;
        _lowCmd->motorCmd[j].tau = 0;

        this->_last_targetPos_rl[j] = _targetPos_rl[j];
    }
}

void State_RL::exit()
{
    this->_percent_1 = 0;

    _threadRunning = false;  // 判定线程循环跳出
    if (_inferenceThread.joinable()) // 如果线程在执行中
        _inferenceThread.join();  // 等待线程执行完，然后将线程变为非可加入状态，即之后joinable会返回false
}

FSMStateName State_RL::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)  // 接收到L2_B则切换到passive状态
    {
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){  // 接收到L2_A则进入fixstand状态
        return FSMStateName::FIXEDSTAND;
    }
    else{  // 否则保持RL
        return FSMStateName::RL;
    }
}