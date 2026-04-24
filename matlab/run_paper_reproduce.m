% 论文复现主脚本：Fixed-Time Networked UAV Topology Reconfiguration
% 输出：Fig3/10/11/12/13/14/15/16 所有图表
clear; clc; close all;
addpath(genpath('./uavrepro')); % 加载所有函数
%% ========== 强制加载所有子文件夹函数（解决函数找不到） ==========
% 获取当前脚本所在目录
current_script_dir = fileparts(mfilename('fullpath'));
% 递归添加所有子文件夹到MATLAB路径
addpath(genpath(current_script_dir));
% 验证加载（可选，查看是否找到函数）
disp('已加载的函数路径：');
disp(which('ascfc_controller')); % 能显示路径则加载成功

%% ====================== 1. 论文核心参数配置 ======================
% UAV数量：0(leader)+1-5(follower) 共6架
params.uav_num = 6; 
params.leader_idx = 1; % UAV0对应索引1
params.tau_d = 1;      % dwell time (s)
params.sim_time = 80;  % 仿真时长80s（匹配论文）
params.dt = 0.01;      % 仿真步长
params.sampling_time = 0.05; % DDPG采样时间（Table III）

% 初始状态（Table II）
params.init_pos = [
    0,10;    % UAV0 (0,[0,10]^T)
    0,10;    % UAV1 ([0,10]^T)
    5,6;     % UAV2 ([5,6]^T)
    -2,3;    % UAV3 ([-2,3]^T)
    2,3;     % UAV4 ([2,3]^T)
    0,0      % UAV5 ([0,0]^T)
];
params.init_theta = [pi/16, pi/10, pi/6, pi/4, pi/15, pi/16]; % 初始角度(rad)

% 扰动参数（论文公式）
params.disturbance_fun = @(t) [
    sin(0.3*t - pi).*cos(pi/10*cos(0.5*t + pi) + pi/5); ... % d_ix
    sin(0.3*t - pi).*sin(pi/10*cos(0.5*t + pi) + pi/5)  ... % d_iy
];

% 控制器参数（论文指定）
params.controller = struct();
% 通用参数
params.controller.kappa = 0.05;   % 滤波系数
params.controller.alpha_ix = 0.1;
params.controller.alpha_iy = 0.1;
params.controller.beta_ix = 0.01;
params.controller.beta_iy = 0.01;
params.controller.p_i = 0.5;
params.controller.m_i = 11;
params.controller.n_i = 10;
params.controller.a1 = 0.2;
params.controller.b1 = 2;
params.controller.c1 = 2;
params.controller.k1 = 5;         % 角速度控制器增益

% DDPG参数（Table III）
params.ddpg = struct();
params.ddpg.lr_Q = 0.001;         % Learning rate α_Q
params.ddpg.lr_mu = 0.005;        % Learning rate α_μ
params.ddpg.tau_s = 0.001;        % Soft updating rate
params.ddpg.replay_capacity = 1e6;% Replay pool capacity
params.ddpg.batch_size = 128;     % Mini-batch size
params.ddpg.gamma = 0.99;         % Discount factor
params.ddpg.episodes = 300;       % 300 episodes
params.ddpg.time_steps = 1600;    % Time steps per episode
params.ddpg.noise_var = 0.6;      % Noise Variance

% 奖励参数
params.reward.r1 = 8;
params.reward.r2 = 8;
params.reward.sigma1 = 1;
params.reward.sigma2 = 1;
% 归一化因子（论文指定）
params.reward.rho_ix = [-1/4, 1/16, 1/7, -1/6, -1/8, 0]; % UAV0-5
params.reward.rho_iy = [1/7, 1/7, 1/13, 1/6, 1/6, 0];

%% ====================== 2. 初始化仿真 ======================
% 状态初始化 [x;y;vx;vy;theta;omega] (6架UAV)
x = zeros(6, params.uav_num);
for i = 1:params.uav_num
    x(1,i) = params.init_pos(i,1); % x位置
    x(2,i) = params.init_pos(i,2); % y位置
    x(5,i) = params.init_theta(i); % 角度
end

% 拓扑初始化（Fig.10的邻接矩阵）
A = [
    0 1 0 1 0 0;
    1 0 1 0 0 0;
    0 1 0 0 0 0;
    1 0 0 0 1 0;
    0 0 0 1 0 0;
    0 0 0 0 0 0
]; 

% 日志初始化
log = struct();
log.time = [];
log.error_ascfc_ix = []; log.error_ascfc_iy = [];
log.error_ficfc_ix = []; log.error_ficfc_iy = [];
log.error_fxcfc_ix = []; log.error_fxcfc_iy = [];
log.ddpg_reward = [];

%% ====================== 3. 模拟通信中断场景 ======================
t = 0;
while t < params.sim_time
    % 更新拓扑（通信中断逻辑）
    if t >= 20 && t < 40
        A(2,4) = 0; A(4,2) = 0; % UAV1(2) ↔ UAV3(4) 中断
    elseif t >= 60 && t < 80
        A(4,5) = 0; A(5,4) = 0; % UAV3(4) ↔ UAV4(5) 中断
    else
        % 恢复初始拓扑
        A(2,4) = 1; A(4,2) = 1;
        A(4,5) = 1; A(5,4) = 1;
    end
    
    % 计算外部扰动
    d = params.disturbance_fun(t);
    d = repmat(d, 1, params.uav_num);
    
    % 1. ASCFC控制器（渐近收敛）
    u_ascfc = ascfc_controller(x, A, params);
    x_dot_ascfc = uav_kinematics(x, u_ascfc, d, params);
    x_ascfc = x + x_dot_ascfc * params.dt;
    e_ascfc = calc_formation_error(x_ascfc, params);
    
    % 2. FICFC控制器（有限时间）
    u_ficfc = ficfc_controller(x, A, params);
    x_dot_ficfc = uav_kinematics(x, u_ficfc, d, params);
    x_ficfc = x + x_dot_ficfc * params.dt;
    e_ficfc = calc_formation_error(x_ficfc, params);
    
    % 3. FXCFC控制器（提出的固定时间）
    u_fxcfc = fxcfc_controller(x, A, params);
    x_dot_fxcfc = uav_kinematics(x, u_fxcfc, d, params);
    x_fxcfc = x + x_dot_fxcfc * params.dt;
    e_fxcfc = calc_formation_error(x_fxcfc, params);
    
    % 记录日志
    log.time = [log.time, t];
    log.error_ascfc_ix = [log.error_ascfc_ix, e_ascfc(1,:)];
    log.error_ascfc_iy = [log.error_ascfc_iy, e_ascfc(2,:)];
    log.error_ficfc_ix = [log.error_ficfc_ix, e_ficfc(1,:)];
    log.error_ficfc_iy = [log.error_ficfc_iy, e_ficfc(2,:)];
    log.error_fxcfc_ix = [log.error_fxcfc_ix, e_fxcfc(1,:)];
    log.error_fxcfc_iy = [log.error_fxcfc_iy, e_fxcfc(2,:)];
    
    % 状态更新（以FXCFC为例）
    x = x_fxcfc;
    t = t + params.dt;
end

%% ====================== 4. 运行DDPG训练（生成Fig.3） ======================
log.ddpg_reward = run_ddpg_training(params);

%% ====================== 5. 绘制论文所有图表 ======================
% Fig.10: 有向通信拓扑图
plot_topology(A, params);

% Fig.3: DDPG累积奖励曲线
plot_ddpg_reward(log.ddpg_reward, params);

% Fig.11: ASCFC e_ix误差
plot_error_curve(log.time, log.error_ascfc_ix, 'ASCFC', 'x', params);

% Fig.12: ASCFC e_iy误差
plot_error_curve(log.time, log.error_ascfc_iy, 'ASCFC', 'y', params);

% Fig.13: FICFC e_ix误差
plot_error_curve(log.time, log.error_ficfc_ix, 'FICFC', 'x', params);

% Fig.14: FICFC e_iy误差
plot_error_curve(log.time, log.error_ficfc_iy, 'FICFC', 'y', params);

% Fig.15: FXCFC e_ix误差
plot_error_curve(log.time, log.error_fxcfc_ix, 'FXCFC', 'x', params);

% Fig.16: FXCFC e_iy误差
plot_error_curve(log.time, log.error_fxcfc_iy, 'FXCFC', 'y', params);

%% ====================== 6. 保存结果 ======================
output_dir = '../outputs/paper_results/';
if ~exist(output_dir, 'dir'), mkdir(output_dir); end
save(fullfile(output_dir, 'paper_results.mat'), 'log', 'params');
savefig(fullfile(output_dir, 'all_figures.fig'));

disp('✅ 论文所有图表生成完成！结果保存在 outputs/paper_results/');