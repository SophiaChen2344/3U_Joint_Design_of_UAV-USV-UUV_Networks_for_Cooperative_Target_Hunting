% 固定拓扑无人机编队控制主脚本（复现论文核心实验）
% 对齐论文：Fixed-Time Networked UAV Topology Reconfiguration With Disturbance Rejection
clear; clc; close all;

%% 1. 配置参数（可从configs/json导入，此处先内置）
params = struct();
% 无人机物理参数
params.mass = 1.0; % 无人机质量(kg)
params.uav_num = 4; % 无人机数量
params.topology_type = 'ring'; % 固定拓扑：环形
% USDE扰动估计参数
params.usde_k1 = 10; 
params.usde_k2 = 5;
params.usde_gamma = 8;
% 固定时间控制器参数（对齐论文Eq.14-16）
params.alpha = 0.8; 
params.beta = 1.2;
params.k = 5.0;
params.p = 1/2; % 固定时间收敛指数
params.q = 2;   % 固定时间收敛指数
% 仿真参数
params.sim_time = 10; % 仿真时长(s)
params.dt = 0.01;     % 仿真步长(s)
% 期望编队构型（4架无人机环形编队，半径1m）
params.desired_formation = [1 0 -1 0; 
                            0 1 0 -1; 
                            0 0 0 0]; % z轴高度固定

%% 2. 初始化
% 拓扑图（拉普拉斯矩阵/邻接矩阵）
[L, A] = formation_graph(params.uav_num, params.topology_type);
% 无人机初始状态：[x,y,z,vx,vy,vz]，4架无人机
x = zeros(6, params.uav_num);
x(1,:) = [0, 1, 0, -1]; % 初始x位置
x(2,:) = [-1, 0, 1, 0]; % 初始y位置
% 扰动初始化（外部风扰动）
d = [0.5*ones(1, params.uav_num); 0.3*ones(1, params.uav_num); zeros(1, params.uav_num)];
% USDE估计器初始化
x_hat = x; % 初始状态估计=实际状态
d_hat = zeros(3, params.uav_num); % 初始扰动估计=0
% 仿真日志
sim_log = struct('time', [], 'x', [], 'e_form', [], 'd_hat', []);

%% 3. 仿真主循环
t = 0;
while t < params.sim_time
    % 记录日志
    sim_log.time = [sim_log.time, t];
    sim_log.x = [sim_log.x, x];
    sim_log.d_hat = [sim_log.d_hat, d_hat];
    
    % 计算编队误差
    e_form = zeros(3, params.uav_num);
    for i = 1:params.uav_num
        % 找到第i架无人机的邻居
        neighbors = find(A(i,:)==1);
        x_j = x(:, neighbors);
        e_form(:,i) = sum(x(1:3,i) - x_j(1:3,:) - params.desired_formation(:,neighbors), 2);
    end
    sim_log.e_form = [sim_log.e_form, e_form];
    
    % 逐无人机计算控制输入 + 更新状态
    u = zeros(3, params.uav_num);
    for i = 1:params.uav_num
        neighbors = find(A(i,:)==1);
        x_j = x(:, neighbors);
        % 固定时间控制器
        u(:,i) = fixed_time_controller(x(:,i), x_j, params.desired_formation(:,neighbors), params, d_hat(:,i));
        % USDE扰动估计更新
        [d_hat_dot(:,i), x_hat_dot(:,i)] = usde_estimator(x(:,i), u(:,i), x_hat(:,i), d_hat(:,i), params);
        d_hat(:,i) = d_hat(:,i) + d_hat_dot(:,i) * params.dt;
        x_hat(:,i) = x_hat(:,i) + x_hat_dot(:,i) * params.dt;
        % 无人机状态更新
        x_dot = uav_kinematics(x(:,i), u(:,i), d(:,i), params);
        x(:,i) = x(:,i) + x_dot * params.dt;
    end
    
    % 时间步进
    t = t + params.dt;
end

%% 4. 结果可视化
% 子图1：编队误差随时间变化
figure('Name','Formation Error');
subplot(2,1,1);
plot(sim_log.time, norm(sim_log.e_form));
xlabel('Time (s)'); ylabel('Formation Error Norm');
title('Fixed-Time Formation Error (With USDE)');
grid on;

% 子图2：扰动估计精度
subplot(2,1,2);
plot(sim_log.time, norm(d_hat - d));
xlabel('Time (s)'); ylabel('Disturbance Estimation Error Norm');
title('USDE Disturbance Estimation Error');
grid on;

% 子图3：无人机位置轨迹
figure('Name','UAV Trajectory');
for i = 1:params.uav_num
    plot(squeeze(sim_log.x(1,i,:)), squeeze(sim_log.x(2,i,:)), 'DisplayName', sprintf('UAV %d', i));
    hold on;
end
xlabel('X Position (m)'); ylabel('Y Position (m)');
title('UAV Formation Trajectory (Fixed Topology)');
legend; grid on; axis equal;

%% 5. 保存结果
output_dir = '../outputs/';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
save(fullfile(output_dir, 'fixed_topology_sim_log.mat'), 'sim_log');
savefig(fullfile(output_dir, 'formation_error.fig'));
savefig(fullfile(output_dir, 'uav_trajectory.fig'));

disp('Fixed topology simulation completed! Results saved to outputs/');