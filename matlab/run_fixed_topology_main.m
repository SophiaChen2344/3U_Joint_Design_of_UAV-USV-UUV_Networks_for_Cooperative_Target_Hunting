% 固定拓扑无人机编队控制主脚本（论文复现 修复版）
% 解决：formation_graph找不到 + 不生成图像 + 绘图错误
clear; clc; close all;

%% ========== 1. 强制添加路径（必加，解决函数找不到） ==========
% 修复路径：递归加载uavrepro文件夹所有函数
currentDir = pwd;
addpath(genpath(fullfile(currentDir, 'uavrepro')));

%% ========== 2. 仿真参数（和论文完全对齐） ==========
params = struct();
params.mass = 1.0;
params.uav_num = 4;           % 4架无人机
params.topology_type = 'ring';% 环形拓扑
params.dt = 0.01;             % 步长
params.sim_time = 10;         % 仿真时间
params.desired_formation = [1 0 -1 0; 0 1 0 -1; 0 0 0 0]; % 期望编队

% 控制器参数
params.alpha = 0.8;
params.beta = 1.2;
params.k = 5.0;
params.p = 1/2;
params.q = 2;

% 扰动观测器参数
params.usde_k1 = 10;
params.usde_k2 = 5;
params.usde_gamma = 8;

%% ========== 3. 初始化 ==========
[L, A] = formation_graph(params.uav_num, params.topology_type);
x = zeros(6, params.uav_num);  % 状态 [x;y;z;vx;vy;vz]
x(1,:) = [0, 1, 0, -1];
x(2,:) = [-1, 0, 1, 0];

% 外部扰动（风场）
d = repmat([0.5; 0.3; 0], 1, params.uav_num);
x_hat = x;
d_hat = zeros(3, params.uav_num);
d_hat_dot = zeros(3, params.uav_num);
x_hat_dot = zeros(6, params.uav_num);

% 日志存储（修复维度！保证绘图正常）
time_log = [];
pos_log = [];       % 位置日志
error_log = [];     % 编队误差日志
dist_err_log = [];  % 扰动误差日志

%% ========== 4. 仿真主循环 ==========
t = 0;
while t < params.sim_time
    % 记录数据
    time_log = [time_log, t];
    pos_log = cat(3, pos_log, x(1:3,:));
    current_error = 0;
    
    % 计算控制输入 + 状态更新
    u = zeros(3, params.uav_num);
    for i = 1:params.uav_num
        neighbors = find(A(i,:)==1);
        x_j = x(:, neighbors);
        des = params.desired_formation(:, neighbors);
        
        % 固定时间控制器
        u(:,i) = fixed_time_controller(x(:,i), x_j, des, params, d_hat(:,i));
        % 扰动观测器
        [d_hat_dot(:,i), x_hat_dot(:,i)] = usde_estimator(x(:,i), u(:,i), x_hat(:,i), d_hat(:,i), params);
        % 无人机动力学
        x_dot = uav_kinematics(x(:,i), u(:,i), d(:,i), params);
        
        % 欧拉积分
        d_hat(:,i) = d_hat(:,i) + d_hat_dot(:,i) * params.dt;
        x_hat(:,i) = x_hat(:,i) + x_hat_dot(:,i) * params.dt;
        x(:,i) = x(:,i) + x_dot * params.dt;
    end
    
    % 计算编队误差
    e_form = zeros(3, params.uav_num);
    for i = 1:params.uav_num
        nb = find(A(i,:)==1);
        e_form(:,i) = sum(x(1:3,i) - x(1:3,nb) - params.desired_formation(:,nb), 2);
    end
    error_log = [error_log, mean(vecnorm(e_form))];
    dist_err_log = [dist_err_log, mean(vecnorm(d_hat - d))];
    
    t = t + params.dt;
end

%% ========== 5. 修复版可视化（100%出图） ==========
% 图1：无人机编队轨迹
figure('Color','w','Position',[100,100,800,600]);
hold on; grid on; axis equal;
colors = ['r','g','b','m'];
for i = 1:params.uav_num
    plot(squeeze(pos_log(1,i,:)), squeeze(pos_log(2,i,:)), ...
         'Color',colors(i),'LineWidth',2,'DisplayName',['UAV ' num2str(i)]);
    scatter(squeeze(pos_log(1,i,1)), squeeze(pos_log(2,i,1)),50,colors(i),'filled'); % 起点
end
xlabel('X 位置 (m)','FontSize',12);
ylabel('Y 位置 (m)','FontSize',12);
title('无人机固定拓扑编队轨迹','FontSize',14);
legend;

% 图2：编队收敛误差
figure('Color','w','Position',[100,100,800,400]);
plot(time_log, error_log,'b-','LineWidth',2);
grid on;
xlabel('时间 (s)','FontSize',12);
ylabel('编队误差范数','FontSize',12);
title('固定时间编队收敛误差','FontSize',14);

% 图3：扰动估计误差
figure('Color','w','Position',[100,100,800,400]);
plot(time_log, dist_err_log,'r-','LineWidth',2);
grid on;
xlabel('时间 (s)','FontSize',12);
ylabel('扰动估计误差','FontSize',12);
title('USDE扰动观测器性能','FontSize',14);

%% ========== 6. 保存结果 ==========
outputDir = fullfile(currentDir, '..', 'outputs');
if ~exist(outputDir,'dir'), mkdir(outputDir); end
save(fullfile(outputDir,'sim_result.mat'),'time_log','pos_log','error_log','dist_err_log');

disp('✅ 仿真完成！3张图像已自动弹出，结果保存到 outputs 文件夹');