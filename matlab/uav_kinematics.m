% 无人机运动学模型：实现论文Eq.(6)-(7)的无人机动力学方程
% 输入：
%   x: 无人机状态 [位置x, 位置y, 位置z, 速度vx, 速度vy, 速度vz]
%   u: 控制输入
%   d: 外部扰动（由USDE估计）
%   params: 无人机物理参数（质量、惯量等）
% 输出：
%   x_dot: 状态导数
function x_dot = uav_kinematics(x, u, d, params)
    % 论文Eq.(6): 无人机二阶动力学模型
    % \dot{p} = v
    % m\dot{v} = u + d (简化版，对齐论文核心逻辑)
    m = params.mass; % 无人机质量
    p = x(1:3);
    v = x(4:6);
    
    p_dot = v;
    v_dot = (u + d) / m;
    
    x_dot = [p_dot; v_dot];
end