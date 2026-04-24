% USDE扰动估计器：实现论文Eq.(8)-(11)的扰动观测器
% 输入：
%   x: 无人机状态
%   u: 控制输入
%   x_hat: 状态估计值
%   d_hat: 扰动估计值
%   params: USDE参数（增益、观测器带宽等）
% 输出：
%   d_hat_dot: 扰动估计导数
%   x_hat_dot: 状态估计导数
function [d_hat_dot, x_hat_dot] = usde_estimator(x, u, x_hat, d_hat, params)
    % 论文Eq.(8)-(9): 状态观测器
    e_x = x - x_hat; % 状态误差
    k1 = params.usde_k1; % USDE增益1
    k2 = params.usde_k2; % USDE增益2
    
    % 状态估计更新律（Eq.9）
    x_hat_dot = [x_hat(4:6); (u + d_hat)/params.mass] + k1 .* e_x + k2 .* sign(e_x) .* abs(e_x).^(1/2);
    
    % 论文Eq.(10)-(11): 扰动估计更新律
    e_d = d_hat - (params.mass * (x_hat_dot(4:6) - (u/params.mass))) ;
    gamma = params.usde_gamma; % 扰动估计增益
    d_hat_dot = gamma .* sign(e_d) .* abs(e_d).^(1/2);
end
