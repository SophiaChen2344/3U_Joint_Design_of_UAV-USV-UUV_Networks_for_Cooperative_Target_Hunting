% 固定时间编队控制器：实现论文Eq.(14)-(16)和Eq.(34)的控制律
% 输入：
%   x_i: 第i架无人机状态
%   x_j: 邻居无人机状态（编队拓扑）
%   desired_formation: 期望编队构型
%   params: 控制器参数（固定时间收敛增益等）
%   d_hat: USDE估计的扰动
% 输出：
%   u_i: 第i架无人机的控制输入
function u_i = fixed_time_controller(x_i, x_j, desired_formation, params, d_hat)
    % 论文Eq.(12)-(13): 编队误差计算
    n = size(x_j, 2); % 邻居数量
    e_form = 0;
    for j = 1:n
        % 相对位置误差：期望构型 - 实际相对位置
        p_ij_des = desired_formation(:,j);
        p_ij_act = x_i(1:3) - x_j(1:3,j);
        e_form = e_form + (p_ij_act - p_ij_des);
    end
    
    % 论文Eq.(14)-(16): 固定时间控制律（含扰动补偿）
    alpha = params.alpha; % 固定时间收敛参数1
    beta = params.beta;   % 固定时间收敛参数2
    k = params.k;         % 控制器增益
    
    % 速度误差（对齐论文Eq.(14)）
    e_v = x_i(4:6);
    % 固定时间控制律 + 扰动补偿（-d_hat抵消外部扰动）
    u_i = -k .* (alpha .* sign(e_form) .* abs(e_form).^(params.p) + beta .* sign(e_form) .* abs(e_form).^(params.q)) ...
          - k .* (alpha .* sign(e_v) .* abs(e_v).^(params.p) + beta .* sign(e_v) .* abs(e_v).^(params.q)) ...
          - d_hat;
end