function u = ascfc_controller(x, A, params)
% ASCFC控制器（论文[19] 渐近收敛）
u = zeros(3, params.uav_num);
leader = params.leader_idx;
kappa = params.controller.kappa;

for i = 1:params.uav_num
    if i == leader, continue; end % leader无控制输入
    % 邻居节点
    neighbors = find(A(i,:)==1);
    if isempty(neighbors), continue; end
    % 编队误差
    e_p = x(1:2,i) - mean(x(1:2,neighbors),2);
    e_v = x(3:4,i) - mean(x(3:4,neighbors),2);
    % ASCFC控制律（比例+微分）
    u(1,i) = -kappa * e_p(1) - 0.5 * e_v(1);
    u(2,i) = -kappa * e_p(2) - 0.5 * e_v(2);
    u(3,i) = 0; % z轴固定
end
end