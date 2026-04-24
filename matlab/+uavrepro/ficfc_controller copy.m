function u = ficfc_controller(x, A, params)
% FICFC控制器（论文[20] 有限时间）
u = zeros(3, params.uav_num);
leader = params.leader_idx;
p_i = params.controller.p_i;
m_i = params.controller.m_i;

for i = 1:params.uav_num
    if i == leader, continue; end
    neighbors = find(A(i,:)==1);
    if isempty(neighbors), continue; end
    % 编队误差
    e_p = x(1:2,i) - mean(x(1:2,neighbors),2);
    e_v = x(3:4,i) - mean(x(3:4,neighbors),2);
    % 有限时间控制律（分数阶项）
    u(1,i) = -m_i * sign(e_p(1)) .* abs(e_p(1)).^p_i - 0.5 * e_v(1);
    u(2,i) = -m_i * sign(e_p(2)) .* abs(e_p(2)).^p_i - 0.5 * e_v(2);
    u(3,i) = 0;
end
end