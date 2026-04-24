function u = fxcfc_controller(x, A, params)
% FXCFC控制器（论文提出的固定时间控制器）
u = zeros(3, params.uav_num);
leader = params.leader_idx;
alpha_ix = params.controller.alpha_ix;
beta_ix = params.controller.beta_ix;
alpha_iy = params.controller.alpha_iy;
beta_iy = params.controller.beta_iy;
k1 = params.controller.k1;

for i = 1:params.uav_num
    if i == leader, continue; end
    neighbors = find(A(i,:)==1);
    if isempty(neighbors), continue; end
    % 期望编队构型
    des_form = [10*cos(2*pi*i/5); 10*sin(2*pi*i/5)];
    % 编队误差
    e_form = x(1:2,i) - mean(x(1:2,neighbors),2) - des_form;
    e_v = x(3:4,i) - mean(x(3:4,neighbors),2);
    
    % 固定时间控制律（论文Eq.14-16）
    u(1,i) = -k1 * (alpha_ix * sign(e_form(1)).*abs(e_form(1)).^(1/2) + ...
                    beta_ix * sign(e_form(1)).*abs(e_form(1)).^2) - 0.5*e_v(1);
    u(2,i) = -k1 * (alpha_iy * sign(e_form(2)).*abs(e_form(2)).^(1/2) + ...
                    beta_iy * sign(e_form(2)).*abs(e_form(2)).^2) - 0.5*e_v(2);
    u(3,i) = 0; % z轴固定
end
end