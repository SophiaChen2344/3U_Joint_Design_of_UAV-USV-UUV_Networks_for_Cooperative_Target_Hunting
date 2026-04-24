function e = calc_formation_error(x, params)
% 计算编队误差 e_ix/e_iy
e = zeros(2, params.uav_num);
leader = params.leader_idx;
for i = 1:params.uav_num
    if i == leader, continue; end
    % 期望位置
    des_x = 10*cos(2*pi*i/5);
    des_y = 10*sin(2*pi*i/5);
    % 误差计算
    e(1,i) = x(1,i) - x(1,leader) - des_x; % e_ix
    e(2,i) = x(2,i) - x(2,leader) - des_y; % e_iy
end
end