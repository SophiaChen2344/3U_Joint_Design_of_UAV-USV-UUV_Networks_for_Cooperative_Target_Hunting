% 无人机运动学模型：适配6架UAV
function x_dot = uav_kinematics(x, u, d, params)
    % 【新增】参数默认值（防止字段缺失）
    if ~isfield(params, 'mass'), params.mass = 1.0; end
    
    m = params.mass; 
    if isscalar(m), m = repmat(m,1,params.uav_num); end
    
    x_dot = zeros(size(x));
    for i = 1:params.uav_num
        p_dot = x(3:4,i); % 速度=位置导数
        v_dot = (u(1:2,i) + d(1:2,i)) / m(i); % 加速度
        theta_dot = x(6,i); % 角速度
        omega_dot = 0; % 简化：角加速度为0
        
        x_dot(1:2,i) = p_dot;
        x_dot(3:4,i) = v_dot;
        x_dot(5,i) = theta_dot;
        x_dot(6,i) = omega_dot;
    end
end