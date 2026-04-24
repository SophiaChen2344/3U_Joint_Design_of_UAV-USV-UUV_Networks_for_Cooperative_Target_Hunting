function plot_topology(A, params)
% 绘制Fig.10 有向通信拓扑图
figure('Color','w','Position',[100,100,800,600]);
hold on; grid on;

% UAV位置（匹配论文拓扑）
uav_pos = [
    0, 5;   % UAV0 (leader)
    2, 5;   % UAV1
    4, 5;   % UAV2
    4, 3;   % UAV3
    2, 3;   % UAV4
    0, 3];  % UAV5

% 绘制UAV节点
colors = ['b','r','g','y','m','c'];
for i = 1:params.uav_num
    scatter(uav_pos(i,1), uav_pos(i,2), 200, colors(i), 'filled');
    text(uav_pos(i,1)+0.1, uav_pos(i,2)+0.1, num2str(i-1), 'FontSize',12);
end

% 绘制有向边
for i = 1:params.uav_num
    for j = 1:params.uav_num
        if A(i,j) == 1
            % 绘制箭头
            quiver(uav_pos(i,1), uav_pos(i,2), ...
                   uav_pos(j,1)-uav_pos(i,1), uav_pos(j,2)-uav_pos(i,2), ...
                   'Color','k','LineWidth',1.5,'MaxHeadSize',0.5);
        end
    end
end

% 显示邻接矩阵
A_display = A(1:5,1:5); % 匹配论文Fig.10
text(5, 5, mat2str(A_display), 'FontSize',10);

xlabel('X Position'); ylabel('Y Position');
title('Directed communication topology among UAVs (Fig.10)');
axis equal; axis([-1,6,-1,7]);
end