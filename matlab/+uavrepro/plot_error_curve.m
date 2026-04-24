function plot_error_curve(time, error, controller, axis_name, params)
% 绘制误差曲线（Fig.11-16）- 修复索引越界问题
figure('Color','w','Position',[300,100,800,500]);
hold on; grid on;

% UAV颜色匹配论文（仅针对UAV1-5，共5架 follower）
colors = ['b','r','y','m','g'];
labels = {'UAV1','UAV2','UAV3','UAV4','UAV5'};

% 绘制每架UAV的误差（i=2到6对应UAV1-5，匹配labels索引）
for i = 2:params.uav_num 
    % 确保error行数足够（防御性编程）
    if i <= size(error,1) && length(time) == size(error,2)
        plot(time, error(i,:), 'Color',colors(i-1),'LineWidth',1.5,'DisplayName',labels{i-1});
    end
end

% 图表配置
xlabel('t(s)','FontSize',12);
if strcmp(axis_name, 'x')
    ylabel(['e_{i',axis_name,'}(m)'],'FontSize',12);
    title(['Evolution of formation error e_{i',axis_name,'} with the ',controller,' method'],'FontSize',12);
else
    ylabel(['e_{i',axis_name,'}(m)'],'FontSize',12);
    title(['Evolution of formation error e_{i',axis_name,'} with the ',controller,' method'],'FontSize',12);
end
ylim([-20, 30]); % 匹配论文刻度
xticks(0:20:80);
legend('Location','best');
end