function plot_ddpg_reward(reward_curve, params)
% 绘制Fig.3 DDPG累积奖励曲线
figure('Color','w','Position',[200,100,800,500]);
plot(1:params.ddpg.episodes, reward_curve, 'b-','LineWidth',1.5);
grid on;
xlabel('Episodes','FontSize',12);
ylabel('Rewards (\times10^4)','FontSize',12);
title('The cumulative reward curve of the proposed algorithm (Fig.3)','FontSize',12);
% 匹配论文刻度
ylim([1e4, 3e4]);
xticks(0:50:300);
end