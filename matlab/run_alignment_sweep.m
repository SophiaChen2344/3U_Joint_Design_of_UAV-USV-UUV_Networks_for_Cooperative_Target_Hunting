clear;
clc;

base_cfg = uavrepro.default_config();

vmax_candidates = base_cfg.alignment.vmax_candidates;
wmax_candidates = base_cfg.alignment.wmax_candidates;
tdot_candidates = base_cfg.alignment.theta_id_dot_limit_candidates;
heading_candidates = base_cfg.alignment.heading_offset_candidates;

results = [];
row = 0;

for i = 1:numel(vmax_candidates)
    for j = 1:numel(wmax_candidates)
        for k = 1:numel(tdot_candidates)
            for m = 1:numel(heading_candidates)
                cfg = base_cfg;
                cfg.controller.vmax = vmax_candidates(i);
                cfg.controller.wmax = wmax_candidates(j);
                cfg.controller.wmin = -wmax_candidates(j);
                cfg.controller.theta_id_dot_limit = tdot_candidates(k);
                cfg.initial.heading_global_offset = heading_candidates(m);

                sim = uavrepro.run_fixed_topology_sim(cfg);
                metrics = uavrepro.compute_alignment_metrics(sim, cfg);

                row = row + 1;
                results(row).vmax = cfg.controller.vmax; %#ok<SAGROW>
                results(row).wmax = cfg.controller.wmax; %#ok<SAGROW>
                results(row).theta_id_dot_limit = cfg.controller.theta_id_dot_limit; %#ok<SAGROW>
                results(row).heading_global_offset = cfg.initial.heading_global_offset; %#ok<SAGROW>
                results(row).settle_time = metrics.settle_time; %#ok<SAGROW>
                results(row).soft_settle_time = metrics.soft_settle_time; %#ok<SAGROW>
                results(row).peak_abs_error = metrics.peak_abs_error; %#ok<SAGROW>
                results(row).final_mean_abs_error = metrics.final_mean_abs_error; %#ok<SAGROW>
                results(row).oscillation_penalty = metrics.oscillation_penalty; %#ok<SAGROW>
                results(row).early_short_mean_abs_error = metrics.early_short_mean_abs_error; %#ok<SAGROW>
                results(row).early_long_mean_abs_error = metrics.early_long_mean_abs_error; %#ok<SAGROW>
                results(row).early_ratio = metrics.early_ratio; %#ok<SAGROW>
                results(row).zero_cross_penalty = metrics.zero_cross_penalty; %#ok<SAGROW>
                results(row).alignment_score = metrics.alignment_score; %#ok<SAGROW>

                fprintf('case %02d | vmax=%.2f wmax=%.2f tdot=%.2f hoff=%.3f | score=%.4f\n', ...
                    row, ...
                    results(row).vmax, ...
                    results(row).wmax, ...
                    results(row).theta_id_dot_limit, ...
                    results(row).heading_global_offset, ...
                    results(row).alignment_score);
            end
        end
    end
end

scores = [results.alignment_score];
[~, order] = sort(scores, 'ascend');

disp('Top 10 alignment candidates:');
for idx = 1:min(10, numel(order))
    r = results(order(idx));
    fprintf('#%d | vmax=%.2f wmax=%.2f tdot=%.2f hoff=%.3f | settle=%.2f soft=%.2f early5=%.3f early10=%.3f ratio=%.3f zc=%.3f final=%.4f osc=%.4f score=%.4f\n', ...
        idx, ...
        r.vmax, ...
        r.wmax, ...
        r.theta_id_dot_limit, ...
        r.heading_global_offset, ...
        r.settle_time, ...
        r.soft_settle_time, ...
        r.early_short_mean_abs_error, ...
        r.early_long_mean_abs_error, ...
        r.early_ratio, ...
        r.zero_cross_penalty, ...
        r.final_mean_abs_error, ...
        r.oscillation_penalty, ...
        r.alignment_score);
        % 参数扫描：验证控制器增益k对收敛时间的影响
clear; clc;

% 扫描的k值范围
k_list = [3, 5, 7, 10];
convergence_time = zeros(size(k_list));
params = struct();
% 基础参数（同run_fixed_topology_main）
params.mass = 1.0; params.uav_num = 4; params.topology_type = 'ring';
params.usde_k1 = 10; params.usde_k2 = 5; params.usde_gamma = 8;
params.alpha = 0.8; params.beta = 1.2; params.p = 1/2; params.q = 2;
params.sim_time = 10; params.dt = 0.01;
params.desired_formation = [1 0 -1 0; 0 1 0 -1; 0 0 0 0];

% 逐参数仿真
for idx = 1:length(k_list)
    params.k = k_list(idx);
    % 调用主仿真逻辑（简化版）
    [~, sim_log] = run_fixed_topology_core(params); % 需抽离run_fixed_topology_main的核心逻辑为函数
    % 计算收敛时间（误差<0.01）
    e_norm = norm(sim_log.e_form);
    convergence_time(idx) = find(e_norm < 0.01, 1) * params.dt;
end

% 绘图
figure;
plot(k_list, convergence_time, 'o-');
xlabel('Controller Gain k');
ylabel('Convergence Time (s)');
title('Convergence Time vs Controller Gain');
grid on;
savefig('../outputs/param_sweep_k.fig');

% 保存扫描结果
save('../outputs/param_sweep_results.mat', 'k_list', 'convergence_time');
disp('Parameter sweep completed! Results saved to outputs/');
end
