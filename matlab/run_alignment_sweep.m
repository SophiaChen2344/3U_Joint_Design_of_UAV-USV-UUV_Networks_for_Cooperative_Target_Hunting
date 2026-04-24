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
end
