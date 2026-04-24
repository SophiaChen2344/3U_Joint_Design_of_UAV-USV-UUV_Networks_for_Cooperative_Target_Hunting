function metrics = compute_alignment_metrics(sim, cfg)
%COMPUTE_ALIGNMENT_METRICS Metrics for matching the qualitative shape of Fig. 11-16.

time = sim.time(:);
err = sim.formation_err;

max_abs_err = squeeze(max(abs(err), [], 3));
max_total = max(max_abs_err, [], 2);

settle_threshold = cfg.alignment.settle_threshold;
soft_threshold = cfg.alignment.soft_threshold;
short_window = cfg.alignment.early_window_short;
long_window = cfg.alignment.early_window_long;

metrics.settle_time = local_settle_time(time, max_total, settle_threshold);
metrics.soft_settle_time = local_settle_time(time, max_total, soft_threshold);
metrics.peak_abs_error = max(max_total);
metrics.final_mean_abs_error = mean(max_total(max(1, end - round(10 / cfg.dt)):end));
metrics.oscillation_penalty = local_oscillation_penalty(err, cfg.dt);
metrics.early_short_mean_abs_error = local_window_mean(max_total, time, short_window);
metrics.early_long_mean_abs_error = local_window_mean(max_total, time, long_window);
metrics.early_ratio = local_early_ratio(max_total, time, short_window);
metrics.zero_cross_penalty = local_zero_cross_penalty(err, time, long_window, cfg.alignment.zero_cross_tolerance);

weights = cfg.alignment.score_weights;
metrics.alignment_score = ...
    weights.settle * metrics.settle_time + ...
    weights.soft_settle * metrics.soft_settle_time + ...
    weights.early_short_mean * metrics.early_short_mean_abs_error + ...
    weights.early_long_mean * metrics.early_long_mean_abs_error + ...
    weights.early_ratio * metrics.early_ratio + ...
    weights.zero_cross * metrics.zero_cross_penalty + ...
    weights.peak * metrics.peak_abs_error + ...
    weights.final_mean * metrics.final_mean_abs_error + ...
    weights.oscillation * metrics.oscillation_penalty;
end

function settle_time = local_settle_time(time, signal, threshold)
settle_time = time(end);
for k = 1:numel(time)
    if all(signal(k:end) < threshold)
        settle_time = time(k);
        return;
    end
end
end

function penalty = local_oscillation_penalty(err, dt)
window = max(2, round(8 / dt));
segment = abs(err(1:window, :, :));
penalty = 0.0;
for axis_id = 1:size(segment, 2)
    for uav_id = 1:size(segment, 3)
        s = squeeze(segment(:, axis_id, uav_id));
        ds = diff(s);
        penalty = penalty + sum(max(ds, 0));
    end
end
penalty = penalty / (size(segment, 2) * size(segment, 3));
end

function value = local_window_mean(signal, time, window_end)
idx = time <= window_end;
value = mean(signal(idx));
end

function ratio = local_early_ratio(signal, time, window_end)
initial_value = max(signal(1), 1.0e-6);
idx = find(time <= window_end, 1, 'last');
ratio = signal(idx) / initial_value;
end

function penalty = local_zero_cross_penalty(err, time, window_end, tol)
idx = find(time <= window_end);
segment = err(idx, :, :);
penalty = 0.0;
count = 0;
for axis_id = 1:size(segment, 2)
    for uav_id = 1:size(segment, 3)
        s = squeeze(segment(:, axis_id, uav_id));
        s(abs(s) < tol) = 0;
        if all(s == 0)
            continue;
        end
        s = local_fill_zeros(s);
        penalty = penalty + sum(abs(diff(sign(s))) > 0);
        count = count + 1;
    end
end
penalty = penalty / max(count, 1);
end

function s = local_fill_zeros(s)
for i = 2:numel(s)
    if s(i) == 0
        s(i) = s(i - 1);
    end
end
for i = numel(s)-1:-1:1
    if s(i) == 0
        s(i) = s(i + 1);
    end
end
end
