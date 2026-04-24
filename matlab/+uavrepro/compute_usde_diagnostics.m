function diag = compute_usde_diagnostics(sim, cfg)
%COMPUTE_USDE_DIAGNOSTICS Quantify disturbance-estimation quality and lag.

time = sim.time(:);
N = cfg.num_followers;
band = cfg.usde.error_band;
plot_uav = min(max(cfg.usde.plot_uav_index, 1), N);

err = sim.dist_hat - sim.dist_true;

rms_axis = zeros(N, 2);
max_abs_axis = zeros(N, 2);
settle_axis = nan(N, 2);

for i = 1:N
    for axis_id = 1:2
        signal = squeeze(err(:, axis_id, i));
        rms_axis(i, axis_id) = sqrt(mean(signal .^ 2));
        max_abs_axis(i, axis_id) = max(abs(signal));
        settle_axis(i, axis_id) = local_settling_time(time, signal, band);
    end
end

true_x = squeeze(sim.dist_true(:, 1, plot_uav));
est_x = squeeze(sim.dist_hat(:, 1, plot_uav));
true_y = squeeze(sim.dist_true(:, 2, plot_uav));
est_y = squeeze(sim.dist_hat(:, 2, plot_uav));

diag.error = err;
diag.rms_axis = rms_axis;
diag.max_abs_axis = max_abs_axis;
diag.settle_axis = settle_axis;
diag.mean_rms = mean(rms_axis(:));
diag.max_rms = max(rms_axis(:));
diag.mean_max_abs = mean(max_abs_axis(:));
diag.max_abs = max(max_abs_axis(:));
diag.mean_settle = mean(settle_axis(~isnan(settle_axis)));
diag.max_settle = max(settle_axis(~isnan(settle_axis)));
diag.plot_uav_index = plot_uav;
diag.lag_seconds = [ ...
    local_best_lag_seconds(true_x, est_x, cfg.dt, cfg.usde.max_lag_seconds), ...
    local_best_lag_seconds(true_y, est_y, cfg.dt, cfg.usde.max_lag_seconds) ...
];
end

function settle_time = local_settling_time(time, signal, band)
settle_time = nan;
for idx = 1:numel(time)
    if all(abs(signal(idx:end)) <= band)
        settle_time = time(idx);
        return;
    end
end
end

function lag_seconds = local_best_lag_seconds(reference, estimate, dt, max_lag_seconds)
max_lag_samples = max(1, round(max_lag_seconds / dt));
best_mse = inf;
best_lag = 0;

for lag = -max_lag_samples:max_lag_samples
    [ref_seg, est_seg] = local_overlap(reference, estimate, lag);
    if numel(ref_seg) < 5
        continue;
    end
    mse = mean((ref_seg - est_seg) .^ 2);
    if mse < best_mse
        best_mse = mse;
        best_lag = lag;
    end
end

lag_seconds = best_lag * dt;
end

function [ref_seg, est_seg] = local_overlap(reference, estimate, lag)
if lag >= 0
    ref_seg = reference(1:end-lag);
    est_seg = estimate(1+lag:end);
else
    lag = abs(lag);
    ref_seg = reference(1+lag:end);
    est_seg = estimate(1:end-lag);
end
end
