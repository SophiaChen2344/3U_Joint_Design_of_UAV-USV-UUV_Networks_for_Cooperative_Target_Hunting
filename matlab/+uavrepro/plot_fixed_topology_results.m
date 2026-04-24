function plot_fixed_topology_results(sim, cfg)
%PLOT_FIXED_TOPOLOGY_RESULTS Debug plots aligned with the paper's error figures.

time = sim.time;
N = cfg.num_followers;
styles = {'o-', 'v--', '*:', 's-.', 'd-'};
labels = arrayfun(@(i) sprintf('UAV%d', i), 1:N, 'UniformOutput', false);
metrics = uavrepro.compute_error_metrics(sim, cfg);
align_metrics = uavrepro.compute_alignment_metrics(sim, cfg);

figure('Name', 'Trajectory');
plot(sim.leader(:, 1), sim.leader(:, 2), 'k-', 'LineWidth', 1.8);
hold on;
for i = 1:N
    traj = squeeze(sim.followers(:, 1:2, i));
    plot(traj(:, 1), traj(:, 2), 'LineWidth', 1.2);
end
grid on;
axis equal;
title('Fixed-Topology Formation Trajectories');
xlabel('x');
ylabel('y');
legend([{'Leader'}, labels], 'Location', 'best');

figure('Name', 'e_ix');
hold on;
for i = 1:N
    err_x = squeeze(sim.formation_err(:, 1, i));
    mark_idx = 1:max(1, floor(numel(time) / 10)):numel(time);
    plot(time, err_x, styles{i}, 'LineWidth', 1.2, 'MarkerSize', 5, 'MarkerIndices', mark_idx);
end
grid on;
xlim([time(1), time(end)]);
ylim([-20, 30]);
xlabel('t(s)');
ylabel('e_{ix}(m)');
title(sprintf('e_{ix} (profile: %s)', cfg.tuning.profile));
legend(labels, 'Location', 'northeast');

figure('Name', 'e_iy');
hold on;
for i = 1:N
    err_y = squeeze(sim.formation_err(:, 2, i));
    mark_idx = 1:max(1, floor(numel(time) / 10)):numel(time);
    plot(time, err_y, styles{i}, 'LineWidth', 1.2, 'MarkerSize', 5, 'MarkerIndices', mark_idx);
end
grid on;
xlim([time(1), time(end)]);
ylim([-20, 30]);
xlabel('Time (s)');
ylabel('e_{iy}(m)');
title(sprintf('e_{iy} (profile: %s)', cfg.tuning.profile));
legend(labels, 'Location', 'northeast');

figure('Name', 'Zoomed Error Comparison');
subplot(2, 1, 1);
hold on;
for i = 1:N
    plot(time, squeeze(sim.formation_err(:, 1, i)), styles{i}, 'LineWidth', 1.1);
end
grid on;
xlim([0, cfg.metrics.zoom_window]);
ylim([-20, 30]);
ylabel('e_{ix}(m)');
title('Early-time error shape against Fig. 15-16');

subplot(2, 1, 2);
hold on;
for i = 1:N
    plot(time, squeeze(sim.formation_err(:, 2, i)), styles{i}, 'LineWidth', 1.1);
end
grid on;
xlim([0, cfg.metrics.zoom_window]);
ylim([-20, 30]);
xlabel('Time (s)');
ylabel('e_{iy}(m)');

figure('Name', 'Early Shape 0-5s');
subplot(2, 1, 1);
hold on;
for i = 1:N
    plot(time, squeeze(sim.formation_err(:, 1, i)), styles{i}, 'LineWidth', 1.1);
end
grid on;
xlim([0, 5]);
ylim([-20, 30]);
ylabel('e_{ix}(m)');
title('Early 0-5s shape against Fig. 15');

subplot(2, 1, 2);
hold on;
for i = 1:N
    plot(time, squeeze(sim.formation_err(:, 2, i)), styles{i}, 'LineWidth', 1.1);
end
grid on;
xlim([0, 5]);
ylim([-20, 30]);
xlabel('Time (s)');
ylabel('e_{iy}(m)');
title('Early 0-5s shape against Fig. 16');

figure('Name', 'Early Shape 0-10s');
subplot(2, 1, 1);
hold on;
for i = 1:N
    plot(time, squeeze(sim.formation_err(:, 1, i)), styles{i}, 'LineWidth', 1.1);
end
grid on;
xlim([0, 10]);
ylim([-20, 30]);
ylabel('e_{ix}(m)');
title(sprintf('0-10s | early mean %.3f | early ratio %.3f', ...
    align_metrics.early_long_mean_abs_error, ...
    align_metrics.early_ratio));

subplot(2, 1, 2);
hold on;
for i = 1:N
    plot(time, squeeze(sim.formation_err(:, 2, i)), styles{i}, 'LineWidth', 1.1);
end
grid on;
xlim([0, 10]);
ylim([-20, 30]);
xlabel('Time (s)');
ylabel('e_{iy}(m)');
title(sprintf('0-10s | zero-cross penalty %.3f', align_metrics.zero_cross_penalty));

figure('Name', 'USDE Disturbance Estimate');
for axis_id = 1:2
    subplot(2, 1, axis_id);
    true_d = squeeze(sim.dist_true(:, axis_id, 1));
    est_d = squeeze(sim.dist_hat(:, axis_id, 1));
    plot(time, true_d, 'k-', time, est_d, 'm--', 'LineWidth', 1.2);
    grid on;
    if axis_id == 1
        title('UAV 1 Disturbance and USDE Estimate');
        ylabel('x-axis');
    else
        ylabel('y-axis');
        xlabel('Time (s)');
    end
end

figure('Name', 'Measured Velocity Signals');
subplot(2, 1, 1);
plot(time, squeeze(sim.measured_vi(:, 1, 1)), 'b-', time, squeeze(sim.measured_pdot(:, 1, 1)), 'r--', 'LineWidth', 1.2);
grid on;
title('UAV 1 x-axis measured signals');
ylabel('x-axis');
legend({'V_i', 'p_i dot'}, 'Location', 'best');

subplot(2, 1, 2);
plot(time, squeeze(sim.measured_vi(:, 2, 1)), 'b-', time, squeeze(sim.measured_pdot(:, 2, 1)), 'r--', 'LineWidth', 1.2);
grid on;
ylabel('y-axis');
xlabel('Time (s)');
legend({'V_i', 'p_i dot'}, 'Location', 'best');

if isfield(sim, 'consistency')
    plot_uav = min(max(cfg.consistency.plot_uav_index, 1), N);
    figure('Name', 'Data Consistency Check');
    subplot(2, 1, 1);
    plot(time, squeeze(sim.measured_pdot(:, 1, plot_uav)), 'k-', ...
        time, squeeze(sim.consistency.model_pdot(:, 1, plot_uav)), 'g--', 'LineWidth', 1.2);
    grid on;
    title(sprintf('UAV %d x-axis: measured p_i dot vs V_i + d_i (%s)', plot_uav, cfg.measurement.profile));
    ylabel('x-axis');
    legend({'measured p_i dot', 'V_i + d_i'}, 'Location', 'best');

    subplot(2, 1, 2);
    hold on;
    for i = 1:N
        plot(time, sim.consistency.residual_norm(:, i), styles{i}, 'LineWidth', 1.1, 'MarkerSize', 4);
    end
    yline(cfg.consistency.warn_threshold, 'r--', 'LineWidth', 1.1);
    grid on;
    title('Consistency residual norm across UAVs');
    ylabel('|| residual ||');
    xlabel('Time (s)');
    legend([labels, {'threshold'}], 'Location', 'best');
end

fprintf('\nFixed-topology error metrics (band = %.2f m)\n', metrics.band);
for i = 1:N
    fprintf('UAV%d: eix settle %.3fs, eiy settle %.3fs, eix RMS %.3f, eiy RMS %.3f\n', ...
        i, metrics.eix_settle_time(i), metrics.eiy_settle_time(i), metrics.eix_rms(i), metrics.eiy_rms(i));
end
fprintf('Alignment: settle %.3fs, soft %.3fs, early5 %.3f, early10 %.3f, ratio %.3f, zero-cross %.3f, score %.3f\n', ...
    align_metrics.settle_time, ...
    align_metrics.soft_settle_time, ...
    align_metrics.early_short_mean_abs_error, ...
    align_metrics.early_long_mean_abs_error, ...
    align_metrics.early_ratio, ...
    align_metrics.zero_cross_penalty, ...
    align_metrics.alignment_score);
if isfield(sim, 'consistency')
    fprintf('Consistency max norm %.3e, RMS component %.3e\n', ...
        sim.consistency.summary.max_norm, sim.consistency.summary.rms_component);
    fprintf('Consistency total exceedances %d, triggered UAVs: %s\n', ...
        sim.consistency.summary.total_exceedances, mat2str(sim.consistency.summary.triggered_uavs));
end
end
