function sim = run_fixed_topology_sim(cfg)
%RUN_FIXED_TOPOLOGY_SIM Fixed-topology simulation with USDE and Eq. (14)-(16).

dt = cfg.dt;
T = cfg.duration;
steps = floor(T / dt) + 1;
time = (0:steps-1) * dt;
N = cfg.num_followers;

if isfield(cfg, 'measurement') && isfield(cfg.measurement, 'seed')
    rng(cfg.measurement.seed);
end

leader = zeros(steps, 3);
followers = zeros(steps, 3, N);
desired = uavrepro.build_desired_offsets(cfg.formation.radius, N);
follower_init = cfg.initial.followers;
follower_init(:, 3) = uavrepro.wrap_to_pi_local( ...
    cfg.initial.heading_scale * follower_init(:, 3) + cfg.initial.heading_global_offset);

leader(1, :) = cfg.initial.leader;
followers(1, :, :) = permute(follower_init, [3, 2, 1]);

usde = repmat(struct('pif', zeros(2, 1), 'vif', zeros(2, 1)), N, 1);
prev_theta_id = squeeze(followers(1, 3, :));
current_d_hat = zeros(N, 2);

dist_true = zeros(steps, 2, N);
dist_hat = zeros(steps, 2, N);
formation_err = zeros(steps, 2, N);
speed_cmd = zeros(steps, N);
turn_cmd = zeros(steps, N);
theta_id_hist = zeros(steps, N);
measured_vi_hist = zeros(steps, 2, N);
measured_pdot_hist = zeros(steps, 2, N);
consistency_disturbance_hist = zeros(steps, 2, N);
true_vi_hist = zeros(steps, 2, N);
true_pdot_hist = zeros(steps, 2, N);
sensed_vi_hist = zeros(steps, 2, N);
sensed_pdot_hist = zeros(steps, 2, N);

A = cfg.topology.A;
B = diag(cfg.topology.B);

current_measured_vi = zeros(N, 2);
current_measured_pdot = zeros(N, 2);

for i = 1:N
    initial_heading = followers(1, 3, i);
    initial_disturbance = uavrepro.wind_disturbance(time(1));
    true_vi_hist(1, :, i) = [0.0, 0.0];
    true_pdot_hist(1, :, i) = initial_disturbance.';
    sensed_vi_hist(1, :, i) = uavrepro.apply_measurement_noise([0.0, 0.0], cfg.measurement.vi_noise_std, cfg.measurement.enable_noise);
    sensed_pdot_hist(1, :, i) = uavrepro.apply_measurement_noise(initial_disturbance.', cfg.measurement.pdot_noise_std, cfg.measurement.enable_noise);
    current_measured_vi(i, :) = squeeze(sensed_vi_hist(1, :, i));
    current_measured_pdot(i, :) = squeeze(sensed_pdot_hist(1, :, i));
    measured_vi_hist(1, :, i) = current_measured_vi(i, :);
    measured_pdot_hist(1, :, i) = current_measured_pdot(i, :);
    consistency_disturbance_hist(1, :, i) = initial_disturbance.';
    dist_true(1, :, i) = initial_disturbance.';
    dist_hat(1, :, i) = current_d_hat(i, :);
    theta_id_hist(1, i) = initial_heading;
end

for k = 1:steps-1
    t = time(k);

    [p0_dot, theta0_ref] = uavrepro.leader_reference(t);
    leader(k, 3) = theta0_ref;
    leader(k+1, 1:2) = leader(k, 1:2) + dt * p0_dot.';
    leader(k+1, 3) = theta0_ref;

    positions = squeeze(followers(k, 1:2, :)).';
    headings = squeeze(followers(k, 3, :));
    d_all = zeros(N, 2);
    v_cmd_all = zeros(N, 1);
    w_cmd_all = zeros(N, 1);
    theta_id_all = zeros(N, 1);

    for i = 1:N
        d_i = uavrepro.wind_disturbance(t);
        d_all(i, :) = d_i.';
        measured_vi_hist(k, :, i) = current_measured_vi(i, :);
        measured_pdot_hist(k, :, i) = current_measured_pdot(i, :);
        consistency_disturbance_hist(k, :, i) = d_i.';
        dist_true(k, :, i) = d_i.';
        dist_hat(k, :, i) = current_d_hat(i, :);
    end

    for i = 1:N
        theta_i = headings(i);
        e_i = uavrepro.formation_error_fixed(i, positions, desired, leader(k, 1:2).', A, B);
        formation_err(k, :, i) = e_i.';

        [v_cmd, w_cmd, theta_id] = uavrepro.fixed_time_control( ...
            i, e_i, current_measured_pdot, p0_dot, theta_i, prev_theta_id(i), current_d_hat(i, :).', A, B, cfg.controller, dt);

        v_cmd_all(i) = v_cmd;
        w_cmd_all(i) = w_cmd;
        theta_id_all(i) = theta_id;
    end

    for i = 1:N
        theta_i = headings(i);
        d_i = d_all(i, :).';
        speed_cmd(k, i) = v_cmd_all(i);
        turn_cmd(k, i) = w_cmd_all(i);
        theta_id_hist(k, i) = theta_id_all(i);

        v_cmd = v_cmd_all(i);
        w_cmd = w_cmd_all(i);
        dx = v_cmd * cos(theta_i) + d_i(1);
        dy = v_cmd * sin(theta_i) + d_i(2);
        dtheta = w_cmd;

        followers(k+1, 1, i) = followers(k, 1, i) + dt * dx;
        followers(k+1, 2, i) = followers(k, 2, i) + dt * dy;
        followers(k+1, 3, i) = uavrepro.wrap_to_pi_local(followers(k, 3, i) + dt * dtheta);
        prev_theta_id(i) = theta_id_all(i);
    end

    next_positions = squeeze(followers(k+1, 1:2, :)).';
    for i = 1:N
        next_p_i = next_positions(i, :).';
        current_p_i = positions(i, :).';
        executed_vi = v_cmd_all(i) * [cos(headings(i)), sin(headings(i))];
        next_measured_pdot = (next_p_i - current_p_i).' / dt;
        true_vi_hist(k+1, :, i) = executed_vi;
        true_pdot_hist(k+1, :, i) = next_measured_pdot;

        sensed_vi_hist(k+1, :, i) = uavrepro.apply_measurement_noise(executed_vi, cfg.measurement.vi_noise_std, cfg.measurement.enable_noise);
        sensed_pdot_hist(k+1, :, i) = uavrepro.apply_measurement_noise(next_measured_pdot, cfg.measurement.pdot_noise_std, cfg.measurement.enable_noise);

        vi_idx = uavrepro.measurement_index(k+1, cfg.measurement.vi_delay_steps, cfg.measurement.enable_delay);
        pdot_idx = uavrepro.measurement_index(k+1, cfg.measurement.pdot_delay_steps, cfg.measurement.enable_delay);

        current_measured_vi(i, :) = squeeze(sensed_vi_hist(vi_idx, :, i));
        current_measured_pdot(i, :) = squeeze(sensed_pdot_hist(pdot_idx, :, i));

        [usde(i), next_d_hat] = uavrepro.usde_update( ...
            usde(i), next_p_i, current_measured_vi(i, :).', dt, cfg.controller.kappa);
        current_d_hat(i, :) = next_d_hat.';

        measured_vi_hist(k+1, :, i) = current_measured_vi(i, :);
        measured_pdot_hist(k+1, :, i) = current_measured_pdot(i, :);
        consistency_disturbance_hist(k+1, :, i) = uavrepro.wind_disturbance(time(k+1)).';
        dist_hat(k+1, :, i) = current_d_hat(i, :);
    end
end

for i = 1:N
    formation_err(end, :, i) = uavrepro.formation_error_fixed(i, squeeze(followers(end, 1:2, :)).', desired, leader(end, 1:2).', A, B).';
    dist_true(end, :, i) = uavrepro.wind_disturbance(time(end)).';
    measured_vi_hist(end, :, i) = current_measured_vi(i, :);
    measured_pdot_hist(end, :, i) = current_measured_pdot(i, :);
    consistency_disturbance_hist(end, :, i) = uavrepro.wind_disturbance(time(end)).';
    dist_hat(end, :, i) = current_d_hat(i, :);
end

consistency_model = measured_vi_hist + consistency_disturbance_hist;
consistency_residual = measured_pdot_hist - consistency_model;
consistency_residual_norm = squeeze(vecnorm(consistency_residual, 2, 2));
exceedance_mask = consistency_residual_norm > cfg.consistency.warn_threshold;

summary.max_abs_component = max(abs(consistency_residual(:)));
summary.rms_component = sqrt(mean(consistency_residual(:).^2));
summary.max_norm = max(consistency_residual_norm(:));
summary.mean_norm = mean(consistency_residual_norm(:));
summary.per_uav_max_norm = max(consistency_residual_norm, [], 1);
summary.per_uav_exceedances = sum(exceedance_mask, 1);
summary.total_exceedances = sum(exceedance_mask(:));
summary.warn_threshold = cfg.consistency.warn_threshold;
summary.passed = summary.max_norm <= summary.warn_threshold;
summary.triggered_uavs = find(summary.per_uav_exceedances > 0);

sim.time = time;
sim.leader = leader;
sim.followers = followers;
sim.desired = desired;
sim.dist_true = dist_true;
sim.dist_hat = dist_hat;
sim.formation_err = formation_err;
sim.speed_cmd = speed_cmd;
sim.turn_cmd = turn_cmd;
sim.theta_id = theta_id_hist;
sim.measured_vi = measured_vi_hist;
sim.measured_pdot = measured_pdot_hist;
sim.true_vi = true_vi_hist;
sim.true_pdot = true_pdot_hist;
sim.sensed_vi = sensed_vi_hist;
sim.sensed_pdot = sensed_pdot_hist;
sim.consistency.disturbance = consistency_disturbance_hist;
sim.consistency.model_pdot = consistency_model;
sim.consistency.residual = consistency_residual;
sim.consistency.residual_norm = consistency_residual_norm;
sim.consistency.summary = summary;
sim.initial_followers = follower_init;
end
