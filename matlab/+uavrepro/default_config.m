function cfg = default_config()
%DEFAULT_CONFIG Parameters for fixed-topology reproduction of Eq. (6)-(16).

cfg.dt = 0.05;
cfg.duration = 80.0;
cfg.num_followers = 5;
cfg.tuning.profile = 'paper_like_fig15_16_best_guess';

cfg.controller.kappa = 0.05;
cfg.controller.alpha = [0.1, 0.1];
cfg.controller.beta = [0.01, 0.01];
cfg.controller.p = 0.5;
cfg.controller.m = 11;
cfg.controller.n = 10;
cfg.controller.a1 = 0.2;
cfg.controller.b1 = 2.0;
cfg.controller.c1 = 2;
cfg.controller.k1 = 5.0;
cfg.controller.vmin = 0.0;
cfg.controller.vmax = 6.0;
cfg.controller.wmin = -8.0;
cfg.controller.wmax = 8.0;
cfg.controller.eps_norm = 1.0e-6;
cfg.controller.theta_id_dot_limit = 5.0;

cfg.metrics.error_band = 0.5;
cfg.metrics.zoom_window = 10.0;
cfg.consistency.warn_threshold = 5.0e-2;
cfg.consistency.plot_uav_index = 1;
cfg.measurement.profile = 'consistency_fault_test';
cfg.measurement.enable_noise = true;
cfg.measurement.enable_delay = true;
cfg.measurement.vi_noise_std = [0.02, 0.02];
cfg.measurement.pdot_noise_std = [0.05, 0.05];
cfg.measurement.vi_delay_steps = 1;
cfg.measurement.pdot_delay_steps = 2;
cfg.measurement.seed = 7;
cfg.usde.error_band = 0.10;
cfg.usde.plot_uav_index = 1;
cfg.usde.max_lag_seconds = 3.0;
cfg.alignment.vmax_candidates = [4.0, 5.0, 6.0, 8.0];
cfg.alignment.wmax_candidates = [4.0, 6.0, 8.0];
cfg.alignment.theta_id_dot_limit_candidates = [5.0, 8.0, 12.0];
cfg.alignment.heading_offset_candidates = [-0.05, 0.0, 0.05];
cfg.alignment.settle_threshold = 0.5;
cfg.alignment.soft_threshold = 1.0;
cfg.alignment.early_window_short = 5.0;
cfg.alignment.early_window_long = 10.0;
cfg.alignment.zero_cross_tolerance = 0.1;
cfg.alignment.score_weights = struct( ...
    'settle', 1.0, ...
    'soft_settle', 0.5, ...
    'early_short_mean', 6.0, ...
    'early_long_mean', 3.0, ...
    'early_ratio', 8.0, ...
    'zero_cross', 2.0, ...
    'peak', 0.1, ...
    'final_mean', 10.0, ...
    'oscillation', 5.0);
cfg.paper_gap.target_soft_settle = 2.0;
cfg.paper_gap.target_hard_settle = 5.0;
cfg.paper_gap.target_early_ratio = 0.05;
cfg.paper_gap.target_zero_cross_penalty = 1.0;
cfg.paper_gap.target_usde_mean_rms = 0.15;
cfg.paper_gap.target_usde_max_abs = 0.50;
cfg.paper_gap.target_usde_lag_seconds = 0.25;
cfg.paper_gap.target_final_mean_abs_error = 0.10;

cfg.formation.radius = 10.0;

% Exact fixed topology digitized from Fig. 10.
cfg.topology.A = [
    0 1 0 1 0;
    1 0 1 0 0;
    0 1 0 0 0;
    1 0 0 0 1;
    0 0 0 1 0
];

% Fixed-topology comparison section in the paper uses B = diag(1, 0, 0, 0, 0).
cfg.topology.B = diag([1 0 0 0 0]);

% Exact initial states digitized from Table II.
cfg.initial.leader = [0.0, 0.0, pi / 16];
cfg.initial.followers = [
     0.0, 10.0, pi / 16;
     5.0,  6.0, pi / 10;
    -5.0,  6.0, pi / 6;
    -2.0,  3.0, pi / 4;
     2.0,  3.0, pi / 15
];
cfg.initial.heading_global_offset = 0.05;
cfg.initial.heading_scale = 1.0;

cfg.paper.eq.dynamics = '(6)-(7)';
cfg.paper.eq.observer = '(8)-(11)';
cfg.paper.eq.formation_error = '(12)-(13)';
cfg.paper.eq.controller = '(14)-(16)';
cfg.paper.fig.fixed_topology = 'Fig. 10';
cfg.paper.table.initial_states = 'Table II';
end
