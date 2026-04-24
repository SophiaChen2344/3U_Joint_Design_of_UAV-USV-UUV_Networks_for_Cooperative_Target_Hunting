clear;
clc;
close all;

cfg = uavrepro.default_config();
sim = uavrepro.run_fixed_topology_sim(cfg);
metrics = uavrepro.compute_alignment_metrics(sim, cfg);
uavrepro.plot_fixed_topology_results(sim, cfg);

disp('MATLAB fixed-topology simulation finished.');
disp(metrics);
fprintf('Measurement profile: %s\n', cfg.measurement.profile);
fprintf('Consistency max norm: %.3e\n', sim.consistency.summary.max_norm);
fprintf('Consistency RMS component: %.3e\n', sim.consistency.summary.rms_component);
fprintf('Consistency total exceedances: %d\n', sim.consistency.summary.total_exceedances);
fprintf('Triggered UAVs: %s\n', mat2str(sim.consistency.summary.triggered_uavs));
if sim.consistency.summary.passed
    disp('Consistency check passed within threshold.');
else
    warning('Consistency check exceeded the configured threshold.');
end
disp('Next step: run run_alignment_sweep.m to rank vmax, wmax, theta_id_dot_limit, and heading offset.');
