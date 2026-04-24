function metrics = compute_error_metrics(sim, cfg)
%COMPUTE_ERROR_METRICS Basic metrics for matching Fig. 15-16 style curves.

time = sim.time(:);
N = cfg.num_followers;
band = cfg.metrics.error_band;

eix = zeros(numel(time), N);
eiy = zeros(numel(time), N);
for i = 1:N
    eix(:, i) = squeeze(sim.formation_err(:, 1, i));
    eiy(:, i) = squeeze(sim.formation_err(:, 2, i));
end

metrics.eix_max_abs = max(abs(eix), [], 1);
metrics.eiy_max_abs = max(abs(eiy), [], 1);
metrics.eix_rms = sqrt(mean(eix .^ 2, 1));
metrics.eiy_rms = sqrt(mean(eiy .^ 2, 1));
metrics.eix_settle_time = local_settling_time(time, eix, band);
metrics.eiy_settle_time = local_settling_time(time, eiy, band);
metrics.band = band;
end

function settle_time = local_settling_time(time, err, band)
num_agents = size(err, 2);
settle_time = nan(1, num_agents);

for i = 1:num_agents
    idx = find(abs(err(:, i)) <= band, 1, 'first');
    if isempty(idx)
        continue;
    end
    if all(abs(err(idx:end, i)) <= band)
        settle_time(i) = time(idx);
    end
end
end
