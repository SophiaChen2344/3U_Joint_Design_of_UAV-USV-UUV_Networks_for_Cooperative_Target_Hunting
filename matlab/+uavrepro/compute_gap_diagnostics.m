function gap = compute_gap_diagnostics(sim, cfg, err_metrics, align_metrics, usde_diag)
%COMPUTE_GAP_DIAGNOSTICS Summarize why the simulation differs from the paper.

gap.soft_settle_ratio = align_metrics.soft_settle_time / max(cfg.paper_gap.target_soft_settle, eps);
gap.hard_settle_ratio = align_metrics.settle_time / max(cfg.paper_gap.target_hard_settle, eps);
gap.early_ratio_ratio = align_metrics.early_ratio / max(cfg.paper_gap.target_early_ratio, eps);
gap.zero_cross_ratio = align_metrics.zero_cross_penalty / max(cfg.paper_gap.target_zero_cross_penalty, eps);
gap.final_mean_ratio = align_metrics.final_mean_abs_error / max(cfg.paper_gap.target_final_mean_abs_error, eps);
gap.usde_mean_rms_ratio = usde_diag.mean_rms / max(cfg.paper_gap.target_usde_mean_rms, eps);
gap.usde_max_abs_ratio = usde_diag.max_abs / max(cfg.paper_gap.target_usde_max_abs, eps);
gap.usde_lag_ratio = max(abs(usde_diag.lag_seconds)) / max(cfg.paper_gap.target_usde_lag_seconds, eps);
gap.consistency_ratio = sim.consistency.summary.max_norm / max(cfg.consistency.warn_threshold, eps);
gap.error_band = err_metrics.band;

gap.shape_gap_score = mean([ ...
    gap.soft_settle_ratio, ...
    gap.hard_settle_ratio, ...
    gap.early_ratio_ratio, ...
    gap.zero_cross_ratio, ...
    gap.final_mean_ratio ...
]);

gap.observer_gap_score = mean([ ...
    gap.usde_mean_rms_ratio, ...
    gap.usde_max_abs_ratio, ...
    gap.usde_lag_ratio ...
]);

gap.instrumentation_ok = sim.consistency.summary.passed;
gap.notes = local_build_notes(sim, cfg, align_metrics, usde_diag, gap);
end

function notes = local_build_notes(sim, cfg, align_metrics, usde_diag, gap)
notes = {};

if ~sim.consistency.summary.passed
    notes{end+1} = 'Data consistency still fails, so paper-gap conclusions are not yet trustworthy.'; %#ok<AGROW>
end

if usde_diag.mean_rms > cfg.paper_gap.target_usde_mean_rms
    notes{end+1} = 'USDE RMS error is above the paper-like target; kappa discretization or velocity-measurement timing is a likely gap source.'; %#ok<AGROW>
end

if max(abs(usde_diag.lag_seconds)) > cfg.paper_gap.target_usde_lag_seconds
    notes{end+1} = 'USDE shows noticeable lag against the true disturbance, which can directly slow the early error decay seen in Fig. 15-16.'; %#ok<AGROW>
end

if align_metrics.soft_settle_time > cfg.paper_gap.target_soft_settle
    notes{end+1} = 'Formation errors settle slower than the paper-like target; controller saturation or heading dynamics are likely limiting factors.'; %#ok<AGROW>
end

if align_metrics.early_ratio > cfg.paper_gap.target_early_ratio
    notes{end+1} = 'Early-time error decay is too slow compared with the paper figures; try larger control authority or lower heading lag.'; %#ok<AGROW>
end

if align_metrics.zero_cross_penalty > cfg.paper_gap.target_zero_cross_penalty
    notes{end+1} = 'The simulated curves cross zero more often than the paper-like traces, suggesting excessive aggressiveness or derivative action.'; %#ok<AGROW>
end

if gap.observer_gap_score <= gap.shape_gap_score && sim.consistency.summary.passed
    notes{end+1} = 'Current evidence suggests the dominant mismatch is in the controller/transient shape, not in the data pipeline.'; %#ok<AGROW>
elseif sim.consistency.summary.passed
    notes{end+1} = 'Current evidence suggests the dominant mismatch is in disturbance estimation rather than plotting or state reconstruction.'; %#ok<AGROW>
end

if isempty(notes)
    notes{1} = 'No major gap flag was triggered by the current heuristics.'; %#ok<AGROW>
end
end
