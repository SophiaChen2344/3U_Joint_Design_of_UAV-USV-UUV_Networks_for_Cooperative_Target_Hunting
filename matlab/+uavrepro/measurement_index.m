function idx = measurement_index(current_step, delay_steps, enabled)
%MEASUREMENT_INDEX Return the delayed sample index for a measurement signal.

if enabled
    idx = max(1, current_step - delay_steps);
else
    idx = current_step;
end
end
