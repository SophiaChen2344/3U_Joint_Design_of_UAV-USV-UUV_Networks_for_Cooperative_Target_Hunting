function signal_out = apply_measurement_noise(signal_in, noise_std, enabled)
%APPLY_MEASUREMENT_NOISE Add Gaussian measurement noise when enabled.

if enabled
    signal_out = signal_in + noise_std .* randn(size(signal_in));
else
    signal_out = signal_in;
end
end
