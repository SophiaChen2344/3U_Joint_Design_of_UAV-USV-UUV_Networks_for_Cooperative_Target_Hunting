function angle = wrap_to_pi_local(angle)
%WRAP_TO_PI_LOCAL Small local replacement to avoid toolbox dependence.

angle = mod(angle + pi, 2 * pi) - pi;
end
