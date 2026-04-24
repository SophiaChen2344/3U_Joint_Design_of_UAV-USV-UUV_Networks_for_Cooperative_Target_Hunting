function d = wind_disturbance(t)
%WIND_DISTURBANCE Unknown disturbance profile from Section IV-A.

dx = sin(0.3 * t - pi) * cos(pi / 10 * cos(0.5 * t + pi) + pi / 5);
dy = sin(0.3 * t - pi) * sin(pi / 10 * cos(0.5 * t + pi) + pi / 5);
d = [dx; dy];
end
