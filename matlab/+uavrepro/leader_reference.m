function [p0_dot, theta0] = leader_reference(t)
%LEADER_REFERENCE Leader velocity profile from Section IV-A.

xdot = 1.5;
ydot = 0.8 * sin(0.1 * t) + 0.2;

p0_dot = [xdot; ydot];
theta0 = atan2(ydot, xdot);
end
