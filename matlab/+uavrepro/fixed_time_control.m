function [v_cmd, w_cmd, theta_id] = fixed_time_control(i, e_i, neighbor_pdot, p0_dot, theta_i, prev_theta_id, d_hat, A, B, ctrl, dt)
%FIXED_TIME_CONTROL Fixed-topology controller based on Eq. (14)-(16).
%
% The OCR text in Eq. (14) drops the summation over neighbors.
% This implementation follows Eq. (13) and uses the weighted average of
% measured neighbor inertial velocities plus the leader velocity term.

N = size(neighbor_pdot, 1);
di = sum(A(i, :));
bi = B(i);
den = max(di + bi, 1);

alpha = diag(ctrl.alpha);
beta = diag(ctrl.beta);
phi_val = local_phi(e_i, ctrl);
q_i = local_q(e_i, ctrl);

norm_e = norm(e_i);
norm_e = max(norm_e, ctrl.eps_norm);

neighbor_ref = zeros(2, 1);
for j = 1:N
    if A(i, j) ~= 0
        neighbor_ref = neighbor_ref + A(i, j) * neighbor_pdot(j, :).';
    end
end
neighbor_ref = (neighbor_ref + bi * p0_dot) / den;

term_p = alpha * e_i / (norm_e^(1 - ctrl.p));
term_q = beta * e_i / (norm_e^(1 - q_i));

f_u = -(term_p + term_q) / (phi_val * den) - e_i + neighbor_ref - d_hat;

v_cmd = min(max(norm(f_u), ctrl.vmin), ctrl.vmax);
theta_id = atan2(f_u(2), f_u(1));
theta_id_dot = uavrepro.wrap_to_pi_local(theta_id - prev_theta_id) / dt;
theta_id_dot = min(max(theta_id_dot, -ctrl.theta_id_dot_limit), ctrl.theta_id_dot_limit);
e_theta = uavrepro.wrap_to_pi_local(theta_i - theta_id);
w_raw = -ctrl.k1 * e_theta + theta_id_dot;
w_cmd = min(max(w_raw, ctrl.wmin), ctrl.wmax);
end

function value = local_phi(e_i, ctrl)
value = ctrl.a1 + (1.0 - ctrl.a1) * exp(-ctrl.b1 * ((e_i.' * e_i) / 2)^ctrl.c1);
end

function value = local_q(e_i, ctrl)
ratio = ctrl.m / ctrl.n;
value = ratio + (ratio - 1.0) * sign((e_i.' * e_i) / 2 - 1.0);
value = max(value, 1.0);
end
