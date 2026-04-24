function [state, d_hat] = usde_update(state, p_i, v_i, dt, kappa)
%USDE_UPDATE Low-pass filter and disturbance estimate from Eq. (8)-(11).

state.pif = state.pif + dt * (p_i - state.pif) / kappa;
state.vif = state.vif + dt * (v_i - state.vif) / kappa;
d_hat = -state.vif + (p_i - state.pif) / kappa;
end
