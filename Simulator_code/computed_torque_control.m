function tau_control = computed_torque_control(M, C, q_e, qd_e)
    qdd_ref = zeros(N,1);
    tau_control = M*(qdd_ref + kd*qd_e + kp*q_e) + C;
    % Restrict torque to actuated joints and limit the magnitude.
    tau_control = saturate(tau_control);
end