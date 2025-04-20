function J = compute_control_effort(u, t)
    % Numerical integration using the trapezoidal rule
    J = trapz(t, u.^2);
end