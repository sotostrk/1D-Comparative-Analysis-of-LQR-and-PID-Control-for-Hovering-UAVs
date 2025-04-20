function [t, x, u] = simulate_closed_loop(A, B, K, x0, m, g)
    [t, x] = ode45(@(t, x) dynamics_closedloop_wind(t, x, A, B, K, m, g), [0 20], x0);
    
    u = zeros(size(t));
    for i = 1:length(t)
        Fw = wind_force(t(i));
        u(i) = -K * x(i,:)' + m * g + Fw;
    end
end

function dxdt = dynamics_closedloop(t, x, A, B, K, m, g)
    u = -K * x + m * g;
    dxdt = A * x + B * u;
end