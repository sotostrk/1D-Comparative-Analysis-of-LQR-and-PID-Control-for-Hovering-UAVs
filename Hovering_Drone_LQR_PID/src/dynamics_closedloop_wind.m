function dxdt = dynamics_closedloop_wind(t, x, A, B, K, m, g)
    Fw = wind_force(t);
    u = -K * x + m * g + Fw;
    dxdt = A * x + B * u;
end