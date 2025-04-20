function [t, x, u] = simulate_pid(x0, T, Kp, Kd, Ki, m, g)
    dt = 0.01;
    t = 0:dt:T;
    
    x = zeros(length(t), 2);   % [z, z_dot]
    u = zeros(length(t), 1);   % thrust
    x(1,:) = x0;

    e_integral = 0;
    z_ref = 0;

    for i = 1:length(t)-1
        z = x(i,1);            % current position
        z_dot = x(i,2);        % current velocity
        
        e = z_ref - z;         % position error
        e_integral = e_integral + e * dt;
        e_derivative = -z_dot; % since z_ref = 0, d(e)/dt = -z_dot

        % PID control + gravity compensation
        u(i) = Kp * e + Kd * e_derivative + Ki * e_integral + m * g;

        % System dynamics (Euler integration)
        xdot1 = x(i,2);
        Fw = wind_force(t(i));
        xdot2 = (1/m) * (u(i) - m * g + Fw);
        x(i+1,1) = x(i,1) + xdot1 * dt;
        x(i+1,2) = x(i,2) + xdot2 * dt;
    end

    % Final control input
    z = x(end,1);
    z_dot = x(end,2);
    e = z_ref - z;
    e_derivative = -z_dot;
    u(end) = Kp * e + Kd * e_derivative + Ki * e_integral + m * g;
end
