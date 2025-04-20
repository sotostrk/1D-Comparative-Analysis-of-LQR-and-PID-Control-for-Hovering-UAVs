function plot_results(t, x, u)
    figure;
    subplot(2,1,1)
    plot(t, x(:,1), 'b', t, x(:,2), 'r', 'LineWidth', 1.5);
    xlabel('Time (s)'), ylabel('States');
    legend('z (Position)', 'ż (Velocity)','location','east');
    title('Closed-Loop State Response (with Wind)');
    grid on

    subplot(2,1,2)
    plot(t, u, 'k', 'LineWidth', 1.5);
    xlabel('Time (s)'), ylabel('Thrust (N)');
    legend('u(t)');
    title('Control Effort (with Wind)');
    grid on
end