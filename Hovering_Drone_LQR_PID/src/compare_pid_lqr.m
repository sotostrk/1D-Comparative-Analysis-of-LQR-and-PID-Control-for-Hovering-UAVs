clear; clc; close all;

%% System parameters
m = 3;
g = 9.81;
x0 = [2; 0];
T = 20;

%% PID Controller Parameters 
Kp = 40;
Kd = 15;
Ki = 10;

%% LQR Parameters 
A = [0 1; 0 0];
B = [0; 1/m];
target_st = 2.5;
target_os = 5;
[best_K, best_Q, best_R] = tune_lqr(m, g, target_st, target_os);
K_lqr = best_K;

%% Simulate PID
[t1, x1, u_pid] = simulate_pid(x0, T, Kp, Kd, Ki, m, g);

%% Simulate LQR
[t2, x2, u_lqr] = simulate_closed_loop(A, B, K_lqr, x0, m, g);

%% Plot Comparison
figure;
subplot(2,1,1)
plot(t1, x1(:,1), 'r--', t2, x2(:,1), 'b', 'LineWidth', 1.5);
legend('PID', 'LQR');
ylabel('Position (m)');
title('PID vs LQR: Position Response');
grid on

subplot(2,1,2)
plot(t1, u_pid, 'r--', t2, u_lqr, 'b', 'LineWidth', 1.5);
legend('PID', 'LQR');
xlabel('Time (s)');
ylabel('Thrust (N)');
title('Control Input Comparison');
grid on
saveas(gcf, '../figs/your_figure_name.png');

%% Contrl Effort Comparison
J_pid = trapz(t1, u_pid.^2);
J_lqr = trapz(t2, u_lqr.^2);
fprintf('Control Effort (J): PID = %.2f, LQR = %.2f\n', J_pid, J_lqr);
