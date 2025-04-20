clear; clc; close all;

%% System parameters
m = 3;
g = 9.81;

%% Target performance specs
target_st = 2.5;  % target settling time (s)
target_os = 5;    % target overshoot (%)

%% Search for optimal LQR gains
[best_K, best_Q, best_R] = tune_lqr(m, g, target_st, target_os);

%% Simulate closed-loop system
A = [0 1; 0 0];
B = [0; 1/m];
x0 = [0; 0];
[t, x, u] = simulate_closed_loop(A, B, best_K, x0, m, g);
%% Compute control effort
J = compute_control_effort(u, t);
fprintf('Total control effort (J = ∫u²dt): %.2f\n', J);

%% Plot results
plot_results(t, x, u);
saveas(gcf, '../figs/your_figure_name.png');