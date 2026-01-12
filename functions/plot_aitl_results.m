function plot_aitl_results(simdata, ctrl, config)
% PLOT_AITL_RESULTS Creates unified plots for AITL simulation results
%
% Inputs:
%   simdata - Simulation data structure (from simulate_aitl)
%   ctrl    - Controller structure (for cost computation)
%   config  - Configuration structure:
%             .visualization.showPlots      - Enable plotting (boolean)
%             .visualization.figPos         - Figure position [x, y, w, h]
%             .visualization.axisFontSize   - Axis font size
%             .visualization.legendLocation - Legend location string

% Quick exit if plotting disabled
if ~config.visualization.showPlots
    fprintf('Plotting disabled (showPlots = false)\n');
    return;
end

%% Extract parameters
time = simdata.time;
x_hist = simdata.x_hist;
u_hist = simdata.u_hist;
theta_hist = simdata.theta_hist;
x_pos_hist = simdata.x_pos_hist;
M = simdata.M;
total_steps = simdata.total_steps;

figPos = config.visualization.figPos;
axisFontSize = config.visualization.axisFontSize;
legendLocation = config.visualization.legendLocation;
sim_time = max(time);

Q = ctrl.Q;

fprintf('\n=== Generating Plots ===\n');

%% 1) Cart displacements
figure('Position', figPos, 'Color', 'w');
plot(time, x_pos_hist', 'LineWidth', 1.5);
xlabel('$t\,[\mathrm{s}]$', 'FontSize', axisFontSize, 'Interpreter', 'latex');
ylabel('$x_i\,[\mathrm{m}]$', 'FontSize', axisFontSize, 'Interpreter', 'latex');
title('Cart Displacements', 'FontSize', axisFontSize);
grid on;
legend(arrayfun(@(i) sprintf('$x_{%d}$', i), 1:M, 'UniformOutput', false), ...
    'Location', legendLocation, 'FontSize', axisFontSize, 'Interpreter', 'latex');
set(gca, 'FontSize', axisFontSize);
xlim([0, sim_time]);

%% 2) Pendulum angles
figure('Position', figPos, 'Color', 'w');
plot(time, rad2deg(theta_hist)', 'LineWidth', 1.5);
xlabel('$t\,[\mathrm{s}]$', 'FontSize', axisFontSize, 'Interpreter', 'latex');
ylabel('$\theta_i\,[\mathrm{deg}]$', 'FontSize', axisFontSize, 'Interpreter', 'latex');
title('Pendulum Angles', 'FontSize', axisFontSize);
grid on;
legend(arrayfun(@(i) sprintf('$\\theta_{%d}$', i), 1:M, 'UniformOutput', false), ...
    'Location', legendLocation, 'FontSize', axisFontSize, 'Interpreter', 'latex');
set(gca, 'FontSize', axisFontSize);
xlim([0, sim_time]);

%% 3) Control inputs (sparse)
figure('Position', figPos, 'Color', 'w');
time_u = time(1:end-1);
stairs(time_u, u_hist', 'LineWidth', 1.5);
xlabel('$t\,[\mathrm{s}]$', 'FontSize', axisFontSize, 'Interpreter', 'latex');
ylabel('$u_i\,[\mathrm{N}]$', 'FontSize', axisFontSize, 'Interpreter', 'latex');
title('Control Input Forces', 'FontSize', axisFontSize);
grid on;
legend(arrayfun(@(i) sprintf('$u_{%d}$', i), 1:M, 'UniformOutput', false), ...
    'Location', legendLocation, 'FontSize', axisFontSize, 'Interpreter', 'latex');
set(gca, 'FontSize', axisFontSize);
xlim([0, sim_time]);

%% 4) Lyapunov Energy + Decay Rate Analysis (2-subplot)
% Compute Lyapunov energy: V(x) = x^T P x
P = ctrl.P;
energy_hist = zeros(total_steps+1, 1);
for k = 1:(total_steps+1)
    energy_hist(k) = x_hist(:,k)' * P * x_hist(:,k);
end

figure('Position', figPos, 'Color', 'w');

% Subplot 1: Lyapunov Energy
subplot(1, 2, 1);
semilogy(time, energy_hist, 'b-', 'LineWidth', 2);
hold on;
plot(time(1), energy_hist(1), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'g');
plot(time(end), energy_hist(end), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'r');
xlabel('$t\,[\mathrm{s}]$', 'FontSize', axisFontSize, 'Interpreter', 'latex');
ylabel('$x^\top P x$', 'FontSize', axisFontSize, 'Interpreter', 'latex');
title('Lyapunov Energy', 'FontSize', axisFontSize);
legend('Energy', 'Initial', 'Final', 'Location', 'best', ...
    'FontSize', axisFontSize-2, 'Interpreter', 'latex');
grid on;
set(gca, 'FontSize', axisFontSize);
xlim([0, sim_time]);

% Subplot 2: Decay Rate Analysis
subplot(1, 2, 2);
log_energy = log10(energy_hist);
plot(time, log_energy, 'r-', 'LineWidth', 2);
hold on;

% Fit linear trend to compute decay rate
p = polyfit(time, log_energy', 1);
decay_rate = p(1);
trend_line = polyval(p, time);
plot(time, trend_line, 'k--', 'LineWidth', 2);

xlabel('$t\,[\mathrm{s}]$', 'FontSize', axisFontSize, 'Interpreter', 'latex');
ylabel('$\log_{10}(x^\top P x)$', 'FontSize', axisFontSize, 'Interpreter', 'latex');
title('Decay Rate Analysis', 'FontSize', axisFontSize);

% Annotate with decay rate and stability verdict
text_str = sprintf('Rate: %.4f decade/s', decay_rate);
if decay_rate < 0
    verdict_color = [0, 0.6, 0];
    verdict = 'STABLE';
else
    verdict_color = [0.8, 0, 0];
    verdict = 'UNSTABLE';
end
text(0.5, 0.95, text_str, 'Units', 'normalized', 'FontSize', axisFontSize-2, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
text(0.5, 0.85, verdict, 'Units', 'normalized', 'FontSize', axisFontSize, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
    'FontWeight', 'bold', 'Color', verdict_color);

legend('$\log_{10}$ Energy', 'Linear Fit', 'Interpreter', 'latex', ...
    'Location', 'best', 'FontSize', axisFontSize-2);
grid on;
set(gca, 'FontSize', axisFontSize);
xlim([0, sim_time]);

fprintf('✓ Generated 4 plots\n');

end
