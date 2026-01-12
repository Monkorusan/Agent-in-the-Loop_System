function [simdata] = simulate_aitl(sys, ctrl, config)
% Run the closed-loop AITL simulation
%
% Inputs:
%   sys    - System structure (from build_aitl_system)
%   ctrl   - Controller structure (from design_*_controller)
%   config - Configuration structure:
%            .simulation.duration   - Simulation time (seconds)
%            .simulation.x0         - Initial state [nx x 1]
%            .walk                  - Agent walk sequence
%
% Outputs:
%   simdata - Simulation data structure:
%             .x_hist         - State history [nx x (steps+1)]
%             .u_hist         - Control history [nu x steps]
%             .agent_location - Agent position at each step [steps x 1]
%             .time           - Time vector [1 x (steps+1)]
%             .total_steps    - Number of simulation steps
%             .num_cycles     - Number of complete walk cycles

%% Extract parameters
A = sys.A;
B = sys.B;
nx = sys.nx;
nu = sys.nu;
dt = sys.dt;
N_walk = sys.N_walk;
walk = config.walk;
K_tilde = ctrl.K_tilde;

sim_time = config.simulation.duration;
x0 = config.simulation.x0;
M = nu;

%% Setup simulation
num_cycles = ceil(sim_time / (N_walk * dt));
total_steps = num_cycles * N_walk;

fprintf('\n=== Running Simulation ===\n');
fprintf('Duration: %.1f seconds\n', sim_time);
fprintf('Cycles: %d | Steps: %d | dt: %.3f s\n', num_cycles, total_steps, dt);

%% Allocate storage
x_hist = zeros(nx, total_steps + 1);
u_hist = zeros(nu, total_steps);
agent_location = zeros(total_steps, 1);
time = (0:total_steps) * dt;

%% Initialize
x_hist(:, 1) = x0;
x_current = x0;
u_resampled = zeros(nu, 1);

%% Simulation loop
for k = 1:total_steps
    % Determine agent location
    step_in_cycle = mod(k-1, N_walk) + 1;
    active_cart = walk(step_in_cycle);
    agent_location(k) = active_cart;
    
    % Compute control at start of each cycle
    if step_in_cycle == 1
        u_resampled = K_tilde * x_current;
    end
    
    % Apply control only to active cart
    u_apply = zeros(nu, 1);
    u_apply(active_cart) = u_resampled(active_cart);
    
    % Store control
    u_hist(:, k) = u_apply;
    
    % Propagate dynamics
    x_current = A * x_current + B * u_apply;
    x_hist(:, k+1) = x_current;
    
    % Progress reporting
    if mod(k, 50) == 0
        max_x = max(abs(x_hist((0:M-1)*4 + 3, k+1)));
        max_theta = max(abs(x_hist((0:M-1)*4 + 1, k+1)));
        fprintf('  Step %3d: max|x|=%.4f m, max|theta|=%.4f rad (%.2f°)\n', ...
            k, max_x, max_theta, rad2deg(max_theta));
    end
end

fprintf('✓ Simulation complete\n');

%% Package outputs
simdata = struct();
simdata.x_hist = x_hist;
simdata.u_hist = u_hist;
simdata.agent_location = agent_location;
simdata.time = time;
simdata.total_steps = total_steps;
simdata.num_cycles = num_cycles;
simdata.M = M;
simdata.dt = dt;

%% Extract per-cart states for convenience
simdata.theta_hist = zeros(M, total_steps+1);
simdata.x_pos_hist = zeros(M, total_steps+1);
for i = 1:M
    simdata.theta_hist(i, :) = x_hist((i-1)*4 + 1, :);
    simdata.x_pos_hist(i, :) = x_hist((i-1)*4 + 3, :);
end

%% Final statistics
fprintf('\n=== Final State ===\n');
fprintf('Max |x_i|: %.4f m\n', max(abs(simdata.x_pos_hist(:,end))));
fprintf('Max |theta_i|: %.4f rad (%.2f°)\n', ...
    max(abs(simdata.theta_hist(:,end))), ...
    rad2deg(max(abs(simdata.theta_hist(:,end)))));

end
