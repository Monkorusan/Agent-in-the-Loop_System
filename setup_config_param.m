function [config] = setup_config_param()
% Returns the default config structure with standard parameters for:
% - System dynamics (masses, stiffness, damping, etc.)
% - Simulation settings (duration, initial conditions)
% - Visualization settings (plots, animation)

config = struct();

%% System parameters
config.M = 5;                               % Number of cart-pendulum subsystems
config.mi = [7.0, 6.0, 6.0, 8.0, 5.0, 7.0, 6.0, 6.0, 8.0, 5.0];     % Cart masses (kg)
config.mp = 1.0;                            % Pendulum mass (kg)
config.l = 1.0;                             % Pendulum length (m)
config.g = 9.81;                            % Gravitational acceleration (m/s^2)
config.kappa = 0.5;                         % Spring stiffness (N/m)
config.h = 0.2;                             % Damping coefficient (N/(m/s))
% config.kappa = 3;
% config.h = 3;
config.dt = 0.03;                           % Sampling time (s)
config.walk = 1:config.M;              % Agent walk sequence

%% reminder: not defining Q_dual and R_dual will let MATLAB use same value as primal

%% this part is only used in noteleport mode! (paper's extension)
config.topology = 'line';           % 'ring' (wrap) or 'line' (no wrap)
config.travel_steps_per_hop = 1;    % integer travel steps per edge (hop)
config.dwell_steps = 0;             % how many steps to stay on a cart

%% Controller costs
config.Q_diag = [10, 1, 300, 1];           % State cost per cart [theta, dtheta, x, dx]
config.R_diag = 1;                         % Control cost (scalar or vector)

%% Greedy control settings (paper's extension)
config.N_pred = 1;                         %for rollout policy
config.D_dwell = 1;                        % Dwell time: commit to selected cart for D steps (1 = re-evaluate every step)

%% LMI-based Common Lyapunov Function (for greedy_rerouting.m as a paper's extension)
% Toggle between DARE-based P (heuristic) and LMI-synthesized CQLF
config.use_lmi_cqlf = true;                % If true, compute P via LMI synthesis; if false, use DARE
config.use_lmi_proxy_gains = true;         % If true, use K_i = Y_i * P from LMI (rigorous)
                                           % Only active when use_lmi_cqlf = true

%% Stability Analysis (Gramian Check in greedy_rerouting.m as a paper's extension)
config.check_gramian = true;                % Enable Gramian analysis
config.gramian_threshold = 1e-6;            % Minimum eigenvalue threshold for controllability

%% Simulation settings
config.simulation = struct();
config.simulation.duration = 7.5;           % Simulation time (seconds)

% Initial conditions
rng(42);  
M = config.M;
x0 = zeros(4*M, 1);
for i = 1:M
    theta_i = (rand - 0.5) * 2 * pi/10;     % Random angle ±36°
    % theta_i = (rand - 0.5) * 2 * pi/50;   % if use super small initial condition
    x_i = (rand - 0.5) * 0.6;               % Random position ±0.3 m
    x0((i-1)*4 + 1) = theta_i;              % theta_i
    x0((i-1)*4 + 2) = 0;                    % dtheta_i
    x0((i-1)*4 + 3) = x_i;                  % x_i
    x0((i-1)*4 + 4) = 0;                    % dx_i
end
config.simulation.x0 = x0;

%% Visualization settings
config.visualization = struct();
config.visualization.showPlots = true;                 % Enable plots
config.visualization.showAnimation = true;             % Enable animation
config.visualization.saveAnimation = false;             % Save video file
config.visualization.videoFilename = 'simulation.mp4'; % Default filename
config.visualization.figPos = [100, 100, 900, 600];    % [x, y, width, height]
config.visualization.axisFontSize = 15;                % Font size for axes
config.visualization.legendLocation = 'best';          % Legend position

end
