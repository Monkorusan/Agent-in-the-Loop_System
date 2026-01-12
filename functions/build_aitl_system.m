function [sys] = build_aitl_system(config)
% BUILD_AITL_SYSTEM Constructs the discrete-time AITL system matrices
% featured in section IV:case study from the paper
%
% Inputs:
%   config - Structure containing system parameters:
%            .M      - Number of cart-pendulum subsystems
%            .mi     - Cart masses [M x 1]
%            .mp     - Pendulum mass (kg)
%            .l      - Pendulum length (m)
%            .g      - Gravitational acceleration (m/s^2)
%            .kappa  - Spring stiffness (N/m)
%            .h      - Damping coefficient (N/(m/s))
%            .dt     - Sampling time (s)
%            .walk   - Agent walk sequence [1 x N_walk]
%
% Outputs:
%   sys - Structure containing:
%         .A         - Discrete-time state matrix [nx x nx]
%         .B         - Discrete-time input matrix [nx x nu]
%         .A_tilde   - Resampled state matrix [nx x nx]
%         .B_tilde   - Resampled input matrix [nx x nu]
%         .P_w       - Agent walk matrix [nu*N x nu]
%         .nx        - Number of states
%         .nu        - Number of inputs
%         .M         - Number of carts
%         .walk      - Agent walk sequence
%         .N_walk    - Walk length
%         .dt        - Sampling time

M = config.M;
mi = config.mi;
mp = config.mp;
l = config.l;
g = config.g;
kappa = config.kappa;
h = config.h;
dt = config.dt;
walk = config.walk;

%% Compute per-cart coefficients
C  = zeros(M,1);
C1 = zeros(M,1);
C2 = zeros(M,1);
C3 = zeros(M,1);
I = 0.0;  % Pendulum moment of inertia: same value as paper

for i = 1:M
    C(i)  = 1/((mi(i)+mp)*(I+mp*l^2)-(mp*l)^2);
    C1(i) = (I+mp*l^2)*C(i);
    C2(i) = -mp*l*C(i);
    C3(i) = (mi(i)+mp)*C(i);
end

%% Build Continuous-Time System
N = M;
Z = zeros(N);
I_n = eye(N);

% Coupling matrices
K  = diag(-2*kappa.*C1) + diag(kappa.*C1(1:end-1), 1) + diag(kappa.*C1(2:end), -1);
H  = diag(-2*h.*C1) + diag(h.*C1(1:end-1), 1) + diag(h.*C1(2:end), -1);
Kth = diag(-2*kappa.*C2) + diag(kappa.*C2(1:end-1), 1) + diag(kappa.*C2(2:end), -1);
Hth = diag(-2*h.*C2) + diag(h.*C2(1:end-1), 1) + diag(h.*C2(2:end), -1);
Gx  = diag(g*mp.*C2);
Gth = diag(g*mp.*C3);

% Continuous dynamics
A_temp = [ Z    Z    I_n  Z;
           Z    Z    Z    I_n;
           K    Gx   H    Z;
           Kth  Gth  Hth  Z ];

B_temp = [ zeros(N, N);
           zeros(N, N);
           diag(C1);
           diag(C2) ];

%% so far, state vector is [ x_i    , theta_i  , dx_i , dtheta_i] for each i
%% Reorder to match paper: [theta_i , dtheta_i , x_i  ,   dx_i  ] for each i
nx = 4*M;
nu = M;

P_reorder = zeros(nx, nx);
for i = 1:M
    P_reorder((i-1)*4 + 1, M + i) = 1;
    P_reorder((i-1)*4 + 2, 3*M + i) = 1;
    P_reorder((i-1)*4 + 3, i) = 1;
    P_reorder((i-1)*4 + 4, 2*M + i) = 1;
end

A_cont = P_reorder * A_temp * P_reorder';
B_cont = P_reorder * B_temp;

%% Discretize System
M_aug = [A_cont, B_cont; zeros(nu, nx+nu)];
M_disc = expm(M_aug * dt);
A = M_disc(1:nx, 1:nx);
B = M_disc(1:nx, nx+1:nx+nu);

%% Build Resampled System
N_walk = length(walk);
A_tilde = A^N_walk;

% B_hat= [A^(N-1)*B, A^(N-2)*B, ..., A*B, B]
B_hat = zeros(nx, nu * N_walk);
for i = 1:N_walk
    power_idx = N_walk - i;
    B_hat(:, (i-1)*nu + 1 : i*nu) = A^power_idx * B;
end

% Agent walk matrix P_w
P_w = zeros(nu * N_walk, nu);
for i = 1:N_walk
    cart_idx = walk(i);
    P_w((i-1)*nu + cart_idx, cart_idx) = 1;
end

B_tilde = B_hat * P_w;

%% Package outputs
sys = struct();
sys.A = A;
sys.B = B;
sys.A_tilde = A_tilde;
sys.B_tilde = B_tilde;
sys.P_w = P_w;
sys.nx = nx;
sys.nu = nu;
sys.M = M;
sys.walk = walk;
sys.N_walk = N_walk;
sys.dt = dt;

% only used in noteleport mode (paper's extension)
if isfield(config, 'cart_spacing')
    spacing = config.cart_spacing;
else
    spacing = 1.0; % Default distance between carts
end
sys.cart_anchors = (0 : config.M - 1) * spacing;

%% Display information
fprintf('=== System Built ===\n');
fprintf('Agent walk: %s (length N = %d)\n', mat2str(walk), N_walk);
fprintf('Agent visits one cart per time step (%.2f seconds)\n', dt);
fprintf('Complete cycle time: %.2f seconds\n', N_walk * dt);
fprintf('Resampled system dimensions: A_tilde %dx%d, B_tilde %dx%d\n', ...
    size(A_tilde,1), size(A_tilde,2), size(B_tilde,1), size(B_tilde,2));

rank_ctrb = rank(ctrb(A_tilde, B_tilde));
fprintf('Controllability rank: %d / %d\n', rank_ctrb, nx);

end
