function [ctrl] = design_lmi_controller(sys, config)
% DESIGN_LMI_CONTROLLER Computes controller via LMI approach (Paper Equation 5)
%
% Inputs:
%   sys    - System structure (from build_aitl_system)
%   config - Configuration structure containing:
%            .Q_diag - State cost diagonal elements [4 x 1] per cart
%            .R_diag - Control cost (scalar or [M x 1])
%
% Outputs:
%   ctrl - Controller structure:
%          .K_tilde - Feedback gain matrix [nu x nx]
%          .P       - Solution to LMI (W inverse) [nx x nx]
%          .Q       - State cost matrix [nx x nx]
%          .R       - Control cost matrix [nu x nu]
%          .method  - 'LMI' string identifier
%          .W_sol   - LMI solution W matrix
%          .Y_sol   - LMI solution Y matrix

M = sys.nu;
nx = sys.nx;
nu = sys.nu;
A_tilde = sys.A_tilde;
B_tilde = sys.B_tilde;

%% Build cost matrices
Q_diag = config.Q_diag;
R_diag = config.R_diag;

% Q matrix: replicate per-cart cost
Q = kron(eye(M), diag(Q_diag));

% R matrix
if isscalar(R_diag)
    R = R_diag * eye(M);
else
    R = diag(R_diag);
end

%% Display info
fprintf('\n=== Designing LMI Controller ===\n');
fprintf('Using LMI approach (Paper Equation 5)\n');
fprintf('Designing on (A_tilde, B_tilde) - resampled system with built-in constraints\n');
fprintf('Note: B_tilde = B_hat * Pw already incorporates the walk structure\n');
fprintf('System dimensions: A_tilde (%dx%d), B_tilde (%dx%d)\n', ...
    size(A_tilde, 1), size(A_tilde, 2), size(B_tilde, 1), size(B_tilde, 2));

%% System analysis
fprintf('Checking system properties...\n');

% Eigenvalue analysis
unstable_eigs = eig(A_tilde);
unstable_count = sum(abs(unstable_eigs) >= 1);
fprintf('  A_tilde has %d unstable eigenvalues (|eig| >= 1)\n', unstable_count);
fprintf('  Max |eig(A_tilde)|: %.4f\n', max(abs(unstable_eigs)));

% Controllability
rank_AB = rank(ctrb(A_tilde, B_tilde));
fprintf('  Controllability rank: %d / %d\n', rank_AB, nx);
if rank_AB < nx
    warning('System is not fully controllable! LMI may be infeasible.');
end

% Condition numbers
cond_A = cond(A_tilde);
cond_B = cond(B_tilde);
fprintf('  Condition number of A_tilde: %.2e\n', cond_A);
fprintf('  Condition number of B_tilde: %.2e\n', cond_B);
if cond_A > 1e10 || cond_B > 1e10
    warning('Matrices are ill-conditioned! This may cause numerical issues.');
end

%% Setup LMI problem
fprintf('Setting up LMI...\n');

Q_sqrt = sqrtm(Q);
R_sqrt = sqrtm(R);

% Initialize LMI system
setlmis([]);

% Define LMI variables
W = lmivar(1, [nx 1]);              % symmetric matrix (nx x nx)
Y = lmivar(2, [nu nx]);             % full matrix (nu x nx)

% LMI constraint (Equation 5 from paper, adapted for B_tilde):
% [W,                           W*A_tilde' + Y'*B_tilde',    W*Q^(1/2),     Y'*R^(1/2);
%  A_tilde*W + B_tilde*Y,      W,                            0,             0;
%  Q^(1/2)*W,                   0,                            I,             0;
%  R^(1/2)*Y,                   0,                            0,             I] >= 0
%
% notice how LMI toolbox encodes constraints as L(x) < 0. So to get M >= 0, write -M < 0.

lmi1 = newlmi;

% Block (1,1): -W
lmiterm([lmi1 1 1 W], -1, 1);

% Block (2,1): -(A_tilde*W + B_tilde*Y)  
lmiterm([lmi1 2 1 W], -A_tilde, 1);
lmiterm([lmi1 2 1 Y], -B_tilde, 1);

% Block (1,3): -W*Q_sqrt
lmiterm([lmi1 1 3 W], -1, Q_sqrt);

% Block (3,1): -Q_sqrt*W (symmetric)
lmiterm([lmi1 3 1 W], -Q_sqrt, 1);

% Block (4,1): -R_sqrt*Y
lmiterm([lmi1 4 1 Y], -R_sqrt, 1);

% Block (2,2): -W
lmiterm([lmi1 2 2 W], -1, 1);

% Block (3,3): -I
lmiterm([lmi1 3 3 0], -eye(nx));

% Block (4,4): -I
lmiterm([lmi1 4 4 0], -eye(nu));

% Get the LMI system
lmisys = getlmis;
n_dec = decnbr(lmisys);
fprintf('LMI system has %d decision variables\n', n_dec);
fprintf('Y dimensions: %d x %d (control gain for resampled system)\n', nu, nx);

%% Objective: maximize trace(W) = minimize -trace(W)
cost = zeros(n_dec, 1);
for m = 1:n_dec
    W_m = defcx(lmisys, m, W);
    cost(m) = -trace(W_m);
end
fprintf('Cost vector has %d non-zero elements (trace objective)\n', sum(cost ~= 0));

%% Solve LMI
fprintf('Solving LMI...\n');

% Step 1: Check feasibility
options_feasp = [0, 300, -1, 5, 0];  % -1 = flexible feasibility radius
[tmin, xfeas] = feasp(lmisys, options_feasp);

if isempty(xfeas) || tmin > 0
    % LMI infeasible, fallback to DARE
    warning('LMI infeasible (tmin = %.4f), falling back to DARE...', tmin);
    [P_dare, ~, K_tilde] = dare(A_tilde, B_tilde, Q, R);
    K_tilde = -K_tilde;
    
    ctrl = struct();
    ctrl.K_tilde = K_tilde;
    ctrl.P = P_dare;
    ctrl.Q = Q;
    ctrl.R = R;
    ctrl.method = 'AITL_LMI_FALLBACK_DARE';
    ctrl.W_sol = [];
    ctrl.Y_sol = [];
    
    fprintf('⚠ Using DARE fallback controller\n');
    return;
end

% Step 2: Optimize
fprintf('LMI feasible, optimizing...\n');
[copt, xopt] = mincx(lmisys, cost);

if isempty(xopt)
    fprintf('Optimization failed, using feasible point\n');
    xopt = xfeas;
end

% Extract solutions
W_sol = dec2mat(lmisys, xopt, W);
Y_sol = dec2mat(lmisys, xopt, Y);

% Compute controller gain
K_tilde = Y_sol / W_sol;

fprintf('✓ LMI solved successfully\n');
fprintf('  Controller gain: %dx%d\n', size(K_tilde, 1), size(K_tilde, 2));
fprintf('  Max |K|: %.4f\n', max(abs(K_tilde(:))));

%% Stability check: test both sign conventions
fprintf('Checking closed-loop stability...\n');

eigs_plus  = eig(A_tilde + B_tilde * K_tilde);
eigs_minus = eig(A_tilde - B_tilde * K_tilde);

max_eig_plus = max(abs(eigs_plus));
max_eig_minus = max(abs(eigs_minus));

fprintf('  Max |eig(A_tilde + B*K)|: %.4f\n', max_eig_plus);
fprintf('  Max |eig(A_tilde - B*K)|: %.4f\n', max_eig_minus);

% Choose sign convention that stabilizes
if max_eig_minus < 1 && max_eig_minus < max_eig_plus
    K_tilde = -K_tilde;
    fprintf('  → Using negative feedback (u = -K*x)\n');
elseif max_eig_plus < 1
    fprintf('  → Using positive feedback (u = K*x)\n');
else
    warning('Closed-loop unstable under both sign conventions!');
    fprintf('  → Keeping original sign\n');
end

%% Package outputs
ctrl = struct();
ctrl.K_tilde = K_tilde;
ctrl.P = inv(W_sol);  % P = W^(-1)
ctrl.Q = Q;
ctrl.R = R;
ctrl.method = 'AITL-LMI';
ctrl.W_sol = W_sol;
ctrl.Y_sol = Y_sol;

fprintf('✓ LMI controller design complete\n');

end
