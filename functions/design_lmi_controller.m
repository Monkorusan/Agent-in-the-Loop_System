function [ctrl] = design_lmi_controller(sys, config)
% DESIGN_LMI_CONTROLLER Computes controller via LMI approach (Paper's Equation 5)
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

M = sys.nu; % num_cart M = dim input u:= nu holds only if we apply input in 1DOF to each carts (in this case, ctrl input is only in x direction)
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

% Define LMI variables
W = sdpvar(nx,nx,'symmetric'); % inverse of P            
Y = sdpvar(nu,nx,'full');      % used in the change of variable 

% LMI constraint (Equation 5 from paper):
% [W,                           W*A_tilde' + Y'*B_tilde',    W*Q^(1/2),     Y'*R^(1/2);
%  A_tilde*W + B_tilde*Y,      W,                            0,             0;
%  Q^(1/2)*W,                   0,                            I,             0;
%  R^(1/2)*Y,                   0,                            0,             I] >= 0

Q_sqrt = sqrtm(Q);
R_sqrt = sqrtm(R);

LMI_matrix = ...
[          W           , W*A_tilde' + Y'*B_tilde' ,  W*Q_sqrt   ,  Y'*R_sqrt ;
 A_tilde*W + B_tilde*Y ,            W             , zeros(nx,nx)' , zeros(nu,nx)' ;
 Q_sqrt*W             ,        zeros(nx,nx)      ,    eye(nx)   , zeros(nu,nx)' ;
 R_sqrt*Y             ,        zeros(nu,nx)      , zeros(nu,nx) ,   eye(nu)   ];

Constraints = LMI_matrix >=0;
% Constraints = [LMI_matrix >=0, W >= 1e-6*eye(nx)]; 
Objective = -trace(W);

options = sdpsettings('verbose', 1, 'solver', 'mosek'); %requires MOSEK
solution = optimize(Constraints, Objective, options);

if solution.problem == 0
    fprintf('✓ LMI solved successfully\n');
else
    fprintf('✗ Solver failed: %s (code: %d)\n', solution.info, solution.problem);
end

% a fallback2LQR version if LMI is infeasible
if isempty(solution) || solution.problem ~= 0
    warning('LMI infeasible , falling back to DARE...');
    [P_dare, ~, K_tilde] = idare(A_tilde, B_tilde, Q, R);
    K_tilde = -K_tilde;
    
    ctrl = struct();
    ctrl.K_tilde = K_tilde;
    ctrl.P = P_dare;
    ctrl.Q = Q;
    ctrl.R = R;
    ctrl.method = 'AITL_LMI_FALLBACK_iDARE';
    ctrl.W_sol = [];
    ctrl.Y_sol = [];
    
    fprintf('⚠ Using DARE fallback controller\n');
    return;
end

% Extract solutions
W_sol = value(W);
Y_sol = value(Y);

% Compute controller gain
K_tilde = Y_sol / W_sol;

fprintf('✓ LMI solved successfully\n');
fprintf('  Controller gain: %dx%d\n', size(K_tilde, 1), size(K_tilde, 2));
fprintf('  Max |K|: %.4f\n', max(abs(K_tilde(:))));
fprintf('  Objective value max trace inverse P: %.6f\n', value(Objective));

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
