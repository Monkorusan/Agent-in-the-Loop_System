clear; clc; close all;

%% Add functions to path
addpath('functions');

%% ==================== CONFIGURATION ====================
config = setup_config_param();
config.visualization.videoFilename = 'AITL_LMI.mp4';

%% ==================== PIPELINE ====================
fprintf('\n');
fprintf('════════════════════════════════════════════════════════\n');
fprintf('     AITL-LMI Simulation Pipeline                       \n');
fprintf('════════════════════════════════════════════════════════\n');
fprintf('\n');

sys = build_aitl_system(config);
ctrl = design_lmi_controller(sys, config);
simdata = simulate_aitl(sys, ctrl, config);
plot_aitl_results(simdata, ctrl, config);

if config.visualization.showAnimation
    animate_aitl(simdata, ctrl, config);
end

%% ==================== SUMMARY ====================
fprintf('\n');
fprintf('════════════════════════════════════════════════════════\n');
fprintf('     Simulation Complete                                \n');
fprintf('════════════════════════════════════════════════════════\n');
fprintf('\nController: %s\n', ctrl.method);
fprintf('Agent completed %d cycles visiting %d carts\n', ...
    simdata.num_cycles, simdata.M);
fprintf('Each cart received control for %.2f seconds total\n', ...
    simdata.num_cycles * config.dt);
fprintf('\n✓ All tasks completed successfully!\n\n');
