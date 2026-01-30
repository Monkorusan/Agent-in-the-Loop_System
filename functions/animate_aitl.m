function animate_aitl(simdata, ctrl, config)
% Creates animation of AITL cart-pendulum system
%
% Inputs:
%   simdata - Simulation data structure (from simulate_aitl)
%   ctrl    - Controller structure (for method name display)
%   config  - Configuration structure:
%             .visualization.showAnimation    - Enable animation (boolean)
%             .visualization.saveAnimation    - Save video (boolean)
%             .visualization.videoFilename    - Video filename
%             .visualization.saveGif          - Save GIF (boolean)
%             .visualization.gifFilename      - GIF filename
%             .visualization.gifDelay         - GIF frame delay (seconds), [] = auto
%             .visualization.figPos           - Figure position [x, y, w, h]
%             .visualization.axisFontSize     - Axis font size
%             .walk                           - Agent walk sequence

showAnimation = config.visualization.showAnimation;
saveVideo = config.visualization.saveAnimation;
saveGif = config.visualization.saveGif;

if ~showAnimation && ~saveVideo && ~saveGif
    fprintf('Animation disabled (showAnimation/saveAnimation/saveGif = false)\n');
    return;
end

%% Extract parameters
time = simdata.time;
x_pos_hist = simdata.x_pos_hist;
theta_hist = simdata.theta_hist;
u_hist = simdata.u_hist;
agent_location = simdata.agent_location;
total_steps = simdata.total_steps;
num_cycles = simdata.num_cycles;
M = simdata.M;
dt = simdata.dt;
N_walk = length(config.walk);
walk = config.walk;

figPos = config.visualization.figPos;
axisFontSize = config.visualization.axisFontSize;
videoFilename = config.visualization.videoFilename;
gifFilename = config.visualization.gifFilename;
gifDelay = config.visualization.gifDelay;

fprintf('\n=== Creating Animation ===\n');

%% Animation parameters
animate_speed = 1;  % real-time
skip_frames = max(1, round(1/(dt*animate_speed*30)));
frames_to_plot = 1:skip_frames:total_steps+1;

cart_width = 0.15;
cart_height = 0.08;
wheel_radius = 0.02;
spring_segments = 12;
spring_amplitude = 0.03;
l = config.l; 

%% Create figure
if showAnimation
    figAnim = figure('Position', figPos, 'Color', 'w');
else
    figAnim = figure('Position', figPos, 'Color', 'w', 'Visible', 'off');
end
figure(figAnim);  % Bring to front

% Determine plot range
x_min = min(x_pos_hist(:)) - 0.5;
x_max = max(x_pos_hist(:)) + 0.5;
x_range = x_max - x_min;
x_center = (x_max + x_min) / 2;
x_plot_range = max(x_range, 2.5);

ax = axes('Position', [0.05, 0.2, 0.9, 0.7]);
axis equal;
xlim([x_center - x_plot_range/2, x_center + x_plot_range/2]);
ylim([-0.3, l + 0.6]);
hold on; grid on;
xlabel('$x\,[\mathrm{m}]$', 'FontSize', axisFontSize, 'Interpreter', 'latex');
ylabel('$y\,[\mathrm{m}]$', 'FontSize', axisFontSize, 'Interpreter', 'latex');
title('Agent-in-the-Loop: Cart-Inverted Pendulum Chain', 'FontSize', axisFontSize);
set(gca, 'FontSize', axisFontSize);

% Ground line
plot(ax, [x_center - x_plot_range, x_center + x_plot_range], [0, 0], 'k-', 'LineWidth', 2);

colors = lines(M);

%% Setup video writer if requested
if saveVideo
    % Try MPEG-4 first, fall back to Motion JPEG AVI if not available
    try
        v = VideoWriter(videoFilename, 'MPEG-4');
        v.FrameRate = 30;
        open(v);
        fprintf('Recording video to: %s\n', videoFilename);
    catch
        % If MPEG-4 fails, try Motion JPEG AVI (more widely supported)
        [~, name, ~] = fileparts(videoFilename);
        videoFilename = [name '.avi'];
        v = VideoWriter(videoFilename, 'Motion JPEG AVI');
        v.FrameRate = 30;
        v.Quality = 95;
        open(v);
        fprintf('MPEG-4 not available, using Motion JPEG AVI instead\n');
        fprintf('Recording video to: %s\n', videoFilename);
    end
else
    v = [];
end

if saveGif
    [~, name, ext] = fileparts(gifFilename);
    if isempty(ext) || ~strcmpi(ext, '.gif')
        gifFilename = [name '.gif'];
    end
    if isempty(gifDelay) || gifDelay <= 0
        gifDelay = dt * skip_frames / animate_speed;
    end
    gifDelay = max(0.01, gifDelay);
    fprintf('Recording GIF to: %s\n', gifFilename);
    firstGifFrame = true;
else
    firstGifFrame = false;
end

%% Animation loop
fprintf('Animating %d frames...\n', length(frames_to_plot));
figure(figAnim);  % Force to front before animation starts
drawnow;  % Process pending graphics events

for frame_idx = 1:length(frames_to_plot)
    k = frames_to_plot(frame_idx);
    
    cla(ax);
    
    x_pos = x_pos_hist(:, k);
    theta = theta_hist(:, k);
    
    if k <= total_steps
        u_current = u_hist(:, k);
        current_agent_cart = agent_location(k);
    else
        u_current = zeros(M, 1);
        current_agent_cart = walk(end);
    end
    
    %% Draw springs and dampers between carts
    for i = 1:M-1
        x1 = x_pos(i) + cart_width/2;
        x2 = x_pos(i+1) - cart_width/2;
        y_spring = cart_height/2 + 0.02;
        
        % Spring (blue, wavy line)
        spring_x = linspace(x1, x2, spring_segments);
        spring_y = y_spring + spring_amplitude * sin(linspace(0, 4*pi, spring_segments));
        plot(ax, spring_x, spring_y, 'b-', 'LineWidth', 1.5);
        
        % Damper (red, straight line)
        y_damper = cart_height/2 - 0.02;
        plot(ax, [x1, x2], [y_damper, y_damper], 'r-', 'LineWidth', 2);
    end
    
    %% Draw each cart-pendulum
    for i = 1:M
        x_cart = x_pos(i);
        theta_i = theta(i);
        
        % Highlight cart if agent is present
        if i == current_agent_cart
            edge_color = [1, 0.5, 0];  % Orange highlight
            edge_width = 3;
        else
            edge_color = 'k';
            edge_width = 2;
        end
        
        % Cart body
        cart_x = [x_cart - cart_width/2, x_cart + cart_width/2, ...
                  x_cart + cart_width/2, x_cart - cart_width/2];
        cart_y = [0, 0, cart_height, cart_height];
        fill(ax, cart_x, cart_y, colors(i,:), 'EdgeColor', edge_color, 'LineWidth', edge_width);
        
        % Wheels
        wheel_offset = cart_width/3;
        rectangle('Position', [x_cart - wheel_offset - wheel_radius, 0, ...
                  2*wheel_radius, 2*wheel_radius], ...
            'Curvature', [1,1], 'EdgeColor', 'k', 'LineWidth', 1.5, 'Parent', ax);
        rectangle('Position', [x_cart + wheel_offset - wheel_radius, 0, ...
                  2*wheel_radius, 2*wheel_radius], ...
            'Curvature', [1,1], 'EdgeColor', 'k', 'LineWidth', 1.5, 'Parent', ax);
        
        % Pendulum rod
        pend_x_top = x_cart + l * sin(theta_i);
        pend_y_top = cart_height + l * cos(theta_i);
        plot(ax, [x_cart, pend_x_top], [cart_height, pend_y_top], ...
            'Color', colors(i,:), 'LineWidth', 3);
        
        % Pendulum mass (circle)
        mass_r = 0.05;
        rectangle('Position', [pend_x_top-mass_r, pend_y_top-mass_r, ...
                  2*mass_r, 2*mass_r], ...
            'Curvature', [1,1], 'EdgeColor', colors(i,:), 'LineWidth', 1, ...
            'FaceColor', colors(i,:), 'Parent', ax);
        
        % Control force arrow
        arrow_scale = 0.015;
        if abs(u_current(i)) > 0.5
            quiver(ax, x_cart, cart_height/2, u_current(i)*arrow_scale, 0, 0, ...
                'Color', [0.8,0,0], 'LineWidth', 2.5, 'MaxHeadSize', 0.5);
        end
        
        % Cart number label
        text(ax, x_cart, -0.12, sprintf('%d', i), ...
            'HorizontalAlignment', 'center', 'FontSize', axisFontSize-2, ...
            'Color', colors(i,:), 'FontWeight', 'bold');
    end
    
    %% Draw agent indicator
    if k <= total_steps
        agent_x = x_pos(current_agent_cart);
        agent_y = cart_height + 0.15;
        plot(ax, agent_x, agent_y, 'p', 'MarkerSize', 20, ...
            'MarkerFaceColor', [1, 0.5, 0], 'MarkerEdgeColor', 'k', 'LineWidth', 2);
        text(ax, agent_x, agent_y + 0.1, 'AGENT', ...
            'HorizontalAlignment', 'center', 'FontSize', axisFontSize-3, ...
            'FontWeight', 'bold', 'Color', [1, 0.5, 0]);
    end
    
    % Title with current state
    cycle_num = floor((k-1) / N_walk) + 1;
    title(ax, sprintf('%s | Time: %.2f s | Cycle: %d/%d | Agent at Cart: %d', ...
        ctrl.method, time(k), cycle_num, num_cycles, current_agent_cart), 'FontSize', axisFontSize+1);
    
    drawnow;
    
    % Save frame to video/GIF
    if ~isempty(v) || saveGif
        frame = getframe(figAnim);
        if ~isempty(v)
            writeVideo(v, frame);
        end
        if saveGif
            [img, cmap] = rgb2ind(frame2im(frame), 256);
            if firstGifFrame
                imwrite(img, cmap, gifFilename, 'gif', 'LoopCount', inf, 'DelayTime', gifDelay);
                firstGifFrame = false;
            else
                imwrite(img, cmap, gifFilename, 'gif', 'WriteMode', 'append', 'DelayTime', gifDelay);
            end
        end
    end
    
    % Pause for animation speed
    if showAnimation && frame_idx < length(frames_to_plot)
        pause(dt * skip_frames / animate_speed);
    end
end

%% Cleanup
if ~isempty(v)
    close(v);
    fprintf('✓ Video saved: %s\n', videoFilename);
end
if saveGif
    fprintf('✓ GIF saved: %s\n', gifFilename);
end

fprintf('✓ Animation complete\n');

end
