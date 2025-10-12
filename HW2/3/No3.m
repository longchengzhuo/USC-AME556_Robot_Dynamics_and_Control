clear; clc; close all;

m = 0.5;      % mass (kg)
g = 9.81;     % gravity (m/s^2)
d = 0.2;      % arm length from center to motor (m)
r = 0.05;     % propeller radius (m)
I = diag([0.01, 0.01, 0.05]); % Moment of inertia matrix (kg.m^2)

tspan = [0 2]; % Simulate for 2 seconds

% 1. Initial position [x; y; z] (m)
p0 = [0.5; 0.5; 1];

% 2. Initial velocity [x_dot; y_dot; z_dot] (m/s)
v0 = [0; 0; 0];

% 3. Initial Euler angles [Roll; Pitch; Yaw] (rad)
euler0_rad = [pi/6; pi/8; pi/4];

% 4. Initial angular velocity in body frame [p; q; r] (rad/s)
omega0_b = [0; -0.1; 0.1];


R0 = eul2rotm(euler0_rad', 'ZYX');


x0 = [p0; R0(:); v0; omega0_b];


ode_fun = @(t, x) uav_dynamics(t, x, m, g, I);
[t, x_out] = ode45(ode_fun, tspan, x0);

% --- Post-processing: Extract results ---
% Position
pos_x = x_out(:, 1);
pos_y = x_out(:, 2);
pos_z = x_out(:, 3);

% Euler Angles
num_steps = length(t);
eulers = zeros(num_steps, 3);
for i = 1:num_steps
    R_vec = x_out(i, 4:12)';
    R = reshape(R_vec, 3, 3);
    eul_zyx = rotm2eul(R, 'ZYX');
    eulers(i, :) = [eul_zyx(3), eul_zyx(2), eul_zyx(1)]; % to Roll, Pitch, Yaw
end

% --- Plotting Results ---
% Plot 1: COM Position over time
figure('Name', 'UAV Position (COM)');
subplot(3,1,1); plot(t, pos_x, 'LineWidth', 1.5); grid on; title('X Position'); ylabel('x (m)');
subplot(3,1,2); plot(t, pos_y, 'LineWidth', 1.5); grid on; title('Y Position'); ylabel('y (m)');
subplot(3,1,3); plot(t, pos_z, 'LineWidth', 1.5); grid on; title('Z Position (Height)'); ylabel('z (m)');
xlabel('Time (s)'); sgtitle('Center of Mass Position vs. Time');

% Plot 2: Body Orientation (Euler Angles) over time
figure('Name', 'UAV Orientation (Euler Angles)');
subplot(3,1,1); plot(t, rad2deg(eulers(:,1)), 'r-', 'LineWidth', 1.5); grid on; title('Roll (\phi)'); ylabel('Angle (deg)');
subplot(3,1,2); plot(t, rad2deg(eulers(:,2)), 'g-', 'LineWidth', 1.5); grid on; title('Pitch (\theta)'); ylabel('Angle (deg)');
subplot(3,1,3); plot(t, rad2deg(eulers(:,3)), 'b-', 'LineWidth', 1.5); grid on; title('Yaw (\psi)'); ylabel('Angle (deg)');
xlabel('Time (s)'); sgtitle('Body Orientation (Euler Angles) vs. Time');

% --- Animation Generation ---
fprintf('Generating animation video... This may take a moment.\n');
anim(t, x_out, d, r);
fprintf('Video HW2_3.mp4 has been saved to your current directory.\n');


%%
function x_dot = uav_dynamics(~, x, m, g, I)
    % Unpack the 18x1 state vector
    % p = x(1:3);         % Position
    R_vec = x(4:12);    % Rotation matrix (vectorized)
    v = x(13:15);       % Velocity
    omega_b = x(16:18); % Angular velocity (body frame)
    R = reshape(R_vec, 3, 3);

    % Control Inputs (zero for this problem)
    T = 0; % Total thrust
    tau_b = [0; 0; 0]; % Torques in body frame

    % State Derivatives Calculation
    p_dot = v;
    S_omega = [  0,       -omega_b(3),  omega_b(2);
               omega_b(3),   0,        -omega_b(1);
              -omega_b(2), omega_b(1),     0      ];
    R_dot = R * S_omega;
    v_dot = [0; 0; -g] + (1/m) * R * [0; 0; T];
    omega_b_dot = I \ (tau_b - cross(omega_b, I * omega_b));

    % Assemble the output state derivative vector
    x_dot = [p_dot; R_dot(:); v_dot; omega_b_dot];
end

%%
function anim(t, x_state, d, r)
    Fs = 30; % Frame rate for the video
    [te, xe] = even_sample(t, x_state, Fs);

    fig_anim = figure('Name', 'UAV Animation', 'Position', [100 100 800 600]);
    axes1 = axes('Parent', fig_anim);
    
    x_min = min(xe(:,1)); x_max = max(xe(:,1));
    y_min = min(xe(:,2)); y_max = max(xe(:,2));
    z_min = min(xe(:,3)); z_max = max(xe(:,3));
    margin = 2 * (d + r); 
    axis_limits = [x_min-margin, x_max+margin, y_min-margin, y_max+margin, z_min-margin, z_max+margin];
    
    video_filename = 'HW2_3';
    v = VideoWriter(video_filename, 'MPEG-4');
    v.FrameRate = Fs;
    open(v);

    for k = 1:numel(te)
        drawone(axes1, xe(k,:), d, r, axis_limits, xe(1:k,:), te(k));   
        drawnow;
        frame = getframe(fig_anim);
        writeVideo(v, frame);
    end
    close(v);
end

%% Even Sampling Function
function [Et, Ex] = even_sample(t, x, Fs)
    t0 = t(1); tf = t(end);
    % Create an evenly spaced time vector
    Et = (t0 : 1/Fs : tf)';
    if Et(end) < tf
        Et(end+1) = tf;
    end
    % Interpolate the state data 'x' at the new time points 'Et'
    Ex = interp1(t, x, Et, 'linear');
end


function drawone(parent, x, d, r, axis_limits, trajectory, current_time)
    cla(parent); % Clear previous frame content
    hold(parent, 'on');

    % Unpack state
    p = x(1:3)';      % Position vector
    R = reshape(x(4:12), 3, 3); % Rotation matrix

    % Define UAV geometry in body frame
    arm1_b = [d; 0; 0]; arm2_b = [0; d; 0]; arm3_b = [-d; 0; 0]; arm4_b = [0; -d; 0];
    theta_prop = linspace(0, 2*pi, 25);
    prop_circle_b = [r*cos(theta_prop); r*sin(theta_prop); zeros(1, 25)];

    % Transform geometry to world frame
    arm1_w = p + R * arm1_b; arm2_w = p + R * arm2_b;
    arm3_w = p + R * arm3_b; arm4_w = p + R * arm4_b;
    prop1_w = p + R * (arm1_b + prop_circle_b); prop2_w = p + R * (arm2_b + prop_circle_b);
    prop3_w = p + R * (arm3_b + prop_circle_b); prop4_w = p + R * (arm4_b + prop_circle_b);

    % Draw the UAV Arms
    plot3(parent, [p(1), arm1_w(1)], [p(2), arm1_w(2)], [p(3), arm1_w(3)], 'r-', 'LineWidth', 2); % Red
    plot3(parent, [p(1), arm2_w(1)], [p(2), arm2_w(2)], [p(3), arm2_w(3)], 'g-', 'LineWidth', 2); % Green
    plot3(parent, [p(1), arm3_w(1)], [p(2), arm3_w(2)], [p(3), arm3_w(3)], 'k-', 'LineWidth', 2); % Black
    plot3(parent, [p(1), arm4_w(1)], [p(2), arm4_w(2)], [p(3), arm4_w(3)], 'b-', 'LineWidth', 2); % Blue
    
    % Draw Propellers
    plot3(parent, prop1_w(1,:), prop1_w(2,:), prop1_w(3,:), 'k-');
    plot3(parent, prop2_w(1,:), prop2_w(2,:), prop2_w(3,:), 'k-');
    plot3(parent, prop3_w(1,:), prop3_w(2,:), prop3_w(3,:), 'k-');
    plot3(parent, prop4_w(1,:), prop4_w(2,:), prop4_w(3,:), 'k-');
    
    % Draw trajectory path using the first 3 columns (x, y, z positions)
    plot3(parent, trajectory(:,1), trajectory(:,2), trajectory(:,3), 'k:');

    hold(parent, 'off');
    grid(parent, 'on');
    axis(parent, 'equal');
    axis(parent, axis_limits);
    view(parent, 30, 20); % 3D view angle
    xlabel(parent, 'X (m)'); ylabel(parent, 'Y (m)'); zlabel(parent, 'Z (m)');
    
    title(parent, sprintf('UAV Animation at t = %.2f s', current_time));
end