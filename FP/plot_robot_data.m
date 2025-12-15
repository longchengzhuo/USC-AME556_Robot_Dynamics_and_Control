%% Plot Robot Data from CSV
% This script reads robot_data.csv and generates 8 figures showing:
% - Figures 1-7: Each DOF position and velocity
% - Figure 8: Four joint torques
%
% Note: Joint angles are negated for display (plot Y-axis is opposite to data)
% Note: Plot Y-axis represents MuJoCo Z-axis

clear; clc; close all;

%% Output settings
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
output_dir = fullfile(script_dir, 'task1_3');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
disp(['Saving figures to: ', output_dir]);

%% Read CSV data
data = readtable('task1_3/robot_data_task_1_3.csv');

% Extract time
t = data.time;

% Extract positions (7 DOF)
q_x = data.q_x;
q_y = data.q_z;           % MuJoCo z -> Plot y
q_pitch = data.q_pitch;
q_lh = -data.q_lh;        % Negate for display
q_lk = -data.q_lk;        % Negate for display
q_rh = -data.q_rh;        % Negate for display
q_rk = -data.q_rk;        % Negate for display

% Extract velocities (7 DOF)
v_x = data.v_x;
v_y = data.v_z;           % MuJoCo z -> Plot y
v_pitch = data.v_pitch;
v_lh = -data.v_lh;        % Negate for display
v_lk = -data.v_lk;        % Negate for display
v_rh = -data.v_rh;        % Negate for display
v_rk = -data.v_rk;        % Negate for display

% Extract torques (4 joints) - Negate to match angle sign convention
tau_lh = -data.tau_lh;
tau_lk = -data.tau_lk;
tau_rh = -data.tau_rh;
tau_rk = -data.tau_rk;

% Convert joint angles to degrees for display
q_lh_deg = rad2deg(q_lh);
q_lk_deg = rad2deg(q_lk);
q_rh_deg = rad2deg(q_rh);
q_rk_deg = rad2deg(q_rk);
q_pitch_deg = rad2deg(q_pitch);

%% Constraint limits
% Hip joints: angle [-120, 30] deg, velocity 30 rad/s, torque 30 Nm
% Knee joints: angle [0, 160] deg, velocity 15 rad/s, torque 60 Nm
HIP_ANGLE_MIN = -120;  % deg
HIP_ANGLE_MAX = 30;    % deg
HIP_VEL_LIMIT = 30;    % rad/s
HIP_TORQUE_LIMIT = 30; % Nm

KNEE_ANGLE_MIN = 0;    % deg
KNEE_ANGLE_MAX = 160;  % deg
KNEE_VEL_LIMIT = 15;   % rad/s
KNEE_TORQUE_LIMIT = 60; % Nm

%% Figure 1: Trunk X Position and Velocity
figure('Name', 'DOF 1: Trunk X', 'Position', [100, 100, 800, 500]);

subplot(2,1,1);
plot(t, q_x, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position (m)');
title('Trunk X Position');
grid on;

subplot(2,1,2);
plot(t, v_x, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Trunk X Velocity');
grid on;
print(gcf, fullfile(output_dir, 'fig1_trunk_x.png'), '-dpng', '-r150');

%% Figure 2: Trunk Y (MuJoCo Z) Position and Velocity
figure('Name', 'DOF 2: Trunk Y (Height)', 'Position', [150, 100, 800, 500]);

subplot(2,1,1);
plot(t, q_y, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position (m)');
title('Trunk Y Position (Height)');
grid on;

subplot(2,1,2);
plot(t, v_y, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Trunk Y Velocity');
grid on;
print(gcf, fullfile(output_dir, 'fig2_trunk_y.png'), '-dpng', '-r150');

%% Figure 3: Trunk Pitch Position and Velocity
figure('Name', 'DOF 3: Trunk Pitch', 'Position', [200, 100, 800, 500]);

subplot(2,1,1);
plot(t, q_pitch_deg, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Trunk Pitch Angle');
grid on;

subplot(2,1,2);
plot(t, v_pitch, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Trunk Pitch Velocity');
grid on;
print(gcf, fullfile(output_dir, 'fig3_trunk_pitch.png'), '-dpng', '-r150');

%% Figure 4: Left Hip Position and Velocity
figure('Name', 'DOF 4: Left Hip', 'Position', [250, 100, 800, 500]);

subplot(2,1,1);
plot(t, q_lh_deg, 'b', 'LineWidth', 1.5);
hold on;
yline(HIP_ANGLE_MIN, 'r--', 'LineWidth', 1.5, 'Label', 'Min Limit');
yline(HIP_ANGLE_MAX, 'r--', 'LineWidth', 1.5, 'Label', 'Max Limit');
hold off;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Left Hip Angle');
legend('Angle', 'Limits', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(t, v_lh, 'b', 'LineWidth', 1.5);
hold on;
yline(HIP_VEL_LIMIT, 'r--', 'LineWidth', 1.5);
yline(-HIP_VEL_LIMIT, 'r--', 'LineWidth', 1.5);
hold off;
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Left Hip Velocity');
legend('Velocity', 'Limits', 'Location', 'best');
grid on;
print(gcf, fullfile(output_dir, 'fig4_left_hip.png'), '-dpng', '-r150');

%% Figure 5: Left Knee Position and Velocity
figure('Name', 'DOF 5: Left Knee', 'Position', [300, 100, 800, 500]);

subplot(2,1,1);
plot(t, q_lk_deg, 'b', 'LineWidth', 1.5);
hold on;
yline(KNEE_ANGLE_MIN, 'r--', 'LineWidth', 1.5, 'Label', 'Min Limit');
yline(KNEE_ANGLE_MAX, 'r--', 'LineWidth', 1.5, 'Label', 'Max Limit');
hold off;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Left Knee Angle');
legend('Angle', 'Limits', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(t, v_lk, 'b', 'LineWidth', 1.5);
hold on;
yline(KNEE_VEL_LIMIT, 'r--', 'LineWidth', 1.5);
yline(-KNEE_VEL_LIMIT, 'r--', 'LineWidth', 1.5);
hold off;
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Left Knee Velocity');
legend('Velocity', 'Limits', 'Location', 'best');
grid on;
print(gcf, fullfile(output_dir, 'fig5_left_knee.png'), '-dpng', '-r150');

%% Figure 6: Right Hip Position and Velocity
figure('Name', 'DOF 6: Right Hip', 'Position', [350, 100, 800, 500]);

subplot(2,1,1);
plot(t, q_rh_deg, 'b', 'LineWidth', 1.5);
hold on;
yline(HIP_ANGLE_MIN, 'r--', 'LineWidth', 1.5, 'Label', 'Min Limit');
yline(HIP_ANGLE_MAX, 'r--', 'LineWidth', 1.5, 'Label', 'Max Limit');
hold off;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Right Hip Angle');
legend('Angle', 'Limits', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(t, v_rh, 'b', 'LineWidth', 1.5);
hold on;
yline(HIP_VEL_LIMIT, 'r--', 'LineWidth', 1.5);
yline(-HIP_VEL_LIMIT, 'r--', 'LineWidth', 1.5);
hold off;
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Right Hip Velocity');
legend('Velocity', 'Limits', 'Location', 'best');
grid on;
print(gcf, fullfile(output_dir, 'fig6_right_hip.png'), '-dpng', '-r150');

%% Figure 7: Right Knee Position and Velocity
figure('Name', 'DOF 7: Right Knee', 'Position', [400, 100, 800, 500]);

subplot(2,1,1);
plot(t, q_rk_deg, 'b', 'LineWidth', 1.5);
hold on;
yline(KNEE_ANGLE_MIN, 'r--', 'LineWidth', 1.5, 'Label', 'Min Limit');
yline(KNEE_ANGLE_MAX, 'r--', 'LineWidth', 1.5, 'Label', 'Max Limit');
hold off;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Right Knee Angle');
legend('Angle', 'Limits', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(t, v_rk, 'b', 'LineWidth', 1.5);
hold on;
yline(KNEE_VEL_LIMIT, 'r--', 'LineWidth', 1.5);
yline(-KNEE_VEL_LIMIT, 'r--', 'LineWidth', 1.5);
hold off;
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Right Knee Velocity');
legend('Velocity', 'Limits', 'Location', 'best');
grid on;
print(gcf, fullfile(output_dir, 'fig7_right_knee.png'), '-dpng', '-r150');

%% Figure 8: Joint Torques
figure('Name', 'Joint Torques', 'Position', [450, 100, 800, 700]);

subplot(4,1,1);
plot(t, tau_lh, 'b', 'LineWidth', 1.5);
hold on;
yline(HIP_TORQUE_LIMIT, 'r--', 'LineWidth', 1.5);
yline(-HIP_TORQUE_LIMIT, 'r--', 'LineWidth', 1.5);
hold off;
xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Left Hip Torque');
legend('Torque', 'Limits', 'Location', 'best');
grid on;

subplot(4,1,2);
plot(t, tau_lk, 'b', 'LineWidth', 1.5);
hold on;
yline(KNEE_TORQUE_LIMIT, 'r--', 'LineWidth', 1.5);
yline(-KNEE_TORQUE_LIMIT, 'r--', 'LineWidth', 1.5);
hold off;
xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Left Knee Torque');
legend('Torque', 'Limits', 'Location', 'best');
grid on;

subplot(4,1,3);
plot(t, tau_rh, 'b', 'LineWidth', 1.5);
hold on;
yline(HIP_TORQUE_LIMIT, 'r--', 'LineWidth', 1.5);
yline(-HIP_TORQUE_LIMIT, 'r--', 'LineWidth', 1.5);
hold off;
xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Right Hip Torque');
legend('Torque', 'Limits', 'Location', 'best');
grid on;

subplot(4,1,4);
plot(t, tau_rk, 'b', 'LineWidth', 1.5);
hold on;
yline(KNEE_TORQUE_LIMIT, 'r--', 'LineWidth', 1.5);
yline(-KNEE_TORQUE_LIMIT, 'r--', 'LineWidth', 1.5);
hold off;
xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Right Knee Torque');
legend('Torque', 'Limits', 'Location', 'best');
grid on;
print(gcf, fullfile(output_dir, 'fig8_joint_torques.png'), '-dpng', '-r150');

disp('All figures generated and saved successfully.');
disp(['Output directory: ', output_dir]);
