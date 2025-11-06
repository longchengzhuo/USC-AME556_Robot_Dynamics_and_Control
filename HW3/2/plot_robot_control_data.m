%% MATLAB script to plot robot simulation data from PD control

% Clear workspace, close all figures, and clear command window
clear; close all; clc;

% Define CSV file and import options
fileName = 'robot_control_data.csv';
opts = detectImportOptions(fileName);
opts.VariableNames = {'time', 'x', 'y', 'theta', 'q1', 'q2', 'q3', 'q4'};

% Load the data from the CSV file
data = readtable(fileName, opts);

% Extract data into variables
time = data.time;
x = data.x;
y = data.y;
theta = data.theta;
q1 = data.q1;
q2 = data.q2;
q3 = data.q3;
q4 = data.q4;

%% Figure 1: Plot x, y, and theta
figure('Name', 'Robot Position and Orientation (PD Control)', 'NumberTitle', 'off');

% Plot x(t)
subplot(3, 1, 1);
plot(time, x, 'r', 'LineWidth', 1.5);
title('X Position vs. Time');
xlabel('Time (s)');
ylabel('x (m)');
grid on;

% Plot y(t)
subplot(3, 1, 2);
plot(time, y, 'g', 'LineWidth', 1.5);
title('Y Position vs. Time');
xlabel('Time (s)');
ylabel('y (m)');
grid on;

% Plot theta(t)
subplot(3, 1, 3);
plot(time, theta, 'b', 'LineWidth', 1.5);
title('Theta (Body Pitch) vs. Time');
xlabel('Time (s)');
ylabel('theta (rad)');
grid on;

% Add a main title to the figure
sgtitle('Robot Position and Orientation (PD Control)');

%% Figure 2: Plot joint angles q1, q2, q3, q4
figure('Name', 'Robot Joint Angles (PD Control)', 'NumberTitle', 'off');

% Plot q1(t) - Left Hip
subplot(4, 1, 1);
plot(time, q1, 'r', 'LineWidth', 1.5);
title('Left Hip Angle (q1) vs. Time');
xlabel('Time (s)');
ylabel('Angle (rad)');
grid on;

% Plot q2(t) - Left Knee
subplot(4, 1, 2);
plot(time, q2, 'g', 'LineWidth', 1.5);
title('Left Knee Angle (q2) vs. Time');
xlabel('Time (s)');
ylabel('Angle (rad)');
grid on;

% Plot q3(t) - Right Hip
subplot(4, 1, 3);
plot(time, q3, 'b', 'LineWidth', 1.5);
title('Right Hip Angle (q3) vs. Time');
xlabel('Time (s)');
ylabel('Angle (rad)');
grid on;

% Plot q4(t) - Right Knee
subplot(4, 1, 4);
plot(time, q4, 'm', 'LineWidth', 1.5);
title('Right Knee Angle (q4) vs. Time');
xlabel('Time (s)');
ylabel('Angle (rad)');
grid on;

% Add a main title to the figure
sgtitle('Robot Joint Angles (PD Control)');

fprintf('MATLAB script finished. Displaying plots.\n');
