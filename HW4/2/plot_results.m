% Clear workspace, close all figures, and clear command window
clear;
clc;
close all;

% Define the filename
filename = 'robot_data.csv';

% Check if the file exists
if ~isfile(filename)
    fprintf('Error: File %s not found in the current directory.\n', filename);
    return;
end

% Read data from the CSV file using automatic header detection
try
    opts = detectImportOptions(filename);
    opts.VariableNamingRule = 'preserve'; % Keep original CSV header names
    data_table = readtable(filename, opts);
catch ME
    fprintf('Error reading the CSV file: %s\n', ME.message);
    return;
end

% --- Data Extraction and Transformation ---
% Extract data columns by name to ensure correctness
time = data_table.time;
x = data_table.x;
z = data_table.z;

% Negate theta and torque values as requested
theta = -data_table.theta;
tau1 = -data_table.tau1;
tau2 = -data_table.tau2;
tau3 = -data_table.tau3;
tau4 = -data_table.tau4;

% --- Plotting Position and Orientation ---
figure('Name', 'Robot State Analysis', 'NumberTitle', 'off');

% 1. Plot x position
subplot(3, 1, 1);
plot(time, x, 'b-', 'LineWidth', 1.5);
title('Trunk Horizontal Position (x)');
xlabel('Time (s)');
ylabel('Position (m)');
grid on;
legend('x(t)', 'Location', 'best');

% 2. Plot z position (labeled as y)
subplot(3, 1, 2);
plot(time, z, 'r-', 'LineWidth', 1.5);
title('Trunk Vertical Position (y)');
xlabel('Time (s)');
ylabel('Position (m)'); % Label z as y as requested
grid on;
legend('y(t)', 'Location', 'best');

% 3. Plot theta orientation
subplot(3, 1, 3);
plot(time, theta, 'g-', 'LineWidth', 1.5);
title('Trunk Orientation (\theta)');
xlabel('Time (s)');
ylabel('Angle (rad)');
grid on;
legend('\theta(t)', 'Location', 'best');

% --- Plotting Joint Torques ---
figure('Name', 'Joint Torque Analysis', 'NumberTitle', 'off');

% 1. Plot tau1
subplot(4, 1, 1);
plot(time, tau1, 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.5);
title('Joint Torque: \tau_1');
xlabel('Time (s)');
ylabel('Torque (Nm)');
grid on;
legend('\tau_1(t)', 'Location', 'best');

% 2. Plot tau2
subplot(4, 1, 2);
plot(time, tau2, 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 1.5);
title('Joint Torque: \tau_2');
xlabel('Time (s)');
ylabel('Torque (Nm)');
grid on;
legend('\tau_2(t)', 'Location', 'best');

% 3. Plot tau3
subplot(4, 1, 3);
plot(time, tau3, 'Color', [0.4660 0.6740 0.1880], 'LineWidth', 1.5);
title('Joint Torque: \tau_3');
xlabel('Time (s)');
ylabel('Torque (Nm)');
grid on;
legend('\tau_3(t)', 'Location', 'best');

% 4. Plot tau4
subplot(4, 1, 4);
plot(time, tau4, 'Color', [0.3010 0.7450 0.9330], 'LineWidth', 1.5);
title('Joint Torque: \tau_4');
xlabel('Time (s)');
ylabel('Torque (Nm)');
grid on;
legend('\tau_4(t)', 'Location', 'best');

fprintf('Successfully generated plots from %s.\n', filename);
