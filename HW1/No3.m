clear;
clc;
close all;

fprintf('=====================================================\n');
fprintf('[Solution] Part a: Rotation Matrix Result\n');
fprintf('=====================================================\n');

% Define Euler angles in radians as per the problem statement.
% Roll (phi, around X-axis): pi/3
% Pitch (theta, around Y-axis): -pi/4
% Yaw (psi, around Z-axis): pi/2
roll  = pi/3;
pitch = -pi/4;
yaw   = pi/2;

% Using the standard ZYX intrinsic rotation convention.
% The angles are provided to eul2rotm in the [Yaw, Pitch, Roll] order
% to match the 'ZYX' sequence.
eul_angles = [yaw, pitch, roll];

% Calculate the rotation matrix using the built-in MATLAB function.
R = eul2rotm(eul_angles, 'ZYX');

% Report the code logic and answer in the command window.
fprintf('Code executed: R = eul2rotm([pi/2, -pi/4, pi/3], ''ZYX'');\n\n');
fprintf('The resulting rotation matrix R is:\n');
disp(R);



fprintf('\n=====================================================\n');
fprintf('[Solution] Part b: Rotated Vector Coordinates and Plot\n');
fprintf('=====================================================\n');

% Define the original vector p.
p = [1; 2; 3];

% Apply the rotation R to get the new vector p1.
p1 = R * p;

% Report the coordinates of the new vector p1 in the command window.
fprintf('Applying the rotation R to vector p = [1; 2; 3]...\n\n');
fprintf('The coordinates of the new vector p1 are:\n');
disp(p1);

% --- Vector Visualization ---

% Create a new figure window with a white background.
figure('Name', 'Vector Rotation Visualization', 'Color', 'w');
hold on; % Allow multiple plots on the same axes.

% Plot the original vector p (blue solid line with circle markers).
plot3([0, p(1)], [0, p(2)], [0, p(3)], ...
      'b-o', 'LineWidth', 2, 'MarkerFaceColor', 'b', 'MarkerSize', 8, ...
      'DisplayName', 'Original Vector p = [1; 2; 3]');

% Plot the rotated vector p1 (red dashed line with square markers).
plot3([0, p1(1)], [0, p1(2)], [0, p1(3)], ...
      'r--s', 'LineWidth', 2, 'MarkerFaceColor', 'r', 'MarkerSize', 8, ...
      'DisplayName', 'Rotated Vector p1');

% --- Plot Formatting and Final Touches ---

% Add title and axis labels with specified font properties.
title('Rotation of a 3D Vector', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('X-axis', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y-axis', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('Z-axis', 'FontSize', 12, 'FontWeight', 'bold');

% Display the legend at the best location.
legend('show', 'Location', 'best', 'FontSize', 11);

% Enable the grid and draw a box around the plot for better 3D perception.
grid on;
box on;

axis equal; 

% Set a clear 3D viewing angle.
view(135, 30); 

hold off; 