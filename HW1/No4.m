clear; clc; close all;

% --- Setup: Define Points and Axis Properties for Plotting ---
P0 = [0, 2, 0;  % X-coordinates for A, B, C
      0, 0, 1;  % Y-coordinates for A, B, C
      0, 0, 0]; % Z-coordinates for A, B, C

% Define properties for drawing the world frame axes
axis_len = 3; % Length of the axis lines
axis_width = 2; % LineWidth for prominence

% --- Part a: Original Triangle ---
figure('Name', 'Part (a): Original Triangle ABC');
hold on;
% Draw World Frame O_0X_0Y_0Z_0
plot3([0 axis_len], [0 0], [0 0], 'r', 'LineWidth', axis_width); % X-axis in Red
plot3([0 0], [0 axis_len], [0 0], 'g', 'LineWidth', axis_width); % Y-axis in Green
plot3([0 0], [0 0], [0 axis_len], 'b', 'LineWidth', axis_width); % Z-axis in Blue
% Draw the triangle
patch(P0(1,:), P0(2,:), P0(3,:), 'k', 'FaceAlpha', 0.5); % Black, semi-transparent
grid on; axis equal; view(3);
title('Part (a): Original Triangle in World Frame');
xlabel('X_0'); ylabel('Y_0'); zlabel('Z_0');
legend('X_0-axis', 'Y_0-axis', 'Z_0-axis', 'Original Triangle');
hold off;

% --- Part b: Rotation about X0 ---
theta_x = pi/6;
Rx = [1,0,0; 0,cos(theta_x),-sin(theta_x); 0,sin(theta_x),cos(theta_x)];
P1 = Rx * P0;
figure('Name', 'Part (b): Rotation about X0');
hold on;
plot3([0 axis_len], [0 0], [0 0], 'r', 'LineWidth', axis_width);
plot3([0 0], [0 axis_len], [0 0], 'g', 'LineWidth', axis_width);
plot3([0 0], [0 0], [0 axis_len], 'b', 'LineWidth', axis_width);
patch(P1(1,:), P1(2,:), P1(3,:), 'g', 'FaceAlpha', 0.5); % Green
grid on; axis equal; view(3);
title('Part (b): After Rotation about X_0-axis');
xlabel('X_0'); ylabel('Y_0'); zlabel('Z_0');
legend('X_0-axis', 'Y_0-axis', 'Z_0-axis', 'Triangle P1');
hold off;

% --- Part c: Rotation about Y0 ---
theta_y = -pi/4;
Ry = [cos(theta_y),0,sin(theta_y); 0,1,0; -sin(theta_y),0,cos(theta_y)];
P2 = Ry * P1;
figure('Name', 'Part (c): Rotation about Y0');
hold on;
plot3([0 axis_len], [0 0], [0 0], 'r', 'LineWidth', axis_width);
plot3([0 0], [0 axis_len], [0 0], 'g', 'LineWidth', axis_width);
plot3([0 0], [0 0], [0 axis_len], 'b', 'LineWidth', axis_width);
patch(P2(1,:), P2(2,:), P2(3,:), 'm', 'FaceAlpha', 0.5); % Magenta
grid on; axis equal; view(3);
title('Part (c): After Rotation about Y_0-axis');
xlabel('X_0'); ylabel('Y_0'); zlabel('Z_0');
legend('X_0-axis', 'Y_0-axis', 'Z_0-axis', 'Triangle P2');
hold off;

% --- Part d: Rotation about Z0 ---
theta_z = 2*pi/3;
Rz = [cos(theta_z),-sin(theta_z),0; sin(theta_z),cos(theta_z),0; 0,0,1];
P3 = Rz * P2;
figure('Name', 'Part (d): Rotation about Z0');
hold on;
plot3([0 axis_len], [0 0], [0 0], 'r', 'LineWidth', axis_width);
plot3([0 0], [0 axis_len], [0 0], 'g', 'LineWidth', axis_width);
plot3([0 0], [0 0], [0 axis_len], 'b', 'LineWidth', axis_width);
patch(P3(1,:), P3(2,:), P3(3,:), 'c', 'FaceAlpha', 0.5); % Cyan
grid on; axis equal; view(3);
title('Part (d): After Rotation about Z_0-axis');
xlabel('X_0'); ylabel('Y_0'); zlabel('Z_0');
legend('X_0-axis', 'Y_0-axis', 'Z_0-axis', 'Triangle P3');
hold off;

% --- Part e: Return to Original Position ---
R_total = Rz * Ry * Rx;
R_inverse = R_total';
P_final = R_inverse * P3;
figure('Name', 'Part (e): Return to Original Position');
hold on;
plot3([0 axis_len], [0 0], [0 0], 'r', 'LineWidth', axis_width);
plot3([0 0], [0 axis_len], [0 0], 'g', 'LineWidth', axis_width);
plot3([0 0], [0 0], [0 axis_len], 'b', 'LineWidth', axis_width);
patch(P_final(1,:), P_final(2,:), P_final(3,:), 'y', 'FaceAlpha', 0.5); % Yellow
grid on; axis equal; view(3);
title('Part (e): Triangle After Applying Final Rotation R');
xlabel('X_0'); ylabel('Y_0'); zlabel('Z_0');
legend('X_0-axis', 'Y_0-axis', 'Z_0-axis', 'Final Triangle');
hold off;
