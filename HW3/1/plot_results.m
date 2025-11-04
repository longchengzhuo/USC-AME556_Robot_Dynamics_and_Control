% Load the data
theta = readmatrix('theta.csv');
x = readmatrix('x.csv');
y = readmatrix('z.csv'); % z.csv is plotted as y(t)

% Take the negative of x data
x = -x;

% Create a time vector
num_points = length(theta);
time = linspace(0, 2, num_points);

% Create the plot
figure;

% Plot x(t)
subplot(3, 1, 1);
plot(time, x);
title('x(t)');
xlabel('Time (s)');
ylabel('x');
grid on;

% Plot y(t)
subplot(3, 1, 2);
plot(time, y);
title('y(t)');
xlabel('Time (s)');
ylabel('y');
grid on;

% Plot theta(t)
subplot(3, 1, 3);
plot(time, theta);
title('\theta(t)');
xlabel('Time (s)');
ylabel('\theta (rad)');
grid on;


