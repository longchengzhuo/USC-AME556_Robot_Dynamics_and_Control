%% 1. Symbolic Setup
clear;
clc;
syms M m l g;
syms t;
syms x(t) theta(t);

%% 2. Lagrangian Formulation
dx = diff(x, t);
dtheta = diff(theta, t);
ddx = diff(dx, t);
ddtheta = diff(dtheta, t);

% L = T - V (Kinetic - Potential)
L = 0.5*(M+m)*dx^2 - m*l*dx*dtheta*cos(theta(t)) + 0.5*m*l^2*dtheta^2 - m*g*l*cos(theta(t));
L = simplify(L);

q = [x(t); theta(t)];
dq = [dx; dtheta];
ddq = [ddx; ddtheta];

%% 3. Derive Equations of Motion (EOM) via Lagrange's Equations
J1 = simplify(jacobian(L, q))';
J2 = simplify(jacobian(L, dq))';
dJ2_dt = simplify(diff(J2, t));

% EOM = d/dt(dL/dq_dot) - dL/dq = 0
EOM = dJ2_dt - J1;
EOM = simplify(EOM);

% EOM is in the form: D(q)*ddq + N(q, dq) = 0
D = simplify(jacobian(EOM, ddq));
N = simplify(subs(EOM, ddq, [0; 0]));

%% 4. Display Symbolic Results
disp('Lagrangian L:'); pretty(L);
disp('Mass Matrix D:'); pretty(D);
disp('Coriolis, Centrifugal, and Gravity terms N:'); pretty(N);

%% 5. Setup for Numerical Simulation
M_val = 1;      % kg
m_val = 0.2;    % kg
l_val = 0.3;    % m
g_val = 9.8;    % m/s^2

syms th thd xh xhd real
old_syms = [theta(t), diff(theta(t), t), x(t), diff(x(t), t)];
new_syms = [th, thd, xh, xhd];
D_sub = subs(D, old_syms, new_syms);
N_sub = subs(N, old_syms, new_syms);

D_func = matlabFunction(D_sub, 'Vars', {t, M, m, l, th, thd, xh, xhd});
N_func = matlabFunction(N_sub, 'Vars', {t, M, m, l, g, th, thd, xh, xhd});

%% 6. Run ODE Simulation
t_span = [0 2];
% State vector: y = [x; theta; x_dot; theta_dot]
y0 = [0; pi/6; 0; 0];

% From EOM, ddq = -inv(D)*N
ode_system = @(t, y) [ y(3); y(4); D_func(t, M_val, m_val, l_val, y(2), y(4), y(1), y(3)) \ -N_func(t, M_val, m_val, l_val, g_val, y(2), y(4), y(1), y(3)) ];
[t_sol, y_sol] = ode45(ode_system, t_span, y0);

%% 7. Plot Results
figure;
subplot(2, 1, 1);
plot(t_sol, y_sol(:, 1), 'b-', 'LineWidth', 1.5);
title('Cart Position vs. Time'); xlabel('Time (s)'); ylabel('Position x (m)'); grid on;
subplot(2, 1, 2);
plot(t_sol, y_sol(:, 2) * 180/pi, 'r-', 'LineWidth', 1.5);
title('Pole Angle vs. Time'); xlabel('Time (s)'); ylabel('Angle \theta (degrees)'); grid on;

%% 8. Generate Animation
ts = 1/60;
file_index = 1;
anim_cart_pole(t_sol, y_sol, ts, file_index, l_val);
disp('Simulation and animation complete.');

function anim_cart_pole(t, x, ts, file_index, l)
    Fs = 1/ts;
    [te, xe] = even_sample(t, x, Fs);
    
    x_min = min(xe(:,1)) - (l + 0.5);
    x_max = max(xe(:,1)) + (l + 0.5);
    y_min = -0.6;
    y_max =  l + 0.6;
    
    fig1 = figure(1001); clf(fig1);
    axes1 = axes('Parent', fig1, 'XLim', [x_min, x_max], 'YLim', [y_min, y_max]);
    axis(axes1, 'equal'); grid(axes1, 'on'); hold(axes1, 'on');
    
    video_filename = sprintf('cartpole_animation_%d', file_index);
    v = VideoWriter(video_filename, 'MPEG-4');
    v.FrameRate = Fs;
    open(v);
    
    cart_w = 0.4; cart_h = 0.2; cart_y = 0.0;
    cart_rect = rectangle(axes1, 'Position', [0,0,cart_w,cart_h], 'FaceColor','k');
    rod_line  = line(axes1, [0,0], [0,0], 'Color', 'r', 'LineWidth', 2);
    
    for k = 1:numel(te)
        x_pos   = xe(k,1);
        theta_k = xe(k,2); % Convention: theta=0 is up, theta>0 is to the left
        
        xc = x_pos;
        yc = cart_y;
        
        set(cart_rect, 'Position', [xc - cart_w/2, yc - cart_h/2, cart_w, cart_h]);
        
        xb = xc - l * sin(theta_k);
        yb = yc + l * cos(theta_k);
        set(rod_line, 'XData', [xc, xb], 'YData', [yc, yb]);
        
        drawnow;
        writeVideo(v, getframe(fig1));
    end
    close(v);
    hold(axes1, 'off');
end

function [Et, Ex] = even_sample(t, x, Fs)
    t_even = linspace(t(1), t(end), round((t(end)-t(1))*Fs))';
    Ex = interp1(t, x, t_even, 'linear');
    Et = t_even;
end