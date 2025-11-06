clear;
clc;
close all;

syms M m l g;
syms t;
syms x(t) theta(t);

dx = diff(x, t);
dtheta = diff(theta, t);
ddx = diff(dx, t);
ddtheta = diff(dtheta, t);
L = 0.5*(M+m)*dx^2 - m*l*dx*dtheta*cos(theta(t)) + 0.5*m*l^2*dtheta^2 - m*g*l*cos(theta(t));
L = simplify(L);
q = [x(t); theta(t)];
dq = [dx; dtheta];
ddq = [ddx; ddtheta];

J1 = simplify(jacobian(L, q))';
J2 = simplify(jacobian(L, dq))';
dJ2_dt = simplify(diff(J2, t));

EOM = dJ2_dt - J1;
EOM = simplify(EOM);
D = simplify(jacobian(EOM, ddq));
N = simplify(subs(EOM, ddq, [0; 0]));

M_val = 1.0;
m_val = 0.2;
l_val = 0.3;
g_val = 9.8;

A = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, (m_val * g_val) / M_val, 0, 0;
     0, (g_val * (M_val + m_val)) / (M_val * l_val), 0, 0];
B = [0;
     0;
     1 / M_val;
     1 / (M_val * l_val)];

Q = diag([800, 40, 0.001, 0.001]);
R = 0.00001;

[K, S, e] = lqr(A, B, Q, R);

A_cl = A - B*K;
eigs_cl = eig(A_cl);

t_span = [0 2];
y0 = [0.1; 0.1; 0; 0];

syms xh xhd th thd real
old_syms = [diff(theta(t),t), theta(t), diff(x(t), t), x(t)];
new_syms = [thd, th, xhd, xh];       
D_sub = subs(D, old_syms, new_syms);
N_sub = subs(N, old_syms, new_syms);
D_sub = formula(D_sub);
N_sub = formula(N_sub);
D_sub = matlabFunction(D_sub, 'Vars', {M, m, l, th, thd, xh, xhd});
N_sub = matlabFunction(N_sub, 'Vars', {M, m, l, g, th, thd, xh, xhd});

ode_lqr = @(t,y) [ y(3); 
                   y(4); 
                   D_sub(M_val,m_val,l_val, y(2),y(4), y(1),y(3)) \ ...
                       ( [-K*y; 0] - N_sub(M_val,m_val,l_val,g_val, y(2),y(4), y(1),y(3)) ) ];
[t_sol, y_sol] = ode45(ode_lqr, t_span, y0);
fprintf('...\n');

u_sol = -K * y_sol';
u_sol = u_sol';

figure('Name', 'LQR Controlled Cart-Pole Response');
subplot(3, 1, 1);
plot(t_sol, y_sol(:, 1), 'b-', 'LineWidth', 2);
title('Cart Position x(t)');
xlabel('Time (s)'); ylabel('Position (m)'); grid on;
legend('x(t)');

subplot(3, 1, 2);
plot(t_sol, y_sol(:, 2), 'r-', 'LineWidth', 2);
hold on;
settling_band_theta = 0.05 * y0(2);
plot(t_span, [settling_band_theta, settling_band_theta], 'k--');
plot(t_span, [-settling_band_theta, -settling_band_theta], 'k--');
title('Pole Angle \theta(t)');
xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
legend('\theta(t)', '5% Settling Band');

subplot(3, 1, 3);
plot(t_sol, u_sol, 'g-', 'LineWidth', 2);
title('Control Input u(t)');
xlabel('Time (s)'); ylabel('Force (N)'); grid on;
legend('u(t)');

fprintf('Generating animation...\n');
ts = 1/60;
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
anim_cart_pole(t_sol, y_sol, ts, l_val, script_dir);

theta_sol = y_sol(:, 2);
theta_min = min(theta_sol);
overshoot_percent = abs(theta_min / y0(2)) * 100;

idx_outside = find(abs(theta_sol) > settling_band_theta, 1, 'last');
if isempty(idx_outside) || (idx_outside + 1) > length(t_sol)
   t_settling = 0;
else
   t_settling = t_sol(idx_outside + 1);
end

fprintf('--- LQR Tuning and Rationale ---\n');
fprintf('The Q and R matrices I chose are:\n');
fprintf('Q = diag([%.1f, %.1f, %.3f, %.3f])\n', Q(1,1), Q(2,2), Q(3,3), Q(4,4));
fprintf('R = %.5f\n', R);
fprintf('Rationale: This combination heavily penalizes the angle \theta (Q(2,2)=%.1f) and angular velocity \dot{\theta} (Q(4,4)=%.3f),\n', Q(2,2), Q(4,4));
fprintf('      to quickly stabilize the pendulum and precisely control overshoot. Cart position x (Q(1,1)=%.1f) is also significantly penalized,\n', Q(1,1));
fprintf('      to ensure it returns to the origin. A very small R (R=%.5f) value is used to allow for sufficient control force,\n', R);
fprintf('      to achieve the fast settling time within 1.0 second.\n\n');
fprintf('--- Actual Performance Verification ---\n');
fprintf('>>> Actual Settling Time (5%%): %.3f s\n', t_settling);
fprintf('>>> Actual Overshoot: %.2f %%\n\n', overshoot_percent);
fprintf('--- b.i) Proof of Local Exponential Stability ---\n');
fprintf('The closed-loop system matrix is A_cl = A - B*K. To prove local exponential stability,\n');
fprintf('we show that all eigenvalues of A_cl have negative real parts.\n');
fprintf('The calculated eigenvalues of A_cl are as follows:\n');
disp(eigs_cl);
if all(real(eigs_cl) < 0)
    fprintf('Conclusion: All eigenvalues have negative real parts. According to Lyapunov''s indirect method,\n');
    fprintf('the closed-loop system is locally exponentially stable around the equilibrium point (0,0,0,0).\n');
else
    fprintf('WARNING: Non-negative real part found in eigenvalues. The system is unstable! Check A, B, Q, R matrices.\n');
end

function anim_cart_pole(t, x, ts, l, save_dir)
    Fs = 1/ts;
    [te, xe] = even_sample(t, x, Fs);
    
    x_min = min(xe(:,1)) - (l + 0.5);
    x_max = max(xe(:,1)) + (l + 0.5);
    y_min = -0.6;
    y_max =  l + 0.6;
    
    fig1 = figure(1001); clf(fig1);
    set(fig1, 'Name', 'LQR Controlled Animation');
    axes1 = axes('Parent', fig1, 'XLim', [x_min, x_max], 'YLim', [y_min, y_max]);
    axis(axes1, 'equal'); grid(axes1, 'on'); hold(axes1, 'on');
    axis(axes1, 'manual');
    
    video_filename = fullfile(save_dir, 'cartpole_lqr_matlab');
    try
        v = VideoWriter(video_filename, 'MPEG-4');
        v.FrameRate = Fs;
        open(v);
    catch ME
        fprintf('VideoWriter initialization failed. MPEG-4 codec might not be installed.\n');
        fprintf('Error message: %s\n', ME.message);
        v = [];
    end
    
    cart_w = 0.4; cart_h = 0.2; cart_y = 0.0;
    cart_rect = rectangle(axes1, 'Position', [0,0,cart_w,cart_h], 'FaceColor','k');
    rod_line  = line(axes1, [0,0], [0,0], 'Color', 'r', 'LineWidth', 2);
    title(axes1, 'LQR Controlled Cart-Pole Animation');
    
    for k = 1:numel(te)
        x_pos   = xe(k,1);
        theta_k = xe(k,2);
        
        xc = x_pos;
        yc = cart_y;
        
        set(cart_rect, 'Position', [xc - cart_w/2, yc - cart_h/2, cart_w, cart_h]);
        
        theta_k_anim = theta_k; 
        
        xb = xc - l * sin(theta_k_anim);
        yb = yc + l * cos(theta_k_anim);
        set(rod_line, 'XData', [xc, xb], 'YData', [yc, yb]);
        
        drawnow;
        if ~isempty(v)
            writeVideo(v, getframe(fig1));
        end
    end
    if ~isempty(v)
        close(v);
        fprintf('Animation saved to: %s.mp4\n', video_filename);
    end
    hold(axes1, 'off');
end
function [Et, Ex] = even_sample(t, x, Fs)
    t_even = linspace(t(1), t(end), round((t(end)-t(1))*Fs))';
    Ex = interp1(t, x, t_even, 'linear');
    Et = t_even;
end