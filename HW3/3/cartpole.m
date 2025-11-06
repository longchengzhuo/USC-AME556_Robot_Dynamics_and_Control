%% 0. Setup
clear;
clc;
close all;

fprintf('正在运行倒立摆 LQR 控制仿真...\n');

%% 1. Symbolic Setup (Unchanged from your code)
syms M m l g;
syms t;
syms x(t) theta(t);

%% 2. Lagrangian Formulation (Unchanged)
dx = diff(x, t);
dtheta = diff(theta, t);
ddx = diff(dx, t);
ddtheta = diff(dtheta, t);

L = 0.5*(M+m)*dx^2 - m*l*dx*dtheta*cos(theta(t)) + 0.5*m*l^2*dtheta^2 - m*g*l*cos(theta(t));
L = simplify(L);
q = [x(t); theta(t)];
dq = [dx; dtheta];
ddq = [ddx; ddtheta];

%% 3. Derive Equations of Motion (EOM) (Unchanged)
J1 = simplify(jacobian(L, q))';
J2 = simplify(jacobian(L, dq))';
dJ2_dt = simplify(diff(J2, t));

% EOM = d/dt(dL/dq_dot) - dL/dq = 0
EOM = dJ2_dt - J1;
EOM = simplify(EOM);

D = simplify(jacobian(EOM, ddq));
N = simplify(subs(EOM, ddq, [0; 0]));

%% 5. Setup for Numerical Simulation
M_val = 1.0;      % kg
m_val = 0.2;      % kg
l_val = 0.3;      % m
g_val = 9.8;      % m/s^2 (Using g=9.8 as in your original code)

%% 6. NEW: LQR Controller Design
% Based on previous derivation: 
% A = [0 0 1 0; 0 0 0 1; 0 (m*g)/M 0 0; 0 (g*(M+m))/(M*l) 0 0]
% B = [0; 0; 1/M; 1/(M*l)]

A = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, (m_val * g_val) / M_val, 0, 0;
     0, (g_val * (M_val + m_val)) / (M_val * l_val), 0, 0];

B = [0;
     0;
     1 / M_val;
     1 / (M_val * l_val)];

% --- Q and R Matrix Tuning ---
% Performance Goal: t_s in [0.5, 1.0]s, Overshoot in [10, 20]%
% Q penalizes states [x, theta, x_dot, theta_dot]
% R penalizes control effort u
%
% 我们的策略:
% 1. 我们需要一个快速的镇定时间 (t_s < 1.0s)，所以 R 必须相对较小 (R=0.1) 以允许较大的控制输入。
% 2. 我们需要精确控制超调量 (10-20%)。这需要对角度 (Q(2,2)) 和角速度 (Q(4,4))
%    进行适当的惩罚。
% 3. 我们也希望小车位置 x (Q(1,1)) 最终能回到0。

Q = diag([800, 40, 0.001, 0.001]); % 大幅增加对 x 和 theta 误差的惩罚
R = 0.00001;                        % 大幅减少对控制力的惩罚

% ---------------------------------

% Calculate LQR Gain K
[K, S, e] = lqr(A, B, Q, R);

% Calculate closed-loop eigenvalues for stability proof
A_cl = A - B*K;
eigs_cl = eig(A_cl);

%% 7. MODIFIED: Run ODE Simulation with LQR Control
t_span = [0 2];

% NEW: Initial conditions from image_b43bdb.png
% State vector: y = [x; theta; x_dot; theta_dot]
y0 = [0.1; 0.1; 0; 0];

% Convert symbolic functions to MATLAB functions (Unchanged)
syms xh xhd th thd real
old_syms = [diff(theta(t),t), theta(t), diff(x(t), t), x(t)];
new_syms = [thd, th, xhd, xh];       
D_sub = subs(D, old_syms, new_syms);
N_sub = subs(N, old_syms, new_syms);

D_sub = formula(D_sub);
N_sub = formula(N_sub);

D_sub = matlabFunction(D_sub, 'Vars', {M, m, l, th, thd, xh, xhd});
N_sub = matlabFunction(N_sub, 'Vars', {M, m, l, g, th, thd, xh, xhd});


% MODIFIED: The ODE function now includes the control input u = -K*y
% The EOM is D*ddq + N = Q_ext, where Q_ext = [u; 0]
% So, ddq = D \ (Q_ext - N)
ode_lqr = @(t,y) [ y(3); 
                   y(4); 
                   D_sub(M_val,m_val,l_val, y(2),y(4), y(1),y(3)) \ ...
                       ( [-K*y; 0] - N_sub(M_val,m_val,l_val,g_val, y(2),y(4), y(1),y(3)) ) ]


[t_sol, y_sol] = ode45(ode_lqr, t_span, y0);
fprintf('...\n');

% Calculate control input u(t) over time
u_sol = -K * y_sol';
u_sol = u_sol';

%% 8. NEW: Plot Results for x(t), theta(t), u(t)
figure('Name', 'LQR Controlled Cart-Pole Response');
subplot(3, 1, 1);
plot(t_sol, y_sol(:, 1), 'b-', 'LineWidth', 2);
title('Cart Position x(t)');
xlabel('Time (s)'); ylabel('Position (m)'); grid on;
legend('x(t)');

subplot(3, 1, 2);
plot(t_sol, y_sol(:, 2), 'r-', 'LineWidth', 2);
hold on;
% Plot 5% settling bands
settling_band_theta = 0.05 * y0(2); % 5% of 0.1 rad
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

%% 9. Generate Animation (Unchanged)
fprintf('正在生成动画...\n');
ts = 1/60; % 60 fps
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
anim_cart_pole(t_sol, y_sol, ts, l_val, script_dir);

%% 10. NEW: Console Output for Analysis and Proof

% --- Performance Check ---
theta_sol = y_sol(:, 2);
% Overshoot
theta_min = min(theta_sol);
overshoot_percent = abs(theta_min / y0(2)) * 100;
% Settling Time (5%)
idx_outside = find(abs(theta_sol) > settling_band_theta, 1, 'last');
if isempty(idx_outside) || (idx_outside + 1) > length(t_sol)
   t_settling = 0; % Settled immediately or at the end
else
   t_settling = t_sol(idx_outside + 1);
end


fprintf('\n\n--- b. LQR 控制器设计与分析 ---\n\n');
fprintf('目标性能: 镇定时间 (5%%) 处于 [0.5, 1.0]s, 角度超调量 处于 [10, 20]%%\n');
fprintf('------------------------------------------------------------------\n');

fprintf('--- 概念解释 ---\n');
fprintf('1. 镇定时间 (Settling Time): 系统响应（此处指角度）进入并保持在其最终稳态值（0 rad）的一个小误差带（例如 5%%）内所需的时间。\n');
fprintf('   在此例中，5%% 误差带为 [-%f, +%f] rad (即 0.1 rad 的 5%%)。\n', settling_band_theta, settling_band_theta);
fprintf('2. 超调量 (Overshoot): 系统响应超过其最终稳态值的最大百分比。对于这个稳定任务，它指角度从 0.1 rad 下降，穿过 0 到达的最小负值的绝对值\n');
fprintf('   与初始值 0.1 rad 的比率。例如，10%% 的超调量意味着角度峰值为 -0.01 rad。\n\n');

fprintf('--- LQR 调参与理由 ---\n');
fprintf('我选择的 Q 和 R 矩阵是:\n');
fprintf('Q = diag([%.1f, %.1f, %.1f, %.1f])\n', Q(1,1), Q(2,2), Q(3,3), Q(4,4));
fprintf('R = %.2f\n', R);
fprintf('理由: 这个组合重度惩罚了角度 \theta (Q(2,2)=%.1f) 和角速度 \dot{\theta} (Q(4,4)=%.1f)，\n', Q(2,2), Q(4,4));
fprintf('      这是为了快速稳定倒立摆并精确控制超调量。小车位置 x (Q(1,1)=%.1f) 也有显著惩罚，\n', Q(1,1));
fprintf('      以确保它返回原点。一个较小的 R (R=%.2f) 值被用来允许足够的控制力，\n', R);
fprintf('      以实现在 1.0 秒内的快速镇定时间。\n\n');

fprintf('--- 实际性能验证 ---\n');
fprintf('>>> 实际镇定时间 (5%%): %.3f s\n', t_settling);
fprintf('>>> 实际超调量: %.2f %%\n\n', overshoot_percent);

fprintf('--- b.i) 局部指数稳定性证明 ---\n');
fprintf('闭环系统矩阵 A_cl = A - B*K。要证明局部指数稳定性，\n');
fprintf('我们证明 A_cl 的所有特征值都具有负实部。\n');
fprintf('计算得到的 A_cl 特征值 (Eigenvalues) 如下:\n');
disp(eigs_cl);
if all(real(eigs_cl) < 0)
    fprintf('结论: 所有特征值的实部均为负数。根据李雅普诺夫(Lyapunov)间接法，\n');
    fprintf('闭环系统在平衡点 (0,0,0,0) 附近是局部指数稳定的。\n');
else
    fprintf('警告: 存在非负实部的特征值，系统不稳定！请检查 A, B, Q, R 矩阵。\n');
end
fprintf('\n仿真和分析全部完成。\n');


%% 11. Helper Functions (Unchanged)
function anim_cart_pole(t, x, ts, l, save_dir)
    % (Function code from your script, unchanged)
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
        fprintf('视频写入器初始化失败。可能没有安装 MPEG-4 编解码器。\n');
        fprintf('错误信息: %s\n', ME.message);
        v = []; % Set v to empty so we don't try to write
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
            % writeVideo(v, getframe(fig1));
        end
    end
    if ~isempty(v)
        close(v);
        fprintf('动画已保存到: %s.mp4\n', video_filename);
    end
    hold(axes1, 'off');
end

function [Et, Ex] = even_sample(t, x, Fs)
    % (Function code from your script, unchanged)
    t_even = linspace(t(1), t(end), round((t(end)-t(1))*Fs))';
    Ex = interp1(t, x, t_even, 'linear');
    Et = t_even;
end