%% 步骤 1: 定义符号变量和函数
% 清理工作区和命令行
clear;
clc;

% 定义常数符号
syms M m l g;

% 定义时间变量
syms t;

% 将 x 和 theta 定义为时间 t 的函数
syms x(t) theta(t);

%% 步骤 2: 建立拉格朗日函数 L
% 计算一阶导数 (速度)
dx = diff(x, t);
dtheta = diff(theta, t);
ddx = diff(dx, t);
ddtheta = diff(dtheta, t);

% 拉格朗日函数 L = T - V (动能 - 势能)
% 您的 L 表达式似乎混合了动能和势能项，这里我们直接使用它
% L = 0.5*(M+m)*dx^2 - m*l*(g+(dx*dtheta))*cos(theta)+l^2*dtheta^2;
% 注意：为了结果更清晰，我们通常将 L 写成最简形式。
% 您的表达式可以化简，但我们先按您的原始形式计算
L = 0.5*(M+m)*dx^2 - m*l*dx*dtheta*cos(theta(t)) + 0.5*m*l^2*dtheta^2 - m*g*l*cos(theta(t));
% 注：我将您的 L 稍微重写成了更标准的 T-V 形式，其中 T = 0.5*(M+m)*dx^2 - m*l*dx*dtheta*cos(theta) + 0.5*m*l^2*dtheta^2
% V = -m*g*l*cos(theta)。这与您原始表达式是等价的（假设 l^2*dtheta^2 项前系数是0.5*m）。
% 如果您的原始L是正确的，请替换回您的那一行。

L = simplify(L);

% 定义广义坐标和广义速度
q = [x(t); theta(t)];
dq = [dx; dtheta];
ddq = [ddx; ddtheta];

%% 步骤 3: 计算拉格朗日方程的各个部分
% 计算 J1 = (dL/dq)' 
J1 = simplify(jacobian(L, q))';

% 计算 J2 = (dL/ddq)'
J2 = simplify(jacobian(L, dq))';

%% 步骤 4: 对 J2 求时间的全导数 (您问题的核心)
% 因为 x 和 theta 已经定义为 t 的函数，可以直接使用 diff
dJ2_dt = simplify(diff(J2, t));

%% 步骤 5: 得到最终的运动方程 (EOM)
% EOM = d/dt(dL/ddq) - dL/dq = 0  (对于无外力系统)
EOM = dJ2_dt - J1;
EOM = simplify(EOM);

D = jacobian(EOM, ddq);
N = subs(EOM, ddq, [0; 0]);
D = simplify(D);
N = simplify(N);


%% 步骤 6: 显示结果
disp('拉格朗日函数 L:');
pretty(L);

disp('J1 = (dL/dq)'' :');
pretty(J1);

disp('J2 = (dL/ddq)'' :');
pretty(J2);

disp('d(J2)/dt (对时间的全导数):');
pretty(dJ2_dt);

disp('最终的运动方程 (EOM = d(J2)/dt - J1):');
pretty(EOM);

disp('D is:');
pretty(D);

disp('N is:');
pretty(N);


%%
%% 步骤 7: 设置数值参数并创建数值函数（包含 t 作为哑变量）
% =========================================================================
M_val = 1;      % kg
m_val = 0.2;    % kg
l_val = 0.3;    % m
g_val = 9.8;    % m/s^2

% 用普通符号承接 x, theta 及其一阶导，避免非法变量名
syms th thd xh xhd real

% 把函数形式替换为普通符号（避免 Vars 中出现 theta(t)、diff(theta,t) 等）
old_syms = [theta(t),           diff(theta(t), t),   x(t),         diff(x(t), t)];
new_syms = [th,                 thd,                 xh,           xhd];

D_sub = simplify(subs(D, old_syms, new_syms));
N_sub = simplify(subs(N, old_syms, new_syms));

% 关键：把 t 也放进 Vars（匿名函数不使用 'Optimize'）
% 为稳妥起见，把所有可能的自变量都列入（即使 D_sub 实际不依赖也没关系）
D_func = matlabFunction(D_sub, 'Vars', {t, M, m, l, th, thd, xh, xhd});
N_func = matlabFunction(N_sub, 'Vars', {t, M, m, l, g, th, thd, xh, xhd});


%% 步骤 8: 使用 ode45 进行仿真
% =========================================================================
t_span = [0 2];                 % 仿真时间 0 到 2 秒
% 状态向量 y = [x; theta; x_dot; theta_dot]
y0 = [0; pi/6; 0; 0];

% y_dot = [x_dot; theta_dot; x_ddot; theta_ddot]
% 注意把 ODE 的 t 作为第一个实参传给 D_func/N_func（"哑变量"）
ode_system = @(t, y) [ ...
    y(3); ...
    y(4); ...
    D_func(t, M_val, m_val, l_val, y(2), y(4), y(1), y(3)) \ ...
   (-N_func(t, M_val, m_val, l_val, g_val, y(2), y(4), y(1), y(3))) ...
];

[t_sol, y_sol] = ode45(ode_system, t_span, y0);


%% 步骤 9: 可视化仿真结果
% =========================================================================
figure;

subplot(2, 1, 1);
plot(t_sol, y_sol(:, 1), 'b-', 'LineWidth', 1.5);
title('Cart Position vs. Time');
xlabel('Time (s)');
ylabel('Position x (m)');
grid on;

subplot(2, 1, 2);
plot(t_sol, y_sol(:, 2) * 180/pi, 'r-', 'LineWidth', 1.5);
title('Pole Angle vs. Time');
xlabel('Time (s)');
ylabel('Angle \theta (degrees)');
grid on;

disp('仿真完成，并已绘制结果图。');


%% 步骤 10: 生成动画并保存视频（红色杆 + 黑色矩形车；theta=0 向上，theta>0 向左）
% =========================================================================
ts = 1/60;           % 目标视频时间步（约 60 FPS）
file_index = 1;      % 输出文件编号
anim_cart_pole(t_sol, y_sol, ts, file_index, l_val);  % 调用动画函数（见下方本文件内局部函数）


% ========== 下面是本脚本内的局部函数（动画） ==========

function anim_cart_pole(t, x, ts, file_index, l)
    % x: [x; theta; x_dot; theta_dot]
    Fs = 1/ts;
    [te, xe] = even_sample_cartpole(t, x, Fs);

    % 根据轨迹计算合适的显示边界
    x_min = min(xe(:,1)) - (l + 0.5);
    x_max = max(xe(:,1)) + (l + 0.5);
    y_min = -0.6;        % 地面下方一点余量
    y_max =  l + 0.6;    % 顶部余量（theta=0 时杆向上）

    fig1 = figure(1001); clf(fig1);
    axes1 = axes('Parent', fig1);
    set(axes1, 'XLim', [x_min, x_max], 'YLim', [y_min, y_max], ...
               'Position', [0 0 1 1], 'Color', 'w');
    axis(axes1, 'equal'); grid(axes1, 'on');
    hold(axes1, 'on');

    % 准备视频写入
    video_filename = sprintf('video_HW2_2_2_demo_%d', file_index);
    v = VideoWriter(video_filename, 'MPEG-4');
    v.FrameRate = Fs;
    open(v);

    % 预创建图元以避免每帧新建
    cart_w = 0.4;                     % 车宽（随意）
    cart_h = 0.2;                     % 车高（随意）
    cart_y = 0.0;                     % 车中心 y 坐标
    cart_rect = rectangle(axes1, 'Position', [0, 0, cart_w, cart_h], ...
                          'Curvature', 0.1, 'FaceColor', 'k', 'EdgeColor', 'k');
    rod_line  = line(axes1, [0,0], [0,0], 'Color', 'r', 'LineWidth', 2);
    pivot_pt  = plot(axes1, 0, cart_y, 'ro', 'MarkerSize', 4, 'LineWidth', 1.0);

    %（可选）地面线
    plot(axes1, [x_min-1, x_max+1], [cart_y - cart_h/2, cart_y - cart_h/2], ...
         '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 1);

    for k = 1:numel(te)
        x_pos   = xe(k,1);
        theta_k = xe(k,2);        % 约定：theta=0 向上，theta>0 向左
        % 如果你的动力学里 theta=0 向下，请改为：
        % theta_k = pi - xe(k,2);

        % 车中心坐标
        xc = x_pos;
        yc = cart_y;

        % 更新车（矩形）的位姿（以中心为基准）
        set(cart_rect, 'Position', [xc - cart_w/2, yc - cart_h/2, cart_w, cart_h]);

        % 杆端点（theta=0 向上；theta>0 向左）
        xb = xc - l * sin(theta_k);
        yb = yc + l * cos(theta_k);

        % 更新杆和枢轴点
        set(rod_line, 'XData', [xc, xb], 'YData', [yc, yb]);
        set(pivot_pt, 'XData', xc, 'YData', yc);

        drawnow;
        writeVideo(v, getframe(fig1));
    end
    close(v);
    hold(axes1, 'off');
end

function [Et, Ex] = even_sample_cartpole(t, x, Fs)
    t0 = t(1); tf = t(end);
    EM = round((tf - t0) * Fs) + 1;
    Et = linspace(t0, tf, EM)';           % 等间隔时间
    Ex = zeros(EM, size(x,2));
    for s = 1:size(x,2)
        Ex(:,s) = interp1(t, x(:,s), Et, 'linear');  % 线性插值为等间隔采样
    end
end


