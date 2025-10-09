% Part (c) with world/body frame axis lines
clear; clc; close all

a1 = 0.1; a2 = 0.2; a3 = 0.2;
theta = [0, pi/4, pi/4];              % joint angles

% helpers
Rz = @(t)[cos(t) -sin(t) 0; sin(t) cos(t) 0; 0 0 1];
Ry = @(t)[cos(t) 0 sin(t); 0 1 0; -sin(t) 0 cos(t)];   % your Ry
Tx = @(d)[eye(3), [d;0;0]; 0 0 0 1];
TR = @(R)[R, zeros(3,1); 0 0 0 1];
T  = @(R,p)[R, p(:); 0 0 0 1];

% arm kinematics relative to its base {0}
t1 = theta(1); t2 = theta(2); t3 = theta(3);
T01 = T(Rz(t1), Rz(t1)*[0;0;a1]);
T12 = T(Ry(t2), Ry(t2)*[a2;0;0]);
T23 = T(Ry(t3), Ry(t3)*[a3;0;0]);

% body rotation from RPY=[pi/2,0,pi/4] using eul2rotm default 'ZYX'
roll = pi/2; pitch = 0; yaw = pi/4;
Rwb = eul2rotm([yaw, pitch, roll]);

% three base poses
Twb1 = eye(4) * Tx(0.5) * TR(Rwb);   % (1) body-translate then rotate
Twb2 = eye(4) * TR(Rwb) * Tx(0.5);   % (2) rotate then body-translate
Twb3 = Tx(0.5) * TR(Rwb);            % (3) rotate then world-translate
Tbase = {Twb1, Twb2, Twb3};

titles = { ...
  '(1) move (body x) 0.5 m, then RPY', ...
  '(2) RPY, then move (body x) 0.5 m', ...
  '(3) RPY, then move (world x) 0.5 m'};

figure('Color','w');
for k = 1:3
    Twb = Tbase{k};
    T0  = Twb;
    T1w = T0*T01; T2w = T1w*T12; T3w = T2w*T23;

    O0 = T0(1:3,4); O1 = T1w(1:3,4); O2 = T2w(1:3,4); O3 = T3w(1:3,4);

    subplot(1,3,k); hold on; grid on; axis equal
    % robot links
    plot3([O0(1) O1(1)],[O0(2) O1(2)],[O0(3) O1(3)],'-o','LineWidth',2);
    plot3([O1(1) O2(1)],[O1(2) O2(2)],[O1(3) O2(3)],'-o','LineWidth',2);
    plot3([O2(1) O3(1)],[O2(2) O3(2)],[O2(3) O3(3)],'-o','LineWidth',2);

    % world frame axes at origin
    draw_axes([0;0;0], eye(3), 0.15, 'W');
    % body frame axes at base origin
    draw_axes(O0, T0(1:3,1:3), 0.15, 'B');

    xlabel('x'); ylabel('y'); zlabel('z'); view(45,30);
    title(titles{k});
end

% ---------- axis drawer ----------
function draw_axes(O, R, s, tag)
    cols = {'r','g','b'};  % x,y,z
    for i = 1:3
        P = O + s*R(:,i);
        plot3([O(1) P(1)], [O(2) P(2)], [O(3) P(3)], ...
              'Color', cols{i}, 'LineWidth', 1.5);
        text(P(1), P(2), P(3), sprintf('%s_%c', tag, 'x'+i-1), 'FontSize', 8);
    end
end
修复问题