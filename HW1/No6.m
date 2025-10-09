% 3-DOF arm (Ry = [c 0 s; 0 1 0; -s 0 c]): forward kinematics + plots
clear; clc; close all

a1 = 0.1; a2 = 0.2; a3 = 0.2;
thetas = [ 0       0        0;
           0       pi/4     pi/4;
           pi/6    pi/4    -pi/2;
           pi      pi/2     pi/2 ];

figure('Color','w');
for k = 1:4
    t = thetas(k,:);
    [O1,O2,O3] = fk_points(t,a1,a2,a3);
    subplot(2,2,k); hold on; grid on; axis equal
    % links O0->O1, O1->O2, O2->O3
    plot3([0 O1(1)],[0 O1(2)],[0 O1(3)],'-o','LineWidth',2);
    plot3([O1(1) O2(1)],[O1(2) O2(2)],[O1(3) O2(3)],'-o','LineWidth',2);
    plot3([O2(1) O3(1)],[O2(2) O3(2)],[O2(3) O3(3)],'-o','LineWidth',2);
    xlabel('x'); ylabel('y'); zlabel('z');
    title(sprintf('\\theta = [%.2f, %.2f, %.2f] rad',t));
    view(45,30);
end

% ---------- helpers ----------
function [O1,O2,O3] = fk_points(theta,a1,a2,a3)
    t1 = theta(1); t2 = theta(2); t3 = theta(3);
    Rz = @(th)[cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
    Ry = @(th)[cos(th) 0 sin(th); 0 1 0; -sin(th) 0 cos(th)]; % given definition
    T  = @(R,p)[R, p(:); 0 0 0 1];

    R01 = Rz(t1);  p01 = R01*[0;0;a1];           T01 = T(R01,p01);
    R12 = Ry(t2);  p12 = R12*[a2;0;0];           T12 = T(R12,p12);
    R23 = Ry(t3);  p23 = R23*[a3;0;0];           T23 = T(R23,p23);

    T02 = T01*T12;  T03 = T02*T23;
    O1 = T01(1:3,4);  O2 = T02(1:3,4);  O3 = T03(1:3,4);
end
