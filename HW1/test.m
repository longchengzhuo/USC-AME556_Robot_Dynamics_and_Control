% 3-DOF arm (Ry = [c 0 s; 0 1 0; -s 0 c]): forward kinematics + plots    % 说明：3自由度机械臂；给定Ry旋转矩阵；做正向运动学并绘图
clear; clc; close all                                                      % 清空变量、清屏、关闭所有图窗

global thetas a1 a2 a3

a1 = 0.1; a2 = 0.2; a3 = 0.2;                                             % 连杆长度：基座到O1的竖直段a1，后两段长度a2、a3
thetas = [ 0       0        0;                                            % 四组关节角（t1,t2,t3），单位：弧度
           0       pi/4     pi/4;
           pi/6    pi/4    -pi/2;
           pi      pi/2     pi/2 ];

figure("color","w")

for k = 1:4 
    [p1,p2,p3] = pose2position(k);

    subplot(2,2,k);hold on;grid on;axis equal
    plot3([0,p1(1)],[0,p1(2)],[0,p1(3)],"-o","LineWidth",2)
    plot3([p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)],"-o","LineWidth",2)
    plot3([p2(1),p3(1)],[p2(2),p3(2)],[p2(3),p3(3)],"-o","LineWidth",2)
    xlabel("x")
    ylabel("y")
    zlabel("z")
    title(sprintf("\\theta=%.2f,%.2f,%.2f",thetas(k,:)))
    view(45,30)
    
end

function [p1,p2,p3] = pose2position(k)
    global thetas a1 a2 a3
    orip = [0;0;0;1];
    Rx = @(degree)[1 0 0;0 cos(degree) -sin(degree);0 sin(degree) cos(degree)];
    Ry = @(degree)[cos(degree) 0 sin(degree);0 1 0;-sin(degree) 0 cos(degree)];
    Rz = @(degree)[cos(degree) -sin(degree) 0;sin(degree) cos(degree) 0;0 0 1];


    T = @(R,p)[R,p;0 0 0 1];

    T01 = T(Rz(thetas(k,1)),Rz(thetas(k,1))*[0; 0; a1]);
    T12 = T(Ry(thetas(k,2)),Ry(thetas(k,2))*[a2; 0; 0]);
    T23 = T(Ry(thetas(k,3)),Ry(thetas(k,3))*[a3; 0; 0]);

    p1 = T01*orip
    p2 = T01*T12*orip;
    p3 = T01*T12*T23*orip;


end
