function HW2_demo()
    clc; clear all; close all;
    global params;
    m = 0.5 ; g = 9.81 ; x0 = [0,0,0;0,0,0;1,2,1;2,1,2]; c = [0.05,0.05,0.5];
    params.m = m;
    params.g = g;
    for i=(1:3)
        params.c = c(i);
        
        tspan=[0, 2]; % simulation time
        [t,x]=ode45(@sys_dynamics,tspan,x0(:,i));
        
        anim(t,x,1/24, i); % animate the system and save the simulation video

         % plot the simulation data
        figure; plot(x(:,1), x(:,2), '-o', 'LineWidth', 1.5, 'MarkerSize', 3); 
        xlabel('x-position (m)');   
        ylabel('y-position (m)');   
        title(['System Trajectory in X-Y Plane for c = ', num2str(c(i))]); 
        legend('Trajectory');       
        axis equal;                 
        grid on;  
    end
end
%%
function dx=sys_dynamics(t,x)
    global params;
    dx_val = x(3);
    dy_val = x(4);
    ddx = -params.c*sqrt(dx_val^2+dy_val^2)*dx_val/params.m;
    ddy = -params.c*sqrt(dx_val^2+dy_val^2)*dy_val/params.m-params.g;
    dx = [dx_val;dy_val;ddx;ddy];
end
%%
function u=controller(t,x)
    global params
    u = 0; % you can put your controller here
end
%%
function anim(t, x, ts, file_index)
    Fs = 1/ts;
    [te, xe] = even_sample(t, x, Fs);
    fig1 = figure(1); clf(fig1);
    axes1 = axes('Parent', fig1);
    set(axes1, 'XLim', [-1, 3], 'YLim', [-2, 2], ...
               'Position', [0 0 1 1], 'Color', 'w');
    axis(axes1, 'equal'); grid(axes1, 'on');
    
    video_filename = sprintf('video_HW2_demo_%d', file_index);
    v = VideoWriter(video_filename, 'MPEG-4');
    
    v.FrameRate = Fs;
    open(v);
    for k = 1:numel(te)
        drawone(axes1, xe(k,:));   
        drawnow;
        writeVideo(v, getframe(fig1));
    end
    close(v);
end
%%
function [Et, Ex] = even_sample(t, x, Fs)
    t0 = t(1); tf = t(end);
    EM = round((tf - t0) * Fs) + 1;   
    Et = linspace(t0, tf, EM)';
    Ex = zeros(EM, size(x,2));
    for s = 1:size(x,2)
        Ex(:,s) = interp1(t, x(:,s), Et, 'linear');
    end
end
%%
function drawone(parent, x)
    % x = [x_pos; y_pos; vx; vy]
    tem = get(parent,'Children');
    delete(tem);
    xpos = x(1);
    ypos = x(2);
    plot(parent, xpos, ypos, 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
    axis(parent, 'equal');  
    grid(parent, 'on');
    xlim(parent, [-1, 3]);
    ylim(parent, [-15, 2]);
end