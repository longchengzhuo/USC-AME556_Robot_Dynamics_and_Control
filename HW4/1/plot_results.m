clear;
clc;
close all;

% 定义三个 Case 的文件名和标题
% files = {'LQR_result_case1.csv', 'LQR_result_case2.csv', 'LQR_result_case3.csv'};
files = {'MPC_result.csv'};
titles = {'MPC control', ...
          'Case 2 (Q=diag(100,0.01,100,0.01), R=10)', ...
          'Case 3 (Q=diag(100,0.01,100,0.01), R=1)'};

% 遍历每个文件进行绘图
for i = 1:length(files)
    filename = files{i};
    case_title = titles{i};
    
    % 检查文件是否存在
    if ~isfile(filename)
        fprintf('Warning: File %s not found. Skipping.\n', filename);
        continue;
    end
    
    % 读取数据 (假设 CSV 包含标题行 t,x,theta,u)
    % DetectImportOptions 用于自动检测标题行
    opts = detectImportOptions(filename);
    opts.VariableNamingRule = 'preserve';
    data_table = readtable(filename, opts);
    
    % 提取数据列
    t_data = data_table.t;
    x_data = data_table.x;
    theta_rad = data_table.theta; % 原始数据是弧度
    u_data = data_table.u;
    
    % --- 开始绘图 ---
    figure('Name', ['LQR Response: ' case_title], 'NumberTitle', 'off');
    
    % 1. Cart Position x(t)
    subplot(3, 1, 1);
    plot(t_data, x_data, 'b-', 'LineWidth', 2);
    title(['Cart Position x(t) - ' case_title]);
    xlabel('Time (s)'); ylabel('Position (m)'); grid on;
    legend('x(t)');
    
    % 2. Pole Angle theta(t) -> 改为显示角度 (Degree)
    subplot(3, 1, 2);
    
    % [关键修改] 将弧度转换为角度
    theta_deg = rad2deg(theta_rad);
    
    plot(t_data, theta_deg, 'r-', 'LineWidth', 2);
    hold on;
    
    % 计算 5% 稳定误差带 (Settling Band)
    % 初始角度取数据的第一个点 (此时已经是角度制)
    theta_0 = theta_deg(1);
    settling_band_val = 0.05 * theta_0;
    
    % 画虚线 (Settling Band)
    t_span = [min(t_data), max(t_data)];
    plot(t_span, [settling_band_val, settling_band_val], 'k--');
    plot(t_span, [-settling_band_val, -settling_band_val], 'k--');
    
    title(['Pole Angle \theta(t) - ' case_title]);
    xlabel('Time (s)'); 
    ylabel('Angle (deg)'); % [关键修改] 标签改为度
    grid on;
    legend('\theta(t)', '5% Settling Band');
    
    % 3. Control Input u(t)
    subplot(3, 1, 3);
    plot(t_data, u_data, 'g-', 'LineWidth', 2);
    title(['Control Input u(t) - ' case_title]);
    xlabel('Time (s)'); ylabel('Force (N)'); grid on;
    legend('u(t)');
    
    fprintf('Plotted %s\n', filename);
end

fprintf('All plots generated.\n');