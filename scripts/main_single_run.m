%% 单次姿态控制仿真（名义工况）
%
%   一次闭环仿真，输出 9 条时域曲线（3 个气流角、3 个角速度、3 个舵偏）。
%   运行前请确认 data/气动数据.xlsx 已放置（见 config/aero_config.m）。
%
%   输出：results/single/
%   参见 SIMULATE_ATTITUDE, MODEL_PARAMS, CONTROLLER_PARAMS。

add_project_paths();
setup_chinese_fonts();

%% 参数
cfg.tend        = 0.2;          % 仿真时长 [s]
cfg.alpha0_deg  = 2;            % 初始攻角 [deg]
cfg.beta0_deg   = 4;            % 初始侧滑角 [deg]
cfg.gammas0_deg = 0;            % 初始速度滚转角 [deg]
cfg.wx0_deg     = 0;            % 初始滚转角速度 [deg/s]
cfg.wy0_deg     = 0;            % 初始偏航角速度 [deg/s]
cfg.wz0_deg     = 25;           % 初始俯仰角速度 [deg/s]
cfg.coeff       = 1;            % 1 = 全补偿 ADRC；0 = PD
cfg.load_results = false;       % true = 复用已有结果，跳过仿真

out_dir = output_dir('single');
results_file = fullfile(output_dir(), 'single_results.mat');

model = model_params();
a_sound = model.sound_speed;

% 速度剖面：0.1 s 内由 4 倍音速加速到 7 倍音速
V_func = @(t) (t < 0.1) .* ((7 - 3 * (0.1 - t) / 0.1) * a_sound) ...
            + (t >= 0.1) .* (7 * a_sound);

%% 仿真
if cfg.load_results && exist(results_file, 'file')
    S = load(results_file);
    t = S.t; x = S.x; u = S.u;
    fprintf('已读取 %s，跳过仿真。\n', results_file);
else
    bias = struct();
    bias.tend        = cfg.tend;
    bias.alpha0      = deg2rad(cfg.alpha0_deg);
    bias.beta0       = deg2rad(cfg.beta0_deg);
    bias.gammas0     = deg2rad(cfg.gammas0_deg);
    bias.wx0         = deg2rad(cfg.wx0_deg);
    bias.wy0         = deg2rad(cfg.wy0_deg);
    bias.wz0         = deg2rad(cfg.wz0_deg);
    bias.V           = V_func;
    bias.coeff       = cfg.coeff;
    bias.hl          = @(t) ones(3, 1);
    bias.disturbance = @(t) zeros(3, 1);

    tic;
    [t, x, u] = simulate_attitude(bias);
    fprintf('仿真完成，用时 %.2f s。\n', toc);

    save(results_file, 't', 'x', 'u');
end

%% 绘图
y = state_to_deg(x);

plot_names = {'01_alpha_攻角跟踪', '02_beta_侧滑角跟踪', '03_gamma_s_弹道倾角跟踪', ...
              '04_omega_x_滚转角速度', '05_omega_y_偏航角速度', '06_omega_z_俯仰角速度', ...
              '07_delta_n_方向舵偏指令', '08_delta_2_左升降舵偏指令', '09_delta_4_右升降舵偏指令'};
plot_ylabels = {'\alpha / (deg)', '\beta / (deg)', '\gamma_s / (deg)', ...
                '\omega_x / (deg/s)', '\omega_y / (deg/s)', '\omega_z / (deg/s)', ...
                '\delta_n / (deg)', '\delta_2 / (deg)', '\delta_4 / (deg)'};
plot_titles = {'Angle of Attack Tracking', 'Sideslip Angle Tracking', 'Bank Angle Tracking', ...
               'Roll Rate (X-axis)', 'Yaw Rate (Y-axis)', 'Pitch Rate (Z-axis)', ...
               'Equivalent Yaw Fin \delta_n', 'Left Pitch Fin \delta_2', 'Right Pitch Fin \delta_4'};

[figs, axs] = create_analysis_figures(plot_names, plot_ylabels, plot_titles);

color_cmd  = [0.8500, 0.3250, 0.0980];
color_real = [0.0000, 0.4470, 0.7410];
color_rate = [0.4660, 0.6740, 0.1880];
color_ctrl = [0.9290, 0.6940, 0.1250];

for i = 1:3
    plot(axs{i}, t, zeros(size(t)), '--', 'Color', color_cmd, 'LineWidth', 1.5);
    plot(axs{i}, t, y(:, i), '-', 'Color', color_real, 'LineWidth', 2);
    legend(axs{i}, {'Cmd', 'True'}, 'Location', 'northeast');
end
for i = 1:3
    plot(axs{3 + i}, t, y(:, 3 + i), '-', 'Color', color_rate, 'LineWidth', 2);
end
for i = 1:3
    plot(axs{6 + i}, t, u(:, i), '-', 'Color', color_ctrl, 'LineWidth', 2);
end

for i = 1:9
    save_figure_3formats(figs{i}, fullfile(out_dir, plot_names{i}));
end

fprintf('图已保存到【%s】。\n', out_dir);
fprintf('终端状态: alpha = %.3f deg, beta = %.3f deg, wz = %.3f deg/s\n', ...
    y(end, 1), y(end, 2), y(end, 6));
