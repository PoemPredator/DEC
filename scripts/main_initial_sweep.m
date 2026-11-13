%% 初始条件拉偏扫描（攻角 × 俯仰角速度）
%
%   遍历 (alpha0, wz0) 网格，统计可收敛工况，绘制响应曲线与收敛性散点图。
%   输出：results/sweep/
%   参见 SIMULATE_ATTITUDE, CREATE_ANALYSIS_FIGURES。

add_project_paths();
setup_chinese_fonts();

%% 参数
cfg.alpha_vals    = -5:0.5:5;      % 初始攻角 [deg]
cfg.wz_vals       = -45:5:45;      % 初始俯仰角速度 [deg/s]
cfg.tend          = 0.5;           % 仿真时长 [s]
cfg.converge_alpha = 5;            % 收敛判据 |alpha_end| <= 5 deg
cfg.converge_beta  = 5;            % 收敛判据 |beta_end|  <= 5 deg
cfg.use_parfor     = true;
cfg.coeff          = 1;

out_dir = output_dir('sweep');

model = model_params();
a_sound = model.sound_speed;

% 速度剖面：0.1 s 内由 4 倍音速加速到 7 倍音速
V_func = @(t) (t < 0.1) .* ((7 - 3 * (0.1 - t) / 0.1) * a_sound) ...
            + (t >= 0.1) .* (7 * a_sound);

%% 参数网格
[A_grid, W_grid] = ndgrid(cfg.alpha_vals, cfg.wz_vals);
A_vec = A_grid(:);
W_vec = W_grid(:);
num_sims = numel(A_vec);

t_all = cell(num_sims, 1);
x_all = cell(num_sims, 1);
u_all = cell(num_sims, 1);

fprintf('开始计算，共 %d 组工况...\n', num_sims);

%% 批量仿真
use_parfor = cfg.use_parfor && (exist('parpool', 'file') == 2);
if cfg.use_parfor && ~use_parfor
    warning('main_initial_sweep:noPCT', '未检测到 Parallel Computing Toolbox，改为串行运行。');
end

if use_parfor
    parfor k = 1:num_sims
        [t_all{k}, x_all{k}, u_all{k}] = run_initial_case(A_vec(k), W_vec(k), ...
            cfg.tend, V_func);
    end
else
    for k = 1:num_sims
        [t_all{k}, x_all{k}, u_all{k}] = run_initial_case(A_vec(k), W_vec(k), ...
            cfg.tend, V_func);
    end
end

disp('计算完成，开始筛选收敛工况并绘图...');

%% 响应曲线
plot_names = {'01_攻角响应', '02_侧滑角响应', '03_速度滚转角响应', ...
              '04_滚转角速度响应', '05_偏航角速度响应', '06_俯仰角速度响应', ...
              '07_delta_n_方向舵偏指令', '08_delta_2_左升降舵偏指令', '09_delta_4_右升降舵偏指令'};
plot_ylabels = {'\alpha / deg', '\beta / deg', '\gamma_s / deg', ...
                '\omega_x / (deg/s)', '\omega_y / (deg/s)', '\omega_z / (deg/s)', ...
                '\delta_n / deg', '\delta_2 / deg', '\delta_4 / deg'};

[figs, axs] = create_analysis_figures(plot_names, plot_ylabels, plot_names);

blue_start = [0.1, 0.2, 0.8];
blue_end   = [0.7, 0.8, 1.0];
color_map = zeros(num_sims, 3);
for i = 1:num_sims
    f = (i - 1) / max(num_sims - 1, 1);
    color_map(i, :) = (1 - f) * blue_start + f * blue_end;
end

plot_count = 0;
for k = 1:num_sims
    y = state_to_deg(x_all{k});
    if abs(y(end, 1)) > cfg.converge_alpha || abs(y(end, 2)) > cfg.converge_beta
        continue;
    end

    plot_count = plot_count + 1;
    current_color = color_map(k, :);

    for ch = 1:6
        plot(axs{ch}, t_all{k}, y(:, ch), 'Color', current_color);
    end
    u = u_all{k};
    if ~isempty(u)
        for ch = 1:3
            plot(axs{6 + ch}, t_all{k}, u(:, ch), 'Color', current_color);
        end
    end
end

for i = 1:9
    save_figure_3formats(figs{i}, fullfile(out_dir, plot_names{i}));
end
fprintf('%d 组工况中成功控住并绘制了 %d 组。\n', num_sims, plot_count);

%% 收敛性统计（拉偏边界点强制判为不可控）
alpha_min = min(cfg.alpha_vals);
alpha_max = max(cfg.alpha_vals);
wz_min = min(cfg.wz_vals);
wz_max = max(cfg.wz_vals);

stable_alpha = [];
stable_wz = [];
unstable_alpha = [];
unstable_wz = [];

for k = 1:num_sims
    x = x_all{k};
    is_boundary = (A_vec(k) == alpha_min || A_vec(k) == alpha_max || ...
                   W_vec(k) == wz_min   || W_vec(k) == wz_max);

    is_stable = false;
    if ~isempty(x)
        y = state_to_deg(x);
        is_stable = (abs(y(end, 1)) <= cfg.converge_alpha && ...
                     abs(y(end, 2)) <= cfg.converge_beta);
    end
    if is_boundary
        is_stable = false;
    end

    if is_stable
        stable_alpha(end+1) = A_vec(k);    %#ok<SAGROW>
        stable_wz(end+1)   = W_vec(k);     %#ok<SAGROW>
    else
        unstable_alpha(end+1) = A_vec(k);  %#ok<SAGROW>
        unstable_wz(end+1)   = W_vec(k);   %#ok<SAGROW>
    end
end

fig4 = figure('Visible', 'off', 'Name', '收敛性散点图');
ax4 = axes('Parent', fig4);
hold(ax4, 'on'); grid(ax4, 'on'); box(ax4, 'on');
scatter(ax4, stable_alpha, stable_wz, 80, 'g', 'filled');
scatter(ax4, unstable_alpha, unstable_wz, 80, 'r', 'x', 'LineWidth', 1.5);
xlabel(ax4, '初始攻角 \alpha_0 / deg');
ylabel(ax4, '初始俯仰角速度 \omega_{z0} / (deg/s)');
title(ax4, '不同初始条件的收敛性', 'FontSize', 9, 'FontWeight', 'normal');
legend(ax4, {'可控', '不可控'}, 'Location', 'best');
save_figure_3formats(fig4, fullfile(out_dir, '10_收敛性散点图'));

save(fullfile(out_dir, 'sweep_metrics.mat'), ...
    'A_vec', 'W_vec', 'stable_alpha', 'stable_wz', ...
    'unstable_alpha', 'unstable_wz');
fprintf('结果已保存到【%s】。\n', out_dir);
