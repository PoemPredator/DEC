function metrics = envelope_report(batch, opts)
%ENVELOPE_REPORT 绘制不同补偿系数下的响应包络带并给出最坏 ISE 报告。
%
%   METRICS = ENVELOPE_REPORT(BATCH, OPTS)
%
%   BATCH 为 RUN_BATCH_SIMULATIONS 的输出；OPTS 字段
%       .out_dir      图片输出目录（必填）
%       .cost         传给 COMPUTE_TRACKING_COST 的配置
%       .converge     .alpha_deg / .beta_deg（默认 5 / 5）
%       .coeff_labels 可选图例标签 cell
%
%   返回 METRICS
%       .coeff_values .coeff_labels .perf_metrics .valid_counts
%       .worst_k .best_idx
%
%   参见 MAIN_MONTE_CARLO, DRAW_ENVELOPE, SAVE_DISTURBANCE_FIGURES。

if nargin < 2 || isempty(opts)
    opts = struct();
end
if ~isfield(opts, 'out_dir') || isempty(opts.out_dir)
    error('analysis:envelope_report:outDir', '必须提供 opts.out_dir。');
end
if ~isfield(opts, 'cost') || isempty(opts.cost)
    opts.cost = struct();
end
if ~isfield(opts, 'converge') || isempty(opts.converge)
    opts.converge = struct('alpha_deg', 5, 'beta_deg', 5);
end

save_folder  = opts.out_dir;
num_sims     = batch.num_sims;
coeff_values = batch.coeff_values(:).';
n_coeff      = numel(coeff_values);
t_common     = linspace(0, batch.tend, 500);

if ~isfield(opts, 'coeff_labels') || isempty(opts.coeff_labels)
    coeff_labels = arrayfun(@(v) sprintf('c=%g', v), coeff_values, ...
        'UniformOutput', false);
else
    coeff_labels = opts.coeff_labels;
end

plot_names = {'01_alpha_攻角包络带', '02_beta_侧滑角包络带', ...
              '03_gamma_s_速度滚转角包络带', '04_omega_x_滚转角速度包络带', ...
              '05_omega_y_偏航角速度包络带', '06_omega_z_俯仰角速度包络带', ...
              '07_delta_n_方向舵偏指令包络带', '08_delta_2_左升降舵偏指令包络带', ...
              '09_delta_4_右升降舵偏指令包络带'};
plot_ylabels = {'\alpha / deg', '\beta / deg', '\gamma_s / deg', ...
                '\omega_x / (deg/s)', '\omega_y / (deg/s)', '\omega_z / (deg/s)', ...
                '\delta_n / deg', '\delta_2 / deg', '\delta_4 / deg'};
plot_titles = {'攻角包络带', '侧滑角包络带', '速度滚转角包络带', ...
               '滚转角速度包络带', '偏航角速度包络带', '俯仰角速度包络带', ...
               '方向舵偏指令包络带', '左升降舵偏指令包络带', '右升降舵偏指令包络带'};

[figs, axs] = create_analysis_figures(plot_names, plot_ylabels, plot_titles);

colors = coeff_colors(n_coeff);

perf_metrics = zeros(1, n_coeff);
valid_counts = zeros(1, n_coeff);
worst_k      = zeros(1, n_coeff);

for c_idx = 1:n_coeff
    y_mat = NaN(num_sims, numel(t_common), 9);
    max_metric = -inf;
    worst_k_current = NaN;

    for k = 1:num_sims
        t = batch.t_all{k, c_idx};
        x = batch.x_all{k, c_idx};
        u = batch.u_all{k, c_idx};
        if isempty(t)
            continue;
        end

        y = state_to_deg(x);
        if abs(y(end, 1)) > opts.converge.alpha_deg || ...
           abs(y(end, 2)) > opts.converge.beta_deg
            continue;
        end

        valid_counts(c_idx) = valid_counts(c_idx) + 1;

        J = compute_tracking_cost(t, x, opts.cost);
        if J > max_metric
            max_metric = J;
            worst_k_current = k;
        end

        [t_u, ia] = unique(t);
        for ch = 1:6
            y_mat(k, :, ch) = interp1(t_u, y(ia, ch), t_common, 'linear', 'extrap');
        end
        if ~isempty(u)
            for ch = 1:3
                y_mat(k, :, 6 + ch) = interp1(t_u, u(ia, ch), t_common, ...
                    'linear', 'extrap');
            end
        end
    end

    if isnan(worst_k_current)
        worst_k_current = 1;
        max_metric = inf;
    end
    perf_metrics(c_idx) = max_metric;
    worst_k(c_idx) = worst_k_current;

    for ch = 1:9
        if ~any(~isnan(y_mat(:, :, ch)), 'all')
            continue;
        end
        draw_envelope(axs{ch}, t_common, y_mat(:, :, ch), ...
            colors(c_idx, :), coeff_labels{c_idx});
    end
end

for i = 1:9
    xline(axs{i}, 0.1, '--k', 't=0.1s', 'LabelHorizontalAlignment', 'left', ...
          'HandleVisibility', 'off', 'LineWidth', 1.2);
    legend(axs{i}, 'Location', 'best', 'FontName', 'Microsoft YaHei', 'FontSize', 9);
    save_figure_3formats(figs{i}, fullfile(save_folder, plot_names{i}));
end
fprintf('包络图已保存到【%s】。\n', save_folder);

%% 性能报告
disp(' ');
disp('================ 性能评估报告 ================');
disp('指标定义: 最坏情形下的过程积分平方误差 ISE');
disp('  J = ∫ z(t)''*Q*z(t) dt, z = [alpha-alpha_cmd; beta-beta_cmd; wx; wy; wz]');
for c_idx = 1:n_coeff
    fprintf('补偿系数 coeff = %-6g | 收敛曲线: %2d/%2d | 最坏代价 J = %.4f\n', ...
        coeff_values(c_idx), valid_counts(c_idx), num_sims, perf_metrics(c_idx));
end
[~, best_idx] = min(perf_metrics);
fprintf('当前候选中的最优补偿系数: coeff = %g (J = %.4f)\n', ...
    coeff_values(best_idx), perf_metrics(best_idx));
disp('==============================================');

metrics = struct();
metrics.coeff_values = coeff_values;
metrics.coeff_labels = coeff_labels;
metrics.perf_metrics = perf_metrics;
metrics.valid_counts = valid_counts;
metrics.worst_k = worst_k;
metrics.best_idx = best_idx;

%% 最坏扰动曲线
if strcmpi(batch.disturbance.type, 'bangbang')
    disp('正在重构并绘制最坏情形下的外部扰动曲线...');
    t_dist = linspace(0, batch.tend, 1000);

    for c_idx = 1:n_coeff
        k = worst_k(c_idx);
        worst_switch_times = batch.switch_times_pool(:, :, k);
        worst_init_signs   = batch.init_sign_pool(:, k);

        dist_history = zeros(3, numel(t_dist));
        for i = 1:numel(t_dist)
            dist_history(:, i) = build_bangbang_dist(t_dist(i), ...
                worst_switch_times, worst_init_signs, ...
                batch.disturbance.M_ub, batch.disturbance.M_lb);
        end

        save_disturbance_figures(t_dist, dist_history, save_folder, ...
            sprintf('Worst_Disturbance_coeff_%g', coeff_values(c_idx)), ...
            sprintf('coeff = %g 时最坏二次型代价对应的扰动', coeff_values(c_idx)));
    end
    fprintf('最坏扰动曲线已保存到【%s】。\n', save_folder);
end
end

% ----------------------------------------------------------------------
function c = coeff_colors(n)
% 候选不多时用固定色板，否则用 TURBO/PARULA。
base = [0.000 0.447 0.741;
        0.850 0.325 0.098;
        0.929 0.694 0.125;
        0.494 0.184 0.556];

if n <= size(base, 1)
    c = base(1:n, :);
else
    try
        c = turbo(n);
    catch
        c = parula(n);
    end
end
end
