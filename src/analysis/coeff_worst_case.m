function results = coeff_worst_case(batch, opts)
%COEFF_WORST_CASE 补偿系数的最坏情形优化与对比。
%
%   对每个候选补偿系数，用同一组 Bang-Bang 扰动样本做蒙特卡洛仿真，
%   取最坏扰动情形的过程积分平方误差 ISE 作为指标，找出最优系数。
%
%   RESULTS = COEFF_WORST_CASE(BATCH, OPTS)
%
%   OPTS
%       .out_dir   图片与结果输出目录（必填）
%       .cost      传给 COMPUTE_TRACKING_COST 的配置
%       .converge  .alpha_deg / .beta_deg（默认 5 / 5）
%
%   RESULTS
%       .coeff_candidates .worst_J .worst_k .valid_counts .J_all
%       .optimal_coeff .best_J .best_idx
%
%   参见 MAIN_MONTE_CARLO, RUN_BATCH_SIMULATIONS, ENVELOPE_REPORT。

if nargin < 2 || isempty(opts)
    opts = struct();
end
if ~isfield(opts, 'out_dir') || isempty(opts.out_dir)
    error('analysis:coeff_worst_case:outDir', '必须提供 opts.out_dir。');
end
if ~isfield(opts, 'cost') || isempty(opts.cost)
    opts.cost = struct();
end
if ~isfield(opts, 'converge') || isempty(opts.converge)
    opts.converge = struct('alpha_deg', 5, 'beta_deg', 5);
end

save_folder = opts.out_dir;
num_sims = batch.num_sims;
coeff_candidates = batch.coeff_values(:).';
n_coeff = numel(coeff_candidates);

%% 1. 逐系数统计最坏代价
J_all = nan(num_sims, n_coeff);
valid_counts = zeros(1, n_coeff);
worst_k = zeros(1, n_coeff);
worst_J = zeros(1, n_coeff);

for c_idx = 1:n_coeff
    max_J = -inf;
    best_k = NaN;

    for k = 1:num_sims
        t = batch.t_all{k, c_idx};
        x = batch.x_all{k, c_idx};
        if isempty(x)
            continue;
        end

        y = state_to_deg(x);
        if abs(y(end, 1)) > opts.converge.alpha_deg || ...
           abs(y(end, 2)) > opts.converge.beta_deg
            J_all(k, c_idx) = inf;
            continue;
        end

        J = compute_tracking_cost(t, x, opts.cost);
        J_all(k, c_idx) = J;
        valid_counts(c_idx) = valid_counts(c_idx) + 1;
        if J > max_J
            max_J = J;
            best_k = k;
        end
    end

    if isnan(best_k)
        best_k = 1;
        max_J = inf;
    end
    worst_k(c_idx) = best_k;
    worst_J(c_idx) = max_J;
end

[best_J, best_idx] = min(worst_J);
optimal_coeff = coeff_candidates(best_idx);

%% 2. 每个系数的最坏响应曲线
try
    cmap = turbo(n_coeff);
catch
    cmap = parula(n_coeff);
end
coeff_labels = arrayfun(@(v) sprintf('c=%.2f', v), coeff_candidates, ...
    'UniformOutput', false);
coeff_labels{best_idx} = sprintf('c=%.2f*', optimal_coeff);

plot_names = {'01_alpha_攻角响应', '02_beta_侧滑角响应', '03_gamma_s_速度滚转角响应', ...
              '04_omega_x_滚转角速度', '05_omega_y_偏航角速度', '06_omega_z_俯仰角速度', ...
              '07_delta_n_方向舵偏指令', '08_delta_2_左升降舵偏指令', '09_delta_4_右升降舵偏指令'};
plot_ylabels = {'\alpha / deg', '\beta / deg', '\gamma_s / deg', ...
                '\omega_x / (deg/s)', '\omega_y / (deg/s)', '\omega_z / (deg/s)', ...
                '\delta_n / deg', '\delta_2 / deg', '\delta_4 / deg'};
plot_titles = {'最坏攻角响应', '最坏侧滑角响应', '最坏速度滚转角响应', ...
               '最坏滚转角速度', '最坏偏航角速度', '最坏俯仰角速度', ...
               '最坏方向舵偏指令', '最坏左升降舵偏指令', '最坏右升降舵偏指令'};

[figs, axs] = create_analysis_figures(plot_names, plot_ylabels, plot_titles);

for c_idx = 1:n_coeff
    k = worst_k(c_idx);
    t = batch.t_all{k, c_idx};
    x = batch.x_all{k, c_idx};
    u = batch.u_all{k, c_idx};
    if isempty(t) || isempty(x)
        continue;
    end

    if c_idx == best_idx
        lw = 2.2;
    else
        lw = 1.0;
    end

    y = state_to_deg(x);
    for ch = 1:6
        plot(axs{ch}, t, y(:, ch), 'Color', cmap(c_idx, :), 'LineWidth', lw, ...
            'DisplayName', coeff_labels{c_idx});
    end
    if ~isempty(u)
        for ch = 1:3
            plot(axs{6 + ch}, t, u(:, ch), 'Color', cmap(c_idx, :), ...
                'LineWidth', lw, 'DisplayName', coeff_labels{c_idx});
        end
    end
end

for i = 1:9
    xline(axs{i}, 0.1, '--k', 't=0.1s', 'LabelHorizontalAlignment', 'left', ...
        'HandleVisibility', 'off', 'LineWidth', 1.2);
    legend(axs{i}, 'Location', 'eastoutside', ...
        'FontSize', 7, 'FontName', 'Microsoft YaHei');
    save_figure_3formats(figs{i}, fullfile(save_folder, plot_names{i}));
end

%% 3. 最坏代价随补偿系数的变化
fig_obj = figure('Name', '最坏代价 vs 补偿系数', 'Color', 'w', 'Visible', 'off');
ax_obj = axes('Parent', fig_obj);
hold(ax_obj, 'on'); grid(ax_obj, 'on'); box(ax_obj, 'on');
J_plot = worst_J;
J_plot(isinf(J_plot)) = NaN;
plot(ax_obj, coeff_candidates, J_plot, 'k-o', 'LineWidth', 1.8, 'MarkerFaceColor', 'k');
plot(ax_obj, optimal_coeff, best_J, 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
xlabel(ax_obj, '补偿系数 coeff');
ylabel(ax_obj, '最坏情形二次型代价 J');
title(ax_obj, '不同补偿系数下的最坏性能指标');
legend(ax_obj, {'J(coeff)', sprintf('最优 coeff=%.2f', optimal_coeff)}, ...
    'Location', 'best');
save_figure_3formats(fig_obj, fullfile(save_folder, '10_最坏代价随补偿系数变化'));

%% 4. 保存数值结果
results = struct();
results.coeff_candidates = coeff_candidates;
results.worst_J = worst_J;
results.worst_k = worst_k;
results.valid_counts = valid_counts;
results.J_all = J_all;
results.optimal_coeff = optimal_coeff;
results.best_J = best_J;
results.best_idx = best_idx;

save(fullfile(save_folder, 'coeff_optimization_worstcase_results.mat'), ...
    '-struct', 'results');

%% 5. 终端报告
fprintf('\n================ 最坏情形二次型优化结果 ================\n');
fprintf('指标: 过程积分平方误差 ISE, J = ∫ z(t)''*Q*z(t) dt\n');
for c_idx = 1:n_coeff
    fprintf('coeff = %6.2f | 收敛 %2d/%2d | 最坏 J = %8.4f | 最坏样本 k = %d\n', ...
        coeff_candidates(c_idx), valid_counts(c_idx), num_sims, ...
        worst_J(c_idx), worst_k(c_idx));
end
fprintf('----------------------------------------------------------\n');
fprintf('最优补偿系数 coeff* = %.4f, 最坏代价 J* = %.4f\n', optimal_coeff, best_J);
fprintf('==========================================================\n');

%% 6. 最优系数对应的最坏扰动曲线
if strcmpi(batch.disturbance.type, 'bangbang')
    best_k_opt = worst_k(best_idx);
    worst_switch_times = batch.switch_times_pool(:, :, best_k_opt);
    worst_init_signs   = batch.init_sign_pool(:, best_k_opt);

    t_dist = linspace(0, batch.tend, 1000);
    dist_history = zeros(3, numel(t_dist));
    for i = 1:numel(t_dist)
        dist_history(:, i) = build_bangbang_dist(t_dist(i), ...
            worst_switch_times, worst_init_signs, ...
            batch.disturbance.M_ub, batch.disturbance.M_lb);
    end

    save_disturbance_figures(t_dist, dist_history, save_folder, ...
        '11_最优系数最坏扰动', ...
        sprintf('最优 coeff=%.2f 的最坏扰动', optimal_coeff));
end

fprintf('\n所有图已保存到【%s】。\n', save_folder);
end
