%% PD 与 ADRC 控制效果对比
%
%   mode = 'nominal'          单次名义工况（无拉偏、无外扰），附带 Cmz 反推
%   mode = 'aero_dispersion'  ±20% 气动力矩拉偏 50 组，比较响应包络
%
%   输出：results/compare_<mode>/
%   参见 RUN_BATCH_SIMULATIONS, DRAW_ENVELOPE, CONTROLLER_PARAMS。

add_project_paths();
setup_chinese_fonts();

%% 参数
mode = 'nominal';

cfg.load_results = false;
cfg.use_parfor   = true;

model = model_params();
a_sound = model.sound_speed;

% 速度剖面：0.1 s 内由 4 倍音速加速到 7 倍音速
V_func = @(t) (t < 0.1) .* ((7 - 3 * (0.1 - t) / 0.1) * a_sound) ...
            + (t >= 0.1) .* (7 * a_sound);

switch mode
    case 'nominal'
        cfg.num_sims  = 1;
        cfg.coeff     = [0, 1];
        cfg.tend      = 0.5;
        cfg.dispersion = 0;
        cfg.subdir    = 'compare_nominal';
        cfg.titles = {'攻角跟踪对比', '侧滑角跟踪对比', '速度滚转角对比', ...
                      '滚转角速度对比', '偏航角速度对比', '俯仰角速度对比', ...
                      '方向舵偏指令对比', '左升降舵偏指令对比', '右升降舵偏指令对比'};
    case 'aero_dispersion'
        cfg.num_sims  = 50;
        cfg.coeff     = [0, 1];
        cfg.tend      = 1;
        cfg.dispersion = 0.2;
        cfg.subdir    = 'compare_aero_dispersion';
        cfg.titles = {'攻角包络', '侧滑角包络', '速度滚转角包络', ...
                      '滚转角速度包络', '偏航角速度包络', '俯仰角速度包络', ...
                      '方向舵偏指令包络', '左升降舵偏指令包络', '右升降舵偏指令包络'};
    otherwise
        error('run_compare:badMode', '未知的 mode: %s', mode);
end

out_dir = output_dir(cfg.subdir);
results_file = fullfile(output_dir(), sprintf('compare_%s.mat', mode));

%% 批量仿真
batch_cfg = struct();
batch_cfg.num_sims = cfg.num_sims;
batch_cfg.coeff_values = cfg.coeff;
batch_cfg.tend = cfg.tend;
batch_cfg.initial_state_deg = struct('alpha', 3, 'beta', 0, 'gammas', 0, ...
                                     'wx', 0, 'wy', 0, 'wz', 15);
batch_cfg.V_func = V_func;
batch_cfg.aero_dispersion_level = cfg.dispersion;
batch_cfg.disturbance = struct('type', 'none');
batch_cfg.seed = 42;
batch_cfg.use_parfor = cfg.use_parfor;

if cfg.load_results && exist(results_file, 'file')
    S = load(results_file);
    batch = S.batch;
    fprintf('已读取 %s，跳过仿真。\n', results_file);
else
    batch = run_batch_simulations(batch_cfg);
    save(results_file, 'batch', '-v7.3');
end

%% 绘制 9 张独立图
plot_names = {'01_alpha_攻角', '02_beta_侧滑角', '03_gamma_s_速度滚转角', ...
              '04_omega_x_滚转角速度', '05_omega_y_偏航角速度', '06_omega_z_俯仰角速度', ...
              '07_delta_n_方向舵偏指令', '08_delta_2_左升降舵偏指令', '09_delta_4_右升降舵偏指令'};
plot_ylabels = {'\alpha / deg', '\beta / deg', '\gamma_s / deg', ...
                '\omega_x / (deg/s)', '\omega_y / (deg/s)', '\omega_z / (deg/s)', ...
                '\delta_n / deg', '\delta_2 / deg', '\delta_4 / deg'};

colors = {[0.850 0.325 0.098], [0.000 0.447 0.741]};
labels = {'PD', 'ADRC'};

[figs, axs] = create_analysis_figures(plot_names, plot_ylabels, cfg.titles);

t_common = linspace(0, cfg.tend, 500);
n_coeff = numel(batch.coeff_values);

for c_idx = 1:n_coeff
    y_mat = NaN(cfg.num_sims, numel(t_common), 9);

    for k = 1:cfg.num_sims
        t = batch.t_all{k, c_idx};
        x = batch.x_all{k, c_idx};
        u = batch.u_all{k, c_idx};
        if isempty(t)
            continue;
        end

        [t_u, ia] = unique(t);
        y = state_to_deg(x(ia, :));

        for ch = 1:6
            y_mat(k, :, ch) = interp1(t_u, y(:, ch), t_common, 'linear', 'extrap');
        end
        if ~isempty(u)
            for ch = 1:3
                y_mat(k, :, 6 + ch) = interp1(t_u, u(ia, ch), t_common, ...
                    'linear', 'extrap');
            end
        end
    end

    for ch = 1:9
        if ~any(~isnan(y_mat(:, :, ch)), 'all')
            continue;
        end
        draw_envelope(axs{ch}, t_common, y_mat(:, :, ch), colors{c_idx}, labels{c_idx});
    end
end

for i = 1:9
    xline(axs{i}, 0.1, '--k', 't=0.1s', 'LabelHorizontalAlignment', 'left', ...
          'HandleVisibility', 'off', 'LineWidth', 1.2);
    legend(axs{i}, 'Location', 'best', 'FontName', 'Microsoft YaHei', 'FontSize', 9);
    save_figure_3formats(figs{i}, fullfile(out_dir, plot_names{i}));
end
fprintf('对比图已保存到【%s】。\n', out_dir);

%% 名义工况附加分析
if strcmp(mode, 'nominal')
    plot_cmz(batch, out_dir);
end

%% 终端性能
fprintf('\n================ 对比结果 ================\n');
for c_idx = 1:n_coeff
    t = batch.t_all{1, c_idx};
    x = batch.x_all{1, c_idx};
    [~, idx] = min(abs(t - 0.1));
    fprintf('coeff = %g | 终端 alpha = %.4f deg, beta = %.4f deg', ...
        batch.coeff_values(c_idx), rad2deg(x(end, 1)), rad2deg(x(end, 2)));
    fprintf(' | t≈0.1s alpha = %.4f deg, beta = %.4f deg\n', ...
        rad2deg(x(idx, 1)), rad2deg(x(idx, 2)));
end
disp('===========================================');

%% ======================================================================
function plot_cmz(batch, out_dir)
% 由名义轨迹反推 Cmz 与 d2 舵效的时变曲线

acfg = aero_config();
aero_model = load_aero_model(acfg);

colors = {[0.850 0.325 0.098], [0.000 0.447 0.741]};
labels = {'PD', 'ADRC'};

fig_cmz = figure('Visible', 'off', 'Name', 'Cmz');
ax_cmz = axes('Parent', fig_cmz);
hold(ax_cmz, 'on'); grid(ax_cmz, 'on'); box(ax_cmz, 'on');
xlabel(ax_cmz, 't/s'); ylabel(ax_cmz, 'C_{mz}');
title(ax_cmz, '俯仰力矩系数对比');

fig_dcmz = figure('Visible', 'off', 'Name', 'dCmz_dd2');
ax_dcmz = axes('Parent', fig_dcmz);
hold(ax_dcmz, 'on'); grid(ax_dcmz, 'on'); box(ax_dcmz, 'on');
xlabel(ax_dcmz, 't/s'); ylabel(ax_dcmz, '\partial C_{mz} / \partial \delta_2');
title(ax_dcmz, '\delta_2 舵效系数对比');

for c_idx = 1:numel(batch.coeff_values)
    t = batch.t_all{1, c_idx};
    x = batch.x_all{1, c_idx};
    u = batch.u_all{1, c_idx};

    Cmz_arr = zeros(length(t), 1);
    dCmz_arr = zeros(length(t), 1);

    for i = 1:length(t)
        if isempty(u)
            dn_i = 0; d2_i = 0; d4_i = 0;
        else
            dn_i = u(i, 1); d2_i = u(i, 2); d4_i = u(i, 3);
        end

        [C_i, dC_i] = aero_model.evaluate(rad2deg(x(i, 1)), rad2deg(x(i, 2)), ...
                                          dn_i, d2_i, d4_i);
        Cmz_arr(i) = C_i(6);
        dCmz_arr(i) = dC_i(6, 2);
    end

    plot(ax_cmz, t, Cmz_arr, 'Color', colors{c_idx}, ...
        'LineWidth', 1.5, 'DisplayName', labels{c_idx});
    plot(ax_dcmz, t, dCmz_arr, 'Color', colors{c_idx}, ...
        'LineWidth', 1.5, 'DisplayName', labels{c_idx});
end

xline(ax_cmz, 0.1, '--k', 't=0.1s', 'LabelHorizontalAlignment', 'left', ...
    'HandleVisibility', 'off', 'LineWidth', 1.2);
xline(ax_dcmz, 0.1, '--k', 't=0.1s', 'LabelHorizontalAlignment', 'left', ...
    'HandleVisibility', 'off', 'LineWidth', 1.2);
legend(ax_cmz, labels, 'Location', 'best', 'FontName', 'Microsoft YaHei');
legend(ax_dcmz, labels, 'Location', 'best', 'FontName', 'Microsoft YaHei');

save_figure_3formats(fig_cmz, fullfile(out_dir, '10_Cmz_俯仰力矩系数'));
save_figure_3formats(fig_dcmz, fullfile(out_dir, '11_dCmz_dd2_d2俯仰舵效'));
end
