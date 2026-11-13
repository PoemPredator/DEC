%% 配平点计算（力矩平衡 + 竖直力平衡检查）
%
%   在给定速度下扫描攻角，用 fmincon 求配平舵偏，输出 5 张结果图。
%   输出：results/trim/
%   依赖 Optimization Toolbox。
%   参见 TRIM_POINT, MODEL_PARAMS, AERO_CONFIG。

add_project_paths();
setup_chinese_fonts();

%% 参数
cfg.alpha_list = -10:0.5:20;    % 攻角扫描范围 [deg]
cfg.trim_mode  = 1;             % 1 = 力矩 + 竖直力联合（原始默认）；2 = 纯力矩
cfg.limit_deg  = 30;            % 舵偏限幅 [deg]
cfg.theta      = 15;            % 弹道倾角（沿用原始程序取值）

out_dir = output_dir('trim');

model = model_params();
acfg = aero_config();

p = model;
p.V = 4 * model.sound_speed;
p.theta = cfg.theta;
p.aero_model = AeroInterpolant(acfg.data_file, acfg.sheet);

%% 逐攻角配平
alpha_list = cfg.alpha_list;
n_alpha = numel(alpha_list);

delta_trim     = zeros(n_alpha, 3);
residual_w     = zeros(n_alpha, 3);
residual_fz    = zeros(n_alpha, 1);
vertical_accel = zeros(n_alpha, 1);
angle_rate_res = zeros(n_alpha, 3);

for i = 1:n_alpha
    out = trim_point(alpha_list(i), p, struct( ...
        'mode', cfg.trim_mode, 'limit_deg', cfg.limit_deg));

    delta_trim(i, :)     = out.delta_deg;
    residual_w(i, :)     = out.rate_res;
    residual_fz(i)       = out.fz_res;
    vertical_accel(i)    = out.vertical_accel;
    angle_rate_res(i, :) = out.angle_rates;

    fprintf(['alpha = %5.1f deg -> delta: [%6.2f, %6.2f, %6.2f] deg, ' ...
             '|w_dot| = %.2e rad/s^2, Fz 残差 = %.2f N\n'], ...
        alpha_list(i), out.delta_deg, norm(out.rate_res), out.fz_res);
end

%% 绘图
fig = figure('Name', '配平舵偏曲线', 'Visible', 'off');
ax = axes('Parent', fig);
hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
plot(ax, alpha_list, delta_trim(:, 1), 'b-o', 'LineWidth', 1.5);
plot(ax, alpha_list, delta_trim(:, 2), 'r-o', 'LineWidth', 1.5);
plot(ax, alpha_list, delta_trim(:, 3), 'g-o', 'LineWidth', 1.5);
xlabel(ax, '攻角 \alpha (deg)'); ylabel(ax, '舵偏角 (deg)');
legend(ax, {'\delta_n', '\delta_2', '\delta_4'}, 'Location', 'best');
title(ax, '配平舵偏随攻角变化');
save_figure_3formats(fig, fullfile(out_dir, 'Trim_Deflections'));

fig = figure('Name', '角加速度残差', 'Visible', 'off');
ax = axes('Parent', fig);
semilogy(ax, alpha_list, vecnorm(residual_w, 2, 2), 'k-s', 'LineWidth', 1.5);
grid(ax, 'on'); box(ax, 'on');
xlabel(ax, '攻角 \alpha (deg)'); ylabel(ax, '角加速度范数 (rad/s^2)');
title(ax, '力矩配平残差');
save_figure_3formats(fig, fullfile(out_dir, 'Angular_Accel_Norm'));

fig = figure('Name', '竖直力残差', 'Visible', 'off');
ax = axes('Parent', fig);
hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
plot(ax, alpha_list, residual_fz, 'r-', 'LineWidth', 2);
plot(ax, alpha_list, zeros(size(alpha_list)), 'k--', 'LineWidth', 1);
xlabel(ax, '攻角 \alpha (deg)'); ylabel(ax, '竖直力残差 (N)');
title(ax, '竖直力平衡残差 (F_z - mg)');
legend(ax, {'残差', '零线'}, 'Location', 'best');
save_figure_3formats(fig, fullfile(out_dir, 'Vertical_Force_Residual'));

fig = figure('Name', '竖直加速度', 'Visible', 'off');
ax = axes('Parent', fig);
hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
plot(ax, alpha_list, vertical_accel / p.gravity, 'b-', 'LineWidth', 1.5);
plot(ax, alpha_list, zeros(size(alpha_list)), 'k--');
xlabel(ax, '攻角 \alpha (deg)'); ylabel(ax, '竖直加速度 (m/s^2)');
title(ax, '竖直加速度');
legend(ax, {'a_z', '零线'}, 'Location', 'best');
save_figure_3formats(fig, fullfile(out_dir, 'Vertical_Accel'));

fig = figure('Name', '角度导数', 'Visible', 'off');
ax = axes('Parent', fig);
hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
plot(ax, alpha_list, rad2deg(angle_rate_res(:, 1)), 'b-', 'LineWidth', 1.2);
plot(ax, alpha_list, rad2deg(angle_rate_res(:, 2)), 'r-', 'LineWidth', 1.2);
plot(ax, alpha_list, rad2deg(angle_rate_res(:, 3)), 'g-', 'LineWidth', 1.2);
xlabel(ax, '攻角 \alpha (deg)'); ylabel(ax, '角度导数 (deg/s)');
legend(ax, {'d\alpha/dt', 'd\beta/dt', 'd\gamma_s/dt'}, 'Location', 'best');
title(ax, '配平点处角度导数（应尽量为 0）');
save_figure_3formats(fig, fullfile(out_dir, 'Angle_Rates'));

%% 表格与结果
T = table(alpha_list', delta_trim(:, 1), delta_trim(:, 2), delta_trim(:, 3), ...
          residual_fz, vertical_accel, ...
          'VariableNames', {'alpha_deg', 'delta_n_deg', 'delta_2_deg', ...
                            'delta_4_deg', 'fz_residual_N', 'vertical_accel_m_s2'});
disp('配平结果表:');
disp(T);

save(fullfile(out_dir, 'trim_results.mat'), ...
    'alpha_list', 'delta_trim', 'residual_w', 'residual_fz', ...
    'vertical_accel', 'angle_rate_res', 'T');

fprintf('配平结果已保存到【%s】。\n', out_dir);
