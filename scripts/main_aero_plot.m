%% 气动特性绘图
%
%   读取单一气动数据表，绘制气动系数随攻角/舵偏的变化，以及标称速度下的
%   有量纲俯仰力矩与俯仰舵效。
%
%   输出：results/aero/
%   参见 AERO_CONFIG, AEROINTERPOLANT, MODEL_PARAMS。

add_project_paths();
setup_chinese_fonts();

%% 参数
cfg.alpha_range = -20:2:20;    % 攻角扫描 [deg]
cfg.alpha_nom   = 5;           % 舵偏特性对应的标称攻角 [deg]
cfg.delta_range = -30:2:30;    % 舵偏扫描 [deg]
cfg.V_nom       = [];          % 标称速度 [m/s]，留空则取 7 倍音速

out_dir = output_dir('aero');

model = model_params();
acfg  = aero_config();
mdl   = AeroInterpolant(acfg.data_file, acfg.sheet);

if isempty(cfg.V_nom)
    cfg.V_nom = 7 * model.sound_speed;
end

q = 0.5 * model.rho * cfg.V_nom^2;
S_ref = model.ref_area;
L_ref = model.ref_length;

%% 计算气动系数
C_alpha = zeros(6, numel(cfg.alpha_range));
dCmz_dd2_alpha = zeros(1, numel(cfg.alpha_range));

for i = 1:numel(cfg.alpha_range)
    [C, dC] = mdl.evaluate(cfg.alpha_range(i), 0, 0, 0, 0);
    C_alpha(:, i) = C;
    dCmz_dd2_alpha(i) = dC(6, 2);
end

C_delta  = zeros(6, numel(cfg.delta_range));
dC_delta = zeros(6, 3, numel(cfg.delta_range));

for i = 1:numel(cfg.delta_range)
    [C, ~] = mdl.evaluate(cfg.alpha_nom, 0, 0, cfg.delta_range(i), 0);
    C_delta(:, i) = C;
    dC_delta(:, :, i) = dC;
end

%% 绘图
% 1. 法向力系数 Cy ~ alpha
fig = figure('Visible', 'off', 'Name', 'Cy_alpha');
ax = axes('Parent', fig);
plot(ax, cfg.alpha_range, C_alpha(2, :), 'b-o', 'LineWidth', 1.5);
grid(ax, 'on'); box(ax, 'on');
xlabel(ax, '攻角 \alpha / deg'); ylabel(ax, 'C_y');
title(ax, '法向力系数');
save_figure_3formats(fig, fullfile(out_dir, '01_Cy_alpha'));

% 2. 俯仰力矩系数 Cmz ~ alpha
fig = figure('Visible', 'off', 'Name', 'Cmz_alpha');
ax = axes('Parent', fig);
plot(ax, cfg.alpha_range, C_alpha(6, :), 'b-o', 'LineWidth', 1.5);
grid(ax, 'on'); box(ax, 'on');
xlabel(ax, '攻角 \alpha / deg'); ylabel(ax, 'C_{mz}');
title(ax, '俯仰力矩系数');
save_figure_3formats(fig, fullfile(out_dir, '02_Cmz_alpha'));

% 3. 俯仰力矩系数 Cmz ~ delta_2
fig = figure('Visible', 'off', 'Name', 'Cmz_delta2');
ax = axes('Parent', fig);
plot(ax, cfg.delta_range, C_delta(6, :), 'r-o', 'LineWidth', 1.5);
grid(ax, 'on'); box(ax, 'on');
xlabel(ax, '\delta_2 / deg'); ylabel(ax, 'C_{mz}');
title(ax, sprintf('俯仰力矩系数（\\alpha = %.0f°）', cfg.alpha_nom));
save_figure_3formats(fig, fullfile(out_dir, '03_Cmz_delta2'));

% 4. 滚转力矩系数 Cmx ~ delta_2
fig = figure('Visible', 'off', 'Name', 'Cmx_delta2');
ax = axes('Parent', fig);
plot(ax, cfg.delta_range, C_delta(4, :), 'r-o', 'LineWidth', 1.5);
grid(ax, 'on'); box(ax, 'on');
xlabel(ax, '\delta_2 / deg'); ylabel(ax, 'C_{mx}');
title(ax, sprintf('滚转力矩系数（\\alpha = %.0f°）', cfg.alpha_nom));
save_figure_3formats(fig, fullfile(out_dir, '04_Cmx_delta2'));

% 5. 有量纲俯仰力矩 Mz ~ delta_2
fig = figure('Visible', 'off', 'Name', 'Mz_delta2');
ax = axes('Parent', fig);
plot(ax, cfg.delta_range, C_delta(6, :) * q * S_ref * L_ref, 'r-o', 'LineWidth', 1.5);
grid(ax, 'on'); box(ax, 'on');
xlabel(ax, '\delta_2 / deg'); ylabel(ax, 'M_z / (N\cdotm)');
title(ax, sprintf('俯仰力矩（V = %.0f m/s）', cfg.V_nom));
save_figure_3formats(fig, fullfile(out_dir, '05_Mz_delta2'));

% 6. 俯仰舵效 dCmz/ddelta_2 ~ alpha
fig = figure('Visible', 'off', 'Name', 'dCmz_dd2_alpha');
ax = axes('Parent', fig);
plot(ax, cfg.alpha_range, dCmz_dd2_alpha, 'k-s', 'LineWidth', 1.5);
grid(ax, 'on'); box(ax, 'on');
xlabel(ax, '攻角 \alpha / deg'); ylabel(ax, '\partial C_{mz} / \partial \delta_2');
title(ax, '俯仰舵效');
save_figure_3formats(fig, fullfile(out_dir, '06_dCmz_dd2_alpha'));

fprintf('气动特性图已保存到【%s】。\n', out_dir);