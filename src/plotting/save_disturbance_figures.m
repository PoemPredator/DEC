function save_disturbance_figures(t, dist_history, out_dir, base_prefix, title_prefix)
%DISTURBANCE 把三通道 Bang-Bang 扰动历史分别保存为独立图。
%
%   T            1×M 时间向量
%   DIST_HISTORY 3×M 扰动矩阵（行对应 d_x, d_y, d_z）
%   OUT_DIR      保存目录
%   BASE_PREFIX  文件名前缀（自动追加 _dx/_dy/_dz）
%   TITLE_PREFIX 图标题前缀
%
%   参见 MAIN_MONTE_CARLO, +ANALYSIS/ENVELOPE_REPORT。

comp_names   = {'dx', 'dy', 'dz'};
comp_colors  = {'r', 'g', 'b'};
comp_ylabels = {'d_x', 'd_y', 'd_z'};

for comp = 1:3
    fig = figure('Visible', 'off', ...
        'Name', sprintf('%s %s', title_prefix, comp_names{comp}));
    ax = axes('Parent', fig);
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');

    stairs(ax, t, dist_history(comp, :), comp_colors{comp}, 'LineWidth', 1.5);
    xlabel(ax, 't/s');
    ylabel(ax, comp_ylabels{comp});
    title(ax, sprintf('%s %s', title_prefix, comp_names{comp}));
    xline(ax, 0.1, '--k', 't=0.1s', 'LabelHorizontalAlignment', 'left', ...
        'HandleVisibility', 'off', 'LineWidth', 1.2);
    ylim(ax, [-18, 18]);

    save_figure_3formats(fig, fullfile(out_dir, ...
        sprintf('%s_%s', base_prefix, comp_names{comp})));
    close(fig);
end
end
