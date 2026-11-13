function [figs, axs] = create_analysis_figures(names, ylabels, titles)
%FIGURES 创建“每个变量一张独立图”的图窗与坐标轴。
%
%   [FIGS, AXS] = FIGURES(NAMES, YLABELS, TITLES)
%
%   NAMES    1×N cell，图窗名称（同时用于保存文件名）
%   YLABELS  1×N cell，纵轴标签
%   TITLES   1×N cell，图标题
%
%   返回的 FIG/AXS 均为 1×N cell。图窗默认不可见，批量导出时不弹窗。
%
%   参见 DRAW_ENVELOPE, MAIN_SINGLE_RUN, MAIN_CONTROLLER_COMPARE。

n = numel(names);
figs = cell(1, n);
axs  = cell(1, n);

for i = 1:n
    fig = figure('Visible', 'off', 'Name', names{i});
    ax  = axes('Parent', fig);
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    xlabel(ax, 't/s');
    ylabel(ax, ylabels{i});
    title(ax, titles{i}, 'FontSize', 9, 'FontWeight', 'normal');
    figs{i} = fig;
    axs{i}  = ax;
end
end
