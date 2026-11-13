function save_figure_3formats(fig, base_path)
%SAVE_FIGURE 把一个 figure 保存为 png / fig / eps 三种格式。
%
%   FIG        figure 句柄
%   BASE_PATH  不带扩展名的完整保存路径
%
%   目录不存在时自动创建。

if ~isgraphics(fig, 'figure')
    error('sim:save_figure:notFigure', '第一个输入必须是 figure 句柄。');
end

[parent_dir, ~, ~] = fileparts(base_path);
if ~isempty(parent_dir) && ~exist(parent_dir, 'dir')
    mkdir(parent_dir);
end

fig.Color = 'w';
try
    fig.InvertHardcopy = 'off';
catch
end

exportgraphics(fig, [base_path, '.png'], 'Resolution', 300);
saveas(fig, [base_path, '.fig']);

old_renderer = fig.Renderer;
fig.Renderer = 'painters';
print(fig, [base_path, '.eps'], '-depsc', '-r300');
fig.Renderer = old_renderer;
end
