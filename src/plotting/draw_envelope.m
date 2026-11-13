function draw_envelope(ax, t, y_mat, color, display_name)
%ENVELOPE 在坐标轴上绘制样本包络带与均值线。
%
%   ENVELOPE(AX, T, Y_MAT, COLOR, DISPLAY_NAME)
%
%   AX           目标坐标轴
%   T            1×M 时间向量
%   Y_MAT        N×M 样本矩阵（每行一条曲线；NaN 自动忽略）
%   COLOR        RGB 颜色
%   DISPLAY_NAME 均值曲线的图例名称
%
%   样本数大于 1 时绘制 max-min 包络带，否则只画均值线。

y_max  = max(y_mat, [], 1, 'omitnan');
y_min  = min(y_mat, [], 1, 'omitnan');
y_mean = mean(y_mat, 1, 'omitnan');

if size(y_mat, 1) > 1
    fill(ax, [t, fliplr(t)], [y_max, fliplr(y_min)], color, ...
        'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
end

plot(ax, t, y_mean, 'Color', color, 'LineWidth', 2.0, 'DisplayName', display_name);
end
