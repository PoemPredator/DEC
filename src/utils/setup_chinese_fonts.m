function setup_chinese_fonts()
%FONTS 设置 MATLAB 根级默认中文字体，避免导出图片中文乱码/缺字。

font_name = 'Microsoft YaHei';
try
    set(0, 'DefaultAxesFontName', font_name);
    set(0, 'DefaultTextFontName', font_name);
    set(0, 'DefaultAxesFontSize', 9);
catch
    % 旧版本或字体缺失时静默回退，不打断绘图
end
end
