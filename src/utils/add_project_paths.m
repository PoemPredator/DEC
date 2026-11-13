function root = add_project_paths()
%ADD_PROJECT_PATHS 把工程源码与配置目录加入 MATLAB 搜索路径。
%
%   ROOT = ADD_PROJECT_PATHS() 返回工程根目录。
%
%   scripts/ 下的主脚本在开头调用本函数即可，无需关心当前文件夹。

root = project_root();
addpath(genpath(fullfile(root, 'src')));
addpath(fullfile(root, 'config'));
end