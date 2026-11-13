function out = output_dir(varargin)
%OUTPUT_DIR 返回（并在需要时创建）results/ 下的输出目录。
%
%   OUT = OUTPUT_DIR()             -> <root>/results
%   OUT = OUTPUT_DIR('single')     -> <root>/results/single
%   OUT = OUTPUT_DIR('a', 'b')     -> <root>/results/a/b
%
%   所有图片与 MAT 结果统一写入 results/，便于 .gitignore 统一忽略。
%
%   参见 PROJECT_ROOT。

root = project_root();
out = fullfile(root, 'results', varargin{:});
if ~exist(out, 'dir')
    mkdir(out);
end
end
