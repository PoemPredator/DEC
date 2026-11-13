function aero = aero_config(override)
%AERO_CONFIG 气动数据文件位置与工作表设置。
%
%   AERO = AERO_CONFIG() 返回默认配置：数据文件为 <工程根>/data/气动数据.xlsx。
%   AERO = AERO_CONFIG(OVERRIDE) 覆盖默认字段。
%
%   可通过环境变量指定路径
%       AERO_DATA_FILE  完整文件路径
%       AERO_DATA_DIR   数据目录（文件名仍取“气动数据.xlsx”）
%
%   字段
%       data_dir    气动数据目录
%       data_file   气动数据文件（单一数据表，不区分马赫数）
%       sheet       Excel 工作表名
%
%   数据列约定见 data/README.md 与 AEROINTERPOLANT。
%
%   参见 MODEL_PARAMS, LOAD_AERO_MODEL, SIMULATE_ATTITUDE。

root = fileparts(fileparts(mfilename('fullpath')));
default_dir  = fullfile(root, 'data');
default_file = fullfile(default_dir, '气动数据.xlsx');

env_file = getenv('AERO_DATA_FILE');
env_dir  = getenv('AERO_DATA_DIR');
if ~isempty(env_file)
    default_file = env_file;
elseif ~isempty(env_dir)
    default_file = fullfile(env_dir, '气动数据.xlsx');
end

aero = struct( ...
    'data_dir',  default_dir, ...
    'data_file', default_file, ...
    'sheet',     'Sheet1');

if nargin >= 1 && ~isempty(override)
    if ~isstruct(override)
        error('config:aero_config:badOverride', 'OVERRIDE 必须是结构体。');
    end
    names = fieldnames(override);
    for i = 1:numel(names)
        aero.(names{i}) = override.(names{i});
    end
end
end
