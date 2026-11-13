function model = model_params(override)
%MODEL_PARAMS 飞行器模型参数（统一入口）。
%
%   MODEL = MODEL_PARAMS() 返回模型参数结构体。
%   MODEL = MODEL_PARAMS(OVERRIDE) 用 OVERRIDE 中的同名字段覆盖默认值。
%
%   参数数值保存在 data/model_params.mat 中（不纳入版本管理）；若该文件
%   不存在，则返回一组中性演示值并给出提示，仅用于保证接口可运行。
%
%   字段（SI 单位）
%       gravity        重力加速度          [m/s^2]
%       mass           质量                [kg]
%       inertia        转动惯量矩阵        [kg*m^2]
%       inv_inertia    I 的逆矩阵（派生）
%       cg_offset_xz   质心 x-z 偏移       [m]
%       ref_area       参考面积            [m^2]
%       ref_length     参考长度            [m]
%       rho            大气密度            [kg/m^3]
%       sound_speed    当地音速            [m/s]
%
%   示例
%       model = model_params();
%       model = model_params(struct('mass', 55));
%
%   参见 CONTROLLER_PARAMS, AERO_CONFIG, SIMULATE_ATTITUDE。

mat_file = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
                    'data', 'model_params.mat');

if exist(mat_file, 'file') == 2
    S = load(mat_file);
    model = S;
    if ~isfield(model, 'inv_inertia') || isempty(model.inv_inertia)
        model.inv_inertia = inv(model.inertia);
    end
else
    model = demo_defaults();
    warn_missing_file();
end

if nargin >= 1 && ~isempty(override)
    if ~isstruct(override)
        error('config:model_params:badOverride', 'OVERRIDE 必须是结构体。');
    end
    names = fieldnames(override);
    for i = 1:numel(names)
        model.(names{i}) = override.(names{i});
    end
    model.inv_inertia = inv(model.inertia);
end
end

% ----------------------------------------------------------------------
function model = demo_defaults()
% 中性演示值：仅保证接口可运行，不代表真实飞行器参数。
model = struct( ...
    'gravity',      9.81, ...
    'mass',         1, ...
    'cg_offset_xz', 0, ...
    'ref_area',     1, ...
    'ref_length',   1, ...
    'rho',          1, ...
    'sound_speed',  340);

model.inertia = eye(3);
model.inv_inertia = inv(model.inertia);
end

function warn_missing_file()
persistent warned
if isempty(warned)
    warning('model_params:missingFile', ...
        ['未找到 data/model_params.mat，已使用中性演示参数。\n' ...
         '请按 config/model_params.m 的字段说明提供本地参数文件。']);
    warned = true;
end
end
