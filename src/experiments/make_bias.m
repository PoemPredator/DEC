function bias = make_bias(state_deg, tend, V_func, opts)
%MAKE_BIAS 由“角度制初值 + 时间 + 速度剖面”构造 SIMULATE_ATTITUDE 的工况结构体。
%
%   BIAS = MAKE_BIAS(STATE_DEG, TEND, V_FUNC)
%   BIAS = MAKE_BIAS(STATE_DEG, TEND, V_FUNC, OPTS)
%
%   STATE_DEG 结构体字段（角度制）
%       .alpha .beta .gammas .wx .wy .wz
%   TEND   仿真时长 [s]
%   V_FUNC 速度函数句柄 V(t) [m/s]
%   OPTS   可选覆盖
%       .coeff        补偿系数（默认 1）
%       .hl           气动力矩拉偏函数句柄（默认全 1）
%       .disturbance  外部扰动函数句柄（默认零）
%       .model        模型参数覆盖结构体
%       .controller   控制器参数覆盖结构体
%       .aero         气动配置覆盖结构体
%
%   返回的 BIAS 可直接传给 SIMULATE_ATTITUDE。

if nargin < 4 || isempty(opts)
    opts = struct();
end

bias = struct();
bias.tend    = tend;
bias.alpha0  = deg2rad(state_deg.alpha);
bias.beta0   = deg2rad(state_deg.beta);
bias.gammas0 = deg2rad(state_deg.gammas);
bias.wx0     = deg2rad(state_deg.wx);
bias.wy0     = deg2rad(state_deg.wy);
bias.wz0     = deg2rad(state_deg.wz);
bias.V       = V_func;

if isfield(opts, 'coeff') && ~isempty(opts.coeff)
    bias.coeff = opts.coeff;
else
    bias.coeff = 1;
end

if isfield(opts, 'hl') && ~isempty(opts.hl)
    bias.hl = opts.hl;
else
    bias.hl = @(t) ones(3, 1);
end

if isfield(opts, 'disturbance') && ~isempty(opts.disturbance)
    bias.disturbance = opts.disturbance;
else
    bias.disturbance = @(t) zeros(3, 1);
end

if isfield(opts, 'model') && ~isempty(opts.model)
    bias.model = opts.model;
end
if isfield(opts, 'controller') && ~isempty(opts.controller)
    bias.controller = opts.controller;
end
if isfield(opts, 'aero') && ~isempty(opts.aero)
    bias.aero = opts.aero;
end
end