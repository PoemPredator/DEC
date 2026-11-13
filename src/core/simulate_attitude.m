function [t, x, u, p] = simulate_attitude(bias)
%SIMULATE 单次姿态控制闭环仿真。
%
%   [T, X, U, P] = SIMULATE(BIAS)
%
%   BIAS 必填字段
%       tend         仿真时长                              [s]
%       alpha0       初始攻角                              [rad]
%       beta0        初始侧滑角                            [rad]
%       gammas0      初始速度滚转角                        [rad]
%       wx0,wy0,wz0  初始机体角速度                        [rad/s]
%       V            速度函数句柄 V(t)                     [m/s]
%       coeff        ESO 补偿系数（0 = PD，1 = 全补偿）
%       hl           气动力矩拉偏函数句柄 hl(t)，返回 3×1
%       disturbance  外部扰动函数句柄 w(t)，返回 3×1
%
%   BIAS 可选字段（覆盖默认配置）
%       model       传给 MODEL_PARAMS 的结构体
%       controller  传给 CONTROLLER_PARAMS 的结构体
%       aero        传给 AERO_CONFIG 的结构体
%
%   输出
%       T  时间向量
%       X  N×18 状态矩阵
%       U  N×3 舵偏角 [deg]
%       P  本次仿真使用的完整参数结构体
%
%   示例
%       bias = struct('tend', 0.2, 'alpha0', deg2rad(2), 'beta0', deg2rad(4), ...
%           'gammas0', 0, 'wx0', 0, 'wy0', 0, 'wz0', deg2rad(25), ...
%           'V', @(t) 7*340.23, 'coeff', 1, ...
%           'hl', @(t) ones(3,1), 'disturbance', @(t) zeros(3,1));
%       [t, x, u] = simulate_attitude(bias);
%
%   参见 ATTITUDE_DYNAMICS, MODEL_PARAMS, CONTROLLER_PARAMS, AERO_CONFIG。

% 0. 输入检查
if nargin < 1 || isempty(bias)
    error('sim:simulate:bias', '必须提供工况结构体 bias。');
end
required = {'tend', 'alpha0', 'beta0', 'gammas0', ...
            'wx0', 'wy0', 'wz0', 'V', 'coeff', 'hl', 'disturbance'};
for i = 1:numel(required)
    if ~isfield(bias, required{i})
        error('sim:simulate:missingField', 'bias 缺少必填字段 "%s"。', required{i});
    end
end

% 1. 读取配置
mdl = model_params();
if isfield(bias, 'model') && ~isempty(bias.model)
    mdl = model_params(bias.model);
end

ctrl = controller_params();
if isfield(bias, 'controller') && ~isempty(bias.controller)
    ctrl = controller_params(bias.controller);
end

acfg = aero_config();
if isfield(bias, 'aero') && ~isempty(bias.aero)
    acfg = aero_config(bias.aero);
end

% 2. 组装参数结构体
p = mdl;
p.Kp = ctrl.Kp;
p.Kd = ctrl.Kd;
p.outer_eso_beta1 = ctrl.outer_eso_beta1;
p.outer_eso_beta2 = ctrl.outer_eso_beta2;
p.inner_eso_beta1 = ctrl.inner_eso_beta1;
p.inner_eso_beta2 = ctrl.inner_eso_beta2;
p.u_limit_deg = ctrl.u_limit_deg;

p.alpha_cmd  = deg2rad(ctrl.alpha_cmd_deg);
p.beta_cmd   = deg2rad(ctrl.beta_cmd_deg);
p.gammas_cmd = deg2rad(ctrl.gammas_cmd_deg);

p.aero_model = load_aero_model(acfg);

p.V_func     = bias.V;
p.theta_func = @(t) 0;
p.w          = bias.disturbance;
p.hl         = bias.hl;
p.coeff      = bias.coeff;

% 3. 18 维初始状态
X1_0 = [bias.alpha0; bias.beta0; bias.gammas0];
X2_0 = [bias.wx0; bias.wy0; bias.wz0];

x0 = [X1_0; X2_0; ...
      X1_0; zeros(3, 1); ...
      zeros(3, 1); zeros(3, 1)];

% 4. 积分
tspan = [0, bias.tend];
[t, x] = ode45(@(tt, xx) attitude_dynamics(tt, xx, p), tspan, x0);

% 5. 舵偏输出（控制律为状态与时间的确定性函数，逐点回代即可）
u = zeros(numel(t), 3);
for k = 1:numel(t)
    [~, ~, u_k] = attitude_dynamics(t(k), x(k, :).', p);
    u(k, :) = u_k.';
end
end
