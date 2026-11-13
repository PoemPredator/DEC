function J = compute_tracking_cost(t, x, cfg)
%COMPUTE_COST 过程积分平方误差 ISE（越小越好）。
%
%   J   : 标量代价
%   T   : N×1 时间向量
%   X   : N×18 状态矩阵（前 6 列为物理状态，rad 与 rad/s）
%   CFG : 可选配置
%         .alpha_cmd_deg    攻角指令           [deg]，默认 3
%         .beta_cmd_deg     侧滑角指令         [deg]，默认 0
%         .angle_scale_deg  气流角归一化尺度   [deg]，默认 3
%         .rate_scale_degs  角速度归一化尺度   [deg/s]，默认 25
%
%   定义
%       z(t) = [alpha-alpha_cmd; beta-beta_cmd; wx; wy; wz]
%       J    = ∫ z(t)' * Q * z(t) dt
%       Q    = diag([1/a^2, 1/a^2, 1/r^2, 1/r^2, 1/r^2])
%
%   参见 +ANALYSIS/ENVELOPE_REPORT, +ANALYSIS/COEFF_WORST_CASE。

if nargin < 3 || isempty(cfg)
    cfg = struct();
end
if ~isfield(cfg, 'alpha_cmd_deg'),   cfg.alpha_cmd_deg   = 3;   end
if ~isfield(cfg, 'beta_cmd_deg'),    cfg.beta_cmd_deg    = 0;   end
if ~isfield(cfg, 'angle_scale_deg'), cfg.angle_scale_deg = 3;   end
if ~isfield(cfg, 'rate_scale_degs'), cfg.rate_scale_degs = 25;  end

if isempty(x)
    J = inf;
    return;
end

alpha_deg = rad2deg(x(:, 1));
beta_deg  = rad2deg(x(:, 2));
wx_deg    = rad2deg(x(:, 4));
wy_deg    = rad2deg(x(:, 5));
wz_deg    = rad2deg(x(:, 6));

z = [alpha_deg - cfg.alpha_cmd_deg, ...
     beta_deg  - cfg.beta_cmd_deg, ...
     wx_deg, wy_deg, wz_deg];

a = cfg.angle_scale_deg;
r = cfg.rate_scale_degs;
w = [1/a^2, 1/a^2, 1/r^2, 1/r^2, 1/r^2];
J = trapz(t, sum(z.^2 .* w, 2));
end
