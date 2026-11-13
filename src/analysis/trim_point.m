function out = trim_point(alpha_deg, p, opts)
%TRIM_POINT 单一攻角下的配平点求解（力矩平衡 + 竖直力残差检查）。
%
%   OUT = TRIM_POINT(ALPHA_DEG, P, OPTS)
%
%   P 必须包含
%       .aero_model  AEROINTERPOLANT 对象
%       .V           速度                                         [m/s]
%       .theta       弹道倾角（沿用原程序约定）
%   以及 MODEL_PARAMS 的质量、惯量、参考量等字段。
%
%   OPTS
%       .mode      1 = 力矩 + 竖直力联合（默认，与原始程序一致）
%                  2 = 纯力矩配平
%       .delta0    舵偏初值 [deg]，默认 [0;0;0]
%       .limit_deg 舵偏上下限 [deg]，默认 30
%       .verbose   是否显示 fmincon 迭代，默认 false
%
%   OUT
%       .alpha_deg .delta_deg .rate_res .fz_res .vertical_accel .angle_rates
%
%   依赖 Optimization Toolbox。
%
%   参见 MAIN_TRIM, MODEL_PARAMS, AEROINTERPOLANT。

if nargin < 3 || isempty(opts)
    opts = struct();
end
if ~isfield(opts, 'mode'),      opts.mode = 1;            end
if ~isfield(opts, 'delta0'),    opts.delta0 = [0; 0; 0];  end
if ~isfield(opts, 'limit_deg'), opts.limit_deg = 30;      end
if ~isfield(opts, 'verbose'),   opts.verbose = false;     end

beta_deg = 0;
w_phys = [0; 0; 0];

if opts.mode == 2
    obj_fun = @(d) sum(moment_residual(d, alpha_deg, beta_deg, w_phys, p).^2);
else
    obj_fun = @(d) sum(joint_residual(d, alpha_deg, beta_deg, w_phys, p).^2);
end

display_mode = 'off';
if opts.verbose
    display_mode = 'iter';
end
fmincon_options = optimoptions('fmincon', 'Display', display_mode, ...
    'Algorithm', 'sqp', 'StepTolerance', 1e-8, 'OptimalityTolerance', 1e-6);

lb = -opts.limit_deg * ones(3, 1);
ub =  opts.limit_deg * ones(3, 1);

[delta_opt, ~] = fmincon(obj_fun, opts.delta0, [], [], [], [], ...
    lb, ub, [], fmincon_options);

[w_rates, fz_res, angle_rates] = residual_terms(delta_opt, alpha_deg, beta_deg, w_phys, p);

out = struct();
out.alpha_deg      = alpha_deg;
out.delta_deg      = delta_opt(:).';
out.rate_res       = w_rates(:).';
out.fz_res         = fz_res;
out.vertical_accel = fz_res / p.mass;
out.angle_rates    = angle_rates(:).';
end

% ----------------------------------------------------------------------
function w_rates = moment_residual(delta_deg, alpha_deg, beta_deg, w_phys, p)
    [w_rates, ~, ~] = residual_terms(delta_deg, alpha_deg, beta_deg, w_phys, p);
end

function residual = joint_residual(delta_deg, alpha_deg, beta_deg, w_phys, p)
    [w_rates, fz_res, ~] = residual_terms(delta_deg, alpha_deg, beta_deg, w_phys, p);
    weight = 10;
    fz_normalized = (fz_res / (p.mass * p.gravity)) * weight;
    residual = [w_rates; fz_normalized];
end

function [w_rates, fz_res, angle_rates] = residual_terms(delta_deg, alpha_deg, ...
                                                          beta_deg, w_phys, p)
    alpha = deg2rad(alpha_deg);
    beta  = deg2rad(beta_deg);
    gammas = 0;
    theta = p.theta;

    V = p.V;
    Q = 0.5 * p.rho * V^2 * p.ref_area;

    [C, ~] = p.aero_model.evaluate(alpha_deg + rad2deg(p.theta), beta_deg, ...
        delta_deg(1), delta_deg(2), delta_deg(3));
    C_vec = C(1:3);
    M_vec = C(4:6);

    % 角加速度
    Omega = [ 0,        -w_phys(3),  w_phys(2);
              w_phys(3),  0,        -w_phys(1);
             -w_phys(2),  w_phys(1),  0 ];
    gyro = -p.inv_inertia * Omega * p.inertia * w_phys;
    aero_moment = p.inv_inertia * (Q * p.ref_length * M_vec);
    offset_moment_vec = [0; -C_vec(3)*p.cg_offset_xz; C_vec(2)*p.cg_offset_xz];
    offset = p.inv_inertia * (Q * offset_moment_vec);
    w_rates = gyro + aero_moment + offset;

    % 竖直力平衡
    Fx = Q * C_vec(1);
    Fz = Q * C_vec(2);
    F_vert = Fz * cos(theta) + Fx * sin(theta);
    fz_res = F_vert - p.mass * p.gravity;

    % 角度导数（运动学验证）
    Mat_w = [ -cos(alpha)*sin(beta)/cos(beta),  sin(alpha)*sin(beta)/cos(beta), 1;
               sin(alpha),                      cos(alpha),                     0;
               cos(alpha)/cos(beta),           -sin(alpha)/cos(beta),           0 ];
    G_term = (p.gravity / V) * [ cos(gammas)*cos(theta);
                                -sin(gammas)*cos(theta);
                                 cos(gammas)*cos(theta)*sin(beta)/cos(beta) ];
    Mat_aero1 = [ -1/cos(beta), 0;
                   0,           1;
                   sin(beta)/cos(beta) + sin(theta)*sin(gammas)/cos(theta), ...
                   sin(theta)*cos(gammas)/cos(theta) ];
    Mat_aero2 = [  sin(alpha)*cos(beta), cos(alpha), sin(alpha)*sin(beta);
                  -sin(beta),            0,          cos(beta) ];
    Aero_term = (1 / (p.mass * V)) * Mat_aero1 * Mat_aero2 * (Q * C_vec);
    angle_rates = Mat_w * w_phys + G_term + Aero_term;
end
