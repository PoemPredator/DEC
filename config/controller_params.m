function ctrl = controller_params(override)
%CONTROLLER_PARAMS PD + 串级 ESO ADRC 控制器参数。
%
%   CTRL = CONTROLLER_PARAMS() 返回默认控制器参数。
%   CTRL = CONTROLLER_PARAMS(OVERRIDE) 用 OVERRIDE 中的同名字段覆盖默认值。
%
%   控制律（见 ADRC_CONTROL）
%       u_acc = -inv(B1) * Kp * (X1 - X1_cmd) - Kd * X2 - f2 - coeff * hat_D2
%
%   字段
%       omega_c, zeta           PD 闭环带宽 / 阻尼比
%       Kp, Kd                  姿态角 / 角速度增益（由 omega_c、zeta 派生）
%       omega_outer/omega_inner 外环 / 内环 ESO 带宽
%       outer_eso_beta1/2       外环 ESO 增益
%       inner_eso_beta1/2       内环 ESO 增益
%       alpha_cmd_deg           攻角指令            [deg]
%       beta_cmd_deg            侧滑角指令          [deg]
%       gammas_cmd_deg          速度滚转角指令      [deg]
%       u_limit_deg             舵偏限幅            [deg]
%
%   参见 MODEL_PARAMS, SIMULATE_ATTITUDE。

ctrl = struct();
ctrl.omega_c = 12;
ctrl.zeta    = 1;

ctrl.omega_outer = 40;
ctrl.omega_inner = 80;

ctrl.alpha_cmd_deg  = 3;
ctrl.beta_cmd_deg   = 0;
ctrl.gammas_cmd_deg = 0;

ctrl.u_limit_deg = 30;

if nargin >= 1 && ~isempty(override)
    if ~isstruct(override)
        error('config:controller_params:badOverride', 'OVERRIDE 必须是结构体。');
    end
    names = fieldnames(override);
    for i = 1:numel(names)
        ctrl.(names{i}) = override.(names{i});
    end
end

ctrl.Kp = diag([ctrl.omega_c^2, ctrl.omega_c^2, ctrl.omega_c^2]);
ctrl.Kd = diag([2*ctrl.zeta*ctrl.omega_c, 2*ctrl.zeta*ctrl.omega_c, ...
                2*ctrl.zeta*ctrl.omega_c]);
ctrl.outer_eso_beta1 = 2 * ctrl.omega_outer;
ctrl.outer_eso_beta2 = ctrl.omega_outer^2;
ctrl.inner_eso_beta1 = 2 * ctrl.omega_inner;
ctrl.inner_eso_beta2 = ctrl.omega_inner^2;
end
