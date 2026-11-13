function [dxdt, u_acc, u_deg] = attitude_dynamics(t, x, p)
%DYNAMICS 18 维闭环姿态动力学 + 双 ESO 观测器。
%
%   [DXDT, U_ACC, U_DEG] = DYNAMICS(T, X, P)
%
%   状态（18×1）
%       x(1:3)    [alpha; beta; gammas]   物理姿态角          [rad]
%       x(4:6)    [wx; wy; wz]            机体角速度          [rad/s]
%       x(7:9)    hat_X1                  外环 ESO 姿态估计
%       x(10:12)  hat_d1                  外环 ESO 扰动估计
%       x(13:15)  hat_Z2                  内环 ESO 角速度误差估计
%       x(16:18)  hat_D2                  内环 ESO 集总扰动估计
%
%   输出
%       DXDT   状态导数
%       U_ACC  加速度域控制量（供线性化 / 裕度分析）
%       U_DEG  舵偏指令 [deg]
%
%   参数 P 由 SIMULATE_ATTITUDE 组装（模型参数来自 MODEL_PARAMS，
%   控制器参数来自 CONTROLLER_PARAMS）。
%
%   参见 ADRC_CONTROL, SIMULATE_ATTITUDE。

% 1. 提取状态
X1 = x(1:3);
X2 = x(4:6);

hat_X1 = x(7:9);
hat_d1 = x(10:12);
hat_Z2 = x(13:15);
hat_D2 = x(16:18);

x_eso = x(7:18);

% 2. 标称模型
[B1, f1, B2, f2] = nominal_model(X1, X2, p, t);

% 3. 控制律
[u_deg, ~, u_acc] = adrc_control(t, X1, X2, x_eso, p, B1, f1, B2, f2);

% 4. 真实动力学
[dX1, dX2] = true_dynamics(X1, X2, u_deg, p, t);

% 5. 外环 ESO
L11 = p.outer_eso_beta1 * eye(3);
L12 = p.outer_eso_beta2 * eye(3);
e1  = X1 - hat_X1;

dhat_X1 = B1 * X2 + f1 + hat_d1 + L11 * e1;
dhat_d1 = L12 * e1;

% 6. 内环 ESO（按角速度误差系统重构）
X2_cmd = zeros(3, 1);
Z2 = X2 - X2_cmd;
L21 = p.inner_eso_beta1 * eye(3);
L22 = p.inner_eso_beta2 * eye(3);
e2  = Z2 - hat_Z2;

dhat_Z2 = B2 * u_deg + f2 + hat_D2 + L21 * e2;
dhat_D2 = L22 * e2;

dxdt = [dX1; dX2; dhat_X1; dhat_d1; dhat_Z2; dhat_D2];
end

% ----------------------------------------------------------------------
function [B1, f1, B2, f2] = nominal_model(X1, X2, p, t)
    % 标称模型：B1(X1), f1(X1), B2(X1,X2), f2(X1,X2)
    alpha = X1(1); beta = X1(2); gammas = X1(3);
    wx = X2(1); wy = X2(2); wz = X2(3);

    V = p.V_func(t); theta = alpha;
    Q = 0.5 * p.rho * V^2 * p.ref_area;

    % 零舵偏气动力
    [C0, dC_ddelta] = p.aero_model.evaluate(rad2deg(alpha), rad2deg(beta), 0, 0, 0);
    C0_vec = C0(1:3);
    M0_vec = C0(4:6);

    % --- B1 ---
    B1 = [ -cos(alpha)*sin(beta)/cos(beta),  sin(alpha)*sin(beta)/cos(beta), 1;
            sin(alpha),                      cos(alpha),                     0;
            cos(alpha)/cos(beta),           -sin(alpha)/cos(beta),           0 ];

    % --- f1 ---
    G_term = (p.gravity / V) * [ cos(gammas)*cos(theta);
                                -sin(gammas)*cos(theta);
                                 cos(gammas)*cos(theta)*sin(beta)/cos(beta) ];
    Mat_aero1 = [ -1/cos(beta), 0;
                   0,           1;
                   sin(beta)/cos(beta) + sin(theta)*sin(gammas)/cos(theta), ...
                   sin(theta)*cos(gammas)/cos(theta) ];
    Mat_aero2 = [  sin(alpha)*cos(beta), cos(alpha), sin(alpha)*sin(beta);
                  -sin(beta),            0,          cos(beta) ];
    Aero_term0 = (1 / (p.mass * V)) * Mat_aero1 * Mat_aero2 * (Q * C0_vec);
    f1 = G_term + Aero_term0;

    % --- f2 ---
    Omega_mat = [ 0, -wz, wy; wz, 0, -wx; -wy, wx, 0 ];
    gyro_term = -p.inv_inertia * Omega_mat * p.inertia * X2;
    aero_moment_term0 = p.inv_inertia * (Q * p.ref_length * M0_vec);
    offset_moment_vec = [0; -C0_vec(3)*p.cg_offset_xz; C0_vec(2)*p.cg_offset_xz];
    offset_term0 = p.inv_inertia * (Q * offset_moment_vec);
    f2 = gyro_term + aero_moment_term0 + offset_term0;

    % --- B2 ---
    % 参考偏移力矩对舵偏的导数项按原设计置零，仅保留纯气动力矩舵效
    dOffset_ddelta = zeros(3);
    dM_ddelta = p.ref_length * dC_ddelta(4:6, :);

    B2 = p.inv_inertia * (Q * (dM_ddelta + dOffset_ddelta));
end

% ----------------------------------------------------------------------
function [dX1, dX2] = true_dynamics(X1, X2, u_deg, p, t)
    alpha = X1(1); beta = X1(2); gammas = X1(3);
    wx = X2(1); wy = X2(2); wz = X2(3);

    V = p.V_func(t); theta = p.theta_func(t);
    Q = 0.5 * p.rho * V^2 * p.ref_area;

    [C, ~] = p.aero_model.evaluate(rad2deg(alpha), rad2deg(beta), ...
                                   u_deg(1), u_deg(2), u_deg(3));
    C_vec = C(1:3);
    M_vec = C(4:6) .* p.hl(t);

    % --- dX1 ---
    B1 = [ -cos(alpha)*sin(beta)/cos(beta),  sin(alpha)*sin(beta)/cos(beta), 1;
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
    dX1 = B1 * X2 + G_term + Aero_term;

    % --- dX2 ---
    Omega_mat = [ 0, -wz, wy; wz, 0, -wx; -wy, wx, 0 ];
    gyro_term = -p.inv_inertia * Omega_mat * p.inertia * X2;
    aero_moment_term = p.inv_inertia * (Q * p.ref_length * M_vec);
    offset_moment_vec = [0; 0; 0];
    offset_term = p.inv_inertia * (Q * offset_moment_vec);
    dX2 = gyro_term + aero_moment_term + offset_term + p.w(t);
end
