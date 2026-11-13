function [u_deg, X2_cmd, u_acc] = adrc_control(t, X1, X2, x_eso, p, B1, f1, B2, f2) %#ok<INUSD>
%CONTROL PD + 前馈 + ESO 补偿控制律（无串级反步）。
%
%   [U_DEG, X2_CMD, U_ACC] = CONTROL(T, X1, X2, X_ESO, P, B1, F1, B2, F2)
%
%   输入
%       T         当前时刻
%       X1        姿态角 [alpha; beta; gammas]     [rad]
%       X2        机体角速度 [wx; wy; wz]          [rad/s]
%       X_ESO     ESO 状态（18 维状态的后 12 维）
%       P         参数结构体（Kp, Kd, coeff, 指令, 限幅）
%       B1,F1,B2,F2 标称模型矩阵，由 ATTITUDE_DYNAMICS 提供
%
%   输出
%       U_DEG   舵偏指令 [deg]（已限幅）
%       X2_CMD  角速度指令（本版本强制为零）
%       U_ACC   加速度域控制量（供线性化分析使用）
%
%   控制律
%       u_acc = -inv(B1) * Kp * Z1 - Kd * X2 - f2 - coeff * hat_D2
%       u_deg = B2 \ u_acc
%
%   参见 ATTITUDE_DYNAMICS, CONTROLLER_PARAMS。

% 1. ESO 总扰动估计（内环）
hat_D2 = x_eso(10:12);

% 2. 期望指令与姿态角误差
X1_cmd = [p.alpha_cmd; p.beta_cmd; p.gammas_cmd];
Z1 = X1 - X1_cmd;

% 3. 角速度指令强制为零，故角速度误差即 X2
X2_cmd = zeros(3, 1);

% 4. B1 求逆（奇异时退化为伪逆）
if rcond(B1) < 1e-6
    inv_B1 = pinv(B1);
else
    inv_B1 = B1 \ eye(3);
end

% 5. 加速度域控制律
u_acc = -inv_B1 * (p.Kp * Z1) - p.Kd * X2 - f2 - p.coeff * hat_D2;

% 可选注入信号：冻结线性化时在 u_acc 处断开回路
if isfield(p, 'u_inj') && ~isempty(p.u_inj)
    u_acc = u_acc + p.u_inj(:);
end

% 6. 控制效能反解舵偏并限幅
u_deg = B2 \ u_acc;
u_deg = max(min(u_deg, p.u_limit_deg), -p.u_limit_deg);
end
