function y = state_to_deg(x)
%STATE_TO_DEG 把状态矩阵前 6 列转换为角度制。
%
%   Y = STATE_TO_DEG(X)
%   X : N×18 状态矩阵（前 6 列为 [alpha; beta; gammas; wx; wy; wz]，弧度制）
%   Y : N×6 矩阵，前 3 列为角度 [deg]，后 3 列为角速度 [deg/s]
%
%   绘图与收敛判据统一使用本函数。

y = rad2deg(x(:, 1:6));
end