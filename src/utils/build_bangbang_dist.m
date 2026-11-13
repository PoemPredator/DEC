function dist = build_bangbang_dist(t, switch_times, init_signs, M_ub, M_lb)
%BANGBANG 生成三分量 Bang-Bang 形式的外部扰动。
%
%   T            当前时刻（标量）
%   SWITCH_TIMES 3×N 矩阵，每一行为该通道的切换时刻
%   INIT_SIGNS   3×1 向量，每通道初始符号（-1 或 1）
%   M_UB, M_LB   3×1 向量，每通道上/下界
%
%   返回 3×1 扰动向量 [d_x; d_y; d_z]。
%
%   参见 RUN_BATCH_SIMULATIONS, MAIN_MONTE_CARLO。

dist = zeros(3, 1);
for i = 1:3
    num_switches_passed = sum(t >= switch_times(i, :));
    current_state = init_signs(i) * (-1)^num_switches_passed;
    if current_state > 0
        dist(i) = M_ub(i);
    else
        dist(i) = M_lb(i);
    end
end
end
