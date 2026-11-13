function [t, x, u] = run_initial_case(alpha0_deg, wz0_deg, tend, V_func)
%RUN_INITIAL_CASE 单个“初始攻角 + 初始俯仰角速度”工况的仿真。
%
%   [T, X, U] = RUN_INITIAL_CASE(ALPHA0_DEG, WZ0_DEG, TEND, V_FUNC)
%
%   其余初始条件（beta, gammas, wx, wy）取零，补偿系数取 1，无拉偏无外扰。
%   单独成文件以便在 parfor 中调用。
%
%   参见 MAIN_INITIAL_SWEEP, MAKE_BIAS, SIMULATE_ATTITUDE。

state_deg = struct('alpha', alpha0_deg, 'beta', 0, 'gammas', 0, ...
                   'wx', 0, 'wy', 0, 'wz', wz0_deg);
bias = make_bias(state_deg, tend, V_func);
[t, x, u] = simulate_attitude(bias);
end