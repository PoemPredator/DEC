function batch = run_batch_simulations(cfg)
%RUN_BATCH 批量（可选并行）运行“补偿系数 × 拉偏样本”仿真。
%
%   BATCH = RUN_BATCH(CFG)
%
%   CFG 必填字段
%       num_sims             每组补偿系数的样本数
%       coeff_values         1×N 补偿系数（0 = PD，1 = 全补偿）
%       tend                 仿真时长                          [s]
%       initial_state_deg    初始状态结构体（deg / deg/s）
%                            .alpha .beta .gammas .wx .wy .wz
%       V_func               速度函数句柄 V(t)                 [m/s]
%
%   CFG 可选字段
%       aero_dispersion_level 气动力矩拉偏幅度（0.2 = ±20%）
%       disturbance           外部扰动配置
%                             .type = 'none' | 'bangbang' | 'custom'
%                             .M_ub, .M_lb   3×1 上下界
%                             .N_switches    切换次数
%                             .fun           自定义函数句柄
%       seed                  随机数种子
%       use_parfor            是否并行
%       model_override        传给 MODEL_PARAMS 的覆盖结构体
%       controller_override   传给 CONTROLLER_PARAMS 的覆盖结构体
%
%   返回 BATCH 结构体
%       t_all / x_all / u_all    num_sims×n_coeff cell
%       coeff_values, num_sims, tend, initial_state_deg, V_func,
%       hl_pool, switch_times_pool, init_sign_pool, disturbance, seed
%
%   所有补偿系数共享同一组拉偏样本，保证比较公平。
%
%   参见 SIMULATE_ATTITUDE, BUILD_BANGBANG_DIST, MAIN_MONTE_CARLO, MAIN_CONTROLLER_COMPARE。

if nargin < 1 || isempty(cfg)
    cfg = struct();
end
cfg = fill_defaults(cfg);

num_sims      = cfg.num_sims;
coeff_values  = cfg.coeff_values;
initial_state = cfg.initial_state_deg;
dist_cfg      = cfg.disturbance;

% 1. 共享随机样本池
rng(cfg.seed);
hl_pool = 1 + cfg.aero_dispersion_level * (2 * rand(3, num_sims) - 1);

if strcmpi(dist_cfg.type, 'bangbang')
    switch_times_pool = rand(3, dist_cfg.N_switches, num_sims) * cfg.tend;
    switch_times_pool = sort(switch_times_pool, 2);
    init_sign_pool = sign(randn(3, num_sims));
    init_sign_pool(init_sign_pool == 0) = 1;
else
    switch_times_pool = [];
    init_sign_pool = [];
end

% 2. 工况模板
bias0 = struct();
bias0.tend        = cfg.tend;
bias0.alpha0      = deg2rad(initial_state.alpha);
bias0.beta0       = deg2rad(initial_state.beta);
bias0.gammas0     = deg2rad(initial_state.gammas);
bias0.wx0         = deg2rad(initial_state.wx);
bias0.wy0         = deg2rad(initial_state.wy);
bias0.wz0         = deg2rad(initial_state.wz);
bias0.V           = cfg.V_func;
bias0.coeff       = 0;
bias0.hl          = @(t) ones(3, 1);
bias0.disturbance = @(t) zeros(3, 1);
if ~isempty(cfg.model_override)
    bias0.model = cfg.model_override;
end
if ~isempty(cfg.controller_override)
    bias0.controller = cfg.controller_override;
end

% 3. 展平循环
n_coeff  = numel(coeff_values);
num_jobs = num_sims * n_coeff;

t_flat = cell(num_jobs, 1);
x_flat = cell(num_jobs, 1);
u_flat = cell(num_jobs, 1);

use_parfor = cfg.use_parfor && (exist('parpool', 'file') == 2);
if cfg.use_parfor && ~use_parfor
    warning('sim:run_batch:noPCT', ...
        '未检测到 Parallel Computing Toolbox，自动改为串行运行。');
end

if use_parfor
    parfor job = 1:num_jobs
        [t_k, x_k, u_k] = run_one(job, num_sims, coeff_values, bias0, ...
            hl_pool, switch_times_pool, init_sign_pool, dist_cfg);
        t_flat{job} = t_k;
        x_flat{job} = x_k;
        u_flat{job} = u_k;
    end
else
    for job = 1:num_jobs
        [t_k, x_k, u_k] = run_one(job, num_sims, coeff_values, bias0, ...
            hl_pool, switch_times_pool, init_sign_pool, dist_cfg);
        t_flat{job} = t_k;
        x_flat{job} = x_k;
        u_flat{job} = u_k;
    end
end

% 4. 输出
batch = struct();
batch.t_all = reshape(t_flat, [num_sims, n_coeff]);
batch.x_all = reshape(x_flat, [num_sims, n_coeff]);
batch.u_all = reshape(u_flat, [num_sims, n_coeff]);

batch.coeff_values       = coeff_values;
batch.num_sims           = num_sims;
batch.tend               = cfg.tend;
batch.initial_state_deg  = initial_state;
batch.V_func             = cfg.V_func;
batch.hl_pool            = hl_pool;
batch.switch_times_pool  = switch_times_pool;
batch.init_sign_pool     = init_sign_pool;
batch.disturbance        = dist_cfg;
batch.aero_dispersion_level = cfg.aero_dispersion_level;
batch.seed               = cfg.seed;
end

% ----------------------------------------------------------------------
function [t, x, u] = run_one(job, num_sims, coeff_values, bias0, ...
                             hl_pool, switch_times_pool, init_sign_pool, dist_cfg)
    c_idx = floor((job - 1) / num_sims) + 1;
    k     = mod(job - 1, num_sims) + 1;

    bias = bias0;
    bias.coeff = coeff_values(c_idx);
    bias.hl    = @(t) hl_pool(:, k);

    switch lower(dist_cfg.type)
        case 'none'
            bias.disturbance = @(t) zeros(3, 1);
        case 'bangbang'
            switch_times = switch_times_pool(:, :, k);
            init_signs   = init_sign_pool(:, k);
            M_ub = dist_cfg.M_ub;
            M_lb = dist_cfg.M_lb;
            bias.disturbance = @(t) build_bangbang_dist(t, switch_times, ...
                init_signs, M_ub, M_lb);
        case 'custom'
            bias.disturbance = dist_cfg.fun;
        otherwise
            error('sim:run_batch:badDisturbance', ...
                '未知的扰动类型 "%s"。', dist_cfg.type);
    end

    [t, x, u] = simulate_attitude(bias);
end

% ----------------------------------------------------------------------
function cfg = fill_defaults(cfg)
    if ~isfield(cfg, 'num_sims') || isempty(cfg.num_sims)
        cfg.num_sims = 1;
    end
    if ~isfield(cfg, 'coeff_values') || isempty(cfg.coeff_values)
        cfg.coeff_values = 1;
    end
    if ~isfield(cfg, 'tend') || isempty(cfg.tend)
        cfg.tend = 0.5;
    end
    if ~isfield(cfg, 'initial_state_deg') || isempty(cfg.initial_state_deg)
        cfg.initial_state_deg = struct('alpha', 3, 'beta', 0, 'gammas', 0, ...
            'wx', 0, 'wy', 0, 'wz', 15);
    end
    if ~isfield(cfg, 'V_func') || isempty(cfg.V_func)
        error('sim:run_batch:missingV', '必须提供 cfg.V_func。');
    end
    if ~isfield(cfg, 'aero_dispersion_level') || isempty(cfg.aero_dispersion_level)
        cfg.aero_dispersion_level = 0;
    end
    if ~isfield(cfg, 'disturbance') || isempty(cfg.disturbance)
        cfg.disturbance = struct('type', 'none');
    end
    if ~isfield(cfg.disturbance, 'type') || isempty(cfg.disturbance.type)
        cfg.disturbance.type = 'none';
    end
    if strcmpi(cfg.disturbance.type, 'bangbang')
        if ~isfield(cfg.disturbance, 'M_ub') || isempty(cfg.disturbance.M_ub)
            cfg.disturbance.M_ub = [0; 0; 15];
        end
        if ~isfield(cfg.disturbance, 'M_lb') || isempty(cfg.disturbance.M_lb)
            cfg.disturbance.M_lb = [0; 0; -15];
        end
        if ~isfield(cfg.disturbance, 'N_switches') || isempty(cfg.disturbance.N_switches)
            cfg.disturbance.N_switches = 12;
        end
    end
    if ~isfield(cfg, 'seed') || isempty(cfg.seed)
        cfg.seed = 42;
    end
    if ~isfield(cfg, 'use_parfor') || isempty(cfg.use_parfor)
        cfg.use_parfor = true;
    end
    if ~isfield(cfg, 'model_override')
        cfg.model_override = [];
    end
    if ~isfield(cfg, 'controller_override')
        cfg.controller_override = [];
    end
end
