%% 补偿系数最优化（核心研究脚本）
%
%   研究重点：ESO 补偿系数 coeff 的最坏情形优化。
%   mode = 'coeff_opt'  在补偿系数网格上求最坏情形最优系数（主结果）
%   mode = 'envelope'   候选补偿系数在 Bang-Bang 外扰下的响应包络与最坏 ISE
%
%   两模式共享同一组随机扰动样本，保证不同系数之间可比。
%   输出：results/coeff_optimization/ 或 results/monte_carlo_envelope/
%   参见 RUN_BATCH_SIMULATIONS, COEFF_WORST_CASE, ENVELOPE_REPORT。

add_project_paths();
setup_chinese_fonts();

%% 参数
mode = 'envelope';

cfg.load_results = false;
cfg.use_parfor   = true;

model = model_params();
a_sound = model.sound_speed;

% 速度剖面：0.1 s 内由 4 倍音速加速到 7 倍音速
V_func = @(t) (t < 0.1) .* ((7 - 3 * (0.1 - t) / 0.1) * a_sound) ...
            + (t >= 0.1) .* (7 * a_sound);

switch mode
    case 'envelope'
        cfg.num_sims = 50;
        cfg.coeff    = [-1, 0, 1, 1.12];
        cfg.tend     = 0.2;
        cfg.cost     = struct('angle_scale_deg', 3, 'rate_scale_degs', 25);
        cfg.converge = struct('alpha_deg', 5, 'beta_deg', 5);
        cfg.subdir   = 'monte_carlo_envelope';
    case 'coeff_opt'
        cfg.num_sims = 40;
        cfg.coeff    = -0.5:0.1:1.8;
        cfg.tend     = 0.2;
        cfg.cost     = struct('angle_scale_deg', 1, 'rate_scale_degs', 1);
        cfg.converge = struct('alpha_deg', 5, 'beta_deg', 5);
        cfg.subdir   = 'coeff_optimization';
    otherwise
        error('run_monte_carlo:badMode', '未知的 mode: %s', mode);
end

out_dir = output_dir(cfg.subdir);
results_file = fullfile(output_dir(), sprintf('monte_carlo_%s.mat', mode));

%% 仿真
batch_cfg = struct();
batch_cfg.num_sims = cfg.num_sims;
batch_cfg.coeff_values = cfg.coeff;
batch_cfg.tend = cfg.tend;
batch_cfg.initial_state_deg = struct('alpha', 3, 'beta', 0, 'gammas', 0, ...
                                     'wx', 0, 'wy', 0, 'wz', 15);
batch_cfg.V_func = V_func;
batch_cfg.aero_dispersion_level = 0;
batch_cfg.disturbance = struct('type', 'bangbang', ...
    'M_ub', [0; 0; 15], 'M_lb', [0; 0; -15], 'N_switches', 12);
batch_cfg.seed = 42;
batch_cfg.use_parfor = cfg.use_parfor;

if cfg.load_results && exist(results_file, 'file')
    S = load(results_file);
    batch = S.batch;
    fprintf('已读取 %s，跳过仿真。\n', results_file);
else
    fprintf('蒙特卡洛仿真：%d 组样本 × %d 个补偿系数 ...\n', ...
        cfg.num_sims, numel(cfg.coeff));
    tic;
    batch = run_batch_simulations(batch_cfg);
    fprintf('仿真完成，用时 %.1f s。\n', toc);
    save(results_file, 'batch', '-v7.3');
end

%% 分析
opts = struct('out_dir', out_dir, 'cost', cfg.cost, 'converge', cfg.converge);

switch mode
    case 'envelope'
        metrics = envelope_report(batch, opts);
        save(fullfile(out_dir, 'envelope_metrics.mat'), 'metrics');
    case 'coeff_opt'
        results = coeff_worst_case(batch, opts);
        save(fullfile(out_dir, 'coeff_opt_summary.mat'), 'results');
end
