# 姿态控制仿真程序

高速固定翼无人机姿态自抗扰控制，面向高速飞行条件下的非线性动力学、气动不确定性及外部扰动，实现姿态运动建模、状态与总扰动在线估计、控制指令生成及扰动补偿，支持典型飞行工况下的姿态控制算法仿真与性能分析。

飞行器姿态控制仿真与分析的 MATLAB 工程：PD + 串级 ESO ADRC 控制律、
气动数据插值、配平计算，以及以补偿系数 `coeff` 为核心的最坏情形优化。

> 本仓库不包含气动数据，也不包含飞行器具体参数（见“参数与数据”一节）。

## 核心贡献：补偿系数最优化

本项目的研究重点是 **ESO 补偿系数 `coeff` 的最坏情形优化**：在外扰与气动
拉偏不确定下，寻找使最坏跟踪误差最小的补偿强度。

**问题定义**

- 控制律中的 ESO 补偿项为 `-coeff * hat_D2`，`coeff` 直接决定补偿强度；
- `coeff = 0` 退化为 PD，`coeff = 1` 为标准 ADRC，其他取值（含负值）对应
  欠补偿 / 过补偿；
- 指标为过程积分平方误差 ISE：`J = ∫ z'Qz dt`，其中
  `z = [α-α_cmd; β; ωx; ωy; ωz]`；
- 对同一组 Bang-Bang 扰动样本，取每个 `coeff` 下最坏样本的 `J` 作为该系数的代价。

**求解流程**

1. `run_batch_simulations` 生成共享扰动样本，并批量仿真各候选 `coeff`；
2. `coeff_worst_case` 计算 `J(coeff)`，在系数网格上取最小值得到最优 `coeff*`；
3. 输出 `J(coeff)` 曲线、最坏响应对比、最坏扰动曲线与数值结果。

**复现**

```matlab
cd scripts
% 打开 main_monte_carlo.m，将 mode 设为 'coeff_opt'
main_monte_carlo
```

结果位于 `results/coeff_optimization/`。

**相关文件**：`scripts/main_monte_carlo.m`（入口）、
`src/analysis/coeff_worst_case.m`（最坏情形优化）、
`src/analysis/envelope_report.m`（候选系数包络对比）、
`src/utils/compute_tracking_cost.m`（ISE 指标）、
`src/core/adrc_control.m`（补偿项实现）。

## 目录结构

```
config/                 参数配置
  model_params.m          模型参数：字段定义与参数加载
  controller_params.m     控制器参数：PD 增益、ESO 带宽、指令、舵偏限幅
  aero_config.m           气动数据文件路径
src/
  core/                   仿真内核
    simulate_attitude.m     单次闭环仿真入口
    attitude_dynamics.m     18 维闭环动力学 + 双 ESO
    adrc_control.m          PD + 前馈 + ESO 补偿控制律
    AeroInterpolant.m       气动系数插值与舵效导数
    load_aero_model.m       气动数据加载与进程内缓存
  experiments/            批量仿真
    run_batch_simulations.m 补偿系数 × 拉偏样本批量仿真
    make_bias.m             构造工况结构体
    run_initial_case.m      单初始条件工况（供并行调用）
  analysis/               分析算法
    trim_point.m            单攻角配平求解
    envelope_report.m       包络带与最坏 ISE 报告
    coeff_worst_case.m      补偿系数最坏情形优化
  plotting/               绘图辅助
    create_analysis_figures.m
    draw_envelope.m
    save_disturbance_figures.m
  utils/                  通用工具
    project_root.m / add_project_paths.m / output_dir.m
    state_to_deg.m / compute_tracking_cost.m
    build_bangbang_dist.m
    save_figure_3formats.m / setup_chinese_fonts.m
scripts/                可运行的主脚本
data/                   气动数据放置目录
results/                运行输出（已忽略）
docs/                   程序说明与新旧对照
```

## 快速开始

1. 用 MATLAB（建议 R2021a 或更高）打开工程根目录；
2. 把气动数据表放到 `data/气动数据.xlsx`（列名要求见 `data/README.md`）；
3. 按 `config/model_params.m` 的字段说明在本地提供模型参数；
4. 进入 `scripts/`，直接运行主脚本：

   ```matlab
   cd scripts
   main_single_run
   ```

脚本开头自动调用 `add_project_paths()`，无需手动 `addpath`，也不依赖当前文件夹。

### 依赖

| 依赖 | 用途 | 是否必需 |
| --- | --- | --- |
| MATLAB R2021a+ | `exportgraphics`、`xline` 等 | 必需 |
| Parallel Computing Toolbox | `parfor` 批量仿真 | 可选（自动退回串行） |
| Optimization Toolbox | `main_trim.m` 的 `fmincon` | 配平必需 |

## 主脚本一览

| 脚本 | 作用 | 输出目录 |
| --- | --- | --- |
| `main_single_run.m` | 单次名义工况仿真，9 条时域曲线 | `results/single/` |
| `main_controller_compare.m` | PD 与 ADRC 对比；`mode` 可选 `nominal` / `aero_dispersion` | `results/compare_*/` |
| `main_monte_carlo.m` | 蒙特卡洛拉偏；`mode` 可选 `envelope` / `coeff_opt` | `results/monte_carlo_*/` |
| `main_initial_sweep.m` | 初始攻角 × 俯仰角速度扫描与收敛性统计 | `results/sweep/` |
| `main_trim.m` | 配平点计算（力矩 + 竖直力） | `results/trim/` |
| `main_aero_plot.m` | 气动系数、俯仰力矩与俯仰舵效曲线 | `results/aero/` |

## 参数与数据

### 气动数据

- 默认位置：`data/气动数据.xlsx`，工作表 `Sheet1`；
- 使用单一数据表，不区分马赫数；
- 列名：`dn, d2, d4, alpha, beta, Cx, Cy, Cz, Cmx, Cmy, Cmz`（详见 `data/README.md`）；
- 已被 `.gitignore` 排除；如放在别处，可设置环境变量 `AERO_DATA_FILE`。

### 飞行器参数

模型参数（质量、惯量、参考面积/长度、大气参数）统一由 `config/model_params.m`
管理，修改飞行器参数只需改此文件。仓库中不保存具体数值；参数文件由本地提供，
不纳入版本管理。若缺失，程序会使用一组中性演示值并给出提示。

控制器参数不属于飞行器模型，仍保存在 `config/controller_params.m`。

> 上传前请确认 `git status` 中没有本地参数文件与 `data/` 下的数据文件；
> 不要用网页“拖拽整个文件夹”的方式上传，否则 `.gitignore` 不生效。

## 输出

- 图片以 `png / fig / eps` 写入 `results/` 对应子目录；
- 大批量结果以 `.mat` 保存（`-v7.3`），默认不纳入版本管理；
- 脚本顶部 `cfg.load_results = true` 可复用已有结果。

## 数据流

```
scripts/main_*.m
   └─> src/experiments/run_batch_simulations.m（可选 parfor）
          └─> src/core/simulate_attitude.m
                 ├─ config/model_params.m（模型参数）
                 ├─ config/controller_params.m
                 ├─ config/aero_config.m    → load_aero_model.m
                 └─ src/core/attitude_dynamics.m ←→ adrc_control.m
```

## 说明与限制

- 本工程由原始多个独立脚本整理而来，物理模型与控制律未做改动；
- 整理后未使用真实气动数据进行完整回归，请以原始结果作为基准复核；
- `attitude_dynamics.m` 中参考偏移力矩对舵偏的导数项按原程序置零；
- `main_trim.m` 的弹道倾角沿用原始取值，其量纲与注释存在历史不一致，未改动物理含义。

## 许可证

MIT，详见 [LICENSE](LICENSE)。

## 新旧文件对照

原 `lib/`、`main/` 目录已并入 `src/` 与 `scripts/`，对照表见
[docs/程序说明.md](docs/程序说明.md)。
