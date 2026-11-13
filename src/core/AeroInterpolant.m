classdef AeroInterpolant < handle
    %AEROINTERPOLANT 气动系数插值与舵效导数计算。
    %
    %   从单一 Excel 气动数据表构建插值模型，提供
    %       [C, dC_ddelta] = OBJ.EVALUATE(alpha, beta, dn, d2, d4)
    %   C        为 6×N 系数矩阵 [Cx; Cy; Cz; Cmx; Cmy; Cmz]
    %   dC_ddelta 为 6×3 舵效导数矩阵（列对应 dn, d2, d4）
    %
    %   Excel 工作表列名（严格一致）
    %       dn, d2, d4, alpha, beta, Cx, Cy, Cz, Cmx, Cmy, Cmz
    %
    %   示例
    %       mdl = AeroInterpolant('data/气动数据.xlsx');
    %       [C, dC] = mdl.evaluate(3, 0, 0, 0, 0);
    %
    %   参见 AERO_CONFIG, LOAD_AERO_MODEL, SIMULATE_ATTITUDE。

    properties (Access = private)
        base_interp
        dn_interp
        d2_interp
        d4_interp
    end

    methods
        function obj = AeroInterpolant(excelFile, sheetName)
            if nargin < 2 || isempty(sheetName)
                sheetName = 'Sheet1';
            end
            obj.buildFromExcel(excelFile, sheetName);
        end

        function [C, dC_ddelta] = evaluate(obj, alpha, beta, dn, d2, d4)
            C_base = obj.base_interp(alpha, beta);

            [deltaC_dn, dDelta_ddn] = obj.dn_interp(alpha, beta, dn);
            [deltaC_d2, dDelta_dd2] = obj.d2_interp(alpha, beta, d2);
            [deltaC_d4, dDelta_dd4] = obj.d4_interp(alpha, beta, d4);

            C = C_base + deltaC_dn + deltaC_d2 + deltaC_d4;

            if nargout > 1
                dC_ddelta = [dDelta_ddn, dDelta_dd2, dDelta_dd4];
            end
        end
    end

    methods (Access = private)
        function buildFromExcel(obj, excelFile, sheetName)
            % 从 Excel 读取数据并构建各插值对象
            data = readtable(excelFile, 'Sheet', sheetName, ...
                'VariableNamingRule', 'preserve');

            dn    = data{:, 'dn'};
            d2    = data{:, 'd2'};
            d4    = data{:, 'd4'};
            alpha = data{:, 'alpha'};
            beta  = data{:, 'beta'};
            C = [data{:, 'Cx'}, data{:, 'Cy'}, data{:, 'Cz'}, ...
                 data{:, 'Cmx'}, data{:, 'Cmy'}, data{:, 'Cmz'}];

            % 1. 基础项：所有舵为零
            idx_base = (dn == 0 & d2 == 0 & d4 == 0);
            alpha_base = alpha(idx_base);
            beta_base  = beta(idx_base);
            C_base = C(idx_base, :)';

            [alpha_grid, ~, ~] = unique(alpha_base);
            [beta_grid,  ~, ~] = unique(beta_base);

            C_base_mat = zeros(6, length(alpha_grid), length(beta_grid));
            for i = 1:length(alpha_base)
                a_idx = find(alpha_grid == alpha_base(i));
                b_idx = find(beta_grid == beta_base(i));
                C_base_mat(:, a_idx, b_idx) = C_base(:, i);
            end
            obj.base_interp = @(a, b) interpBase(a, b, alpha_grid, beta_grid, C_base_mat);

            % 2. 方向舵增量：d2=0, d4=0, dn 非零
            idx_dn = (d2 == 0 & d4 == 0);
            obj.dn_interp = buildDelta(obj, alpha, beta, dn, C, idx_dn, ...
                alpha_grid, beta_grid, C_base_mat);

            % 3. 左升降舵增量：dn=0, d4=0, d2 非零
            idx_d2 = (dn == 0 & d4 == 0);
            obj.d2_interp = buildDelta(obj, alpha, beta, d2, C, idx_d2, ...
                alpha_grid, beta_grid, C_base_mat);

            % 4. 右升降舵增量：dn=0, d2=0, d4 非零
            idx_d4 = (dn == 0 & d2 == 0);
            obj.d4_interp = buildDelta(obj, alpha, beta, d4, C, idx_d4, ...
                alpha_grid, beta_grid, C_base_mat);
        end
    end
end

% ----------------------------------------------------------------------
function fcn = buildDelta(~, alpha, beta, delta, C, idx, alpha_grid, beta_grid, C_base_mat)
    % 构建单个舵面的增量插值器（基础项之上叠加舵偏增量）
    alpha_d = alpha(idx);
    beta_d  = beta(idx);
    delta_d = delta(idx);
    C_raw = C(idx, :)';

    C_base_at = interpBase(alpha_d, beta_d, alpha_grid, beta_grid, C_base_mat);
    deltaC = C_raw - C_base_at;

    [alpha_grid_d, ~, ~] = unique(alpha_d);
    [beta_grid_d,  ~, ~] = unique(beta_d);
    delta_grid = unique(delta_d);

    C_mat = zeros(6, length(alpha_grid_d), length(beta_grid_d), length(delta_grid));
    for i = 1:length(alpha_d)
        a_idx = find(alpha_grid_d == alpha_d(i));
        b_idx = find(beta_grid_d == beta_d(i));
        d_idx = find(delta_grid == delta_d(i));
        C_mat(:, a_idx, b_idx, d_idx) = deltaC(:, i);
    end

    fcn = @(a, b, d) interpDelta(a, b, d, alpha_grid_d, beta_grid_d, delta_grid, C_mat);
end

function C_base = interpBase(alpha, beta, alpha_grid, beta_grid, C_mat)
    n = numel(alpha);
    C_base = zeros(6, n);
    for i = 1:6
        F = griddedInterpolant({alpha_grid, beta_grid}, ...
            squeeze(C_mat(i, :, :)), 'linear', 'linear');
        C_base(i, :) = F(alpha, beta);
    end
end

function [deltaC, dDelta] = interpDelta(alpha, beta, delta, alpha_grid, beta_grid, delta_grid, C_mat)
    n = numel(alpha);
    deltaC = zeros(6, n);
    dDelta = zeros(6, n);
    for i = 1:6
        F = griddedInterpolant({alpha_grid, beta_grid, delta_grid}, ...
            squeeze(C_mat(i, :, :, :)), 'linear', 'linear');
        deltaC(i, :) = F(alpha, beta, delta);
        dDelta(i, :) = computeDeltaDerivative(F, alpha, beta, delta);
    end
end

function deriv = computeDeltaDerivative(F, alpha, beta, delta)
    % 全局割线法计算宏观舵效导数：取 0 点两侧最大对称点做差商，
    % 以抑制气动表格局部噪声导致的导数符号反转。
    n = numel(delta);
    deriv = zeros(1, n);

    delta_grid = F.GridVectors{3};
    d_min = delta_grid(1);
    d_max = delta_grid(end);

    if d_min < 0 && d_max > 0
        d_sym = min(abs(d_min), abs(d_max));
        d_left  = -d_sym;
        d_right =  d_sym;
    else
        d_left  = d_min;
        d_right = d_max;
    end

    for k = 1:n
        if abs(d_right - d_left) < 1e-6
            deriv(k) = 0;
        else
            F_left  = F(alpha(k), beta(k), d_left);
            F_right = F(alpha(k), beta(k), d_right);
            deriv(k) = (F_right - F_left) / (d_right - d_left);
        end
    end
end
