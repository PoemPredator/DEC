function mdl = load_aero_model(a)
%LOAD_AERO 加载（并缓存）气动数据表，返回 AEROINTERPOLANT 对象。
%
%   MDL = LOAD_AERO(A)
%   A 为 AERO_CONFIG 返回的结构体。
%
%   同一 MATLAB 进程（或 parfor worker）内只读取一次 Excel。

persistent cache

key = sprintf('%s|%s', a.data_file, a.sheet);

if isempty(cache) || ~strcmp(cache.key, key)
    cache.key = key;
    cache.model = AeroInterpolant(a.data_file, a.sheet);
end

mdl = cache.model;
end
