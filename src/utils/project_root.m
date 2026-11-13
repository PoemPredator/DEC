function root = project_root()
%PROJECT_ROOT 返回工程根目录的绝对路径。
%
%   本文件位于 <root>/src/utils/，向上三级即为工程根目录。
%   需要拼装路径的脚本/函数应通过本函数定位，避免写死绝对路径。

here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
end