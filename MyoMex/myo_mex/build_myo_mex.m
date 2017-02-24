function build_myo_mex(sdk_path)
%BUILD_MYO_MEX(SDK_PATH) - Builds myo_mex
%   This functions builds the mex function myo_mex against the Windows
%   Myo SDK with root directory located at sdk_path.
%
%   Currently, only Windows is supported in this script and the source code
%   myo_mex.cpp has Win API dependencies.
%
% Usage:
%
%   sdk_path = fullfile('c:','path','to','sdk','root');
%   build_myo_mex(sdk_path)
%
%   build_myo_mex c:\myo-sdk-win-0.9.0
%
%   build_myo_mex default % same as above (hardcoded path)
%

if strcmp('default',sdk_path)
  sdk_path = 'c:\myo-sdk-win-0.9.0';
end

lib_name = get_lib_name(); % decides on myo32 or myo64 from computer arch

inc_path = fullfile(sdk_path,'include');
lib_path = fullfile(sdk_path,'lib');
if ~exist(sdk_path,'dir')
  error('sdk_path = ''%s'' is not a valid directory.',sdk_path);
elseif ~exist(inc_path,'dir')
  error('sdk_path = ''%s'' does not contain subdirectory ''%s''',sdk_path,'include');
elseif ~exist(lib_path,'dir')
  error('sdk_path = ''%s'' does not contain subdirectory ''%s''',sdk_path,'lib');
end

cur_path = pwd;
bld_path = fileparts(mfilename('fullpath'));

if ~strcmp(bld_path,cur_path)
  fprintf('\nChanging directory to build directory:\n\t''%s''\n',bld_path);
  cd(bld_path);
end

% build

% source files
src_files = {...
  'myo_mex.cpp',...
  'myo_sfun.cpp'};

for ii=1:length(src_files)
  % mex command (add ' -v -g' switches for verbose output and debug symbols)
  cmd = sprintf('mex -O -I"%s" -L"%s" -l%s %s',...
    inc_path,lib_path,lib_name,src_files{ii});
  fprintf('\nEvaluating mex command:\n\t''%s''\n',cmd);
  try
    eval(cmd);
  catch err
    % NOTE: I'm not really sure where the cd()'s are going in this catch
    fprintf('\nMEX-file ''%s'' did not build successfully!\n',...
      src_files{ii});
    % cd back to original path
    if ~strcmp(bld_path,cur_path)
      fprintf('\nChanging directory to original directory:\n\t''%s''\n',bld_path);
      cd(cur_path);
    end
    fprintf('\nChanging directory to build directory:\n\t''%s''\n',bld_path);
    throw(err);
  end
  fprintf('\nMEX-file ''%s'' built successfully!\n\n',...
    src_files{ii});
end

% cd back to original path
if ~strcmp(bld_path,cur_path)
  fprintf('\nChanging directory to original directory:\n\t''%s''\n',bld_path);
  cd(cur_path);
end


end


% infer lib variant from computer architecture (i.e. 32 or 64 bit)
% only windows is supported currently
function name = get_lib_name()
if ispc
  if strcmp('win64',computer('arch'))
    name = 'myo64';
  else
    name = 'myo32';
  end
else
  error('build_myo_mex only supports windows pc');
end
end
