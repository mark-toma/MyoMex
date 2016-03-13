function install_myo_mex(spec)
%INSTALL_MYO_MEX(SPEC)  Adds directories to MATLAB search path.
%   Specify SPEC = 'save' to save the path for persistence. Otherwise,
%   these additions to MATLAB search path will only last for the duration
%   of the current MATLAB session.
%
%   This function must remain in the root direcory of MyoMex.

if nargin < 1
  spec = false;
elseif ischar(spec) && isvector(spec) && strcmp('save',spec)
  spec = true;
else
  error('Input spec must have the value ''save'' or not be supplied.');
end

% assume this function is in the root of MyoMex and add all subdirectories
addpath(genpath(fileparts(mfilename('fullpath'))))

if spec, savepath; end

end