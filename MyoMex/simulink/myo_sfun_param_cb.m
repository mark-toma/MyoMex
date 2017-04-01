function myo_sfun_param_cb(blk)
% myo_sfun_param_cb(blk)  Callback for S-Function block myo_sfun
%   Validates parameters, recovers from bad input, and issues warnings

DEFAULT_EMG_ENABLED = 1;
DEFAULT_COUNT_MYOS = 1;

isBadParams = false;

% get all params
emgEnabled = get_param(blk,'emgEnabled');
countMyos = get_param(blk,'countMyos');

try
  emgEnabled = str2num(emgEnabled);
  countMyos = str2num(countMyos);
catch e
  warndlg('Param value conversion to double failed. Setting params to default state.','Bad Parameter Value','modal');
  isBadParams = true;
end

% validate params
if ~any(emgEnabled==[0,1])
  warndlg('Invalid value for emgEnabled. Setting params to default state.','Bad Parameter Value','modal');
  isBadParams = true;
end
if ~any(countMyos==[1,2])
  warndlg('Invalid value for countMyos. Setting params to default state.','Bad Parameter Value','modal');
  isBadParams = true;
end
if (countMyos==2)&&(emgEnabled==1)
  warndlg('EMG cannot be enabled when using 2 Myos. Disabling EMG.','Bad Parameter Value','modal');
  emgEnabled = 0;
end


% set default param state
if isBadParams
  disp('bad params');
  emgEnabled = DEFAULT_EMG_ENABLED;
  countMyos = DEFAULT_COUNT_MYOS;
end

set_param(blk,'emgEnabled',num2str(emgEnabled));
set_param(blk,'countMyos',num2str(countMyos));

end
