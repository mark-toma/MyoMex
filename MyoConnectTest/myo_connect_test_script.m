
cmd = sprintf('mex -I"%s" -L"%s" -l%s %s',...
  'c:\myo-sdk-win-0.9.0\include\',...
  'c:\myo-sdk-win-0.9.0\lib\',...
  'myo64',...
  'myo_connect_test.cpp');

eval(cmd)

%%
disp('One Myo: (A)');
myo_connect_test

%%
disp('Two Myo: (A) and (B)');
myo_connect_test

%%
disp('One Myo: (A)');
myo_connect_test





