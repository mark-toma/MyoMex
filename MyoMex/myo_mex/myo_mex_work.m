%% myo_mex_work.m


myo_mex init



%%

myo_mex start_streaming


%%

pause(0.2);
d = myo_mex('get_streaming_data');
nIMU = size(d.quat,1)
nEMG = size(d.emg,1)

pause(0.2);
d = myo_mex('get_streaming_data');
nIMU = size(d.quat,1)
nEMG = size(d.emg,1)

pause(0.2);
d = myo_mex('get_streaming_data');
nIMU = size(d.quat,1)
nEMG = size(d.emg,1)

pause(0.2);
d = myo_mex('get_streaming_data');
nIMU = size(d.quat,1)
nEMG = size(d.emg,1)

%%

myo_mex stop_streaming

%%


myo_mex start_streaming
pause(0.5)
d = myo_mex('get_streaming_data');
nIMU1 = size(d(1).quat,1)
nEMG1 = size(d(1).emg,1)
if length(d)>1
  nIMU2 = size(d(2).quat,1)
  nEMG2 = size(d(2).emg,1)
end

myo_mex stop_streaming



%%

figure;
subplot(2,1,1);
plot(d(1).emg);
subplot(2,1,2);
plot(d(2).emg);




%%

myo_mex delete
clear myo_mex






