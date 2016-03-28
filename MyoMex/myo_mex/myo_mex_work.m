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
d = myo_mex('get_streaming_data');
nIMU = size(d.quat,1)
nEMG = size(d.emg,1)
myo_mex stop_streaming






%%

myo_mex delete
clear myo_mex






