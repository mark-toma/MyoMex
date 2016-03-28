%% MyoMex_work.m

mm = MyoMex()
m1 = mm.myoData(1)

%%

tic;
mm.startStreaming();
pause(20);
mm.stopStreaming();
T = toc;

%%
mm.delete();
clear mm;

%%

nIMU = size(m1.quat_log,1);
nEMG = size(m1.emg_log,1);

nEMG/nIMU

nIMU/T

nEMG/T


%%

figure;
subplot(3,1,1);
plot(m1.quat_log)
subplot(3,1,2);
plot(m1.gyro_log)
subplot(3,1,3);
plot(m1.accel_log)

figure;
subplot(2,1,1);
plot(m1.emg_log);
subplot(2,1,2);
plot(m1.pose_log);
