%% MyoMex_work.m

mm = MyoMex(2);

m1 = mm.myo_data(1);
m2 = mm.myo_data(2);
pause(30);

%%
mm.delete();
clear mm myo_mex;

nIMU = size(m1.quat_log,1)
nEMG = size(m1.emg_log,1)


%%



% figure;
% subplot(3,1,1);
% plot(m1.quat_log)
% subplot(3,1,2);
% plot(m1.gyro_log)
% subplot(3,1,3);
% plot(m1.accel_log)
% 
% figure;
% subplot(2,1,1);
% plot(m1.emg_log);
% 
% figure;
% subplot(3,1,1);
% plot(m2.quat_log)
% subplot(3,1,2);
% plot(m2.gyro_log)
% subplot(3,1,3);
% plot(m2.accel_log)
% 
% figure;
% subplot(2,1,1);
% plot(m2.emg_log);






