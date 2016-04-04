%% MyoMexExample_DataContinuity
% In this script, we log samples from countMyos devices and then inspect
% the data to detect cases in which data may have been lost. Since missing
% data is interpolated by zero order hold, we can only say for sure if no
% samples have been missed.


%% Log Data
% Configure |TIME_DURATION| (10 seconds is a good choice) and |countMyos|
% before running this script.

TIME_DURATION = 20; % seconds
countMyos = 2;

fprintf('Collecting data from %d Myos for roughly %5.2f[s] ... ',...
  countMyos,TIME_DURATION);
mm = MyoMex(countMyos);
tic;
m = mm.myoData;
pause(TIME_DURATION-toc);
mm.delete();
fprintf('Done!\n\n');

%%  Test Data for Missed Samples
% Anonymous function returns logical scalar representing the validity of
% samples. It returns true if no two consecutive rows have the same values
% in all columns. Otherwise it returns false.

hasDataContinuity = @(x)~any(all((~diff(x))'));

flags = [];
for ii = 1:countMyos
  flags = [flags,hasDataContinuity(m(ii).quat_log)];
  flags = [flags,hasDataContinuity(m(ii).gyro_log)];
  flags = [flags,hasDataContinuity(m(ii).accel_log)];
  if countMyos == 1
    flags = [flags,hasDataContinuity(m(ii).emg_log)];
  end
  fprintf('Estimated sample rate (Myo %d):\n\tIMU: %5.2f[Hz]\tEMG: %5.2f[Hz]\n',...
    ii,length(m(ii).timeIMU_log)/TIME_DURATION,length(m(ii).timeEMG_log)/TIME_DURATION);
end
fprintf('\n');

fprintf('Data continuity test result:\n\t');
if all(flags)
  fprintf('PASS: No samples were missed\n\n');
else
  fprintf('FAIL: At least one sample was missed\n\n');
end


%% Plot Duplicate Samples
% Usually we find that the duplicates are in up to about 15% of the
% quaternion samples. It may be that the quaternion estimate is acually
% updated less frquently than it is sampled. Over many 60 second trials, I
% seldom witness lost data in gyro, accel, or emg. This leads me to believe
% that we're not actually missing quaternion data.

idDuplicates = @(x)all((~diff(x))');

tIMU1 = m(1).timeIMU_log(2:end);
qd1 = idDuplicates(m(1).quat_log);
gd1 = idDuplicates(m(1).gyro_log);
ad1 = idDuplicates(m(1).accel_log);

if countMyos == 2
  tIMU2 = m(2).timeIMU_log(2:end);
  qd2 = idDuplicates(m(2).quat_log);
  gd2 = idDuplicates(m(2).gyro_log);
  ad2 = idDuplicates(m(2).accel_log);
elseif countMyos==1
  tEMG1 = m(1).timeEMG_log(2:end);
  ed1 = idDuplicates(m(1).emg_log);
end

figure;
if countMyos == 2, subplot(2,1,1); end
plot(tIMU1,1*qd1,'r',tIMU1,2*gd1,'g',tIMU1,3*ad1,'b'); legCell = {'quat','gyro','accel'};
if countMyos == 1, hold on; plot(tEMG1,4*ed1,'k'); legCell{4} = 'emg'; end 
legend(legCell);
if countMyos == 2
  subplot(2,1,2);
  plot(tIMU2,1*qd2,'r',tIMU2,2*gd2,'g',tIMU2,3*ad2,'b');
  legend('quat','gyro','accel');
end

%% Plot Acceleration Magnitude
% If logging from two Myos, plot the 2 norm of kinematic acceleration to
% inspect the time-synchronization of signals. You may choose to perform
% similar motions on the two Myos to visualize the results.

if countMyos == 2
  figure;
  an1 = sqrt(sum((m(1).accel_log.^2)'))-1;
  an2 = sqrt(sum((m(2).accel_log.^2)'))-1;
  plot(...
    m(1).timeIMU_log,...
    an1-mean(an1),'r',...
    m(2).timeIMU_log,...
    an2-mean(an2),'b');
  legend('|a|_1','|a|_2');
end
















