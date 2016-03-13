%% MyoMex Quickstart
% Before you begin, please read through READ_ME.txt and follow all steps
% for setting up the Myo Connect application, Myo SDK, and building the
% MEX function myo_mex.

%% Before Using |MyoMex|
% If you decided not to read through |READ_ME.txt|, let's at least show you
% the quickest possible way to get started.

install_myo_mex; % adds directories to MATLAB search path
% install_myo_mex save % additionally saves the path

sdk_path = 'C:\myo-sdk-win-0.9.0'; % root path to Myo SDK
build_myo_mex(sdk_path); % builds myo_mex

%% Simplest Usage
% This m-code class MyoMex is the intended interface for the myo_mex file
% that was just built. The simplest lifecycle for a MyoMex object is,

m = MyoMex() % instantiate MyoMex (calls into myo_mex init)
m.delete()   % destroy MyoMex (calls into myo_mex delete)
clear m

%% Polling Data
% The next thing we can do is poll for a single sampling of Myo data.
% First, instantiate |MyoMex| again.

m = MyoMex();

%%
% Call the method |getData()| get current data from Myo and populate
% |MyoMex| data properties.

m.getData();

%%
% The most recent data from Myo will be stored in the relevant properties
% of the |MyoMex| object (i.e. |quat|, |gyro|, |accel|, |emg|, |pose|,
% etc.).

m.time
m.quat
m.gyro
m.accel
m.emg
m.pose
m.pose_rest
m.pose_fist
m.pose_wave_in
m.pose_wave_out
m.pose_fingers_spread
m.pose_double_tap

%%
% After many subsequent calls to |getData()|, you'll notice that the
% |data_log| properties of are keeping track of the data received from
% Myo.

for ii = 1:10, m.getData(); end % poll for data ten times

m.time_log
m.accel_log

%%
% As the logs become large in size you may want to clear them using the
% |clearLogs()| method.

m.clearLogs(); % sets *_log properties back to empty

m.time_log
m.accel_log

%%
% And the most recent data received is always maintained in the |data|
% properties (it's not cleared with the logs).

m.time
m.accel

%% Streaming Data
% In addition to manually polling data, the |myo_mex| interface also
% supports a state in which it continuously polls Myo for data at an
% (assumed) constant rate in its own thread. This is referred to as the
% streaming mode. You set and get the effective sampling rate of this
% feature by accessing the |streaming_data_time| property. This is the
% number of seconds between data samples.

sample_rate = 30; % desired data sampling rate in Hz
m.streaming_data_time = 1/sample_rate;
m.streaming_data_time % print the value

%%
% Notice that the time is restricted to millisecond precision as indicated
% by the warning message.
%
% As the MEX file polls Myo for data every |streaming_data_time| seconds,
% the |MyoMex| object will set up a timer that calls into |myo_mex| to
% fetch the data every |streaming_frame_time| seconds,

m.streaming_frame_time = 1/10; % fetch data frames at 10 Hz

%%
% Begin and end a streaming data session by calling the methods
% |startStreaming()| and |stopStreaming()|. Since the |MyoMex| object takes
% care of fetching the data using a |timer| object, you're free to access
% the command line for other program behavior.

m.startStreaming();
% other program behavior
tic;
while toc<5
  fprintf('Number of samples: %5d\n',length(m.time_log));
  pause(1);
end
m.stopStreaming();

%%
% You'll notice that as time goes on, the size of the |data_log|
% properties grows as data is fetched by |MyoMex|. You can read data from
% these properties while streaming, but most other methods are not
% functional in this state.
%
% Since we've accumulated some logged data, let's look at a plot of the
% accelerometer data.

figure;
plot(m.time_log,m.accel_log);
ylabel('Accelerometer in sensor frame [g]'); xlabel('Time [s]');

%%
% We can also transform the |gyro| and |accel| data from sensor frame to
% fixed frame by getting the dependent property |rot_log|. This is a 3D
% array in which each 3x3 2D slice is the rotation matrix corresponding to
% the rows of |quat_log|.

R = m.rot_log;
accel_fixed = zeros(size(m.accel_log));
for kk = 1:size(R,3)
  accel_fixed(kk,:) = (R(:,:,kk)*m.accel_log(kk,:)')';
end

figure;
plot(m.time_log,accel_fixed);
ylabel('Accelerometer in fixed frame [g]'); xlabel('Time [s]');


%%
% Finally, when you're done with |MyoMex|, don't forget to clean up!

m.delete;
clear m
  
%%
% Finally, take advantages of the following resources for additional
% information about |MyoMex|!

% MyoMexGUI_Monitor
% properties MyoMex
% methods MyoMex
% help MyoMex
% help MyoMex.time
% help MyoMex.quat
% help MyoMex.getData
% help MyoMex.startStreaming


  