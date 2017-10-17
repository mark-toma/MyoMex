function myNewDataFcn(src,~)
% myNewDataFcn(src,evt)  Callback that executes after MyoMex sends new data
% to its MyoData objects
% 
% Inputs:
%   src  Handle to the MyoMex object

% get 
mm = src; % MyoMex object
m = mm.myoData; % MyoData object

% compute on data
W = 100; % window size
emg_log = m.emg_log; % emg data copy
[M,N] = size(emg_log); 
inds = M-W+1:M;
inds(inds<1) = []; % remove invalid indices
emg_window = emg_log(inds,:); % Wx8 array of the most recent W EMG samples
emg_mean = mean(emg_window); % 1x8 array of emg mean values computed on the window

% store results back in MyoMex/userData in a structure
ud.x = emg_window;
ud.features = emg_mean;
mm.userData = ud; % store ud struct in userData

end