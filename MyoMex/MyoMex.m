classdef MyoMex < handle
  % MyoMex  This is an m-code wrapper for a MEX wrapper for Myo SDK.
  %
  %   The class methods provide access to Myo functionality and raw data
  %   while the class properties expose the data to the MATLAB workspace
  %   and allow for configuration of some Myo program state.
  %
  %   The basic utility of the MyoMex object is that it provides a simple
  %   programmatic interface to acquire raw data from Myo (in near-real
  %   time) using either a polling or a streaming model without blocking
  %   the MATLAB command line unecessarily.
  %
  % Example (polling data):
  %
  %   m = MyoMex(); % instantiate
  %   m.getData();  % poll for data
  %
  %   % inspect most recent data
  %   m.time
  %   m.quat
  %   m.gyro
  %   m.accel
  %   m.emg
  %   m.pose
  %   m.pose_rest
  %   m.pose_fist
  %   m.pose_wave_in
  %   m.pose_wave_out
  %   m.pose_fingers_spread
  %   m.pose_double_tap
  %
  %   m.delete(); % eplicitly clean up the MyoMex object
  %   clear m
  %
  % Example (streaming Data):
  %
  %   m = MyoMex();
  %
  %   % Set desired sample time (reciprocal of rate) in seconds.
  %   % This determines the rate at which data samples are polled from Myo 
  %   % SDK in the MEX function myo_mex.
  %   m.streaming_data_time = 1/50;
  %
  %   % Set desired frame time (reciprocal of rate) in seconds
  %   % This determined the rate at which the data buffer is fetched from
  %   % the MEX function myo_mex as triggered by a MATLAB timer.
  %   % This should always be greater than the streaming_data_time and is
  %   % typically chosen to be no less than 0.040[s] or 40[ms].
  %   m.streaming_frame_time = 1/25;
  %
  %   % Start streaming
  %   % This starts a thread in the MEX function myo_mex that runs every 
  %   % streaming_data_time seconds. While this is happening, a MATLAB
  %   % timer is automatically calling m.getData every streaming_frame_time
  %   % seconds.
  %   m.startStreaming();
  %
  %   % Inspect data
  %   % While in streaming mode, the most recent data can be read from the
  %   % properties used in the previous example. But additionally, this
  %   % all data is pushed into properties with the same names and the
  %   % suffix "_log"
  %   m.time_log
  %   m.quat_log
  %   % ... and so on ...
  %
  %   % Stop streaming
  %   m.stopStreaming();
  %
  %   % Plot some data
  %   plot(m.time_log,m.accel_log,'-',m.time_log,m.emg_log,'-');
  %
  %   m.delete();
  %   clear m
  
  properties
    % streaming_frame_time - Time between buffered read of streaming data.
    %   This value should be greater than 0.04[s] and streaming_data_time.
    streaming_frame_time;
    % streaming_data_time - Time between sampling of Myo data.
    %   This value should be in the range 0.02[s] and 1[s].
    streaming_data_time;
  end
    
  properties (SetAccess = private)
    % myoData  Data objects for physical Myo devices
    myoData
    % is_streaming  Streaming status
    %   Read this property to test the streaming status. This is set by
    %   startStreaming() and unset by stopStreaming(). If is_streaming is
    %   set then the only valid method call is to stopStreaming().
    is_streaming = false;
  end
  
  properties (SetAccess=private,Hidden=true)
    TimerStreamingData = [];
    now_init;
    image_pose_fist;
    image_pose_wave_in;
    image_pose_wave_out;
    image_pose_fingers_spread;
    image_pose_double_tap;
  end
  
  properties (Dependent)
    curr_time; % Seconds since MyoMex construction
  end
  
  properties (Access=private)
    streaming_data_init_time;    
    % constants
    STREAMING_FRAME_TIME_DEF = 0.040; %    40ms =  25Hz
    STREAMING_FRAME_TIME_MIN = 0.040; %    40ms =  25Hz
    STREAMING_DATA_TIME_DEF  = 0.040; %   40ms =  25Hz
    STREAMING_DATA_TIME_MIN  = 0.020; %   20ms =  50Hz
    STREAMING_DATA_TIME_MAX  = 1.000; % 1000ms =   1Hz
  end
  
  methods
    
    function m = MyoMex()
      % MyoMex  Construct a MyoMex object.
      
      % TODO
      %   Is this the correct/best way to instantiate myoData?
      tmp(1) = MyoDataClass_devel();
      tmp(2) = MyoDataClass_devel();      
      m.myoData = tmp;
      
      % we depend on finding resources in the root directory for this class
      class_root_path = fileparts(mfilename('fullpath'));
      
      % check that myo_mex exists as a mex file in the expected location
      if exist(fullfile(class_root_path,'myo_mex/myo_mex'))~=3
        error('MEX-file ''myo_mex'' is not on MATLAB''s search path and is required for MyoMexClass_devel.');
      end
      
      try % myo_mex_assert_ready_to_init returns iff myo_mex is unlocked
        m.myo_mex_assert_ready_to_init();
      catch err % myo_mex_assert_ready_to_init couldn't unlock myo_mex
        error('MyoMex failed to initialize because myo_mex_assert_ready_to_init() could not bring myo_mex into a known unlocked state with failure message:\n\t''%s''',err.message);
      end
      
      [fail,emsg] = m.myo_mex_init;
      if fail
        if strcmp(emsg,'Myo failed to init!') % extra hint
          warning('Myo will fail to init if it is not connected to your system via Myo Connect.');
        end
        error('MEX-file ''myo_mex'' failed to initialize with error:\n\t''%s''',emsg);
      end
      
      % at this point, myo_mex should be alive!
      %m.streaming_data_time = m.STREAMING_DATA_TIME_DEF;
      m.streaming_frame_time = m.STREAMING_FRAME_TIME_DEF;
      m.now_init = now;
      
      % load image resources
      if ~exist(fullfile(class_root_path,'resources'),'dir')
        warning('MyoMex resources directory not found. Failed to initialize properties, ''image_pose_<pose-name>''');
        m.image_pose_fist = [];
        m.image_pose_wave_in = [];
        m.image_pose_wave_out = [];
        m.image_pose_fingers_spread = [];
        m.image_pose_double_tap = [];
        return;
      end
      % assume the images are actually in resources
      m.image_pose_fist = imread(fullfile(class_root_path,'resources','image_pose_fist.png'));
      m.image_pose_wave_in = imread(fullfile(class_root_path,'resources','image_pose_wave_in.png'));
      m.image_pose_wave_out = imread(fullfile(class_root_path,'resources','image_pose_wave_out.png'));
      m.image_pose_fingers_spread = imread(fullfile(class_root_path,'resources','image_pose_fingers_spread.png'));
      m.image_pose_double_tap = imread(fullfile(class_root_path,'resources','image_pose_double_tap.png'));
      
    end
        
    function delete(m)
      % delete  Clean up MyoMex instance of MEX function myo_mex
      %   This must be called to unlock the MEX function myo_mex. Failure
      %   to call into myo_mex('delete') before attempting to reinitialize
      %   MyoMex is undefined. Although this might work (the MyoMex
      %   constructor attempts to brute-force myo_mex unlocked), the
      %   command window fill up with warning text to indicate that
      %   something's not quite right.
      [fail,emsg] = m.myo_mex_delete;
      if fail
        error('myo_mex delete failed with message:\n\t''%s''',emsg);
      end
      m.myo_mex_clear();
    end
    
    function val = get.curr_time(m)
      val = (now-m.now_init)*24*60*60; % seconds since init
    end
    
    function set.streaming_data_time(m,val)
      
      if (nargin < 2) 
        error('Input val is required.');
      end
      if ~isnumeric(val) || ~isscalar(val)
        error('Input val must be a numerical scalar.');
      end
      
      if m.is_streaming
        warning('MyoMex will not change streaming data time while streaming.');
        return;
      end
      
      v = round(val*1000)/1000;
      if v ~= val
        warning('Desired value, %7.4f, rounded to millisecond precision: %7.4f.',...
          val,v);
      end
      
      % attempt to set time
      [fail,emsg] = m.myo_mex_set_streaming_time(v*1000);
      if fail
        warning('myo_mex failed to set streaming data time to %d[ms] with error message:\n\t''%s''',...
          v*1000,emsg);
        return;
      end
      [fail,emsg,vnew_millis] = m.myo_mex_get_streaming_time();
      vnew = vnew_millis/1000;
      if fail
        error('myo_mex failed to validate streaming data time change (resulting in inconsistent state) with error message:\n\t''%s''',...
          emsg);
      end
      if vnew~=v
        warning('myo_mex failed to set streaming data time to %d[ms]. Instead, the value is %d[ms].',...
          v*1000,vnew*1000);
      end
      
      m.streaming_data_time = vnew; % update property value
      
    end
    
    function set.streaming_frame_time(m,val)
      if m.is_streaming
        warning('MyoMex will not change streaming frame time while streaming.');
        return;
      end
      
      v = round(val*1000)/1000;
      if v ~= val
        warning('Desired value, %7.4f, rounded to millisecond precision: %7.4f.',...
          val,v);
      end
      
      if v<m.STREAMING_FRAME_TIME_MIN
        warning('Desirred value less than min value. Using min value.');
        v = m.STREAMING_FRAME_TIME_MIN;
      end
      
      if v<m.streaming_data_time
        warning('Desirred value less than streaming_data_time value. Using this value.');
        v = m.streaming_data_time;
      end
      
      m.streaming_frame_time = v; % update property value
      
    end
    
    function fail = getData(m)
      % getData  Get data from Myo by polling
      %   When not streaming, use this method explicitly to poll for data
      %   from Myo. When MyoMex is_streaming, it is not necessary (and not
      %   recommended) to call this method. There is a MATLAB timer set up
      %   to do that for you!
      if m.is_streaming
        [fail,emsg,data] = m.myo_mex_get_streaming_data();
      end
      
      % print number of myos
      %fprintf('Received data for %d devices.\n',length(data));
      
      % bail if getting data failed
      if fail, return; end
      
      curr_time = m.curr_time;
      % add data to each myoData element
      for ii=1:length(data)
        % supply getData with base time and sample time
        % getData(data,curr_time,is_streaming,Dt)
        m.myoData(ii).addData(data(ii),curr_time);
      end
      
    end
    
    function startStreaming(m)
      % startStreaming  Start streaming data
      %   While MyoMex is_streaming, data from Myo will be sampled at
      %   streaming_data_time into a buffer in MEX function myo_mex. Then a
      %   MATLAB timer will call into MEX function myo_mex every
      %   streaming_frame_time seconds to fetch the buffered data and push
      %   it into the properties of MyoMexClass_devel.
      if m.is_streaming, return; end
      
      m.is_streaming = true; % block
      
      % construct timer
      if isempty(m.TimerStreamingData)
        % create timer
        m.TimerStreamingData = timer(...
          'busymode','drop',...
          'executionmode','fixedrate',...
          'name','MyoMex-TimerStreamingData',...
          'period',m.streaming_frame_time,...
          'timerfcn',@(src,evt)m.timerStreamingDataCallback(src,evt));
      end
      
      [fail,emsg] = m.myo_mex_start_streaming();
      if fail
        warning('myo_mex start_streaming failed with message\n\t''%s''',emsg);
        delete(m.TimerStreamingData);
        m.TimerStreamingData = [];
        return;
      end
      
      start(m.TimerStreamingData);
      m.streaming_data_init_time = true;
      
    end
    
    function stopStreaming(m)
      % stopStreaming  Stop streaming data
      %   This method terminates streaming mode, and is expected to return
      %   all buffered data stored in the MEX function myo_mex.
      if ~m.is_streaming
        return;
      end
      
      [fail,emsg] = m.myo_mex_stop_streaming();
      if fail
        error('myo_mex start_streaming failed with message\n\t''%s''',emsg);
      end
      
      stop(m.TimerStreamingData);      
      delete(m.TimerStreamingData);
      m.TimerStreamingData = [];
      m.getData(); % one more call for good measure
      m.is_streaming = false;      
      
    end

  end
  
  methods (Access=private,Hidden=true)
    
    function timerStreamingDataCallback(m,~,~)
      m.getData;
    end
    
  end
  
  % MEX-file implementation wrapped up in static methods for simpler syntax
  % in writing the class above... I was having a bunch of trouble writing
  % the single quotes
  methods (Access=private,Static=true,Hidden=true)
    
    function [fail,emsg] = myo_mex_init()
      fail = false; emsg = [];
      try 
        myo_mex('init');
      catch err
        fail = true; emsg = err.message;
      end
    end
   
    function [fail,emsg] = myo_mex_start_streaming()
      fail = false; emsg = [];
      try 
        myo_mex('start_streaming');
      catch err
        fail = true; emsg = err.message;
      end
    end
    
    function [fail,emsg,data] = myo_mex_get_streaming_data()
      fail = false; emsg = [];
      data = [];
      try
        data = myo_mex('get_streaming_data');
      catch err
        fail = true; emsg = err.message;
      end
    end
    
    function [fail,emsg] = myo_mex_stop_streaming()
      fail = false; emsg = [];
      try 
        myo_mex('stop_streaming');
      catch err
        fail = true; emsg = err.message;
      end
    end
    
    function [fail,emsg] = myo_mex_delete()
      fail = false; emsg = [];
      try 
        myo_mex('delete');
      catch err
        fail = true; emsg = err.message;
      end
    end
    
    function [out] = myo_mex_is_locked()
      out = mislocked('myo_mex');
    end
    
    function myo_mex_clear()
      clear('myo_mex');
    end
    
    function myo_mex_assert_ready_to_init()
      % The purpose of this function is to validate that myo_mex is not
      % locked before calling into init myo_mex.
      %
      % If myo_mex isn't locked, we return gracefully. In perfect use
      % scenarios, this is the expected outcome. However, for sloppy use
      % and/or unexpected behavior of the matlab and/or the mex api, we
      % implement these checks to raise awareness for users.
      %
      % If myo_mex is locked then somethings wrong with one of: user code,
      % matlab, or mex api! Anywya, we try two different approches to
      % returning myo_mex to an expected unlocked state and issue warnings
      % to inform the user of this process (and thus fill the command
      % window with ugliness to indicate that something is not right).
      % Finally, if these attempts don't recover an unlocked state, then we
      % error out.
      %
      % The only way to return from this function without error is to pass
      % the mexislocked==false test. However, if you see a bunch of warning
      % text pop up, you should probably go back to the drawing board with
      % your code or investigate other causes to this ebnormal behavior.      
      
      emsg_bad_state = 'myo_mex is locked in an unknown state!';
      
      if ~MyoMexClass_devel.myo_mex_is_locked();
        return;
      end
      
      warning('MEX-file ''myo_mex'' is locked. Attemping to unlock and re-initialize.');
      
      % see if delete brings us out of locked status
      % this should work if myo_mex isn't in streaming mode
      [fail,emsg] = MyoMexClass_devel.myo_mex_delete();
      if fail
        warning('myo_mex_delete failed with message:\n\t''%s''\n',emsg);
      else
        % if it's still locked 
        if ~MyoMexClass_devel.myo_mex_is_locked(), return; else error(emsg_bad_state); end
      end
      
      % see if we can stop streaming then call delete
      [fail,emsg] = MyoMexClass_devel.myo_mex_stop_streaming();
      if fail
        warning('myo_mex_stop_streaming failed with message:\n\t''%s''\n',emsg);
        error(emsg_bad_state);
      else
        % try the delete routine all over again
        [fail,emsg] = MyoMexClass_devel.myo_mex_delete();
        if fail
          warning('myo_mex_delete failed again with message:\n\t''%s''\n',emsg);
          error(emsg_bad_state);
        else
          % if it's still locked
          if ~MyoMexClass_devel.myo_mex_is_locked(), return; else error(emsg_bad_state); end
        end
      end   
      
    end
    
  end
  
end

