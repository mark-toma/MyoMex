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
  
  properties (SetAccess = private)
    % myo_data  Data objects for physical Myo devices
    myo_data = [MyoData,MyoData];
    % is_streaming  Streaming status
    %   Read this property to test the streaming status. This is set by
    %   startStreaming() and unset by stopStreaming(). If is_streaming is
    %   set then the only valid method call is to stopStreaming().
    is_streaming = false;
  end
  
  properties (Dependent)

  end
  properties (Dependent,Hidden=true)
    curr_time;
  end
  properties (Access=private,Hidden=true)
    TimerStreamingData = [];
    now_init;
    streaming_data_init_time;
    DEFAULT_STREAMING_FRAME_TIME = 0.040;
  end
  
  methods
    
    %% --- Object Management
    function m = MyoMex(num_myos)
      % MyoMex  Construct a MyoMex object.
      
      if nargin<1
        num_myos = 1;
      elseif ~any(num_myos==[1,2])
        error('MyoMex only supports 1 or 2 Myo devices.');
      end
      
      % we depend on finding resources in the root directory for this class
      class_root_path = fileparts(mfilename('fullpath'));
      % check that myo_mex exists as a mex file in the expected location
      if exist(fullfile(class_root_path,'myo_mex/myo_mex'))~=3
        error('MEX-file ''myo_mex'' is not on MATLAB''s search path and is required for MyoMex.');
      end
      
      try % myo_mex_assert_ready_to_init returns iff myo_mex is unlocked
        m.myo_mex_assert_ready_to_init();
      catch err % myo_mex_assert_ready_to_init couldn't unlock myo_mex
        error('MyoMex failed to initialize because myo_mex_assert_ready_to_init() could not bring myo_mex into a known unlocked state with failure message:\n\t''%s''',err.message);
      end
      
      [fail,emsg,data] = m.myo_mex_init;
      if fail
        if strcmp(emsg,'Myo failed to init!') % extra hint
          warning('Myo will fail to init if it is not connected to your system via Myo Connect.');
        end
        error('MEX-file ''myo_mex'' failed to initialize with error:\n\t''%s''',emsg);
      end
      
      if data~=num_myos
        error('MyoMex failed to initialize %d Myos. myo_mex initialized to %d Myos instead.',...
          num_myos,data);
      end
      
      if num_myos==1
        m.myo_data = MyoData;
      elseif num_myos==2
        m.myo_data = [MyoData,MyoData];
      end
      
      % at this point, myo_mex should be alive!
      m.now_init = now;
      
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
    
        
    %% --- Streaming
    function startStreaming(m)
      % startStreaming  Start streaming data
      %   While MyoMex is_streaming, data from Myo will be sampled at
      %   streaming_data_time into a buffer in MEX function myo_mex. Then a
      %   MATLAB timer will call into MEX function myo_mex every
      %   streaming_frame_time seconds to fetch the buffered data and push
      %   it into the properties of MyoMex.
      if m.is_streaming, return; end
      m.is_streaming = true; % block
      
      % construct timer
      if isempty(m.TimerStreamingData)
        % create timer
        m.TimerStreamingData = timer(...
          'busymode','drop',...
          'executionmode','fixedrate',...
          'name','MyoMex-TimerStreamingData',...
          'period',m.DEFAULT_STREAMING_FRAME_TIME,...
          'startdelay',m.DEFAULT_STREAMING_FRAME_TIME,...
          'timerfcn',@(src,evt)m.timerStreamingDataCallback(src,evt));
      end
      
      [fail,emsg] = m.myo_mex_start_streaming();
      if fail
        m.is_streaming = false;
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
      if ~m.is_streaming, return; end
      
      [fail,emsg] = m.myo_mex_stop_streaming();
      if fail
        error('myo_mex stop_streaming failed with message\n\t''%s''',emsg);
      end
      
      stop(m.TimerStreamingData);
      delete(m.TimerStreamingData);
      m.TimerStreamingData = [];
      m.is_streaming = false;
      
    end
    
  end
  
  methods (Access=private,Hidden=true)
    
    function timerStreamingDataCallback(m,~,~)
      
      [fail,emsg,data] = m.myo_mex_get_streaming_data();
            
      % bail if getting data failed
      if fail, return; end
      
      % Then push addData to all existing
      for ii=1:length(data)
        m.myo_data(ii).addData(data(ii));
      end
      
    end
        
  end
  
  %% --- Wrappers for myo_mex Interface
  % MEX-file implementation wrapped up in static methods for simpler syntax
  % in writing the class above... I was having a bunch of trouble writing
  % the single quotes
  methods (Access=private,Static=true,Hidden=true)
    
    function [fail,emsg,data] = myo_mex_init()
      fail = false; emsg = [];
      data = [];
      try
        data = myo_mex('init');
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
      
      if ~MyoMex.myo_mex_is_locked();
        return;
      end
      
      warning('MEX-file ''myo_mex'' is locked. Attemping to unlock and re-initialize.');
      
      % see if delete brings us out of locked status
      % this should work if myo_mex isn't in streaming mode
      [fail,emsg] = MyoMex.myo_mex_delete();
      if fail
        warning('myo_mex_delete failed with message:\n\t''%s''\n',emsg);
      else
        % if it's still locked
        if ~MyoMex.myo_mex_is_locked(), return; else error(emsg_bad_state); end
      end
      
      % see if we can stop streaming then call delete
      [fail,emsg] = MyoMex.myo_mex_stop_streaming();
      if fail
        warning('myo_mex_stop_streaming failed with message:\n\t''%s''\n',emsg);
        error(emsg_bad_state);
      else
        % try the delete routine all over again
        [fail,emsg] = MyoMex.myo_mex_delete();
        if fail
          warning('myo_mex_delete failed again with message:\n\t''%s''\n',emsg);
          error(emsg_bad_state);
        else
          % if it's still locked
          if ~MyoMex.myo_mex_is_locked(), return; else error(emsg_bad_state); end
        end
      end
      
    end
    
  end
  
end

