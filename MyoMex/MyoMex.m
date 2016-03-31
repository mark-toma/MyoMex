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
  %   plot(this.time_log,m.accel_log,'-',m.time_log,m.emg_log,'-');
  %
  %   m.delete();
  %   clear m
  
  properties (SetAccess = private)
    % myo_data  Data objects for physical Myo devices
    myoData;
  end
  properties (Dependent,Hidden=true)
    currTime;
  end
  properties (Access=private,Hidden=true)
    timerStreamingData = [];
    nowInit;
    DEFAULT_STREAMING_FRAME_TIME = 0.040;
  end
  
  methods
    
    %% --- Object Management
    function this = MyoMex(countMyos)
      % MyoMex  Construct a MyoMex object.
      
      if nargin<1, countMyos = 1; end
      
      assert(isnumeric(countMyos) && isscalar(countMyos) && any(countMyos==[1,2]),...
        'Input countMyos must be a numeric scalar in [1,2].');
      
      % we depend on finding resources in the root directory for this class
      class_root_path = fileparts(mfilename('fullpath'));
      % check that myo_mex exists as a mex file in the expected location
      assert(3==exist(fullfile(class_root_path,'myo_mex/myo_mex')),...
        'MEX-file ''myo_mex'' is not on MATLAB''s search path and is required for MyoMex.');
      
      % check to see if myo_mex is in an initializable state
      try % myo_mex_assert_ready_to_init returns iff myo_mex is unlocked
        MyoMex.myo_mex_assert_ready_to_init();
      catch err % myo_mex_assert_ready_to_init couldn't unlock myo_mex
        error('MyoMex failed to initialize because myo_mex_assert_ready_to_init() could not bring myo_mex into a known unlocked state with failure message:\n\t''%s''',err.message);
      end
      
      % call into myo_mex init
      [fail,emsg,countMyosInit] = this.myo_mex_init();
      if fail
        if strcmp(emsg,'Myo failed to init!') % extra hint
          warning('Myo will fail to init if it is not connected to your system via Myo Connect.');
        end
        error('MEX-file ''myo_mex'' failed to initialize with error:\n\t''%s''',emsg);
      end
      
      % error out if myo_mex failed to initialize with desired countMyos
      if countMyosInit ~= countMyos
        this.myo_mex_delete(); % clean up myo_mex internal state
        this.myo_mex_clear(); % clean up mex file myo_mex
        error('MyoMex failed to initialize %d Myos. myo_mex initialized to %d Myos instead.',...
          countMyos,countMyosInit);
      end
      
      this.myoData = MyoData(countMyos);
      
      % at this point, myo_mex should be alive!
      this.nowInit = now;
      
      this.startStreaming();
      
    end
    
    function delete(this)
      % delete  Clean up MyoMex instance of MEX function myo_mex
      this.stopStreaming();
      [fail,emsg] = MyoMex.myo_mex_delete;
      assert(~fail,...
        'myo_mex delete failed with message:\n\t''%s''',emsg);
      MyoMex.myo_mex_clear();
    end
    
    function val = get.currTime(this)
      val = (now - this.nowInit)*24*60*60;
    end
    
  end
  
  methods (Access=private,Hidden=true)
    
    %% --- Streaming
    function startStreaming(this)
      % startStreaming  Start streaming data from myo_mex
      assert(isempty(this.timerStreamingData),...
        'MyoMex is already streaming.');
      this.timerStreamingData = timer(...
        'busymode','drop',...
        'executionmode','fixedrate',...
        'name','MyoMex-timerStreamingData',...
        'period',this.DEFAULT_STREAMING_FRAME_TIME,...
        'startdelay',this.DEFAULT_STREAMING_FRAME_TIME,...
        'timerfcn',@(src,evt)this.timerStreamingDataCallback(src,evt));
      [fail,emsg] = this.myo_mex_start_streaming();
      if fail
        delete(this.timerStreamingData);
        this.timerStreamingData = [];
        warning('myo_mex start_streaming failed with message\n\t''%s''',emsg);
        return;
      end
      start(this.timerStreamingData);
    end
    
    function stopStreaming(this)
      % stopStreaming  Stop streaming data from myo_mex
      assert(~isempty(this.timerStreamingData),...
        'MyoMex is not streaming.');
      stop(this.timerStreamingData);
      delete(this.timerStreamingData);
      this.timerStreamingData = [];
      [fail,emsg] = this.myo_mex_stop_streaming();
      assert(~fail,...
        'myo_mex stop_streaming failed with message\n\t''%s''',emsg);
    end
    
    function timerStreamingDataCallback(this,~,~)
      % timerStreamingDataCallback  Fetch streaming data from myo_mex
      %   MyoMex.timerStreamingData triggers this callback to schedule
      %   regular fetching of the data from the Myo devices in myo_mex.
      %   Subsequently, the fetched data is sent into myoData for logging
      %   and future access.
      %
      %   If an error is thrown by myo_mex during a call
      %   into get_streaming_data, then the callback cleans up myo_mex and
      %   MyoMex, thus invalidating this object. This is known to happen
      %   when a Myo device goes to sleep. Applications built on MyoMex
      %   should manage the MyoMex lifetime around scenarios in which the
      %   Myo is being utilized by the user.
      [fail,emsg,data] = this.myo_mex_get_streaming_data();
      if fail, this.delete(); end
      assert(~fail,...
        'myo_mex get_streaming_data failed with message\n\t''%s''\n%s',emsg,...
        sprintf('MyoMex has been cleaned up and destroyed.'));
      this.myoData.addData(data,this.currTime);
    end
    
  end
  
  %% --- Wrappers for myo_mex Interface
  % MEX-file implementation wrapped up in static methods for simpler syntax
  % in writing the class above... I was having a bunch of trouble writing
  % the single quotes
  methods (Static=true,Access=private,Hidden=true)
    
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

