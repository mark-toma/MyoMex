classdef MyoMex < handle
  % MyoMex  This is an m-code wrapper for a MEX wrapper for Myo SDK.
  %   This class manages the state of the MEX function myo_mex which
  %   provides the interface to Myo SDK, and thus the data from physical
  %   Myo devices. MyoMex is the glue between the Myo SDK data and the
  %   devices' MATLAB m-code objects of type MyoData.
  %
  %   Using MyoMex is very simple, (1) Instantiate MyoMex, (2) access
  %   MyoData objects, and (3) clean up the MyoMex instance when done.
  %
  % Example (minimal):
  %
  %  m = MyoMex(); % default expects one Myo paired with Myo Connect
  %  pause(1);     % wait a second
  %  m.myoData     % display the MyoData object
  %  m.delete();   % destruct MyoMex (thus cleaning up myo_mex resources)
  %
  % More Information:
  %
  % MyoMex calls into myo_mex to initialize a data connection to Myo SDK at
  % runtime. The MyoMex constructor take an optional input to specify the
  % number of Myos expected in Myo SDK. Once successfully initialized,
  % MyoMex will guarantee data from all Myos unless disconnected from
  % MyoConnect. In that case, MyoMex terminates itself.
  %
  % After initialization of myo_mex, MyoMex then calls back into myo_mex to
  % start a new worked thread. In the myo_mex environment, all samples of
  % data from Myo SDK are queued in a FIFO buffer waiting for MyoMex to
  % call back in and fetch the data.
  %
  % The last thing that happens in MyoMex construction is that a MATLAB
  % timer is started. This timer schedules calls back into myo_mex to fetch
  % data out of the FIFO. The timer callback then calls back into the
  % MyoMex property myoData (a MyoData object) to provide it with the
  % latest batch of samples.
  
  properties
    deleteFcn = []
  end
  properties (SetAccess = private)
    % myo_data  Data objects for physical Myo devices
    myoData
  end
  properties (Dependent,Hidden=true)
    currTime
  end
  properties (Access=private,Hidden=true)
    timerStreamingData
    nowInit
    DEFAULT_STREAMING_FRAME_TIME = 0.040
    NUM_INIT_SAMPLES = 4
    fatalErrorSource
  end
  
  methods
    
    %% --- Object Management
    function this = MyoMex(countMyos)
      % MyoMex  Construct a MyoMex object
      %
      % Inputs:
      %   countMyos - Number of Myos
      %     Numerical scalar specifying the number of physical Myo devices
      %     expected to be found from Myo Connect. Construction will fail
      %     if Myo Connect provides the Myo SDK with more or less Myos than
      %     countMyos. This parameter is optional with a default value 1.
      %
      % Outputs:
      %   this - Handle to MyoMex instance
      %     The output argument is required! Users should maintain this
      %     variable throughout the lifecycle of MyoMex and then explicitly
      %     call the delete() method on the object when finished.
      assert(nargout==1,...
        'MyoMex must be assigned to an output variable.');
      
      if nargin<1 || isempty(countMyos), countMyos = 1; end
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
      [fail,emsg] = this.myo_mex_init(countMyos);
      if fail
        if strcmp(emsg,'Myo failed to init!') % extra hint
          warning('Myo will fail to init if it is not connected to your system via Myo Connect.');
        end
        this.myo_mex_clear();
        error('MEX-file ''myo_mex'' failed to initialize with error:\n\t''%s''',emsg);
      end
      
      this.myoData = MyoData(countMyos);
      
      % at this point, myo_mex should be alive!
      this.nowInit = now;
      
      this.startStreaming();
      
    end
    
    function delete(this)
      % delete  Clean up MyoMex instance of MEX function myo_mex
      this.onDelete();
      this.stopStreaming();
      for ii = 1:length(this.myoData)
        delete(this.myoData(ii));
      end
      
      [fail,emsg] = MyoMex.myo_mex_delete;
      assert(~fail,...
        'myo_mex delete failed with message:\n\t''%s''',emsg);
      MyoMex.myo_mex_clear();
    end
    
    %% --- Setters
    function set.deleteFcn(this,val)
      assert( (isa(val,'function_handle') && 2==nargin(val)) || isempty(val),...
        'Property ''deleteFcn'' must be a function handle with two input arguments or the empty matrix.');
      this.deleteFcn = val;
    end
    
    %% --- Dependent Getters
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
        'startdelay',this.DEFAULT_STREAMING_FRAME_TIME*this.NUM_INIT_SAMPLES,...
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
      if fail
        this.fatalErrorSource = 'timerStreamingDataCallback';
        this.delete();
      end
      assert(~fail,...
        'myo_mex get_streaming_data failed with message\n\t''%s''\n%s',emsg,...
        sprintf('MyoMex has been cleaned up and destroyed.'));
      this.myoData.addData(data,this.currTime);
    end
    
    function onDelete(this)
      % onDelete  Calls deleteFcn from within the delete method
      %   Use the fatalErrorSource field of the evt structure to determine
      %   the cause of object destruction.
      %
      %   timerStreamingDataCallback
      %     Failure in timerStreamingData during call to get_streaming_data
      evt.fatalErrorSource = this.fatalErrorSource;
      if ~isempty(this.deleteFcn) && ...
          isa(this.deleteFcn,'function_handle')
        this.deleteFcn(this,evt);
      end
    end
  end
  
  %% --- Wrappers for myo_mex Interface
  % MEX-file implementation wrapped up in static methods for simpler syntax
  % in writing the class above... I was having a bunch of trouble writing
  % the single quotes
  methods (Static=true,Access=private,Hidden=true)
    
    function [fail,emsg] = myo_mex_init(countMyos)
      assert( (nargin==1) && isnumeric(countMyos) && isscalar(countMyos) && any(countMyos==[1,2]),...
        'Input countMyos must be a numeric scalar in [1,2].');
      fail = false; emsg = [];
      try
        myo_mex('init',countMyos);
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
      % matlab, or mex api! Anyway, we try two different approches to
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
        assert(~MyoMex.myo_mex_is_locked(),emsg_bad_state);
        if ~MyoMex.myo_mex_is_locked(), return; end
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

