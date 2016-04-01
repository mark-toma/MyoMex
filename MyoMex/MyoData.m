classdef MyoData < handle
  % MyoData  Data collector for a physical Myo device
  %   This class exposes Myo data to users by way of so-called <data> and
  %   <data>_log properties (See the example below). It is assumed that
  %   MyoData is owned by a data provider such as a MyoMex object. Users
  %   simply acquire the handle to a MyoData object from the provider, and
  %   then use the data. Under the hood, the MyoMex object is repeatedly
  %   calling into MyoData to give it more data as it becomes available.
  %
  % Example:
  %
  %   mm = MyoMex(); % instantiate MyoMex with one Myo
  %   m = mm.myoData; % acquire the MyoData handle
  %
  %   % Now the MyoData object is automatically accumulating data logs from
  %   % Myo as they are being provided by MyoMex.
  %
  %   % Inspect the <data> properties' most recent samples
  %   m.timeIMU % timeIMU corresponds to IMU data ONLY
  %   m.quat
  %   m.rot         % rotation matrix   - computed from quat
  %   m.gyro        % sensor frame gyro
  %   m.gyro_fixed  % fixed frame gyro  - computed from gyro and quat
  %   m.accel       % sensor frame accel
  %   m.accel_fixed % fixed frame accel - computed from accel and quat
  
  %
  %   m.timeEMG % corresponds to all other data listed below
  %   m.emg
  %   m.pose        % enumerated data - logical values below
  %   m.pose_rest
  %   m.pose_fist
  %   m.pose_wave_in
  %   m.pose_wave_out
  %   m.pose_fingers_spread
  %   m.arm         % enumerated data - logical cases below
  %   m.arm_right
  %   m.arm_left
  %   m.arm_unknown
  %   m.xDir        % enumerated data - logical values below
  %   m.xDir_wrist
  %   m.xDir_elbow
  %   m.xDir_unknown
  %
  %   % Inspect the <data>_log properties
  %   % Just append "_log" to all of the <data> property names above. Take
  %   % care to use the correct time vectors. Here, I'll generate a plot.
  %
  %   figure;
  %   subplot(3,1,1); plot(m.timeIMU_log,m.gyro_log);  title('gyro');
  %   subplot(3,1,2); plot(m.timeIMU_log,m.accel_log); title('accel');
  %   subplot(3,1,2); plot(m.timeEMG_log,m.emg_log);   title('emg');
  %
  %   % Don't forget to close your session with Myo!
  %   mm.delete();
  %
  %   % Now the MyoData objects aren't receiving new data, but you can
  %   % still use them to analyze, save, etc. the data you collected.  
    
  properties (SetAccess = private)
    
    timeIMU
    quat
    gyro
    gyro_fixed
    accel
    accel_fixed
    
    timeEMG
    emg
    pose
    
    arm
    xDir
    
    pose_rest
    pose_fist
    pose_wave_in
    pose_wave_out
    pose_fingers_spread
    pose_double_tap
    pose_unknown
    
    arm_left
    arm_right
    arm_unknown
    
    xDir_wrist
    xDir_elbow
    xDir_unknown
    
    isStreaming = true;
    
  end
  
  properties (SetAccess=private,Hidden=true)
    timeIMU_log     = [];
    quat_log        = [];
    gyro_log        = [];
    gyro_fixed_log  = [];
    accel_log       = [];
    accel_fixed_log = [];
    timeEMG_log     = [];
    emg_log         = [];
    pose_log        = [];
    arm_log         = [];
    xDir_log        = [];
    POSE_REST           = 0;
    POSE_FIST           = 1;
    POSE_WAVE_IN        = 2;
    POSE_WAVE_OUT       = 3;
    POSE_FINGERS_SPREAD = 4;
    POSE_DOUBLE_TAP     = 5;
    POSE_UNKNOWN        = hex2dec('ffff');
  end
  
  properties (Dependent)
    rot;
  end
  
  properties (Dependent,Hidden=true)
    rot_log;
    pose_rest_log;
    pose_fist_log;
    pose_wave_in_log;
    pose_wave_out_log;
    pose_fingers_spread_log;
    pose_double_tap_log;
    pose_unknown_log;
    arm_left_log;
    arm_right_log;
    arm_unknown_log;
    xDir_wrist_log;
    xDir_elbow_log;
    xDir_unknown_log;
  end
  
  properties (Access=private,Hidden=true)
    prevTimeIMU = [];
    prevTimeEMG = [];
    
    % libmyo.h enum order: right, left, unknown
    % DeviceListener.hpp  enum order: left, right, unknown
    ARM_RIGHT     = 0;
    ARM_LEFT      = 1;
    ARM_UNKNOWN   = 2;
    
    XDIR_WRIST    = 0;
    XDIR_ELBOW    = 1;
    XDIR_UNKNOWN  = 2;
    
    IMU_SAMPLE_TIME = 0.020; %  50Hz
    EMG_SAMPLE_TIME = 0.005; % 200Hz
    EMG_SCALE = 128;
    RESTART_DELAY = 0.5;
  end
  
  methods
    %% --- Object Management
    function this = MyoData(countMyos)
      % MyoData  Instantiate a vector of MyoData objects
      %   A MyoData object represents the data for a unique physical Myo
      %   device. These objects are used by MyoMex to store and interact
      %   with data as it's collected from a device. This constructor takes
      %   one scalar number, countMyos, to instantiate a vector of objects
      %   with this length. The resulting length of MyoData is at least 1.
      if nargin<1
        countMyos = 0;
      end
      
      assert(isnumeric(countMyos) && isscalar(countMyos) && ~mod(countMyos,1),...
        'Input countMyos must be numeric scalar in {0,1,2,...}.');
      
      if countMyos > 0
        this(countMyos) = MyoData();
      end
      
    end
    
    function delete(this)
      
    end
    
    %% --- Dependent Getters
    function val = get.rot(this)
      if isempty(this.quat)
        val = [];
        return;
      end
      val = this.q2r(this.quat);
    end
    function val = get.rot_log(this)
      if isempty(this.quat_log)
        val = [];
        return;
      end
      val = this.q2r(this.quat);
    end
    function val = get.pose_rest(this)
      val = this.pose == this.POSE_REST;
    end
    function val = get.pose_fist(this)
      val = this.pose == this.POSE_FIST;
    end
    function val = get.pose_wave_in(this)
      val = this.pose == this.POSE_WAVE_IN;
    end
    function val = get.pose_wave_out(this)
      val = this.pose == this.POSE_WAVE_OUT;
    end
    function val = get.pose_fingers_spread(this)
      val = this.pose == this.POSE_FINGERS_SPREAD;
    end
    function val = get.pose_double_tap(this)
      val = this.pose == this.POSE_DOUBLE_TAP;
    end
    function val = get.pose_unknown(this)
      val = this.pose == this.POSE_UNKNOWN;
    end
    function val = get.pose_rest_log(this)
      val = this.pose_log == this.POSE_REST;
    end
    function val = get.pose_fist_log(this)
      val = this.pose_log == this.POSE_FIST;
    end
    function val = get.pose_wave_in_log(this)
      val = this.pose_log == this.POSE_WAVE_IN;
    end
    function val = get.pose_wave_out_log(this)
      val = this.pose_log == this.POSE_WAVE_OUT;
    end
    function val = get.pose_fingers_spread_log(this)
      val = this.pose_log == this.POSE_FINGERS_SPREAD;
    end
    function val = get.pose_double_tap_log(this)
      val = this.pose_log == this.POSE_DOUBLE_TAP;
    end
    function val = get.pose_unknown_log(this)
      val = this.pose_log == this.POSE_UNKNOWN;
    end
    function val = get.arm_right(this)
      val = this.arm == this.ARM_RIGHT;
    end
    function val = get.arm_left(this)
      val = this.arm == this.ARM_LEFT;
    end
    function val = get.arm_unknown(this)
      val = this.arm == this.ARM_UNKNOWN;
    end
    function val = get.arm_right_log(this)
      val = this.arm_log == this.ARM_RIGHT;
    end
    function val = get.arm_left_log(this)
      val = this.arm_log == this.ARM_LEFT;
    end
    function val = get.arm_unknown_log(this)
      val = this.arm_log == this.ARM_UNKNOWN;
    end
    function val = get.xDir_wrist(this)
      val = this.xDir == this.XDIR_WRIST;
    end
    function val = get.xDir_elbow(this)
      val = this.xDir == this.XDIR_ELBOW;
    end
    function val = get.xDir_unknown(this)
      val = this.xDir == this.XDIR_UNKNOWN;
    end
    function val = get.xDir_wrist_log(this)
      val = this.xDir_log == this.XDIR_WRIST;
    end
    function val = get.xDir_elbow_log(this)
      val = this.xDir_log == this.XDIR_ELBOW;
    end
    function val = get.xDir_unknown_log(this)
      val = this.xDir_log == this.XDIR_UNKNOWN;
    end
    
    %% ---
    function clearLogs(this)
      % clearLogs  Clears logged data
      %   Sets all <data>_log properties to the empty matrix. Do not call
      %   this method while MyoMex is_streaming.
      for ii = 1:length(this)
        this(ii).timeIMU_log     = [];
        this(ii).quat_log        = [];
        this(ii).gyro_log        = [];
        this(ii).gyro_fixed_log  = [];
        this(ii).accel_log       = [];
        this(ii).accel_fixed_log = [];
        this(ii).timeEMG_log     = [];
        this(ii).emg_log         = [];
        this(ii).pose_log        = [];
        this(ii).arm_log         = [];
        this(ii).xDir_log        = [];
      end
    end
    
    function startStreaming(this), this.isStreaming = true; end
    
    function stopStreaming(this), this.isStreaming = false; end
    
  end
  
  methods (Access={?MyoMex})
    %% --- Data Provider Access
    function addData(this,data,currTime)
      % addData  Adds new data
      assert(length(this)==length(data),...
        'Input data must be the same length as MyoData');
      
      for ii=1:length(this)
        this(ii).addDataIMU(data(ii),currTime);
        this(ii).addDataEMG(data(ii),currTime);
      end
    end
    
  end
  
  methods (Access=private)
    %% --- Internal Data Management
    function addDataIMU(this,data,currTime)
      if isempty(data.quat), return; end
      
      N = size(data.quat,1);
      t = (1:1:N)' * this.IMU_SAMPLE_TIME;
      
      q = data.quat;
      g = data.gyro;
      a = data.accel;
      q = this.qRenorm(q); %renormalize quaterions
      gf = this.qRot(q,g);
      af = this.qRot(q,a);
      
      if ~isempty(this.prevTimeIMU)
        t = t + this.prevTimeIMU;
      else % init time
        t = t - t(end) + currTime;
        t = t(2:end,:);
        q = q(2:end,:);
        g = g(2:end,:);
        gf = gf(2:end,:);
        a = a(2:end,:);
        af = af(2:end,:);
      end
      
      this.prevTimeIMU = t(end);
      
      this.timeIMU = t(end,:);
      this.quat = q(end,:);
      this.gyro = g(end,:);
      this.gyro_fixed = gf(end,:);
      this.accel = a(end,:);
      this.accel_fixed = af(end,:);
      
      if this.isStreaming, this.pushLogsIMU(t,q,g,gf,a,af); end
      
    end
    
    function addDataEMG(this,data,currTime)
      if isempty(data.emg), return; end
      
      N = size(data.emg,1);
      t = (1:1:N)' * this.EMG_SAMPLE_TIME;
      
      e = data.emg./this.EMG_SCALE;
      p = data.pose;
      a = data.arm;
      x = data.xDir;
      
      if ~isempty(this.prevTimeEMG)
        t = t + this.prevTimeIMU;
      else % init time
        t = t - t(end) + currTime;
        % chop off first data point
        t = t(2:end,:);
        e = e(2:end,:);
        p = p(2:end,:);
        a = a(2:end,:);
        x = x(2:end,:);
      end
      this.prevTimeEMG = t(end);
      

      
      this.timeEMG = t(end,:);
      this.emg = e(end,:);
      this.pose = p(end,:);
      this.arm = a(end,:);
      this.xDir = x(end,:);
      
      if this.isStreaming, this.pushLogsEMG(t,e,p,a,x); end
      
    end
    
    function pushLogsIMU(this,t,q,g,gf,a,af)
      this.timeIMU_log     = [ this.timeIMU_log     ; t  ];
      this.quat_log        = [ this.quat_log        ; q  ];
      this.gyro_log        = [ this.gyro_log        ; g  ];
      this.gyro_fixed_log  = [ this.gyro_fixed_log  ; gf ];
      this.accel_log       = [ this.accel_log       ; a  ];
      this.accel_fixed_log = [ this.accel_fixed_log ; af ];
    end
    
    function pushLogsEMG(this,t,e,p,a,x)
      this.timeEMG_log   = [ this.timeEMG_log ; t ];
      this.emg_log       = [ this.emg_log     ; e ];
      this.pose_log      = [ this.pose_log    ; p ];
      this.arm_log       = [ this.arm_log     ; a ];
      this.xDir_log      = [ this.xDir_log    ; x ];
    end
    
  end
  
  methods (Static=true)
    %% --- Quaternion Operations
    function qn = qRenorm(q)
      % qRenorm  Normalized quaternion q to unit magnitude in qn
      n = sqrt(sum(q'.^2))'; % column vector of quaternion norms
      N = repmat(n,[1,4]);
      qn = q./N;
    end
    function R = q2r(q)
      % q2r  Convert unit quaternions to rotation matrices
      %   Input q is Kx4 where each k-th row is a unit quaternion and the
      %   scalar is listed first. Output R is 3x3xK where each kk-th 2D
      %   slice is the rotation matrix representing q(kk,:).
      %
      %   R(:,:,kk) premultiplies a 3x1 vector p to perform the same
      %   rotation as quaternion pre-multiplication by q(kk,:) and
      %   post-multiplication by the inverse of q(kk,:).
      R = zeros(3,3,size(q,1));
      for kk = 1:size(R,3)
        s = q(1); v = q(2:4)';
        vt = [0,-v(3),v(2);v(3),0,-v(1);-v(2),v(1),0]; % cross matrix
        R(:,:,kk) = eye(3) + 2*v*v' - 2*v'*v*eye(3) + 2*s*vt;
      end
    end
    function r = qRot(q,p)
      % qRot  Rotate vectors by unit quaternions
      %   Input q is Kx4 where each k-th row is a unit quaternion and the
      %   scalar is listed first. Input p is Kx3 where each k-th row is a
      %   1x3 vector. Output r is Kx3 where r(kk,:) is the image of p(kk,:)
      %   under rotation by quaternion q(kk,:).
      %
      %   The quaternion rotation of vector p by quaternion q is performed
      %   by pre-multiplication of p by q and post-multiplcation by the
      %   inverse of q.
      pq = [zeros(size(p,1),1),p]; % vector quaternion with zero scalar
      qi = MyoData.qInv(q);
      rq = MyoData.qMult(MyoData.qMult(q,pq),qi);
      r = rq(:,2:4);
    end
    function qp = qMult(ql,qr)
      % qMult  Multiply quaternions
      %   Inputs ql and qr are Kx4 where each k-th row is a unit quaternion
      %   and the scalar is listed first. Output qp is Kx4 where each kk-th
      %   row contains the quaternion product of the kk-th rows of the
      %   inputs ql and qr.
      sl = ql(:,1); vl = ql(:,2:4);
      sr = qr(:,1); vr = qr(:,2:4);
      sp = sl.*sr - dot(vl,vr,2);
      vp = repmat(sl,[1,3]).*vr + ...
        +  repmat(sr,[1,3]).*vl + ...
        + cross(vl,vr,2);
      qp = [sp,vp];
    end
    function qi = qInv(q)
      % qInv  Inverse of unit quaternions
      %   Input q is Kx4 where each k-th row is a unit quaternion and the
      %   scalar is listed first. Since q(kk,:) are assumed to be unit
      %   magnitude, the inverse is simply the conjugate.
      qi = q.*repmat([1,-1,-1,-1],[size(q,1),1]);
    end
  end
  
end

