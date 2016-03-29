classdef MyoData < handle
  % MyoData  This class represents a myo device
  %
  
  properties
    
  end
  
  properties (SetAccess = private)
    
    timeIMU = [];
    quat = [];
    gyro = [];
    gyro_fixed = [];
    accel = [];
    accel_fixed = [];
    
    timeEMG = [];
    emg = [];
    pose = [];
    
    pose_rest;
    pose_fist;
    pose_wave_in;
    pose_wave_out;
    pose_fingers_spread;
    pose_double_tap;
    pose_unknown;
    
  end
  
  properties (SetAccess=private,Hidden=true)
    timeIMU_log     = [];
    quat_log        = [];
    gyro_log        = [];
    gyro_fixed_log  = [];
    accel_log       = [];
    accel_fixed_log = [];
    timeEMG_log    = [];
    emg_log         = [];
    pose_log        = [];
    POSE_NUM_REST           = 0;
    POSE_NUM_FIST           = 1;
    POSE_NUM_WAVE_IN        = 2;
    POSE_NUM_WAVE_OUT       = 3;
    POSE_NUM_FINGERS_SPREAD = 4;
    POSE_NUM_DOUBLE_TAP     = 5;
    POSE_NUM_UNKNOWN        = 6;
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
  end
  
  properties (Access=private,Hidden=true)
    IMU_SAMPLE_TIME = 0.020; %  50Hz
    EMG_SAMPLE_TIME = 0.005; % 200Hz
    EMG_SCALE = 128;
    RESTART_DELAY = 0.5;
  end
  
  methods
    
    function m = MyoData()
      
    end
    
    function delete(m)
      
    end
    
    function val = get.rot(m)
      if isempty(m.quat)
        val = [];
        return;
      end
      val = m.q2r(m.quat);
    end
    function val = get.rot_log(m)
      if isempty(m.quat_log)
        val = [];
        return;
      end
      val = m.q2r(m.quat);
    end
    function val = get.pose_rest(m)
      val = m.pose == m.POSE_NUM_REST;
    end
    function val = get.pose_fist(m)
      val = m.pose == m.POSE_NUM_FIST;
    end
    function val = get.pose_wave_in(m)
      val = m.pose == m.POSE_NUM_WAVE_IN;
    end
    function val = get.pose_wave_out(m)
      val = m.pose == m.POSE_NUM_WAVE_OUT;
    end
    function val = get.pose_fingers_spread(m)
      val = m.pose == m.POSE_NUM_FINGERS_SPREAD;
    end
    function val = get.pose_double_tap(m)
      val = m.pose == m.POSE_NUM_DOUBLE_TAP;
    end
    function val = get.pose_unknown(m)
      val = m.pose == m.POSE_NUM_UNKNOWN;
    end
    function val = get.pose_rest_log(m)
      val = m.pose_log == m.POSE_NUM_REST;
    end
    function val = get.pose_fist_log(m)
      val = m.pose_log == m.POSE_NUM_FIST;
    end
    function val = get.pose_wave_in_log(m)
      val = m.pose_log == m.POSE_NUM_WAVE_IN;
    end
    function val = get.pose_wave_out_log(m)
      val = m.pose_log == m.POSE_NUM_WAVE_OUT;
    end
    function val = get.pose_fingers_spread_log(m)
      val = m.pose_log == m.POSE_NUM_FINGERS_SPREAD;
    end
    function val = get.pose_double_tap_log(m)
      val = m.pose_log == m.POSE_NUM_DOUBLE_TAP;
    end
    function val = get.pose_unknown_log(m)
      val = m.pose_log == m.POSE_NUM_UNKNOWN;
    end
        
    function clearLogs(m)
      % clearLogs  Clears logged data
      %   Sets all <data>_log properties to the empty matrix. Do not call
      %   this method while MyoMex is_streaming.
      m.timeIMU_log    = [];
      m.quat_log        = [];
      m.gyro_log        = [];
      m.gyro_fixed_log  = [];
      m.accel_log       = [];
      m.accel_fixed_log = [];
      m.timeEMG_log    = [];
      m.emg_log         = [];
      m.pose_log        = [];
    end
    
    function addData(m,data)
      % addData  Adds new data
      m.addDataIMU(data);
      m.addDataEMG(data);
    end
        
  end
  
  methods (Access=private)
    
    function addDataIMU(m,data)
%       data
      if isempty(data.quat), return; end
      t = data.timeIMU;
      q = data.quat;
      g = data.gyro;
      a = data.accel;
      q = m.qRenorm(q); %renormalize quaterions
      gf = m.qRot(q,g);
      af = m.qRot(q,a);
      m.timeIMU = t(end,:);
      m.quat = q(end,:);
      m.gyro = g(end,:);
      m.gyro_fixed = gf(end,:);
      m.accel = a(end,:);
      m.accel_fixed = af(end,:);
      m.pushLogsIMU(t,q,g,gf,a,af);
    end
    
    
    function addDataEMG(m,data)
      if isempty(data.timeEMG), return; end
      t = data.timeEMG;
      e = data.emg;
      p = data.pose;
      e = e./m.EMG_SCALE; % normalize emg values
      m.timeEMG = t(end,:);
      m.emg = e(end,:);
      m.pose = p(end,:);
      m.pushLogsEMG(t,e,p);      
    end
    
    function pushLogsIMU(m,t,q,g,gf,a,af)
      m.timeIMU_log     = [ m.timeIMU_log     ; t  ];
      m.quat_log        = [ m.quat_log        ; q  ];
      m.gyro_log        = [ m.gyro_log        ; g  ];
      m.gyro_fixed_log  = [ m.gyro_fixed_log  ; gf ];
      m.accel_log       = [ m.accel_log       ; a  ];
      m.accel_fixed_log = [ m.accel_fixed_log ; af ];
    end
    
    function pushLogsEMG(m,t,e,p)
      m.timeEMG_log   = [ m.timeEMG_log ; t ];
      m.emg_log       = [ m.emg_log     ; e ];
      m.pose_log      = [ m.pose_log    ; p ];
    end
  end
  
  methods (Static=true)
    
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

