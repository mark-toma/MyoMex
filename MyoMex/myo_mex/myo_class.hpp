
#ifndef MYO_CLASS_HPP
#define MYO_CLASS_HPP
#endif // ndef MYO_CLASS_HPP

#include "myo/myo.hpp"  // myo sdk cpp binding
#include <array>        // myo sdk emg data

#define POSE_NUM_REST           0
#define POSE_NUM_FIST           1
#define POSE_NUM_WAVE_IN        2
#define POSE_NUM_WAVE_OUT       3
#define POSE_NUM_FINGERS_SPREAD 4
#define POSE_NUM_DOUBLE_TAP     5
#define POSE_NUM_UNKNOWN        6



// holds a frame of data from myo collected in a call to hub.run(...)
struct Frame
{
  myo::Quaternion<float> quat;
  myo::Vector3<float> gyro, accel;
  std::array<int8_t,8> emg;
  bool onArm, isUnlocked, whichArm;
  float poseNum;
};

// implementation of object used to grab data from myo sdk api
// this class is registered with the hub and its member functions are
// called during hub.run(...) with relevant myo data
class DataCollector : public myo::DeviceListener
{
private:
  Frame frame;
public:
  
  myo::Quaternion<float> quat;
  myo::Vector3<float> gyro;
  myo::Vector3<float> accel;
  std::array<int8_t,8> emg;
  float poseNum;
  bool isUnlocked;
  bool onArm;
  
  DataCollector()
  : quat(), gyro(), accel(), emg(), poseNum(POSE_NUM_UNKNOWN), 
          isUnlocked(true), onArm(false)
  {
  }
  
  ~DataCollector()
  {
  }
  
  // get current data frame
  // this should be called after hub.run(...) to get current data
  const Frame &getFrame ()
  {
    frame.quat        = quat;
    frame.gyro        = gyro;
    frame.accel       = accel;
    frame.emg         = emg;
    frame.poseNum     = poseNum;
    frame.isUnlocked  = isUnlocked;
    frame.onArm       = onArm;
    return frame;
  }
  
  // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
  void onUnpair(myo::Myo* myo, uint64_t timestamp)
  {
    const myo::Quaternion<float> quatIdentity;
    quat = quatIdentity; // set quat to identity
    emg.fill(0);
    isUnlocked = false;
    onArm = false;
    poseNum = POSE_NUM_UNKNOWN;
  }
  
  // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
  // as a unit quaternion.
  void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& q)
  {
    quat = q;
  }
  
  // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
  // making a fist, or not making a fist anymore.
  void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
  {
    switch (pose.type()) {
      case myo::Pose::unknown :
        poseNum = POSE_NUM_UNKNOWN;
      case myo::Pose::rest :
        poseNum = POSE_NUM_REST;
        break;
      case myo::Pose::fist :
        poseNum = POSE_NUM_FIST;
        break;
      case myo::Pose::waveIn :
        poseNum = POSE_NUM_WAVE_IN;
        break;
      case myo::Pose::waveOut :
        poseNum = POSE_NUM_WAVE_OUT;
        break;
      case myo::Pose::fingersSpread :
        poseNum = POSE_NUM_FINGERS_SPREAD;
        break;
      case myo::Pose::doubleTap :
        poseNum = POSE_NUM_DOUBLE_TAP;
        break;
    }
  }
  
  // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
  // arm. This lets Myo know which arm it's on and which way it's facing.
  void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
  {
    onArm = true;
  }
  
  // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
  // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
  // when Myo is moved around on the arm.
  void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
  {
    onArm = false;
  }
  
  // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
  void onUnlock(myo::Myo* myo, uint64_t timestamp)
  {
    isUnlocked = true;
  }
  void onLock(myo::Myo* myo, uint64_t timestamp)
  {
    isUnlocked = false;
  }
  void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t  *e)
  {
    for (int i = 0; i < 8; i++)
      emg[i] = e[i];
  }
  void onAccelerometerData (myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& a)
  {
    accel=a;
  }
  void onGyroscopeData (myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& g)
  {
    gyro=g;
  }
  
}; // DataCollector