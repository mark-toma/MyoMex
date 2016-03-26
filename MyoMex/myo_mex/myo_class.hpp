
#ifndef MYO_CLASS_HPP
#define MYO_CLASS_HPP
#endif // ndef MYO_CLASS_HPP

#include "mex.h"
#include "myo/myo.hpp"  // myo sdk cpp binding
#include <array>        // myo sdk emg data
#include <vector>

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
public:
  std::vector<Frame> frame;
  
  std::vector<myo::Quaternion<float>> quat;
  std::vector<myo::Vector3<float>> gyro;
  std::vector<myo::Vector3<float>> accel;
  std::vector<std::array<int8_t,8>> emg;
  std::vector<float> poseNum;
  std::vector<bool> isUnlocked;
  std::vector<bool> onArm;
  std::vector<myo::Myo*> knownMyos;
  
  DataCollector()
  /*: quat(), gyro(), accel(), emg(), poseNum(POSE_NUM_UNKNOWN),
   * isUnlocked(true), onArm(false)*/
  {
    
  }
  
  ~DataCollector()
  {
  }
  
  // get current data frame
  // this should be called after hub.run(...) to get current data
  const Frame &getFrame ( int id )
  {
    
    //mexPrintf("getFrame()\n");
    
    // another workaround... return the wrong frame if id is bad and a 
    // proper frame exists
    if (id>getMyoCount()) { id = getMyoCount(); } // basically truncate id
    
    frame[id-1].quat        = quat[id-1];
    frame[id-1].gyro        = gyro[id-1];
    frame[id-1].accel       = accel[id-1];
    frame[id-1].emg         = emg[id-1];
    frame[id-1].poseNum     = poseNum[id-1];
    frame[id-1].isUnlocked  = isUnlocked[id-1];
    frame[id-1].onArm       = onArm[id-1];
    return frame[id-1];
  }
  
  // get current number of myos
  const size_t getMyoCount() {
    //mexPrintf("getMyoCount()\n");
    return knownMyos.size();
  }
  
  const size_t getMyoID(myo::Myo* myo) {
    //mexPrintf("getMyoID()\tcount = %d\n",getMyoCount());
    
    for (size_t i = 0; i < knownMyos.size(); ++i) {
      // If two Myo pointers compare equal, they refer to the same Myo device.
      if (knownMyos[i] == myo) { return i+1; }
    }
    
    // here there's no ID for this myo yet
    // let's make one
    addMyo(myo); // pushes this myo onto knownMyos
    return knownMyos.size();
    
  }
  
  void addMyo(myo::Myo* myo) {
    
    // add myo pointer to knownMyos
    // get the myo index later with getMyoID(myo)
    knownMyos.push_back(myo);
    
    // pushback in other data members too
    Frame f;
    myo::Quaternion<float> q;
    myo::Vector3<float> g;
    myo::Vector3<float> a;
    std::array<int8_t,8> e;
    float p = POSE_NUM_UNKNOWN;
    bool u = false;
    bool o = false;
    
    frame.push_back(f);
    quat.push_back(q);
    gyro.push_back(g);
    accel.push_back(a);
    emg.push_back(e);
    poseNum.push_back(p);
    isUnlocked.push_back(u);
    onArm.push_back(o);
    
  }
  
  void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
  {
    //mexPrintf("onPair()\n");

    addMyo(myo);
    
  }
  
  // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
  void onUnpair(myo::Myo* myo, uint64_t timestamp)
  {
    // TODO: we should probably pop this myo out of the datas...
    
    /*
     * // reset data state
     * const myo::Quaternion<float> quatIdentity;
     * quat = quatIdentity; // set quat to identity
     * emg.fill(0);
     * isUnlocked = false;
     * onArm = false;
     * poseNum = POSE_NUM_UNKNOWN;
     */
  }
  
  // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
  // as a unit quaternion.
  void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& q)
  {
    size_t id = getMyoID(myo);
    if (id == 0) { return; }
    
    quat[id-1] = q;
  }
  
  // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
  // making a fist, or not making a fist anymore.
  void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
  {
    size_t id = getMyoID(myo);
    if (id == 0) { return; }
        
    float p;
    switch (pose.type()) {
      case myo::Pose::unknown :
        p = POSE_NUM_UNKNOWN;
      case myo::Pose::rest :
        p = POSE_NUM_REST;
        break;
      case myo::Pose::fist :
        p = POSE_NUM_FIST;
        break;
      case myo::Pose::waveIn :
        p = POSE_NUM_WAVE_IN;
        break;
      case myo::Pose::waveOut :
        p = POSE_NUM_WAVE_OUT;
        break;
      case myo::Pose::fingersSpread :
        p = POSE_NUM_FINGERS_SPREAD;
        break;
      case myo::Pose::doubleTap :
        p = POSE_NUM_DOUBLE_TAP;
        break;
    }
    
    poseNum[id-1] = p;
  }
  
  // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
  // arm. This lets Myo know which arm it's on and which way it's facing.
  void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
  {
    size_t id = getMyoID(myo);
    if (id == 0) { return; }
    
    onArm[id-1] = true;
  }
  
  // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
  // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
  // when Myo is moved around on the arm.
  void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
  {
    size_t id = getMyoID(myo);
    if (id == 0) { return; }
    
    onArm[id-1] = false;
  }
  
  // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
  void onUnlock(myo::Myo* myo, uint64_t timestamp)
  {
    size_t id = getMyoID(myo);
    if (id == 0) { return; }
    
    isUnlocked[id-1] = true;
  }
  void onLock(myo::Myo* myo, uint64_t timestamp)
  {
    size_t id = getMyoID(myo);
    if (id == 0) { return; }
    
    isUnlocked[id-1] = false;
  }
  void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t  *e)
  {
    size_t id = getMyoID(myo);
    if (id == 0) { return; }
    
    for (int i = 0; i < 8; i++)
      emg[id-1][i] = e[i];
  }
  void onAccelerometerData (myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& a)
  {
    size_t id = getMyoID(myo);
    if (id == 0) { return; }
    
    accel[id-1]=a;
  }
  void onGyroscopeData (myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& g)
  {
    size_t id = getMyoID(myo);
    if (id == 0) { return; }
    
    gyro[id-1]=g;
  }
  
}; // DataCollector