
#ifndef MYO_CLASS_HPP
#define MYO_CLASS_HPP

// comment the following line to remove debug output via DB_MYO_CLASS()
#define DEBUG_MYO_CLASS
#ifdef DEBUG_MYO_CLASS
#define DB_MYO_CLASS(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define DB_MYO_CLASS(fmt, ...)
#endif

#include "mex.h"
#include "myo/myo.hpp"  // myo sdk cpp binding

#include <array>        // myo sdk emg data
#include <vector>
#include <queue>

#define POSE_NUM_REST           0
#define POSE_NUM_FIST           1
#define POSE_NUM_WAVE_IN        2
#define POSE_NUM_WAVE_OUT       3
#define POSE_NUM_FINGERS_SPREAD 4
#define POSE_NUM_DOUBLE_TAP     5
#define POSE_NUM_UNKNOWN        6


// --- Data Frames
// These structures are used as return types in MyoData and DataCollector
// methods to return a single sample of data from one time instance

// IMU data frame
struct FrameIMU
{
  float timeIMU;
  myo::Quaternion<float> quat;
  myo::Vector3<float> gyro, accel;
}; // FrameIMU

// EMG data frame
struct FrameEMG
{
  float timeEMG;
  std::array<int8_t,8> emg;
  unsigned int pose;
}; // FrameEMG

// Meta data frame
struct FrameMeta
{
  bool onArm;
  bool isUnlocked;
  bool whichArm;
}; // FrameMeta

// END Data Frames


// --- MyoData
// This class is used to keep track of the data for one physical Myo.
// Typical use may be to instantiate a MyoData for each unique myo received
// by a DataCollector. Then, subsequent calls into add<data> and get<data>
// can be used to receive contiguous collections of IMU, EMG, and Meta data

class MyoData
{
  
  // Data frames for returning data samples
  FrameIMU frameIMU;
  FrameEMG frameEMG;
  FrameMeta frameMeta;
  
  // Pointer to a Myo device instance provided by hub
  myo::Myo* pMyo;
  uint64_t timestampMyoInit;
  
  // IMU data queues and state information
  std::deque<float> timeIMU;
  std::deque<myo::Quaternion<float>> quat;
  std::deque<myo::Vector3<float>> gyro;
  std::deque<myo::Vector3<float>> accel;
  unsigned int countIMU;
  
  // EMG data queues and state information
  std::deque<float> timeEMG;
  std::deque<std::array<int8_t,8>> emg;
  std::deque<unsigned int> pose;
  unsigned int countEMG;
  unsigned int semEMG;
  
  // TODO
  //   Implement Meta data
  // Meta data queues and state information
  // ...
  // ...
  
  float timeSinceInit(uint64_t timestamp)
  {
    float retval = (0.000001)*(timestamp-timestampMyoInit);
    return retval;
  }
  
  void syncIMU(uint64_t ts)
  {
    float tsInit = timeSinceInit(ts);
    if ( tsInit > timeIMU.back()) {
      size_t sz = timeIMU.size();
      // pad onto IMU data to make them all maximum length
      while ( quat.size() < sz ) {
        myo::Quaternion<float> q = quat.back();
        quat.push_back(q);
      }
      while ( gyro.size() < sz ) {
        myo::Vector3<float> g = gyro.back();
        gyro.push_back(g);
      }
      while ( accel.size() < sz ) {
        myo::Vector3<float> a = accel.back();
        accel.push_back(a);
      }
      countIMU++;
      timeIMU.push_back(tsInit);
    }
  }
  
  // Helper for syncEMG(...,isPoseSync)
  void syncPose(uint64_t ts)
  {
    // pose is just tagging along with EMG and arbitrarily being filled
    // at the same rate
    unsigned int p = pose.back();
    while(pose.size()< (timeEMG.size()-1) ) { pose.push_back(p); }
  }
  
  
  void syncEMG(uint64_t ts)
  {
    float tsInit = timeSinceInit(ts);
    size_t sz = timeEMG.size();
    if ( tsInit>timeEMG.back() ) { // new timestamp
      if ( 0==(semEMG%2) ) {
        std::array<int8_t,8> e = emg.back();
        emg.push_back(e);
        float t = timeEMG.back();
        timeEMG.push_back(t);
      }
      semEMG = 0; // reset sem
    } else {
      semEMG++; // increment sem
    }
    countEMG++;
    timeEMG.push_back(tsInit);
    
    // copy pose to bring it up to same size as new timeEMG and emg
    unsigned int p = pose.back();
    while (pose.size() < timeEMG.size() ) { pose.push_back(p); }
  }
  
public:
  
  // Construct a MyoData
  // The constructor must be passed a single myo::Myo* input argument.
  MyoData(myo::Myo* myo, uint64_t timestamp)
  : countIMU(1), countEMG(1), semEMG(0)
  {
    pMyo = myo; // pointer to myo::Myo
    timestampMyoInit = timestamp;
    
    // perform some operations on myo to set it up before subsequent use
    pMyo->setStreamEmg(myo::Myo::streamEmgEnabled);
    pMyo->unlock(myo::Myo::unlockHold);
    
    // fill up the other private members
    myo::Quaternion<float> q; // dummy default objects
    myo::Vector3<float> g;
    myo::Vector3<float> a;
    std::array<int8_t,8> e;
    quat.push_back(q);        // push them back onto queues
    gyro.push_back(g);
    accel.push_back(a);
    emg.push_back(e);
    pose.push_back(POSE_NUM_UNKNOWN);
    
    timeIMU.push_back(timeSinceInit(timestamp));
    timeEMG.push_back(timeSinceInit(timestamp));
  }
  
  // Myo is owned by hub... no cleanup necessary here
  ~MyoData() {}
  
  
  FrameIMU &getFrameIMU()
  {
    countIMU = countIMU - 1;
    frameIMU.timeIMU     = timeIMU.front();
    frameIMU.quat        = quat.front();
    frameIMU.gyro        = gyro.front();
    frameIMU.accel       = accel.front();
    timeIMU.pop_front();
    quat.pop_front();
    gyro.pop_front();
    accel.pop_front();
    return frameIMU;
  }
  
  
  FrameEMG &getFrameEMG()
  {
    countEMG = countEMG - 1;
    frameEMG.timeEMG = timeEMG.front();
    frameEMG.emg     = emg.front();
    frameEMG.pose    = pose.front();
    timeEMG.pop_front();
    emg.pop_front();
    pose.pop_front();
    return frameEMG;
  }
  
  // TODO
  //   Implement getFrameMeta()
  //  const FrameMeta &getFrameMeta()
  //  {
  // // assign frameMeta
  // return frameMeta;
  // }
  
  
  myo::Myo* getInstance() { return pMyo; }
  
  
  unsigned int getCountIMU() { return countIMU; }
  
  
  unsigned int getCountEMG() { return countEMG; }
  
  // pop_front on all queues until size==1
  void syncDataSources()
  {
    FrameIMU frameIMU;
    while ( getCountIMU() > 1 )
      frameIMU = getFrameIMU();
    FrameEMG frameEMG;
    while ( getCountEMG() > 1 )
      frameEMG = getFrameEMG();
  }
  
  
  void addQuat(const myo::Quaternion<float>& _quat, uint64_t timestamp)
  {
    syncIMU(timestamp);
    quat.push_back(_quat);
  }
  
  
  void addGyro(const myo::Vector3<float>& _gyro, uint64_t timestamp)
  {
    syncIMU(timestamp);
    gyro.push_back(_gyro);
  }
  
  
  void addAccel(const myo::Vector3<float>& _accel, uint64_t timestamp)
  {
    syncIMU(timestamp);
    accel.push_back(_accel);
  }
  
  
  void addEmg(const int8_t  *_emg, uint64_t timestamp)
  {
    syncEMG(timestamp);
    std::array<int8_t,8> tmp;
    int ii = 0;
    for (ii;ii<8;ii++) {tmp[ii]=_emg[ii];}
    emg.push_back(tmp);
  }
  
  
  void addPose(myo::Pose _pose, uint64_t timestamp)
  {
    syncPose(timestamp);
    pose.push_back(lookupPose(_pose));
  }
  
  // Utility to lookup pose number from myo::Pose
  static unsigned int lookupPose(myo::Pose _pose)
  {
    switch (_pose.type())
    {
      case myo::Pose::unknown :
        return POSE_NUM_UNKNOWN;
      case myo::Pose::rest :
        return POSE_NUM_REST;
      case myo::Pose::fist :
        return POSE_NUM_FIST;
      case myo::Pose::waveIn :
        return POSE_NUM_WAVE_IN;
      case myo::Pose::waveOut :
        return POSE_NUM_WAVE_OUT;
      case myo::Pose::fingersSpread :
        return POSE_NUM_FINGERS_SPREAD;
      case myo::Pose::doubleTap :
        return POSE_NUM_DOUBLE_TAP;
      default :
        return 42;
    }
  }
  
}; // MyoData

// END MyoData


// --- DataCollector
// implementation of object used to grab data from myo sdk api
// this class is registered with the hub and its member functions are
// called during hub.run(...) with relevant myo data
class DataCollector : public myo::DeviceListener
{
  
  std::vector<MyoData*> knownMyos;
  
public:
  
  bool addDataEnabled;
  
  DataCollector()
  : addDataEnabled(false)
  {
  }
  
  
  ~DataCollector()
  {
    // destruct all MyoData* in knownMyos
    int ii=0;
    for (ii;ii<knownMyos.size();ii++)
    {
      delete knownMyos[ii];
    }
  }
  
  
  unsigned int getCountIMU(int id) { return knownMyos[id-1]->getCountIMU(); }
  
  
  unsigned int getCountEMG(int id) { return knownMyos[id-1]->getCountEMG(); }
  
  
  const FrameIMU &getFrameIMU( int id ) { return knownMyos[id-1]->getFrameIMU(); }
  
  
  const FrameEMG &getFrameEMG( int id ) { return knownMyos[id-1]->getFrameEMG(); }
  
  // TODO
  //   Implement getFrame() in MyoData
  // const FrameMeta &getFrameMeta( int id ) { return knownMyos[id-1]->getFrameMeta(); }
  
  void syncDataSources()
  {
    int ii = 0;
    for (ii;ii<knownMyos.size();ii++)
      knownMyos[ii]->syncDataSources();
  }
  
  
  // get current number of myos
  const unsigned int getCountMyos() { return knownMyos.size(); }
  
  
  const unsigned int getMyoID(myo::Myo* myo,uint64_t timestamp)
  {
    // search myos in knownMyos for myo
    for (size_t ii = 0; ii < knownMyos.size(); ii++)
      if (knownMyos[ii]->getInstance() == myo) { return ii+1; }
    
    // add myo to a new MyoData* in knowmMyos if it doesn't exist yet
    knownMyos.push_back(new MyoData(myo,timestamp));
    return knownMyos.size();
  }
  
  
  void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
  {
    if (!getMyoID(myo,timestamp)) mexErrMsgTxt("Failed to add Myo\n");
  }
  
  // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
  void onUnpair(myo::Myo* myo, uint64_t timestamp)
  {
    // TODO: we should probably pop this myo out of the datas...
  }
  
  
  void onConnect(myo::Myo *myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
  {
    myo->setStreamEmg(myo::Myo::streamEmgEnabled);
    if (!getMyoID(myo,timestamp)) mexErrMsgTxt("Failed to add Myo\n");
  }
  
  
  void onDisconnect(myo::Myo* myo, uint64_t timestamp)
  {
    // TODO
    //   Remove from knownMyos
  }
  
  
  void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& q)
  {
    if (!addDataEnabled) { return; }
    knownMyos[getMyoID(myo,timestamp)-1]->addQuat(q,timestamp);
  }
  
  
  void onGyroscopeData (myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& g)
  {
    if (!addDataEnabled) { return; }
    knownMyos[getMyoID(myo,timestamp)-1]->addGyro(g,timestamp);
  }
  
  
  void onAccelerometerData (myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& a)
  {
    if (!addDataEnabled) { return; }
    knownMyos[getMyoID(myo,timestamp)-1]->addAccel(a,timestamp);
  }
  
  
  void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t  *e)
  {
    if (!addDataEnabled) { return; }
    knownMyos[getMyoID(myo,timestamp)-1]->addEmg(e,timestamp);
  }
  
  
  void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose p)
  {
    if (!addDataEnabled) { return; }
    knownMyos[getMyoID(myo,timestamp)-1]->addPose(p,timestamp);
  }
  
  
  void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
  {
  }
  
  
  void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
  {
  }
  
  
  void onUnlock(myo::Myo* myo, uint64_t timestamp)
  {
  }
  
  
  void onLock(myo::Myo* myo, uint64_t timestamp)
  {
    // shamelessly unlock the device
    myo->unlock(myo::Myo::unlockHold);
  }
  
}; // DataCollector

// END DataCollector


#endif // ndef MYO_CLASS_HPP