
#ifndef MYO_CLASS_HPP
#define MYO_CLASS_HPP

// comment the following line to remove debug output via DB_MYO_CLASS()
//#define DEBUG_MYO_CLASS
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


// --- Data Frames
// These structures are used as return types in MyoData and DataCollector
// methods to return a single sample of data from one time instance

// IMU data frame
struct FrameIMU
{
  myo::Quaternion<float> quat;
  myo::Vector3<float>    gyro;
  myo::Vector3<float>    accel;
}; // FrameIMU

// EMG data frame
struct FrameEMG
{
  std::array<int8_t,8> emg;
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
  
  // IMU data queues and state information
  std::queue<myo::Quaternion<float>,std::deque<myo::Quaternion<float>>> quat;
  std::queue<myo::Vector3<float>,std::deque<myo::Vector3<float>>> gyro;
  std::queue<myo::Vector3<float>,std::deque<myo::Vector3<float>>> accel;
  uint64_t timestampIMU;
  unsigned int countIMU;
  
  // EMG data queues and state information
  std::queue<std::array<int8_t,8>,std::deque<std::array<int8_t,8>>> emg;
  unsigned int semEMG;
  unsigned int countEMG;
  uint64_t timestampEMG;
      
  void syncIMU(uint64_t ts)
  {
    if ( ts > timestampIMU ) {
      // fill IMU data (only if we missed samples)
      while ( quat.size() < countIMU ) {
        myo::Quaternion<float> q = quat.back();
        quat.push(q);
      }
      while ( gyro.size() < countIMU ) {
        myo::Vector3<float> g = gyro.back();
        gyro.push(g);
      }
      while ( accel.size() < countIMU ) {
        myo::Vector3<float> a = accel.back();
        accel.push(a);
      }
      countIMU++;
      timestampIMU = ts;
    }
  }
  
  void syncEMG(uint64_t ts)
  {
    if ( ts>timestampEMG ) { // new timestamp
      if ( 0==(semEMG%2) ) {
        std::array<int8_t,8> e = emg.back();
        emg.push(e);
      }
      semEMG = 0; // reset sem
    } else {
      semEMG++; // increment sem
    }
    countEMG++;
    timestampEMG = ts;
  }
  
public:
  
  // Construct a MyoData
  // The constructor must be passed a single myo::Myo* input argument.
  MyoData(myo::Myo* myo, uint64_t timestamp)
  : countIMU(1), countEMG(1), semEMG(0)
  {
    pMyo = myo; // pointer to myo::Myo
        
    // perform some operations on myo to set it up before subsequent use
    pMyo->setStreamEmg(myo::Myo::streamEmgEnabled);
    pMyo->unlock(myo::Myo::unlockHold);
    
    // fill up the other private members
    myo::Quaternion<float> q; // dummy default objects
    myo::Vector3<float> g;
    myo::Vector3<float> a;
    std::array<int8_t,8> e;
    quat.push(q);        // push them back onto queues
    gyro.push(g);
    accel.push(a);
    emg.push(e);
    
    timestampIMU = timestamp;
    timestampEMG = timestamp;
  }
  
  // Myo is owned by hub... no cleanup necessary here
  ~MyoData() {}
  
  
  FrameIMU &getFrameIMU()
  {
    countIMU = countIMU - 1;
    frameIMU.quat        = quat.front();
    frameIMU.gyro        = gyro.front();
    frameIMU.accel       = accel.front();
    quat.pop();
    gyro.pop();
    accel.pop();
    return frameIMU;
  }
    
  FrameEMG &getFrameEMG()
  {
    countEMG = countEMG - 1;
    frameEMG.emg     = emg.front();
    emg.pop();
    return frameEMG;
  }
  
  
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
    quat.push(_quat);
  }
  
  
  void addGyro(const myo::Vector3<float>& _gyro, uint64_t timestamp)
  {
    syncIMU(timestamp);
    gyro.push(_gyro);
  }
  
  
  void addAccel(const myo::Vector3<float>& _accel, uint64_t timestamp)
  {
    syncIMU(timestamp);
    accel.push(_accel);
  }
  
  
  void addEmg(const int8_t  *_emg, uint64_t timestamp)
  {
    syncEMG(timestamp);
    std::array<int8_t,8> tmp;
    int ii = 0;
    for (ii;ii<8;ii++) {tmp[ii]=_emg[ii];}
    emg.push(tmp);
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
    unsigned int tmp = getMyoID(myo,timestamp);
  }
  
  void onConnect(myo::Myo *myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
  {
    unsigned int tmp =  getMyoID(myo,timestamp);
  }
  
  void onDisconnect(myo::Myo* myo, uint64_t timestamp)
  {
    knownMyos.erase(knownMyos.begin()+getMyoID(myo,timestamp)-1);
  }
    
  void onLock(myo::Myo* myo, uint64_t timestamp)
  {
    // shamelessly unlock the device
    myo->unlock(myo::Myo::unlockHold);
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
  
  //void onUnpair(myo::Myo* myo, uint64_t timestamp) {}
  //void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose p) {}
  //void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection) {}
  //void onArmUnsync(myo::Myo* myo, uint64_t timestamp) {}
  //void onUnlock(myo::Myo* myo, uint64_t timestamp) {}
  
}; // DataCollector

// END DataCollector


#endif // ndef MYO_CLASS_HPP