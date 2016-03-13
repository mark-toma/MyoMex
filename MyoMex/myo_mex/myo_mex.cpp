
#include <mex.h>          // mex api
#include <windows.h>      // win api for threading support
#include <process.h>      // process/thread support
#include <queue>          // standard type for fifo queue
#include "myo/myo.hpp"
#include "myo_class.hpp"  // implementation of myo::DeviceListener
                          // enables access to myo sdk api and data

// indeces of output args (into plhs[*])
#define SIZE_OUT_NUM        0
#define QUAT_OUT_NUM        1
#define GYRO_OUT_NUM        2
#define ACCEL_OUT_NUM       3
#define EMG_OUT_NUM         4
#define POSE_OUT_NUM        5
#define ON_ARM_OUT_NUM      6
#define IS_UNLOCKED_OUT_NUM 7
#define DATA_OUT_NUM        8 // number of data outputs
#define STREAMING_TIME_OUT_NUM 0

// program behavior parameters
#define STREAMING_TIME_DEF   40L //   40 ms =  25 Hz
#define STREAMING_TIME_MAX 1000L // 1000 ms =   1 Hz
#define STREAMING_TIME_MIN   20L //   20 ms =  50 Hz
#define POLLING_DATA_TIME    20L // duration hub.run(.) [ms]

#define INIT_DELAY 1000

// program state
volatile bool isStreaming = false;
volatile unsigned streaming_time = STREAMING_TIME_DEF;

// global data
std::queue<Frame,std::deque<Frame>> frameLog;
DataCollector collector;
myo::Hub* pHub = NULL;
myo::Myo* pMyo = NULL;

// threading
unsigned int threadID;
HANDLE hThread;
HANDLE hMutex;

// thread routine
unsigned __stdcall runThreadFunc( void* pArguments ) {
  while ( isStreaming ) { // unset isStreaming to terminate thread
    pHub->run(streaming_time); // run callbacks to collector
    const Frame &f = collector.getFrame(); // get new data
    
    // acquire lock then write data into queue
    DWORD dwWaitResult;
    dwWaitResult = WaitForSingleObject(hMutex,INFINITE);
    switch (dwWaitResult)
    {
      case WAIT_OBJECT_0: // The thread got ownership of the mutex
        __try {
          frameLog.push(f); // write to queue
        }
        __finally {
          if (! ReleaseMutex(hMutex)) // release lock
          {
            // Handle error.
            // I don't know how to do this here.
            // Hopefully nothing worse than a reboot is required.
          }
        }
        break;
      case WAIT_ABANDONED:
        return FALSE; // acquired bad mutex
    }
  }
  // isStreaming unset (or couldn't release lock...)
  // end thread and return
  _endthreadex(0); // 
  return 0;
}

// allocates a bunch of mxArrays for time varying output data
// additionally, assign the size since it's only a scalar and it is
// passed into this function anyway
void init_output_data(mxArray *plhs[],unsigned int sz) {
  plhs[SIZE_OUT_NUM]        = mxCreateNumericMatrix(1 ,1,mxDOUBLE_CLASS,mxREAL);
  plhs[QUAT_OUT_NUM]        = mxCreateNumericMatrix(sz,4,mxDOUBLE_CLASS,mxREAL);
  plhs[GYRO_OUT_NUM]        = mxCreateNumericMatrix(sz,3,mxDOUBLE_CLASS,mxREAL);
  plhs[ACCEL_OUT_NUM]       = mxCreateNumericMatrix(sz,3,mxDOUBLE_CLASS,mxREAL);
  plhs[EMG_OUT_NUM]         = mxCreateNumericMatrix(sz,8,mxDOUBLE_CLASS,mxREAL);
  plhs[POSE_OUT_NUM]        = mxCreateNumericMatrix(sz,1,mxDOUBLE_CLASS,mxREAL);
  plhs[ON_ARM_OUT_NUM]      = mxCreateNumericMatrix(sz,1,mxDOUBLE_CLASS,mxREAL);
  plhs[IS_UNLOCKED_OUT_NUM] = mxCreateNumericMatrix(sz,1,mxDOUBLE_CLASS,mxREAL);
  
  *(mxGetPr(plhs[SIZE_OUT_NUM])) = sz;
}

// fill the ii-th row of the data outputs
// init_output_data must be called before this function
void fill_output_data_row(Frame f, mxArray *plhs[],
        unsigned int ii,unsigned int sz) {
  
  *( mxGetPr(plhs[QUAT_OUT_NUM])  + ii+sz*0 ) = f.quat.w();
  *( mxGetPr(plhs[QUAT_OUT_NUM])  + ii+sz*1 ) = f.quat.x();
  *( mxGetPr(plhs[QUAT_OUT_NUM])  + ii+sz*2 ) = f.quat.y();
  *( mxGetPr(plhs[QUAT_OUT_NUM])  + ii+sz*3 ) = f.quat.z();
  
  *( mxGetPr(plhs[GYRO_OUT_NUM])  + ii+sz*0 ) = f.gyro.x();
  *( mxGetPr(plhs[GYRO_OUT_NUM])  + ii+sz*1 ) = f.gyro.y();
  *( mxGetPr(plhs[GYRO_OUT_NUM])  + ii+sz*2 ) = f.gyro.z();
  
  *( mxGetPr(plhs[ACCEL_OUT_NUM]) + ii+sz*0 ) = f.accel.x();
  *( mxGetPr(plhs[ACCEL_OUT_NUM]) + ii+sz*1 ) = f.accel.y();
  *( mxGetPr(plhs[ACCEL_OUT_NUM]) + ii+sz*2 ) = f.accel.z();
  
  int jj = 0;
  for (jj;jj<8;jj++)
    *( mxGetPr(plhs[EMG_OUT_NUM]) + ii+sz*jj )  = f.emg[jj];
  
  *( mxGetPr(plhs[POSE_OUT_NUM])        + ii ) = f.poseNum;
  *( mxGetPr(plhs[ON_ARM_OUT_NUM])      + ii ) = f.onArm;
  *( mxGetPr(plhs[IS_UNLOCKED_OUT_NUM]) + ii ) = f.isUnlocked;
  
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

  // check for proper number of arguments
  if( nrhs<1 )
    mexErrMsgTxt("myo_mex requires at least one input.");
  if ( !mxIsChar(prhs[0]) )
    mexErrMsgTxt("myo_mex requires a char command as the first input.");
  if(nlhs>DATA_OUT_NUM)
    mexErrMsgTxt("myo_mex cannot provide the specified number of outputs.");
    
  char* cmd = mxArrayToString(prhs[0]);
  
  if ( !strcmp("init",cmd) ) {
    // ----------------------------------------- myo_mex init -------------
    if ( mexIsLocked() )
      mexErrMsgTxt("myo_mex is already initialized.\n");
    pHub = new myo::Hub("com.example.myo_mex");
    if ( !pHub )
      mexErrMsgTxt("Hub failed to init!");
    pMyo = pHub->waitForMyo(5);
    if ( !pMyo )
      mexErrMsgTxt("Myo failed to init!");
        
    // configure myo and hub
    pMyo->setStreamEmg(myo::Myo::streamEmgEnabled);
    pMyo->unlock(myo::Myo::unlockHold);
    pHub->setLockingPolicy(myo::Hub::lockingPolicyNone);
    pHub->addListener(&collector);
    
    // instantiate mutex
    hMutex = CreateMutex(NULL,FALSE,NULL);
    
    mexLock();
    
    Sleep(INIT_DELAY); // wait briefly for Myo to come up
  
  } else if ( !strcmp("get_polling_data",cmd) ) {
    // ----------------------------------------- myo_mex get_polling_data -
    
    if ( !mexIsLocked() )
      mexErrMsgTxt("myo_mex is not initialized.\n");
    if ( isStreaming )
      mexErrMsgTxt("myo_mex cannot get polling data while streaming.\n");
    
    pHub->run(POLLING_DATA_TIME);
    const Frame &f = collector.getFrame();
    
    init_output_data(plhs,1); // allocate output data and assign size
    fill_output_data_row(f,plhs,0,1); // assign data frame to outputs
    
  } else if ( !strcmp("set_streaming_time",cmd) ) {
    // ----------------------------------------- myo_mex set_streaming_time
    
    if ( !mexIsLocked() )
      mexErrMsgTxt("myo_mex is not initialized.\n");
    if ( isStreaming )
      mexErrMsgTxt("myo_mex cannot set streaming time while streaming.\n");
    
    // get second input parameter
    if (nrhs<2)
      mexErrMsgTxt("myo_mex requires a second input parameter to set streaming time.\n");
   
    double time = mxGetScalar( prhs[1] );
    
    if ( (time<STREAMING_TIME_MIN) || (time>STREAMING_TIME_MAX) )
      mexErrMsgTxt("myo_mex streaming time value out of range.\n");
    
    streaming_time = (unsigned int)time; // set time
  } else if ( !strcmp("get_streaming_time",cmd) ) {
    // ----------------------------------------- myo_mex get_streaming_time
    
    if ( !mexIsLocked() )
      mexErrMsgTxt("myo_mex is not initialized.\n");
    if ( isStreaming )
      mexErrMsgTxt("myo_mex cannot get streaming time while streaming.\n");
    
    double *time;
    
    plhs[STREAMING_TIME_OUT_NUM] = mxCreateNumericMatrix(1,1,mxDOUBLE_CLASS,mxREAL);
    time = mxGetPr(plhs[STREAMING_TIME_OUT_NUM]);
    
    time[0] = streaming_time;
    
  } else if ( !strcmp("start_streaming",cmd) ) {
    // ----------------------------------------- myo_mex start_streaming --
    
    if ( !mexIsLocked() )
      mexErrMsgTxt("myo_mex is not initialized.\n");
    if ( isStreaming )
      return; //bail
    
    // dispatch concurrent task
    isStreaming = true;
    hThread = (HANDLE)_beginthreadex( NULL, 0, &runThreadFunc, NULL, 0, &threadID );
    if ( !hThread )
      mexErrMsgTxt("Failed to create streaming thread!\n");
    // end dispatch thread
    
  } else if ( !strcmp("get_streaming_data",cmd) ) {
    // ----------------------------------------- myo_mex get_streaming_data
    if ( !mexIsLocked() )
      mexErrMsgTxt("myo_mex is not initialized.\n");
    if ( !isStreaming && frameLog.empty() )
      mexErrMsgTxt("myo_mex is not streaming and the queue is empty.\n");
        
    unsigned int sz = frameLog.size();
    unsigned int ii = 0;
    
    // allocate output data and assign size value
    init_output_data(plhs,sz);
    
    while ( !frameLog.empty() && (ii < sz) ) {
      
      Frame f;
      DWORD dwWaitResult;
      
      // acquire lock to write into queue
      dwWaitResult = WaitForSingleObject(
              hMutex,    // handle to mutex
              INFINITE);
      switch (dwWaitResult)
      {
        case WAIT_OBJECT_0:
            f = frameLog.front();
            frameLog.pop();
            // Release ownership of the mutex object
            if (! ReleaseMutex(hMutex))
            {
              mexErrMsgTxt("Failed to release lock\n");
            }
            break;
          // The thread got ownership of an abandoned mutex
          // The database is in an indeterminate state
        case WAIT_ABANDONED:        
          mexErrMsgTxt("Acquired abandoned lock\n");
      }
      
      // fill the ii-th row of the outputs with this data frame
      fill_output_data_row(f,plhs,ii,sz);
      
      ii++;
    }
        
  } else if ( !strcmp("stop_streaming",cmd) ) {
    // ----------------------------------------- myo_mex stop_streaming ---
    
    if ( !mexIsLocked() )
      mexErrMsgTxt("myo_mex is not initialized.\n");
    if ( !isStreaming ) // bail
      return;
    
    isStreaming = false;
    
    WaitForSingleObject( hThread, INFINITE );
    CloseHandle( hThread );
    hThread = NULL;

  } else if ( !strcmp("delete",cmd) ) {
    // ----------------------------------------- myo_mex delete -----------
    
    if ( !mexIsLocked() )
      mexErrMsgTxt("myo_mex is not initialized.\n");
    if ( isStreaming )
      mexErrMsgTxt("myo_mex cannot be deleted while streaming. Call stop_streaming first.\n");
    
    CloseHandle (hMutex);
    hMutex = NULL;
        
    mexUnlock();
    
    if (pHub!=NULL)
      delete(pHub);
    pHub=NULL;
    
  } else {
    mexErrMsgTxt("unknown command!\n");
  }
     
  return;
}



