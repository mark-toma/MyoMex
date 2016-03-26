
#include <mex.h>          // mex api
#include <windows.h>      // win api for threading support
#include <process.h>      // process/thread support
#include <queue>          // standard type for fifo queue
#include "myo/myo.hpp"
#include "myo_class.hpp"  // implementation of myo::DeviceListener
// enables access to myo sdk api and data


// indeces of output args (into plhs[*])
#define DATA_STRUCT_OUT_NUM    0
#define STREAMING_TIME_OUT_NUM 0

// indeces of data fields into data output struct
#define SIZE_FIELD_NUM        0
#define QUAT_FIELD_NUM        1
#define GYRO_FIELD_NUM        2
#define ACCEL_FIELD_NUM       3
#define EMG_FIELD_NUM         4
#define POSE_FIELD_NUM        5
#define ON_ARM_FIELD_NUM      6
#define IS_UNLOCKED_FIELD_NUM 7
#define NUM_FIELDS            8 // number of data outputs

// field names for data output struct
const char* output_fields[] = {"size","quat","gyro","accel","emg","pose",
"on_arm","is_unlocked"};

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
std::queue<Frame,std::deque<Frame>> frameLog1;
std::queue<Frame,std::deque<Frame>> frameLog2;
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
    const Frame &f1 = collector.getFrame(1);
    unsigned num_myos = collector.getMyoCount();
    // f2 might be invalid... if
    const Frame &f2 = collector.getFrame(2);
    
    // acquire lock then write data into queue
    DWORD dwWaitResult;
    dwWaitResult = WaitForSingleObject(hMutex,INFINITE);
    switch (dwWaitResult)
    {
      case WAIT_OBJECT_0: // The thread got ownership of the mutex
        __try {
          frameLog1.push(f1); // write to queue
          if (num_myos>1) {
            frameLog2.push(f2);
          }
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
void make_output_data(mxArray *outData[], unsigned int sz) {
  
  outData[SIZE_FIELD_NUM]        = mxCreateNumericMatrix(1 ,1,mxDOUBLE_CLASS,mxREAL);
  outData[QUAT_FIELD_NUM]        = mxCreateNumericMatrix(sz,4,mxDOUBLE_CLASS,mxREAL);
  outData[GYRO_FIELD_NUM]        = mxCreateNumericMatrix(sz,3,mxDOUBLE_CLASS,mxREAL);
  outData[ACCEL_FIELD_NUM]       = mxCreateNumericMatrix(sz,3,mxDOUBLE_CLASS,mxREAL);
  outData[EMG_FIELD_NUM]         = mxCreateNumericMatrix(sz,8,mxDOUBLE_CLASS,mxREAL);
  outData[POSE_FIELD_NUM]        = mxCreateNumericMatrix(sz,1,mxDOUBLE_CLASS,mxREAL);
  outData[ON_ARM_FIELD_NUM]      = mxCreateNumericMatrix(sz,1,mxDOUBLE_CLASS,mxREAL);
  outData[IS_UNLOCKED_FIELD_NUM] = mxCreateNumericMatrix(sz,1,mxDOUBLE_CLASS,mxREAL);
  
  *(mxGetPr(outData[SIZE_FIELD_NUM])) = sz;
  
}

// fill the ii-th row of the data outputs
// init_output_data must be called before this function
void fill_output_data_row(Frame f, mxArray *outData[],
        unsigned int ii,unsigned int sz) {
  
  *( mxGetPr(outData[QUAT_FIELD_NUM])  + ii+sz*0 ) = f.quat.w();
  *( mxGetPr(outData[QUAT_FIELD_NUM])  + ii+sz*1 ) = f.quat.x();
  *( mxGetPr(outData[QUAT_FIELD_NUM])  + ii+sz*2 ) = f.quat.y();
  *( mxGetPr(outData[QUAT_FIELD_NUM])  + ii+sz*3 ) = f.quat.z();
  
  *( mxGetPr(outData[GYRO_FIELD_NUM])  + ii+sz*0 ) = f.gyro.x();
  *( mxGetPr(outData[GYRO_FIELD_NUM])  + ii+sz*1 ) = f.gyro.y();
  *( mxGetPr(outData[GYRO_FIELD_NUM])  + ii+sz*2 ) = f.gyro.z();
  
  *( mxGetPr(outData[ACCEL_FIELD_NUM]) + ii+sz*0 ) = f.accel.x();
  *( mxGetPr(outData[ACCEL_FIELD_NUM]) + ii+sz*1 ) = f.accel.y();
  *( mxGetPr(outData[ACCEL_FIELD_NUM]) + ii+sz*2 ) = f.accel.z();
  
  int jj = 0;
  for (jj;jj<8;jj++)
    *( mxGetPr(outData[EMG_FIELD_NUM]) + ii+sz*jj )  = f.emg[jj];
  
  *( mxGetPr(outData[POSE_FIELD_NUM])        + ii ) = f.poseNum;
  *( mxGetPr(outData[ON_ARM_FIELD_NUM])      + ii ) = f.onArm;
  *( mxGetPr(outData[IS_UNLOCKED_FIELD_NUM]) + ii ) = f.isUnlocked;
  
}

// Assigns Myo data in matrices d to element id of struct s
void assn_output_data_struct(mxArray *dst, mxArray *d[], int id) {
  
  int ii = 0;
  for (ii;ii<NUM_FIELDS;ii++) {
    mxSetFieldByNumber(dst,id,ii,d[ii]);
  }
}


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  
  // check for proper number of arguments
  if( nrhs<1 )
    mexErrMsgTxt("myo_mex requires at least one input.");
  if ( !mxIsChar(prhs[0]) )
    mexErrMsgTxt("myo_mex requires a char command as the first input.");
  if(nlhs>NUM_FIELDS)
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
    
    //Sleep(INIT_DELAY); // wait briefly for Myo to come up
    
    pHub->run(INIT_DELAY);
    
    mexPrintf("Number of Myos = %d",collector.getMyoCount());
    
  } else if ( !strcmp("get_polling_data",cmd) ) {
    // ----------------------------------------- myo_mex get_polling_data -
    
    if ( !mexIsLocked() )
      mexErrMsgTxt("myo_mex is not initialized.\n");
    if ( isStreaming )
      mexErrMsgTxt("myo_mex cannot get polling data while streaming.\n");
    
    pHub->run(POLLING_DATA_TIME);
    unsigned num_myos = collector.getMyoCount();
    const Frame &f1 = collector.getFrame(1);
    const Frame &f2 = collector.getFrame(2);
    mxArray *outData1[NUM_FIELDS];
    mxArray *outData2[NUM_FIELDS];
    plhs[DATA_STRUCT_OUT_NUM] = mxCreateStructMatrix(1,num_myos,NUM_FIELDS,output_fields);
    
    make_output_data(outData1,1); // allocate output data and assign size
    fill_output_data_row(f1,outData1,0,1); // assign data frame to outputs
    assn_output_data_struct(plhs[DATA_STRUCT_OUT_NUM], outData1, 0);
    
    if (num_myos>1) {
      make_output_data(outData1,2); // allocate output data and assign size
      fill_output_data_row(f2,outData2,0,1); // assign data frame to outputs
      assn_output_data_struct(plhs[DATA_STRUCT_OUT_NUM], outData2, 0);
    }
    
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
    if ( !isStreaming && frameLog1.empty() && frameLog2.empty() )
      mexErrMsgTxt("myo_mex is not streaming and the queue is empty.\n");
    
    unsigned num_myos = collector.getMyoCount();
    unsigned int sz1 = frameLog1.size();
    unsigned int sz2 = frameLog2.size();
    // allocate output data and assign size value
    mxArray *outData1[NUM_FIELDS];
    mxArray *outData2[NUM_FIELDS];
    // create a (1xnum_myos) output struct as first output argument
    plhs[DATA_STRUCT_OUT_NUM] = mxCreateStructMatrix(1,num_myos,NUM_FIELDS,output_fields);
    
    
    // drain frameLog1
    make_output_data(outData1,sz1);
    unsigned int ii = 0;
    while ( !frameLog1.empty() && (ii < sz1) ) {
      
      Frame f;
      DWORD dwWaitResult;
      
      // acquire lock to write into queue
      dwWaitResult = WaitForSingleObject(
              hMutex,    // handle to mutex
              INFINITE);
      switch (dwWaitResult)
      {
        case WAIT_OBJECT_0:
          f = frameLog1.front();
          frameLog1.pop();
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
      fill_output_data_row(f,outData1,ii,sz1);
      
      ii++;
    }
    assn_output_data_struct(plhs[DATA_STRUCT_OUT_NUM], outData1, 0);
    
    // conditionally drain frameLog2
    if (num_myos>1) {
      make_output_data(outData2,sz2);
      ii = 0;
      while ( !frameLog2.empty() && (ii < sz2) ) {
        
        Frame f;
        DWORD dwWaitResult;
        
        // acquire lock to write into queue
        dwWaitResult = WaitForSingleObject(
                hMutex,    // handle to mutex
                INFINITE);
        switch (dwWaitResult)
        {
          case WAIT_OBJECT_0:
            f = frameLog2.front();
            frameLog2.pop();
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
        fill_output_data_row(f,outData2,ii,sz2);
        
        ii++;
      }
      assn_output_data_struct(plhs[DATA_STRUCT_OUT_NUM], outData2, 1);
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



