
// comment the following line to remove debug output via mexPrintf()
//#define DEBUG_MYO_MEX
#ifdef DEBUG_MYO_MEX
#define DB_MYO_MEX(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define DB_MYO_MEX(fmt, ...)
#endif

#include <mex.h>          // mex api
#include <windows.h>      // win api for threading support
#include <process.h>      // process/thread support
#include <queue>          // standard type for fifo queue
#include "myo/myo.hpp"
#include "myo_class.hpp"

// indeces of output args (into plhs[*])
#define DATA_STRUCT_OUT_NUM    0
#define NUM_MYOS_OUT_NUM       0

// indeces of data fields into data output struct
#define QUAT_FIELD_NUM        0
#define GYRO_FIELD_NUM        1
#define ACCEL_FIELD_NUM       2
#define EMG_FIELD_NUM         3
#define POSE_FIELD_NUM        4
#define ARM_FIELD_NUM         5
#define XDIR_FIELD_NUM        6
#define NUM_FIELDS            7
const char* output_fields[] = {"quat","gyro","accel","emg","pose","arm","xDir"};

// program behavior parameters
#define STREAMING_TIME   5L
#define INIT_DELAY 1000
#define RESTART_DELAY 500

// program state
volatile bool runThreadFlag = false;

// global data
DataCollector collector;
myo::Hub* pHub = NULL;
myo::Myo* pMyo = NULL;

unsigned int countMyosRequired = 1;

// threading
unsigned int threadID;
HANDLE hThread;
HANDLE hMutex;

// thread routine
unsigned __stdcall runThreadFunc( void* pArguments ) {
  while ( runThreadFlag ) { // unset isStreaming to terminate thread
    // acquire lock then write data into queue
    DWORD dwWaitResult;
    dwWaitResult = WaitForSingleObject(hMutex,INFINITE);
    switch (dwWaitResult)
    {
      case WAIT_OBJECT_0: // The thread got ownership of the mutex
        // holding lock
        pHub->runOnce(STREAMING_TIME); // run callbacks to collector
        // release lock
        if (! ReleaseMutex(hMutex)) // release lock
        {
          return FALSE; // acquired bad mutex
        }
        break;
      case WAIT_ABANDONED:
        return FALSE; // acquired bad mutex
    }
  } // end thread and return
  _endthreadex(0); //
  return 0;
}

void makeOutputIMU(mxArray *outData[], unsigned int sz) {
  outData[QUAT_FIELD_NUM]        = mxCreateNumericMatrix(sz,4,mxDOUBLE_CLASS,mxREAL);
  outData[GYRO_FIELD_NUM]        = mxCreateNumericMatrix(sz,3,mxDOUBLE_CLASS,mxREAL);
  outData[ACCEL_FIELD_NUM]       = mxCreateNumericMatrix(sz,3,mxDOUBLE_CLASS,mxREAL);
}
void makeOutputEMG(mxArray *outData[], unsigned int sz) {
  outData[EMG_FIELD_NUM]         = mxCreateNumericMatrix(sz,8,mxDOUBLE_CLASS,mxREAL);
  outData[POSE_FIELD_NUM]        = mxCreateNumericMatrix(sz,1,mxDOUBLE_CLASS,mxREAL);
  outData[ARM_FIELD_NUM]         = mxCreateNumericMatrix(sz,1,mxDOUBLE_CLASS,mxREAL);
  outData[XDIR_FIELD_NUM]        = mxCreateNumericMatrix(sz,1,mxDOUBLE_CLASS,mxREAL);
}
void fillOutputIMU(FrameIMU f, mxArray *outData[],
        unsigned int row,unsigned int sz) {
  *( mxGetPr(outData[QUAT_FIELD_NUM])  + row+sz*0 ) = f.quat.w();
  *( mxGetPr(outData[QUAT_FIELD_NUM])  + row+sz*1 ) = f.quat.x();
  *( mxGetPr(outData[QUAT_FIELD_NUM])  + row+sz*2 ) = f.quat.y();
  *( mxGetPr(outData[QUAT_FIELD_NUM])  + row+sz*3 ) = f.quat.z();
  *( mxGetPr(outData[GYRO_FIELD_NUM])  + row+sz*0 ) = f.gyro.x();
  *( mxGetPr(outData[GYRO_FIELD_NUM])  + row+sz*1 ) = f.gyro.y();
  *( mxGetPr(outData[GYRO_FIELD_NUM])  + row+sz*2 ) = f.gyro.z();
  *( mxGetPr(outData[ACCEL_FIELD_NUM]) + row+sz*0 ) = f.accel.x();
  *( mxGetPr(outData[ACCEL_FIELD_NUM]) + row+sz*1 ) = f.accel.y();
  *( mxGetPr(outData[ACCEL_FIELD_NUM]) + row+sz*2 ) = f.accel.z();
}
void fillOutputEMG(FrameEMG f, mxArray *outData[],
        unsigned int row,unsigned int sz) {
  int jj = 0;
  for (jj;jj<8;jj++)
    *( mxGetPr(outData[EMG_FIELD_NUM]) + row+sz*jj ) = f.emg[jj];
  *( mxGetPr(outData[POSE_FIELD_NUM])  + row )       = f.pose.type();
  *( mxGetPr(outData[ARM_FIELD_NUM])   + row )       = f.arm;
  *( mxGetPr(outData[XDIR_FIELD_NUM])  + row )       = f.xDir;
}
// Assigns Myo data in matrices d to element id of struct s
void assnOutputStruct(mxArray *s, mxArray *d[], int id) {
  int ii = 0;
  for (ii;ii<NUM_FIELDS;ii++) {
    DB_MYO_MEX("Setting field %d of struct element %d\n",ii+1,id);
    mxSetFieldByNumber(s,id-1,ii,d[ii]);
  }
}


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  
  // check for proper number of arguments
  if( nrhs<1 )
    mexErrMsgTxt("myo_mex requires at least one input.");
  if ( !mxIsChar(prhs[0]) )
    mexErrMsgTxt("myo_mex requires a char command as the first input.");
  if(nlhs>1)
    mexErrMsgTxt("myo_mex cannot provide the specified number of outputs.");
  
  char* cmd = mxArrayToString(prhs[0]);
  
  if ( !strcmp("init",cmd) ) {
    // ----------------------------------------- myo_mex init -------------
    if ( mexIsLocked() )
      mexErrMsgTxt("myo_mex is already initialized.\n");
    
    
    pHub = new myo::Hub("com.mark-toma.myo_mex");
    if ( !pHub )
      mexErrMsgTxt("Hub failed to init!");
    pMyo = pHub->waitForMyo(5);
    if ( !pMyo )
      mexErrMsgTxt("Myo failed to init!");
    
    // configure myo and hub
    DB_MYO_MEX("myo_mex init:\n\tConfiguring myo and hub\n");
    pHub->setLockingPolicy(myo::Hub::lockingPolicyNone);
    pHub->addListener(&collector);
    
    // instantiate mutex
    DB_MYO_MEX("myo_mex init:\n\tSetting up mutex\n");
    hMutex = CreateMutex(NULL,FALSE,NULL);
    if (hMutex == NULL)
      mexErrMsgTxt("Failed to set up mutex.\n");
    mexLock();
    
    // wait briefly for Myo to come up
    DB_MYO_MEX("myo_mex init:\n\tRunning hub\n");
    pHub->run(INIT_DELAY);
    
    unsigned int countMyos = collector.getCountMyos();
    DB_MYO_MEX("myo_mex init:\n\tNumber of Myos = %d\n",
            countMyos);
    
    collector.syncDataSources();
    DB_MYO_MEX("myo_mex init:\n\tData sources synchronized!\n");
    
    // assign output num_myos
    plhs[NUM_MYOS_OUT_NUM] = mxCreateNumericMatrix(1,1,mxDOUBLE_CLASS,mxREAL);
    *mxGetPr(plhs[NUM_MYOS_OUT_NUM]) = countMyos;
    
    // keep track of this value
    countMyosRequired = countMyos;
    
  } else if ( !strcmp("start_streaming",cmd) ) {
    // ----------------------------------------- myo_mex start_streaming --
    
    if ( !mexIsLocked() )
      mexErrMsgTxt("myo_mex is not initialized.\n");
    if ( runThreadFlag )
      mexErrMsgTxt("myo_mex is already streaming.\n");
    if ( nlhs>0 )
      mexErrMsgTxt("myo_mex too many outputs specified.\n");
    
    collector.addDataEnabled = true; // lets collector handle data events
    // dispatch concurrent task
    runThreadFlag = true;
    hThread = (HANDLE)_beginthreadex( NULL, 0, &runThreadFunc, NULL, 0, &threadID );
    if ( !hThread )
      mexErrMsgTxt("Failed to create streaming thread!\n");
    DB_MYO_MEX("myo_mex start_streaming:\n\tSuccess\n");
    
  } else if ( !strcmp("get_streaming_data",cmd) ) {
    // ----------------------------------------- myo_mex get_streaming_data
    if ( !mexIsLocked() )
      mexErrMsgTxt("myo_mex is not initialized.\n");
    if ( !runThreadFlag )
      mexErrMsgTxt("myo_mex is not streaming.\n");
    if ( nlhs>1 )
      mexErrMsgTxt("myo_mex too many outputs specified.\n");
    
    unsigned int countMyos = collector.getCountMyos();
    DB_MYO_MEX("myo_mex get_streaming_data:\n\tReading from %d Myos\n",
            countMyos);
    
    if ( countMyos != countMyosRequired )
      mexErrMsgTxt("myo_mex countMyos is inconsistent with initialization... We lost a Myo!");
    
    unsigned int iiIMU1=0;
    unsigned int iiEMG1=0;
    unsigned int iiIMU2=0;
    unsigned int iiEMG2=0;
    unsigned int szIMU1 = 0;
    unsigned int szEMG1 = 0;
    unsigned int szIMU2 = 0;
    unsigned int szEMG2 = 0;
    FrameIMU frameIMU1, frameIMU2;
    FrameEMG frameEMG1, frameEMG2;
    mxArray *outData1[NUM_FIELDS];
    mxArray *outData2[NUM_FIELDS];
    
    // initialize output matrices for myo 1
    szIMU1 = collector.getCountIMU(1)-1;
    szEMG1 = collector.getCountEMG(1)-1;
    makeOutputIMU(outData1,szIMU1);
    makeOutputEMG(outData1,szEMG1);
    DB_MYO_MEX("myo_mex get_streaming_data:\n\tszIMU1=%u\tszEMG1=%u\n",
            szIMU1,szEMG1);
    
    // initialize output matrices for myo 2
    if (countMyos>1) {
      szIMU2 = collector.getCountIMU(2)-1;
      szEMG2 = collector.getCountEMG(2)-1;
      makeOutputIMU(outData2,szIMU2);
      makeOutputEMG(outData2,szEMG2);
      DB_MYO_MEX("myo_mex get_streaming_data:\n\tszIMU2=%u\tszEMG2=%u\n",
              szIMU2,szEMG2);
    }
    
    DWORD dwWaitResult;
    dwWaitResult = WaitForSingleObject(hMutex,INFINITE);
    DB_MYO_MEX("myo_mex get_streaming_data:\n\tAcquired lock\n");
    switch (dwWaitResult)
    {
      case WAIT_OBJECT_0: // The thread got ownership of the mutex
        DB_MYO_MEX("myo_mex get_streaming_data:\n\tHolding lock\n");
        // holding lock
        DB_MYO_MEX("myo_mex get_streaming_data:\n\tEntering iterative read loop\n");
        // read sz<src><id> elements out of each of these sources
        while ( (iiIMU1<szIMU1) || (iiEMG1<szEMG1) || (iiIMU2<szIMU2) || (iiEMG2<szEMG2) )
        {
          if (iiIMU1<szIMU1) {
            //DB_MYO_MEX("myo_mex get_streaming_data:\n\tgetFrameIMU1\n");
            frameIMU1 = collector.getFrameIMU(1);
            //DB_MYO_MEX("myo_mex get_streaming_data:\n\tfillOutputIMU1\n");
            fillOutputIMU(frameIMU1,outData1,iiIMU1,szIMU1);
            iiIMU1++;
          }
          if (iiEMG1<szEMG1) {
            //DB_MYO_MEX("myo_mex get_streaming_data:\n\tgetFrameEMG1\n");
            frameEMG1 = collector.getFrameEMG(1);
            //DB_MYO_MEX("myo_mex get_streaming_data:\n\tfillOutputEMG1\n");
            fillOutputEMG(frameEMG1,outData1,iiEMG1,szEMG1);
            iiEMG1++;
          }
          if (iiIMU2<szIMU2) {
            //DB_MYO_MEX("myo_mex get_streaming_data:\n\tgetFrameIMU2\n");
            frameIMU2 = collector.getFrameIMU(2);
            //DB_MYO_MEX("myo_mex get_streaming_data:\n\tfillOutputIMU2\n");
            fillOutputIMU(frameIMU2,outData2,iiIMU2,szIMU2);
            iiIMU2++;
          }
          if (iiEMG2<szEMG2) {
            //DB_MYO_MEX("myo_mex get_streaming_data:\n\tgetFrameEMG2\n");
            frameEMG2 = collector.getFrameEMG(2);
            //DB_MYO_MEX("myo_mex get_streaming_data:\n\tfillOutputEMG2\n");
            fillOutputEMG(frameEMG2,outData2,iiEMG2,szEMG2);
            iiEMG2++;
          }          
        }
        DB_MYO_MEX("myo_mex get_streaming_data:\n\tReleasing lock\n");
        // release lock if it was acquired
        if ( !ReleaseMutex(hMutex)) // release lock
          mexErrMsgTxt("Failed to release lock\n");
        break;
      case WAIT_ABANDONED:
        DB_MYO_MEX("myo_mex get_streaming_data:\n\tBad lock\n");
        mexErrMsgTxt("Acquired abandoned lock\n");
        break;
    }
    DB_MYO_MEX("myo_mex get_streaming_data:\n\tAssigning output struct\n");
    // assign output matrices to struct array
    plhs[DATA_STRUCT_OUT_NUM] = mxCreateStructMatrix(1,countMyos,NUM_FIELDS,output_fields);
    DB_MYO_MEX("myo_mex get_streaming_data:\nAssigning outData1\n");
    assnOutputStruct(plhs[DATA_STRUCT_OUT_NUM], outData1, 1);
    if (countMyos>1) {
      DB_MYO_MEX("myo_mex get_streaming_data:\nAssigning outData2\n");
      assnOutputStruct(plhs[DATA_STRUCT_OUT_NUM], outData2, 2);
    }
    
  } else if ( !strcmp("stop_streaming",cmd) ) {
    // ----------------------------------------- myo_mex stop_streaming ---
    
    if ( !mexIsLocked() )
      mexErrMsgTxt("myo_mex is not initialized.\n");
    if ( !runThreadFlag )
      mexErrMsgTxt("myo_mex is not streaming.\n");
    if ( nlhs>0 )
      mexErrMsgTxt("myo_mex too many outputs specified.\n");
    
    runThreadFlag = false; // thread sees this flag and exits
    WaitForSingleObject( hThread, INFINITE );
    CloseHandle( hThread );
    hThread = NULL;
    collector.addDataEnabled = false; // stop handling data events
    collector.syncDataSources(); // sync data up again (flushes queue)
    Sleep(RESTART_DELAY);
    
  } else if ( !strcmp("delete",cmd) ) {
    // ----------------------------------------- myo_mex delete -----------
    
    if ( !mexIsLocked() )
      mexErrMsgTxt("myo_mex is not initialized.\n");
    if ( runThreadFlag )
      mexErrMsgTxt("myo_mex cannot be deleted while streaming. Call stop_streaming first.\n");
    if ( nlhs>0 )
      mexErrMsgTxt("myo_mex too many outputs specified.\n");
    
    CloseHandle (hMutex);
    hMutex = NULL;
    mexUnlock();
    if (pHub!=NULL)
      delete pHub;
    
  } else {
    mexErrMsgTxt("unknown command!\n");
  }
  
  return;
}



