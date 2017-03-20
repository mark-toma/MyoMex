// myo_sfun.cpp
// 
#define S_FUNCTION_NAME  myo_sfun
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

#include "myo_sfun.h"        // macros and defines for general behavior
#include "myo_sfun_wiring.h" // macros and defines for sfcn block and data

#include "myo/myo.hpp"   // Myo SDK bindings
#include "myo_class.hpp" // application class for Myo implementation

#include <windows.h>      // win api for threading support
#include <process.h>      // process/thread support


// ==================================================
// Global variables
unsigned int threadID;
HANDLE ghThread;
HANDLE ghMutex;
volatile bool gRunThreadFlag = false;
real_T gCountMyosRequired = 1;
real_T gEmgEnabledRequired = 1;


// ==================================================
// Thread function
//   This thread calls myo::Hub::runOnce() for the
//   lifetime of the application
unsigned __stdcall runThreadFunc( void* S_ ) {
  SimStruct *S = (SimStruct *)S_;
  myo::Hub* pHub = (myo::Hub *) ssGetPWork(S)[IDX_HUB];
  while ( gRunThreadFlag ) { // unset isStreaming to terminate thread
    // acquire lock then write data into queue
    DWORD dwWaitResult;
    dwWaitResult = WaitForSingleObject(ghMutex,INFINITE);
    switch (dwWaitResult)
    {
      case WAIT_OBJECT_0: // The thread got ownership of the mutex
        // --- CRITICAL SECTION - holding lock
        pHub->runOnce(STREAMING_TIMEOUT); // run callbacks to collector
        // END CRITICAL SECTION - release lock
        if (! ReleaseMutex(ghMutex)) { return FALSE; } // acquired bad mutex
        break;
      case WAIT_ABANDONED:
        return FALSE; // acquired bad mutex
    }
  } // end thread and return
  _endthreadex(0); //
  return 0;
}


// ==================================================
// Utility functions
//   User defined functions for convenience
static void setOutputDimensionInfo(SimStruct *S, int_T port, int_T len, int_T sz)
{
  DECL_AND_INIT_DIMSINFO(di);
  int_T dims[2];
  di.numDims = 2;
  dims[0] = sz;
  dims[1] = len;
  di.dims = dims;
  di.width = sz*len;
  ssSetOutputPortDimensionInfo(S, port, &di);
}


// ==================================================
// Model callback functions
#define MDL_CHECK_PARAMETERS   /* Change to #undef to remove function */
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
        static void mdlCheckParameters(SimStruct *S)
{
  DB_MYO_SFUN("ENTER mdlCheckParameters()\n",);
  // check data types
  const mxArray *pEmgEnabled = ssGetSFcnParam(S,IDX_EMG_ENABLED_REQUIRED);
  const mxArray *pCountMyos = ssGetSFcnParam(S,IDX_COUNT_MYOS_REQUIRED);
  if ( !IS_PARAM_SCALAR_DOUBLE(pEmgEnabled)) {
    ssSetErrorStatus(S,"EMG Enabled parameter must be a scalar int8");
    return;
  }
  if ( !IS_PARAM_SCALAR_DOUBLE(pCountMyos)) {
    ssSetErrorStatus(S,"Count Myos parameter must be a scalar int8");
    return;
  }
  DB_MYO_SFUN("Parameter datatypes OK\n");
  // check values
  real_T emgEnabled = *mxGetPr(pEmgEnabled);
  real_T countMyos = *mxGetPr(pCountMyos);
  if ( !((emgEnabled==0.0)||(emgEnabled==1.0)) ) {
    ssSetErrorStatus(S,"EMG Enabled parameter must be 0 or 1");
    return;
  }
  if ( !((countMyos==1.0)||(countMyos==2.0)) ) {
    ssSetErrorStatus(S,"Count Myos parameter must be 1 or 2");
    return;
  }
  DB_MYO_SFUN("Parameter values OK\n");
  DB_MYO_SFUN("EXIT  mdlCheckParameters()\n",);
}
#endif /* MDL_CHECK_PARAMETERS */


static void mdlInitializeSizes(SimStruct *S)
{
  DB_MYO_SFUN("ENTER mdlInitializeSizes()\n",);
  int_T numOutputPorts;
  ssSetNumContStates(    S, 0);
  ssSetNumDiscStates(    S, 0);
  ssSetNumSFcnParams(S,NUM_SFCN_PARAMS);
  ssSetSFcnParamTunable(S,IDX_EMG_ENABLED_REQUIRED,SS_PRM_NOT_TUNABLE);
  ssSetSFcnParamTunable(S,IDX_COUNT_MYOS_REQUIRED,SS_PRM_NOT_TUNABLE);
  DB_MYO_SFUN("Checking parameters ...\n");
#if defined(MATLAB_MEX_FILE)
  if(ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S) ) {
    mdlCheckParameters(S);
    if(ssGetErrorStatus(S) != NULL) return;
  } else {
    return; /* The Simulink engine reports a mismatch error. */
  }
#endif
  
  // get parameters from block
  gEmgEnabledRequired = *mxGetPr(ssGetSFcnParam(S,IDX_EMG_ENABLED_REQUIRED));
  gCountMyosRequired = *mxGetPr(ssGetSFcnParam(S,IDX_COUNT_MYOS_REQUIRED));
  DB_MYO_SFUN("Parameter values:\n");
  DB_MYO_SFUN("\tgEmgEnabledRequired = %f\n",gEmgEnabledRequired);
  DB_MYO_SFUN("\tgCountMyosRequired  = %f\n",gCountMyosRequired);
  // Determine number of output ports based on parameters
  //   This is the process parameters routine to calculate numOutputPorts
  // gCountMyosRequired
  //   |  gEmgEnabledRequired
  //   |    |  numOutputPorts
  //   1    0    6              Default - IMU for Myo 1
  //   1    1    7              Adds EMG for Myo 1
  //   2    0    12             Adds IMU for Myo 2
  //   2    1    ERROR
  if ((gEmgEnabledRequired==0.0)&&(gCountMyosRequired==1.0)) {
    numOutputPorts = NUM_OUTPUT_PORTS_IMU;
  } else if ((gEmgEnabledRequired==1.0)&&(gCountMyosRequired==1.0)) {
    numOutputPorts = NUM_OUTPUT_PORTS_IMU+NUM_OUTPUT_PORTS_EMG;
  } else if ((gEmgEnabledRequired==0.0)&&(gCountMyosRequired==2.0)) {
    numOutputPorts = 2*NUM_OUTPUT_PORTS_IMU;
  } else if ((gEmgEnabledRequired==1.0)&&(gCountMyosRequired==2.0)) {
    ssSetErrorStatus(S,"EMG Cannot be enabled with more than one Myo.");
    return;
  }
  DB_MYO_SFUN("Computed number of output ports:\n");
  DB_MYO_SFUN("\tnumOutputPorts = %d\n",numOutputPorts);
  if (!ssSetNumOutputPorts(S, numOutputPorts)) return;
  // MYO 1 IMU - These ports are always hooked up
  DB_MYO_SFUN("Configuring ports for Myo 1 IMU ...\n");
  setOutputDimensionInfo(S,OUTPUT_PORT_IDX_QUAT, LEN_QUAT ,SZ_IMU);
  setOutputDimensionInfo(S,OUTPUT_PORT_IDX_GYRO,LEN_GYRO  ,SZ_IMU);
  setOutputDimensionInfo(S,OUTPUT_PORT_IDX_ACCEL,LEN_ACCEL,SZ_IMU);
  setOutputDimensionInfo(S,OUTPUT_PORT_IDX_POSE,LEN_POSE  ,SZ_IMU);
  setOutputDimensionInfo(S,OUTPUT_PORT_IDX_ARM,LEN_ARM    ,SZ_IMU);
  setOutputDimensionInfo(S,OUTPUT_PORT_IDX_XDIR,LEN_XDIR  ,SZ_IMU);
  if (gEmgEnabledRequired==1.0) {
    DB_MYO_SFUN("Configuring ports for Myo 1 EMG ...\n");
    // Add EMG port for Myo 1
    setOutputDimensionInfo(S,OUTPUT_PORT_IDX_EMG,LEN_EMG,SZ_EMG);
  } else if (gCountMyosRequired==2.0) {
    DB_MYO_SFUN("Configuring ports for Myo 2 IMU ...\n");
    // Add IMU ports for Myo 2
    setOutputDimensionInfo(S,NUM_OUTPUT_PORTS_IMU+OUTPUT_PORT_IDX_QUAT,LEN_QUAT  ,SZ_IMU);
    setOutputDimensionInfo(S,NUM_OUTPUT_PORTS_IMU+OUTPUT_PORT_IDX_GYRO,LEN_GYRO  ,SZ_IMU);
    setOutputDimensionInfo(S,NUM_OUTPUT_PORTS_IMU+OUTPUT_PORT_IDX_ACCEL,LEN_ACCEL,SZ_IMU);
    setOutputDimensionInfo(S,NUM_OUTPUT_PORTS_IMU+OUTPUT_PORT_IDX_POSE,LEN_POSE  ,SZ_IMU);
    setOutputDimensionInfo(S,NUM_OUTPUT_PORTS_IMU+OUTPUT_PORT_IDX_ARM,LEN_ARM    ,SZ_IMU);
    setOutputDimensionInfo(S,NUM_OUTPUT_PORTS_IMU+OUTPUT_PORT_IDX_XDIR,LEN_XDIR  ,SZ_IMU);
  }
  ssSetNumSampleTimes(    S, 1);
  ssSetNumRWork(          S, 0);   /* number of real work vector elements   */
  ssSetNumIWork(          S, NUM_IWORK);   /* number of integer work vector elements*/
  ssSetNumPWork(          S, NUM_PWORK);   /* number of pointer work vector elements*/
  ssSetNumModes(          S, 0);   /* number of mode work vector elements   */
  ssSetNumNonsampledZCs(  S, 0);   /* number of nonsampled zero crossings   */
  ssSetSimStateCompliance(S, USE_CUSTOM_SIM_STATE);
  ssSetOptions(          S, 0);   /* general options (SS_OPTION_xx)        */
  DB_MYO_SFUN("EXIT  mdlInitializeSizes\n");
} /* end mdlInitializeSizes */


static void mdlInitializeSampleTimes(SimStruct *S)
{
  DB_MYO_SFUN("ENTER mdlInitializeSampleTimes\n");
  ssSetSampleTime(S, 0, 0.001*SAMPLE_TIME_BLK);
  ssSetOffsetTime(S, 0, 0.0);
  DB_MYO_SFUN("EXIT  mdlInitializeSampleTimes()\n");
} /* end mdlInitializeSampleTimes */


#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START)
static void mdlStart(SimStruct *S)
{
  DB_MYO_SFUN("ENTER mdlStart()\n");
  // declare pointers
  DataCollector* pCollector = NULL;
  myo::Hub* pHub = NULL;
  myo::Myo* pMyo = NULL;
  
  DB_MYO_SFUN("Setting up DataCollector ...\n");
  ssGetPWork(S)[IDX_COLLECTOR] = (void *) new DataCollector;
  pCollector = (DataCollector *)ssGetPWork(S)[IDX_COLLECTOR];
  if (gEmgEnabledRequired==1.0)
    pCollector->addEmgEnabled = true; // lets collector handle data events
  DB_MYO_SFUN("Setting up Hub ...\n");
  // Instantiate a Hub and get a Myo
  ssGetPWork(S)[IDX_HUB] = (void *) new myo::Hub("com.mark-toma.myo_sfun");
  pHub = (myo::Hub *)ssGetPWork(S)[IDX_HUB];
  if ( !pHub ) {
    ssSetErrorStatus(S,"Hub failed to init!");
    return;
  }
  pHub->setLockingPolicy(myo::Hub::lockingPolicyNone); // TODO: What does this do?
  pHub->addListener(pCollector);
  DB_MYO_SFUN("Setting up Myo ...\n");
  pMyo = pHub->waitForMyo(5);
  if ( !pMyo ) {
    ssSetErrorStatus(S,"Myo failed to init!");
    return;
  }
  DB_MYO_SFUN("Setting up mutex lock ...\n");
  // instantiate mutex
  ghMutex = CreateMutex(NULL,FALSE,NULL);
  if (ghMutex == NULL) {
    ssSetErrorStatus(S,"Failed to set up mutex.\n");
    return;
  }
  DB_MYO_SFUN("Running Hub for INIT_DELAY to validate countMyos ...\n");
  // Let Hub run callbacks on collector so we can figure out how many
  // Myos are connected to Myo Connect so we can assert gCountMyosRequired
  pHub->run(INIT_DELAY);
  if (gCountMyosRequired != pCollector->getCountMyos()) {
    ssSetErrorStatus(S,"myo_sfun failed to initialize with countMyos.\n");
    return;
  }
  DB_MYO_SFUN("Running Hub for BUFFER_DELAY to preload buffers ...\n");
  // Flush the data queues with syncDataSources
  pCollector->syncDataSources();
  // Enabled data and initialize buffer
  pCollector->addDataEnabled = true;
  pHub->run(BUFFER_DELAY);
  DB_MYO_SFUN("Dispatching thread to run Hub ...\n");
  // dispatch concurrent task
  gRunThreadFlag = true;
  ghThread = (HANDLE)_beginthreadex( NULL, 0, &runThreadFunc, S, 0, &threadID );
  if ( !ghThread ) {
    ssSetErrorStatus(S,"Failed to create streaming thread!\n");
    return;
  }
  DB_MYO_SFUN("EXIT  mdlStart()\n");
}
#endif /*  MDL_START */


static void mdlOutputs(SimStruct *S, int_T tid)
{
  DB_MYO_SFUN("ENTER mdlOutputs()\n");
  
  // get pHub and pCollector
  myo::Hub* pHub = (myo::Hub *) ssGetPWork(S)[IDX_HUB];
  DataCollector* pCollector = (DataCollector *) ssGetPWork(S)[IDX_COLLECTOR];
  int_T iter = ssGetIWork(S)[IDX_ITER]; // get iteration counter
  
  int_T ii, jj;
  real_T *pQuat1, *pGyro1, *pAccel1, *pPose1, *pArm1, *pXDir1;
  real_T *pQuat2, *pGyro2, *pAccel2, *pPose2, *pArm2, *pXDir2;
  real_T *pEMG1;
  FrameIMU frameIMU1[SAMPLES_PER_FRAME_IMU];
  FrameIMU frameIMU2[SAMPLES_PER_FRAME_IMU];
  FrameEMG frameEMG1[SAMPLES_PER_FRAME_EMG];
  int_T countIMU1 = 0;
  int_T countIMU2 = 0;
  int_T countEMG1 = 0;
  countIMU1 = pCollector->getCountIMU(1);
  if (gCountMyosRequired==2)
    countIMU2 = pCollector->getCountIMU(1);
  else if ((gCountMyosRequired==1)&&(gEmgEnabledRequired==1))
    countEMG1 = pCollector->getCountEMG(1);
  
  // fail if the queue is falling behind
  if (countIMU1 < 1+SAMPLES_PER_FRAME_IMU*BUFFER_FRAMES_MIN) {
    ssSetErrorStatus(S,"IMU1 buffer is less than minimum size.");
    return;
  }
  if ( (gEmgEnabledRequired==1) &&
          (countEMG1 < 1+SAMPLES_PER_FRAME_EMG*BUFFER_FRAMES_MIN) ) {
    ssPrintf("EMG Buffer is %d samples.\n",countEMG1);
    ssSetErrorStatus(S,"EMG buffer is less than minimum size");
    return;
  }
  if ( (gCountMyosRequired==2) &&
          (countIMU2 < 1+SAMPLES_PER_FRAME_IMU*BUFFER_FRAMES_MIN) ) {
    ssSetErrorStatus(S,"IMU2 buffer is less than minimum size.");
    return;
  }
  // Verify that collector still has all of its Myos, otherwise error out
  if ( pCollector->getCountMyos() != gCountMyosRequired ) {
    ssSetErrorStatus(S,"myo_sfun countMyos is inconsistent with initialization... We lost a Myo!");
    return;
  }
  // move above into another method
  DB_MYO_SFUN_ITER("IMU1=%d\tEMG1=%d\n",countIMU1,countEMG1);
  DWORD dwWaitResult = WaitForSingleObject(ghMutex,INFINITE);
  switch (dwWaitResult)
  {
    case WAIT_OBJECT_0: // The thread got ownership of the mutex
      // --- CRITICAL SECTION - holding lock
      // in initial run set the buffer lengths
      if (iter==0) {
        DB_MYO_SFUN("Initializing data buffers on iteration zero ...\n");
        while(pCollector->getCountIMU(1)>1+SAMPLES_PER_FRAME_IMU*BUFFER_FRAMES_DES )
          *frameIMU1 = pCollector->getFrameIMU(1);
        if(gEmgEnabledRequired) {
          while(pCollector->getCountEMG(1)>1+SAMPLES_PER_FRAME_EMG*BUFFER_FRAMES_DES )
            *frameEMG1 = pCollector->getFrameEMG(1);
        }
        if (gCountMyosRequired==2) {
          while(pCollector->getCountIMU(2)>1+SAMPLES_PER_FRAME_IMU*BUFFER_FRAMES_DES)
            *frameIMU2 = pCollector->getFrameIMU(2);
        }
      }
      for (ii=0;ii<SAMPLES_PER_FRAME_IMU;ii++) {
        frameIMU1[ii] = pCollector->getFrameIMU(1);
      }
      if ( gEmgEnabledRequired ) {
        for (ii=0;ii<SAMPLES_PER_FRAME_EMG;ii++) {
          frameEMG1[ii] = pCollector->getFrameEMG(1);
        }
      }
      if (gCountMyosRequired==2) {
        for (ii=0;ii<SAMPLES_PER_FRAME_IMU;ii++) {
          frameIMU2[ii] = pCollector->getFrameIMU(2);
        }
      }
      // END CRITICAL SECTION - release lock
      if ( !ReleaseMutex(ghMutex)) {
        ssSetErrorStatus(S,"Failed to release lock\n");
        return;
      }
      break;
    case WAIT_ABANDONED:
      ssSetErrorStatus(S,"Acquired abandoned lock\n");
      return;
      break;
  }
  
  pQuat1 = ssGetOutputPortRealSignal(S,OUTPUT_PORT_IDX_QUAT);
  pQuat1[0] = frameIMU1[0].quat.w();
  pQuat1[1] = frameIMU1[1].quat.w();
  pQuat1[2] = frameIMU1[0].quat.x();
  pQuat1[3] = frameIMU1[1].quat.x();
  pQuat1[4] = frameIMU1[0].quat.y();
  pQuat1[5] = frameIMU1[1].quat.y();
  pQuat1[6] = frameIMU1[0].quat.z();
  pQuat1[7] = frameIMU1[1].quat.z();
  pGyro1 = ssGetOutputPortRealSignal(S,OUTPUT_PORT_IDX_GYRO);
  pGyro1[0] = frameIMU1[0].gyro.x();
  pGyro1[1] = frameIMU1[1].gyro.x();
  pGyro1[2] = frameIMU1[0].gyro.y();
  pGyro1[3] = frameIMU1[1].gyro.y();
  pGyro1[4] = frameIMU1[0].gyro.z();
  pGyro1[5] = frameIMU1[1].gyro.z();
  pAccel1 = ssGetOutputPortRealSignal(S,OUTPUT_PORT_IDX_ACCEL);
  pAccel1[0] = frameIMU1[0].accel.x();
  pAccel1[1] = frameIMU1[1].accel.x();
  pAccel1[2] = frameIMU1[0].accel.y();
  pAccel1[3] = frameIMU1[1].accel.y();
  pAccel1[4] = frameIMU1[0].accel.z();
  pAccel1[5] = frameIMU1[1].accel.z();
  pPose1 = ssGetOutputPortRealSignal(S,OUTPUT_PORT_IDX_POSE);
  pPose1[0] = frameIMU1[0].pose.type();
  pPose1[1] = frameIMU1[1].pose.type();
  pArm1 = ssGetOutputPortRealSignal(S,OUTPUT_PORT_IDX_ARM);
  pArm1[0] = frameIMU1[0].arm;
  pArm1[1] = frameIMU1[1].arm;
  pXDir1 = ssGetOutputPortRealSignal(S,OUTPUT_PORT_IDX_XDIR);
  pXDir1[0] = frameIMU1[0].xDir;
  pXDir1[1] = frameIMU1[1].xDir;
  
  if ( gEmgEnabledRequired ) {
    pEMG1 = ssGetOutputPortRealSignal(S,OUTPUT_PORT_IDX_EMG);
    for (ii=0;ii<8;ii++) {
      for (jj=0;jj<8;jj++) {
        //DB_MYO_SFUN_ITER("ii=%d\tjj=%d\tkk=%d\n",ii,jj,ii+8*jj);
        pEMG1[ii+8*jj] = frameEMG1[ii].emg[jj];
      }
    }
  }
  if (gCountMyosRequired==2) {
    pQuat2 = ssGetOutputPortRealSignal(S,NUM_OUTPUT_PORTS_IMU+OUTPUT_PORT_IDX_QUAT);
    pQuat2[0] = frameIMU2[0].quat.w();
    pQuat2[1] = frameIMU2[1].quat.w();
    pQuat2[2] = frameIMU2[0].quat.x();
    pQuat2[3] = frameIMU2[1].quat.x();
    pQuat2[4] = frameIMU2[0].quat.y();
    pQuat2[5] = frameIMU2[1].quat.y();
    pQuat2[6] = frameIMU2[0].quat.z();
    pQuat2[7] = frameIMU2[1].quat.z();
    pGyro2 = ssGetOutputPortRealSignal(S,NUM_OUTPUT_PORTS_IMU+OUTPUT_PORT_IDX_GYRO);
    pGyro2[0] = frameIMU2[0].gyro.x();
    pGyro2[1] = frameIMU2[1].gyro.x();
    pGyro2[2] = frameIMU2[0].gyro.y();
    pGyro2[3] = frameIMU2[1].gyro.y();
    pGyro2[4] = frameIMU2[0].gyro.z();
    pGyro2[5] = frameIMU2[1].gyro.z();
    pAccel2 = ssGetOutputPortRealSignal(S,NUM_OUTPUT_PORTS_IMU+OUTPUT_PORT_IDX_ACCEL);
    pAccel2[0] = frameIMU2[0].accel.x();
    pAccel2[1] = frameIMU2[1].accel.x();
    pAccel2[2] = frameIMU2[0].accel.y();
    pAccel2[3] = frameIMU2[1].accel.y();
    pAccel2[4] = frameIMU2[0].accel.z();
    pAccel2[5] = frameIMU2[1].accel.z();
    pPose2 = ssGetOutputPortRealSignal(S,NUM_OUTPUT_PORTS_IMU+OUTPUT_PORT_IDX_POSE);
    pPose2[0] = frameIMU2[0].pose.type();
    pPose2[1] = frameIMU2[1].pose.type();
    pArm2 = ssGetOutputPortRealSignal(S,NUM_OUTPUT_PORTS_IMU+OUTPUT_PORT_IDX_ARM);
    pArm2[0] = frameIMU2[0].arm;
    pArm2[1] = frameIMU2[1].arm;
    pXDir2 = ssGetOutputPortRealSignal(S,NUM_OUTPUT_PORTS_IMU+OUTPUT_PORT_IDX_XDIR);
    pXDir2[0] = frameIMU2[0].xDir;
    pXDir2[1] = frameIMU2[1].xDir;
  }
  // increment and store new iteration value
  ssGetIWork(S)[IDX_ITER] = ++iter;
  DB_MYO_SFUN("EXIT  mdlOutputs()\n");
} /* end mdlOutputs */


static void mdlTerminate(SimStruct *S)
{
  DB_MYO_SFUN("EXIT  mlTerminate()\n");
  DB_MYO_SFUN("Fetch Hub and DataCollector pointers ...\n");
  myo::Hub* pHub = (myo::Hub *) ssGetPWork(S)[IDX_HUB];
  DataCollector* pCollector = (DataCollector *) ssGetPWork(S)[IDX_COLLECTOR];
  DB_MYO_SFUN("Unsetting runThreadFlag, waiting for thread, deleting thread ...\n");
  gRunThreadFlag = false; // thread sees this flag and exits
  WaitForSingleObject( ghThread, INFINITE );
  CloseHandle( ghThread );
  ghThread = NULL;
  CloseHandle (ghMutex);
  ghMutex = NULL;
  DB_MYO_SFUN("Deleting Hub and DataCollector ...\n");
  if (pHub!=NULL)
    delete pHub;
  if (pCollector!=NULL)
    delete pCollector;
  DB_MYO_SFUN("EXIT  mlTerminate()\n");
}


#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
