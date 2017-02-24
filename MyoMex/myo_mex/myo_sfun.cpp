

//  mxGetScalar(ssGetSFcnParam(S, 0))



//    INPUTS         myo_sfun         OUTPUTS
//            +--------------------+
//            |             quat ->+--(1x4 double)-->
//            |             gyro ->+--(1x3 double)-->
//            |            accel ->+--(1x3 double)-->
//            |             pose ->+--(1x1 double)-->
//            |              arm ->+--(1x1 double)-->
//            |             xDir ->+--(1x1 double)-->
//            |              emg ->+--(1x8 double)-->
//            +--------------------+
//


// https://www.mathworks.com/help/simulink/create-cc-s-functions.html
#define S_FUNCTION_NAME  myo_sfun
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
//  *       ssSetErrorStatus(S,"error encountered due to ...");
//  *       return;
//  *      ssSetOptions(S, SS_OPTION_RUNTIME_EXCEPTION_FREE_CODE);
//  *   You can use ssWarning(S,msg) to display a warning.
//  *   You can use ssPrintf(fmt, ...) to print a message.

#include <windows.h>      // win api for threading support
#include <process.h>      // process/thread support
#include "myo/myo.hpp"
#include "myo_class.hpp"

#define STREAMING_TIMEOUT    5
#define INIT_DELAY 1000
#define BUFFER_DELAY 200
const int_T countMyosRequired = 1;
#define SAMPLE_TIME_IMU 0.02
#define SAMPLE_TIME_EMG 0.005

#define BUFFER_SAMPLES_IMU BUFFER_DELAY/SAMPLE_TIME_IMU/1000
#define BUFFER_SAMPLES_EMG BUFFER_DELAY/SAMPLE_TIME_EMG/1000
        
#define IDX_HUB 0
#define IDX_COLLECTOR 1
#define NUM_PWORK 2

#define IDX_ITER 0
#define NUM_IWORK 1

// threading
unsigned int threadID;
HANDLE hThread;
HANDLE hMutex;

// program state
volatile bool runThreadFlag = false;

// thread routine
unsigned __stdcall runThreadFunc( void* _S ) {
  SimStruct *S = (SimStruct *)_S;
  myo::Hub* pHub = (myo::Hub *) ssGetPWork(S)[IDX_HUB];
  DataCollector* pCollector = (DataCollector *) ssGetPWork(S)[IDX_COLLECTOR];
  while ( runThreadFlag ) { // unset isStreaming to terminate thread
    // acquire lock then write data into queue
    DWORD dwWaitResult;
    dwWaitResult = WaitForSingleObject(hMutex,INFINITE);
    switch (dwWaitResult)
    {
      case WAIT_OBJECT_0: // The thread got ownership of the mutex
        // --- CRITICAL SECTION - holding lock
        pHub->runOnce(STREAMING_TIMEOUT); // run callbacks to collector
        // END CRITICAL SECTION - release lock
        if (! ReleaseMutex(hMutex)) { return FALSE; } // acquired bad mutex
        break;
      case WAIT_ABANDONED:
        return FALSE; // acquired bad mutex
    }
  } // end thread and return
  _endthreadex(0); //
  return 0;
}

const int_T numInputPorts  = 0;
const int_T numOutputPorts = 7;
const int_T numSFcnParams  = 1; // number of Myo devices
const int_T lenQuat  = 4;
const int_T lenGyro  = 3;
const int_T lenAccel = 3;
const int_T lenPose  = 1;
const int_T lenArm   = 1;
const int_T lenXDir  = 1;
const int_T lenEMG   = 8;

const int_T outputPortIdxQuat  = 0;
const int_T outputPortIdxGyro  = 1;
const int_T outputPortIdxAccel = 2;
const int_T outputPortIdxPose  = 3;
const int_T outputPortIdxArm   = 4;
const int_T outputPortIdxXDir  = 5;
const int_T outputPortIdxEMG   = 6;


/*=====================================*
 * Configuration and execution methods *
 *=====================================*/
/*
 *    The NumContStates, NumDiscStates, NumInputs, NumOutputs, NumRWork,
 *    NumIWork, NumPWork NumModes, and NumNonsampledZCs widths can be set to:
 *       DYNAMICALLY_SIZED    - In this case, they will be set to the actual
 *                              input width, unless you are have a
 *                              mdlSetWorkWidths to set the widths.
 *       0 or positive number - This explicitly sets item to the specified
 *                              value.
 */
static void mdlInitializeSizes(SimStruct *S)
{
  
  ssSetNumSFcnParams(S, 0);
  ssSetNumContStates(    S, 0);   /* number of continuous states           */
  ssSetNumDiscStates(    S, 0);   /* number of discrete states             */
  
  if (!ssSetNumOutputPorts(S, numOutputPorts)) return;
  ssSetOutputPortWidth(S,outputPortIdxQuat,lenQuat);
  ssSetOutputPortWidth(S,outputPortIdxGyro,lenGyro);
  ssSetOutputPortWidth(S,outputPortIdxAccel,lenAccel);
  ssSetOutputPortWidth(S,outputPortIdxPose,lenPose);
  ssSetOutputPortWidth(S,outputPortIdxArm,lenArm);
  ssSetOutputPortWidth(S,outputPortIdxXDir,lenXDir);
  ssSetOutputPortWidth(S,outputPortIdxEMG,lenEMG);
  
  ssSetNumSampleTimes(S, 1);
  
  ssSetNumRWork(         S, 0);   /* number of real work vector elements   */
  ssSetNumIWork(         S, NUM_IWORK);   /* number of integer work vector elements*/
  ssSetNumPWork(         S, NUM_PWORK);   /* number of pointer work vector elements*/
  ssSetNumModes(         S, 0);   /* number of mode work vector elements   */
  ssSetNumNonsampledZCs( S, 0);   /* number of nonsampled zero crossings   */
  
  ssSetSimStateCompliance(S, USE_CUSTOM_SIM_STATE);
  
  ssSetOptions(          S, 0);   /* general options (SS_OPTION_xx)        */
  
} /* end mdlInitializeSizes */

static void mdlInitializeSampleTimes(SimStruct *S)
{
  ssSetSampleTime(S, 0, SAMPLE_TIME_EMG);
  ssSetOffsetTime(S, 0, 0.0);
  ssSetModelReferenceSampleTimeDefaultInheritance(S);
} /* end mdlInitializeSampleTimes */


#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START)
/* Function: mdlStart =======================================================
 * Abstract:
 *    This function is called once at start of model execution. If you
 *    have states that should be initialized once, this is the place
 *    to do it.
 */
static void mdlStart(SimStruct *S)
{
  
  // global data
  DataCollector* pCollector = NULL;
  myo::Hub* pHub = NULL;
  myo::Myo* pMyo = NULL;
  
  ssGetPWork(S)[IDX_COLLECTOR] = (void *) new DataCollector;
  pCollector = (DataCollector *)ssGetPWork(S)[IDX_COLLECTOR];
  pCollector->addEmgEnabled = true; // lets collector handle data events

  // Instantiate a Hub and get a Myo
  ssGetPWork(S)[IDX_HUB] = (void *) new myo::Hub("com.mark-toma.myo_sfun");
  pHub = (myo::Hub *)ssGetPWork(S)[IDX_HUB];
  if ( !pHub )
    ssSetErrorStatus(S,"Hub failed to init!");
  pMyo = pHub->waitForMyo(5);
  if ( !pMyo )
    ssSetErrorStatus(S,"Myo failed to init!");
  
  // configure myo and hub
  pHub->setLockingPolicy(myo::Hub::lockingPolicyNone); // TODO: What does this do?
  pHub->addListener(pCollector);
  
  // instantiate mutex
  hMutex = CreateMutex(NULL,FALSE,NULL);
  if (hMutex == NULL)
    ssSetErrorStatus(S,"Failed to set up mutex.\n");
  
  // Let Hub run callbacks on collector so we can figure out how many
  // Myos are connected to Myo Connect so we can assert countMyosRequired
  pHub->run(INIT_DELAY);
  if (countMyosRequired != pCollector->getCountMyos())
    ssSetErrorStatus(S,"myo_sfun failed to initialize with countMyos.\n");
  
  // Flush the data queues with syncDataSources
  // Note: This pops the oldest samples of data off the front of all
  //   queues until only the most recent data remains
  pCollector->syncDataSources();
  
  // Enabled data and initialize buffer
  pCollector->addDataEnabled = true;
  pHub->run(BUFFER_DELAY);
  
  // dispatch concurrent task
  runThreadFlag = true;
  hThread = (HANDLE)_beginthreadex( NULL, 0, &runThreadFunc, S, 0, &threadID );
  if ( !hThread )
    ssSetErrorStatus(S,"Failed to create streaming thread!\n");
  
}
#endif /*  MDL_START */


/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block. Generally outputs are placed in the output vector(s),
 *    ssGetOutputPortSignal.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
  int_T ii;
  real_T *pOutputQuat;
  real_T *pOutputGyro;
  real_T *pOutputAccel;
  real_T *pOutputEMG;

  // get pHub and pCollector
  myo::Hub* pHub = (myo::Hub *) ssGetPWork(S)[IDX_HUB];
  DataCollector* pCollector = (DataCollector *) ssGetPWork(S)[IDX_COLLECTOR];
  
  // get iteration counter
  int_T iter = ssGetIWork(S)[IDX_ITER];
  
  int_T countIMU = pCollector->getCountIMU(1);
  int_T countEMG = pCollector->getCountEMG(1);
  ssPrintf("iter %d\t IMU %d\t EMG %d\n",
          iter,countIMU,countEMG);
  
  // fail if the queue is falling behind
  if (countIMU < 2)//0.5*BUFFER_SAMPLES_IMU)
    ssSetErrorStatus(S,"IMU buffer is less than half full.");
  if (countEMG < 2)//0.5*BUFFER_SAMPLES_EMG)
    ssSetErrorStatus(S,"EMG buffer is less than half full.");
  
  // grab mutex and pop out some data
  // Verify that collector still has all of its Myos, otherwise error out
  unsigned int countMyos = pCollector->getCountMyos();
  if ( countMyos != countMyosRequired )
    ssSetErrorStatus(S,"myo_sfun countMyos is inconsistent with initialization... We lost a Myo!");
  
  FrameIMU frameIMU1, frameIMU2; // Data structures returned from queue read
  FrameEMG frameEMG1, frameEMG2;
  
  bool isHitIMU = (0==(iter % 4));
  // bool isHitIMU = (0==(iter % 4));
  if (isHitIMU)
    ssPrintf("isHitIMU!\n");
  
  // Now get ahold of the lock and iteratively drain the queue while
  // filling outDataN matrices
  DWORD dwWaitResult;
  dwWaitResult = WaitForSingleObject(hMutex,INFINITE);
  switch (dwWaitResult)
  {
    case WAIT_OBJECT_0: // The thread got ownership of the mutex
      // --- CRITICAL SECTION - holding lock
      if (isHitIMU)
        frameIMU1 = pCollector->getFrameIMU(1);
        frameEMG1 = pCollector->getFrameEMG(1);
      // END CRITICAL SECTION - release lock
      if ( !ReleaseMutex(hMutex))
        ssSetErrorStatus(S,"Failed to release lock\n");
      break;
    case WAIT_ABANDONED:
      ssSetErrorStatus(S,"Acquired abandoned lock\n");
      break;
  }
  
  if (isHitIMU) {
    pOutputQuat = ssGetOutputPortRealSignal(S,outputPortIdxQuat);
    pOutputQuat[0] = frameIMU1.quat.w();
    pOutputQuat[1] = frameIMU1.quat.x();
    pOutputQuat[1] = frameIMU1.quat.y();
    pOutputQuat[3] = frameIMU1.quat.z();
    pOutputGyro = ssGetOutputPortRealSignal(S,outputPortIdxGyro);
    pOutputGyro[0] = frameIMU1.gyro.x();
    pOutputGyro[1] = frameIMU1.gyro.y();
    pOutputGyro[2] = frameIMU1.gyro.z();
    pOutputAccel = ssGetOutputPortRealSignal(S,outputPortIdxAccel);
    pOutputAccel[0] = frameIMU1.accel.x();
    pOutputAccel[1] = frameIMU1.accel.y();
    pOutputAccel[2] = frameIMU1.accel.z();
  }
  
  pOutputEMG = ssGetOutputPortRealSignal(S,outputPortIdxEMG);
  ii = 0;
  for (ii;ii<8;ii++)
    pOutputEMG[ii] = frameEMG1.emg[ii];
  
  // increment and store new iteration value
  ssGetIWork(S)[IDX_ITER] = ++iter;
  
} /* end mdlOutputs */


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was allocated
 *    in mdlStart, this is the place to free it.
 *
 *    Suppose your S-function allocates a few few chunks of memory in mdlStart
 *    and saves them in PWork. The following code fragment would free this
 *    memory.
 *        {
 *            int i;
 *            for (i = 0; i<ssGetNumPWork(S); i++) {
 *                if (ssGetPWorkValue(S,i) != NULL) {
 *                    free(ssGetPWorkValue(S,i));
 *                }
 *            }
 *        }
 */
static void mdlTerminate(SimStruct *S)
{
  myo::Hub* pHub = (myo::Hub *) ssGetPWork(S)[IDX_HUB];
  DataCollector* pCollector = (DataCollector *) ssGetPWork(S)[IDX_COLLECTOR];
  
// stop streaming and clean up
  // Terminate thread and reset state
  runThreadFlag = false; // thread sees this flag and exits
  WaitForSingleObject( hThread, INFINITE );
  CloseHandle( hThread );
  hThread = NULL;
  
  // Terminate data logging and reset state
  pCollector->addDataEnabled = false; // stop handling data events
  pCollector->syncDataSources(); // sync data up again (flushes queue)
  
  CloseHandle (hMutex);
  hMutex = NULL;
  if (pHub!=NULL)
    delete pHub;
  
}




/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
