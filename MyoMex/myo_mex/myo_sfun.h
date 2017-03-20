// myo_sfun.h
#ifndef MYO_SFUN
#define MYO_SFUN


// ==================================================
// Debug macros
//#define DEBUG_MYO_SFUN
//#define DEBUG_MYO_SFUN_ITER
#ifdef DEBUG_MYO_SFUN
#define DB_MYO_SFUN(fmt, ...) ssPrintf(fmt, ##__VA_ARGS__)
#else
#define DB_MYO_SFUN(fmt, ...)
#endif
#ifdef DEBUG_MYO_SFUN_ITER
#define DB_MYO_SFUN_ITER(fmt, ...) ssPrintf(fmt, ##__VA_ARGS__)
#else
#define DB_MYO_SFUN_ITER(fmt, ...)
#endif


// ==================================================
// Validation macros
#define IS_PARAM_SCALAR_DOUBLE(pVal) ( \
mxIsDouble(pVal) && !mxIsComplex(pVal) && \
        (mxGetNumberOfDimensions(pVal)==2) && \
        (mxGetM(pVal)==1 && mxGetN(pVal)==1))
        

// ==================================================
// Configuration
#define BUFFER_FRAMES_DES 25 // initial size of data buffer in frames
                             // the frame rate is 25Hz so this default
                             // value 25 results in 1s latency


// program behavior
#define STREAMING_TIMEOUT 5
#define INIT_DELAY 1000 // [ms] to wait for Myo
#define BUFFER_FRAMES_MIN 1
#define SAMPLE_TIME_BLK 40  // [ms] sample time for the block
#define SAMPLE_TIME_IMU 20  // [ms]  50Hz
#define SAMPLE_TIME_EMG  5  // [ms] 200Hz
#define SAMPLES_PER_FRAME_IMU SAMPLE_TIME_BLK/SAMPLE_TIME_IMU
#define SAMPLES_PER_FRAME_EMG SAMPLE_TIME_BLK/SAMPLE_TIME_EMG
#define BUFFER_DELAY ((1+BUFFER_FRAMES_DES)*SAMPLE_TIME_BLK)

        
#endif // MYO_SFUN