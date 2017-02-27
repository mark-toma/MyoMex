// myo_sfun_wiring.h
#ifndef _MYO_SFUN_WIRING_
#define _MYO_SFUN_WIRING_

#define NUM_INPUT_PORTS 0

#define OUTPUT_PORT_IDX_QUAT  0
#define OUTPUT_PORT_IDX_GYRO  1
#define OUTPUT_PORT_IDX_ACCEL 2
#define OUTPUT_PORT_IDX_POSE  3
#define OUTPUT_PORT_IDX_ARM   4
#define OUTPUT_PORT_IDX_XDIR  5
#define OUTPUT_PORT_IDX_EMG   6
#define NUM_OUTPUT_PORTS_IMU  6
#define NUM_OUTPUT_PORTS_EMG  1

#define LEN_QUAT  4
#define LEN_GYRO  3
#define LEN_ACCEL 3
#define LEN_POSE  1
#define LEN_ARM   1
#define LEN_XDIR  1
#define LEN_EMG   8


#define IDX_HUB 0
#define IDX_COLLECTOR 1
#define NUM_PWORK 2

#define IDX_ITER 0
#define IDX_COUNT_MYOS_REQUIRED 1
#define NUM_IWORK 2

#define IDX_EMG_ENABLED_REQUIRED 0
#define IDX_COUNT_MYOS_REQUIRED 1
#define NUM_SFCN_PARAMS 2

#endif // _MYO_SFUN_WIRING_