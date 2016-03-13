# Myo_SDK_MEX_Wrapper
Access data from Thalmic Labs' Myo Gesture Control Armband in m-code!

**Myo SDK MEX Wrapper** is also available on MathWorks' File Exchange [here](http://www.mathworks.com/matlabcentral/fileexchange/55817-myo-sdk-mex-wrapper).

## Description

Thalmic Labs' Myo Gesture Control Armband (myo.com) features an Inertial Measurement Unit (IMU) and 8 surface Electromyography sensors (sEMG) in addition to a nice Windows SDK that allows developers to obtain access to this data! 

On the surface, this package contains a simplified m-code class, MyoMex, that enables MATLAB users to stream data from Myo at up to 50Hz with as few as 4 commands:

`
m = MyoMex();

m.startStreaming(); 

% Data is now being pushed into log properties named, 

% quat_log, gyro_log, accel_log, emg_log, etc. 

% Data acquisition is non-blocking, too! 

m.stopStreaming(); 

m.delete();
`

The IMU data includes estimated quaternion (orientation), three-axis gyroscope (angular velocity), and three-axis accelerometer (linear acceleration).

The sEMG data includes 8 raw data channels plus the output of Myo's built-in gesture detection.

Here are some of the things that you'll find in this package, 

* READ_ME.txt - Step-by-step instructions for prerequisite configuration 
* install_myo_mex() - installation tool 
* build_myo_mex() - MEX-file build tool 
* MyoMex_Quickstart - Quickstart guide script with example code and liberal comments 
* MyoMexGUI_Monitor - GUI implementation of streaming data with visualization of all available data

Under the hood, the MyoMex class depends on a MEX-file named myo_mex. This MEX-file implements the Myo SDK API to communicate with Myo and provide the interface between C++ and m-code.
