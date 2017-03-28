# Myo SDK MATLAB MEX Wrapper
Access data from Thalmic Labs' Myo Gesture Control Armband in m-code!

**Myo SDK MATLAB MEX Wrapper** is also available on MathWorks File Exchange [here](http://www.mathworks.com/matlabcentral/fileexchange/55817-myo-sdk-mex-wrapper).

## Description

Thalmic Labs' Myo Gesture Control Armband (myo.com) features an Inertial Measurement Unit (IMU) and 8 surface Electromyography sensors (sEMG) in addition to a nice Windows SDK that allows developers to obtain access to this data! 

On the surface, this package contains a simplified m-code class, MyoMex, that enables MATLAB users to stream data from Myo at up to 50Hz (IMU) and 200Hz (EMG and meta data) with only 1 command!

    mm = MyoMex(); % Upon construction, MyoMex starts accumulating streaming data in its myoData property
    m = mm.myoData; % get MyoData object
    % Data is now being pushed into log properties of m named, 
    % quat_log, gyro_log, accel_log, emg_log, etc. 
    % Data acquisition is non-blocking, too! 
    mm.delete(); % clean up

The IMU data includes estimated quaternion (orientation), three-axis gyroscope (angular velocity), and three-axis accelerometer (linear acceleration).

The sEMG data includes 8 raw data channels plus the output of Myo's built-in gesture detection.

Here are some of the things that you'll find in this package, 

* README.txt - Step-by-step instructions for prerequisite configuration 
* install_myo_mex() - installation tool 
* build_myo_mex() - MEX-file build tool 
* MyoMex_Quickstart - Quickstart guide script with example code and liberal comments 
* MyoMexGUI_Monitor - GUI implementation of streaming data with visualization of all available data

## Common Solutions

#### Add Myo SDK to `PATH`

The location of the files myo32.dll and myo64.dll must be indicated in the `PATH` environment variable in Windows. The following error results from failure to perform this step of configuration.

English:
```
Error using MyoMex (line XXX) 
MEX-file 'myo_mex' failed to initialize with error: 
'Invalid MEX-file '<absolute path to MyoMex>\MyoMex\myo_mex\myo_mex.mexw64': Can't find specified module.' 
```

Spanish:
```
Error using MyoMex (line XXX) 
MEX-file 'myo_mex' failed to initialize with error:
'Invalid MEX-file '<absolute path to MyoMex>\MyoMex\myo_mex\myo_mex.mexw64': No se puede encontrar el módulo especificado.' 
```

The solution for this problem is to add the location of the Myo SDK bin directory to the `PATH` environment variable in Windows.

First, we must determine the absolute path of this directory by looking at two popular locations where you may have extracted the Myo SDK files.

The root of `C:\`:
```
C:\myo-sdk-win-0.9.0\bin\
```

Your user folder (and your username is "AliceAndBob")
```
C:\Users\AliceAndBob\myo-sdk-win-0.9.0\bin\
```

Now you've determined the absolute path to the Myo SDK bin directory, you must append this path to the value of the `PATH` environment variable in Windows. You can follow [this tutorial](http://www.computerhope.com/issues/ch000549.htm) to learn how to add this path to `PATH` in your version of Windows. A special thanks goes out to C. Spiewak for providing this link in the comments [here](http://www.mathworks.com/matlabcentral/fileexchange/55817-myo-sdk-matlab-mex-wrapper).

