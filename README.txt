###########################################################################
# README.txt - Myo SDK MEX Wrapper MATLAB Package Guide

Written by: Mark Tomaszewski, mark@mark-toma.com, marktoma@buffalo.edu

Read through this file before going into the code and/or going willy-nilly 
typing commands in MATLAB. There are some PREREQUISITES (download and 
install dependencies from Thalmic Labs), followed by MATLAB path setup, 
MEX-file build instructions, and finally usage examples of this package. 
After you make it through this document, you can find some additional 
information in the help for MyoMex.

###########################################################################
# CHANGELOG - Release Notes

2016-04-01 2.0
  Major update - Added support for multiple Myos and 200Hz EMG data. Every 
  file underwent substantial changes, and most of the previous API should 
  still function the same way. But there are some breaking changes. See API
  CHANGES below
  * NEW FEATURE in 
  * UPDATED DOCS in MyoMex_Quickstart.m, README.txt
  * API CHANGES in MyoMex.m, MyoData.m
    All of the data properties that used to be in MyoMex are now in
    MyoData. The MyoMex instance simply manages the myo_mex interface and
    passes new data for each Myo device into an instance of MyoData in its
    myoData property. That is, mm = MyoMex(countMyos) results in,
    m = mm.myoData where m is a 1 -by- countMyos object of type MyoData.
    Then, m1 = mm.myoData(1) can be used to access data from Myo almost
    identically to the interface provided by MyoMex previously.
  * NEW CLASS MyoData.m
    Provides access to data collected from a physical Myo device.
  * DEPRECATED FEATURE in MyoMex.m, MyoData.m
    Polling for data is no longer supported, i.e. MyoMex.getData().
  * NEW FEATURE
    Multiple Myos supported. MyoMex is now instantiated with an optional
    parameter, countMyos. This numeric scalar specifies the number of Myos
    to use. Currently, only one or two Myos are supported. The number of
    Myos in Myo Connect be exactly countMyos for MyoMex construction to be 
    successful.
  * NEW FEATURE
    All data is now collected at the maximum sample rate. That is 50Hz for
    IMU data, 200Hz for EMG data, and additional meta data is reported at
    200Hz at the time instants of EMG samples.
  * NEW FEATURE in build_myo_mex.m
    If input sdk_path is the string 'default' then it is assumed to be
    C:\myo-sdk-win-0.9.0\. This is useful for those who have extracted the 
    Myo SDK to C:\.

2016-03-12 1.1
  Some new features in MyoMex (no changes to myo_mex) and a slew of 
  modifications to the example GUI. The myo_mex build has also been tested 
  successfully with Visual Studio 2012 Professional.
  * BUGFIX in MyoMexGUI_Monitor.m
    Added call to getData() immediately after instantiation of MyoMex to 
    initialize data properties non-empty before the first figure update.
  * FEATURE in MyoMex.m
    Added static methods to perform quaternion math, q2r(), qRot(), 
    qMult(), qInv(). These assume Kx4 quaternion arrays, Kx3 vectors, and 
    3x3xK rotation matrices.
  * FEATURE in MyoMex.m
    Added properties to store the representation of gyro[_log] and 
    accel[_log] in the fixed frame as represented by the quat property.
    The new properties are named gyro_fixed, accel_fixed, gyro_fixed_log, 
    and accel_fixed_log and the rotation is performed when values are set.
    NOTE: This trades off additional storage for reduced computation in 
    repeated coordinate transformation of the gyro and accel properties.
  * CHANGE in MyoMexGUI_Monitor.m
    Re-implemented plotting of gyro and accel values in the fixed frame by
    leveraging the new fixed frame properties.
  * CHANGE in MyoMexGUI_Monitor.m
    Gave the figure update timer a descriptive name and removed startdelay.
  * CHANGE in MyoMexGUI_Monitor.m
    Flipped the cylinder representation of Myo coordinate frame so that now
    users can imagine their elbows are fixed at the origin. I shaded the 
    wireframe also.
  * CHANGE in MyoMexGUI_Monitor.m
    Saved the figure as proportional layout rather than non-resizable for 
    maximizing to fill large displays.
  * ISSUE in MyoMexGUI_Monitor.m
    Something's not quite right in r2015a. The figure is updating slow and 
    choppy as if the event queue was clogging up. Since the GUI isn't core 
    functionality of this package, I don't intend to fix this any time 
    soon.
    
2016-03-06 1.0
  Initial release.

###########################################################################
# PREREQUISITES - Myo Connect, Myo SDK, and environment setup

Start by downloading some required resources from Thalmic labs at,

https://developer.thalmic.com/downloads

Specifically, you'll need the Myo Connect application and the Windows SDK. 
You may need to register a free developer account with Thalmic Labs in 
order to download the SDK.

Myo Connect for Windows 1.0.1 (fetched 2016-03-02):
https://s3.amazonaws.com/thalmicdownloads/windows/1.0.1/Myo+Connect+Installer.exe

Windows SDK 0.9.0 (fetched 2016-03-02):
http://developer.thalmic.com/login/redirect/?next=/downloads

Procede with installation of Myo Connect by launching the installed and 
following its prompts.

Extract the SDK in your desired location in the filesystem, and take note 
of the resulting file structure. For instance, you may choose to extract 
the SDK contents to C:\ so that the resulting file structure looks like,

C:\myo-sdk-win-0.9.0\
  bin\
  ...
  include\
  lib\
  ...

Following this example, we'll refer to "C:\myo-sdk-win-0.9.0" as the 
SDK_PATH.

Next, add "SDK_PATH\bin" to your PATH environment variable. This allows 
your compiler to find the required DLL (i.e. myo32.dll or myo64.dll) when
linking against the Myo SDK. In this example, we just add 
";C:\myo-sdk-win-0.9.0\bin" to the end of the current PATH variable.


###########################################################################
# MATLAB PACKAGE INSTALLATION

Navigate to the location where you have extracted the contents of this 
package. Add the required directories to MATLAB's search path by typing,

  >> install_myo_mex

Alternatively, you may choose to have this command save the path so that 
you don't have to repeat this step in every new MATLAB session,

  >> install_myo_mex save


###########################################################################
# BUILDING MEX

Before you can build the mex file, you need to have a valid C++ compiler
installed on your system and configure mex to use this compiler. Assuming
that the former is already taken care of, type the following command and
follow the prompts to configure your C++ compiler.

  >> mex -setup

To build the mex interface, you need to specify the location of the Myo 
SDK. Recall your location for SDK_PATH from above and type the command,

  >> build_myo_mex SDK_PATH

Which in this example look like,

  >> build_myo_mex C:\myo-sdk-win-0.9.0\

Now hopefully this completes without error. Upon success, you'll see 
command window output similar to the following,

  >> build_myo_mex c:\myo-sdk-win-0.9.0

  Changing directory to build directory:
    'C:\path\to\matlab\package\Myo_SDK_MEX_Wrapper\MyoMex\myo_mex'

  Evaluating mex command:
    'mex -Ic:\myo-sdk-win-0.9.0\include -Lc:\myo-sdk-win-0.9.0\lib -lmyo64 myo_mex.cpp'

  Changing directory to original directory:
    'C:\path\where\you\started'

  MEX-file 'myo_mex' built successfully!

Possible errors are due to:

* Incorrect specification of SDK_PATH
* Corrupted Myo SDK file structure
* Failure to run 'install_myo_mex'
* Corrupted Myo_SDK_MEX_Wrapper file structure

If you did not add "SDK_PATH\bin" to the PATH environment variable, it is
possible to build successfully but experience failures when using MyoMex.

###########################################################################
# ACKNOWLEDGEMENTS

Thank you to boyali for sharing your implementation of a MEX wrapper for
Myo SDK! Your code is what got me started with this project!
https://github.com/boyali/matMYO

