# VRPN-FreeD-OpenVR
VRPN and FreeD server for OpenVR (based on a VRPN-OpenVR from https://github.com/zhouhs88/vrpn-openvr)

# Acknowledgements
VRPN-OpenVR project https://github.com/zhouhs88/vrpn-openvr

# Downloads
Latest build available for download from http://research.m1stereo.tv/ue/VRPN-FreeD-OpenVR.7z

# Usage
This version of VRPN server provide original functionality of **VRPN-OpenVR** and some more usefull functions for using with UE4 (Unreal Engine) VRPN and FreeD services.

It is recommended to use *.bat* file to run server. For example:
```
VRPN-FreeD-OpenVR.exe ^
    port 3885 ^
    ref 0.0 0.0 1.51 ^
    cam CAMERA-78 LHR-971C5478 0.0 0.0 -0.4 ^
        freed 127.0.0.1:20000 ^
        freed 10.1.5.221:20001 ^
    cam TRACKER-78 LHR-971C5478 0.0 0.0 0.0 ^
        freed 127.0.0.1:20000 ^
        freed 10.1.5.221:20002 ^
    cam CAMERA-54 LHR-731BED54 0.0 0.0 -0.4 ^
        freed 127.0.0.1:20000 ^
        freed 10.1.5.221:20003 ^
    cam TRACKER-54 LHR-731BED54 0.0 0.0 0.0 ^
        freed 127.0.0.1:20000 ^
        freed 10.1.5.221:20004
```

Where:

* *port 3885* - TCP port to listen for VRPN server
* *ref 0.0 0.0 1.51* - reference point coordinate (in UE coordiantes X, Y, Z)
* *cam CAMERA-78 LHR-971C5478 0.0 0.0 -0.4* - adding a virtual camera with name **CAMERA-78** (it will be availabe with VRPN name *virtual/CAMERA-78@127.0.0.1:3885*), that assigned to tracker/controller with serial **LHR-971C5478** and it (camera's) nodal point shifted with vector **0.0 0.0 -0.4** (X, Y, Z) (40cm bellow tracker)
* *freed 127.0.0.1:20000* - request to send FreeD data packes to host **127.0.0.1** on UDP port **20000**

After starting application it will display all it works and status in a text console:
![running_app](/docs/ui1.png?raw=true "Running App")

# Virtual Space Calibration

That is actually a main goal of this app. Virtual space's camera coordinates and rotation are in terms of UE4 (this mean no need to remap axis for using it). Calibration of virtual space performed by putting tracking into Real space position that relates to virtual space ref point specified at argument. Tracker should **look forward** to **X** axes. After putting tracker into reference position, you need to press a key that relates to tracker's index. On a screen above it is **1**.

# UE4 integration

nDisplay has support for VRPN. But if you need to use it in Editor, you can use LiveLink plugins:
* https://github.com/max-verem/VRPNLiveLink
* https://github.com/max-verem/FreeDLiveLink



