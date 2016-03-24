# OSVR-SharedIMU
> Maintained at <http://gitlab.motionreality.com/cry/OSVR-SharedIMU>

A minimal OSVR device plugin for the OSVR HDK IMU that opens the HID-compliant device in shared mode. This is needed to share the IMU with CryEngine/Dauntless, which uses direct access to the IMU to facilitate sensor fusion with tracking data.

## Building
The plug-in is built using CMake and has two direct dependencies.

* [OSVR-Core](https://github.com/OSVR/OSVR-Core "OSVR-Core")
* [Eigen3](https://github.com/OSVR/OSVR-Core/tree/master/vendor/eigen "Eigen 3")

At the time of writing, the plug-in was built and tested against OSVR-Core v0.6-818-gd437ac1 which uses/provides Eigen 3.2.8. Transitively, this depends on Boost via the OSVR-Core dependency. This was built and tested against Boost 1.55.

You will need to point CMake at all three dependencies. Example steps:

1. Install Boost 1.55 from binary to a common location, e.g. `C:\boost\boost_1.55\`
2. Install OSVR-Core from binary snapshot
3. Clone [OSVR-Core](https://github.com/OSVR/OSVR-Core "OSVR-Core") to get their vendored version of Eigen3
4. Clone this repo
5. Run CMake on this repo (it will fail) for VS 2013 x64
6. Point `osvr_DIR` at the root of the OSVR-Core binary snapshot
7. Point `Eigen3_DIR` at `vendor/eigen/` under the source clone of `osvr/OSVR-Core1
8. Run CMake again
9. Open the VS solution and build in `Release` or `RelWithDebugInfo`
9. ...
10. Profit.

## Use/installation

The build process will generate a DLL plug-in that needs to be installed in the OSVR Core plugins folder. This is `osvr-plugins-0` relative to the `osvr_server.exe`. This plug-in replaces the `com_osvr_Multiserver.dll` plugin, which must be removed from the folder. The `com_osvr_Multiserver` plugin also attempts to open the OSVR HDK IMU but in exclusive mode. By removing the `com_osvr_Multiserver` plugin, you have also disabled the automatic loading of the Dead Reckoning plug-in (`org_osvr_filter_deadreckoningrotation`). You will need to modify the `osvr_server_config.json` to enable Dead Reckoning for the IMU again. See the example [`osvr_server_config.json`](osvr_server_config.json) in this repo for how to do that.
   


