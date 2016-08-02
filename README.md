# Installation (Windows)
Documentation is for Microsoft Visual Studio 14 64bit

## Install 3rd party libraries

### Boost

- go to boost.org and download the prebuilt windows binaries (e.g. boost_1_61_0-msvc-14.0-64.exe)
- Install

Set environment variables:
    BOOST_INCLUDEDIR    C:\SDKs\boost_1_58_0\
    BOOST_LIBRARYDIR    C:\SDKs\boost_1_58_0\lib64-msvc-12.0
    BOOST_ROOT          C:\SDKs\boost_1_58_0\boost


### GTSAM

- go to: https://bitbucket.org/gtborg/gtsam
- download and unpack
- open folder in cmake gui. Set output path to /build
- Configure and generate


```
mdir build
cd build
cmake .. -G "Visual Studio 14 2015 Win64" -DCMAKE_INSTALL_PREFIX="C:\lib\gtsam-3.2.1\installation"
```
- Open in MVS and compile Debug + Release configuration
- Install

## Build project




```
mdir build
cd build
cmake .. -G "Visual Studio 14 2015 Win64"
```