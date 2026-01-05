## ExtIO_sddc.dll (software digital down converter)

> **Warning: Custom Fork**
>
> This is a custom fork specifically modified for the **RX-888 Mk II**. Changes include VHF tuner fixes, performance optimizations, and API enhancements that have **not been tested against other SDR hardware** (HF103, BBRF103, RX888 Mk I, RX999, etc.). If you are using a different SDR, please use the [upstream repository](https://github.com/ik1xpv/ExtIO_sddc) instead.

Based on the original work by Oscar Steila (IK1XPV).

## RX-888 Documents and Support

If you are looking for RX-888 documents or support, please navigate to: https://www.rx-888.com/rx/

## Installation Instructions

This fork must be built from source. See the build instructions below.

For pre-built binaries from the upstream project, visit: https://github.com/ik1xpv/ExtIO_sddc/releases (note: these will not include the fixes in this fork).


## Build Instructions for ExtIO

1. Install Visual Studio 2019 with Visual C++ support. You can use the free community version, which can be downloaded from: https://visualstudio.microsoft.com/downloads/
1. Install CMake 3.19+, https://cmake.org/download/
1. Running the following commands in the root folder of the cloned repro:

```bash
> mkdir build
> cd build
> cmake ..
> cmake --build .
or
> cmake --build . --config Release
> cmake --build . --config RelWithDebInfo
```

* You need to download **32bit version** of fftw library from fftw website http://www.fftw.org/install/windows.html. Copy libfftw3f-3.dll from the downloaded zip package to the same folder of extio DLL.

* If you are running **64bit** OS, you need to run the following different commands instead of "cmake .." based on your Visual Studio Version:
```
VS2022: >cmake .. -G "Visual Studio 17 2022" -A Win32
VS2019: >cmake .. -G "Visual Studio 16 2019" -A Win32
VS2017: >cmake .. -G "Visual Studio 15 2017 Win32"
VS2015: >cmake .. -G "Visual Studio 14 2015 Win32"
```

## Build Instructions for firmware

- download latest Cypress EZ-USB FX3 SDK from here: https://www.cypress.com/documentation/software-and-drivers/ez-usb-fx3-software-development-kit
- follow the installation instructions in the PDF document 'Getting Started with FX3 SDK'; on Windows the default installation path will be 'C:\Program Files (x86)\Cypress\EZ-USB FX3 SDK\1.3' (see pages 17-18) - on Linux the installation path could be something like '/opt/Cypress/cyfx3sdk'
- add the following environment variables:
```
export FX3FWROOT=<installation path>
export ARMGCC_INSTALL_PATH=<ARM GCC installation path>
export ARMGCC_VERSION=4.8.1
```
(on Linux you may want to add those variables to your '.bash_profile' or '.profile')
- all the previous steps need to be done only once (or when you want to upgrade the version of the Cypress EZ-USB FX3 SDK)
- to compile the firmware run:
```
cd SDDC_FX3
make
```

## Build Instruction for Linux

1. Install development packages:
```bash
> sudo apt install libfftw3-dev
```

1. Follow Windows Build Instruction to run cmake to build Linux libaray


## Directory structure:
    \Core\           > Core logic of the component
        r2iq.cpp			> The logic to demodulize IQ from ADC real samples
        FX3handler.cpp		> Interface with firmware
        RadioHandler.cpp    > The abstraction of different radios
        Radio\*.cpp         > Hardware specific logic
    \ExtIO_sddc\ 		> ExtIO_sddc sources,
        extio_sddc.cpp     > The implementation of EXTIO contract 
        tdialog.cpp			> The Configuration GUI Dialog
    \libsddc\        > libsddc lib
    \SDDC_FX3\          > Firmware sources

## Fork Change Log

### RX-888 Mk II Fork (January 2026)

**VHF Tuner Fixes:**
- Fix VHF mode spectrum mirroring caused by incorrect sideband inversion
- R828D tuner uses low-side injection, spectrum should not be inverted at IF

**Performance Optimizations:**
- Replace byte-by-byte ring buffer copy with memcpy (10-100x sync read improvement)
- Increase FFT processing threads from 1 to 4 for multi-core utilization
- SSE/AVX vectorization for int16→float conversion, complex multiply, and complex copy
- Lock-free ring buffer using std::atomic indices
- Reduced mutex contention in fine-tune mixer path

**libsddc API Enhancements:**
- Add VGA (AD8370) gain control functions
- Add GPIO control functions for direct hardware access
- Add tuner IF frequency configuration
- Add error handling with sddc_get_last_error()
- Improved sample rate documentation

## Upstream Project

For references, acknowledgments, and full project history, see the [upstream repository](https://github.com/ik1xpv/ExtIO_sddc).
