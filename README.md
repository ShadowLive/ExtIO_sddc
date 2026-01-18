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
> sudo apt install libfftw3-dev libusb-1.0-0-dev pkg-config
```

2. Build with CMake:
```bash
> mkdir build && cd build
> cmake -DCMAKE_BUILD_TYPE=Release ..
> make -j$(nproc)
```

**FFT Backend Selection:**

The project supports multiple FFT backends. Choose one at build time:

```bash
# FFTW (default, best compatibility)
> cmake -DFFT_BACKEND=FFTW ..

# Intel MKL (best performance on x86_64 with MKL installed)
> cmake -DFFT_BACKEND=MKL ..

# Apple Accelerate (best performance on macOS/Apple Silicon)
> cmake -DFFT_BACKEND=Accelerate ..
```

**Apple Silicon / M4 Build:**
```bash
> mkdir build && cd build
> cmake -DFFT_BACKEND=Accelerate \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_OSX_DEPLOYMENT_TARGET=15.0 ..
> make -j8
```

The Apple Silicon build includes ARM NEON intrinsics for maximum performance.


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
- AVX2 vectorization for int16→float conversion, complex multiply, and complex copy (x86_64)
- ARM NEON intrinsics for maximum performance on Apple Silicon M4 (~4.5x speedup)
- Apple Accelerate framework integration for optimized FFT on macOS
- FFT backend abstraction supporting FFTW, Intel MKL, and Apple Accelerate
- Lock-free ring buffer using std::atomic indices
- Reduced mutex contention in fine-tune mixer path
- Fixed FFT thread race conditions for reliable 128 Msps streaming

**libsddc API Enhancements:**
- Add VGA (AD8370) gain control functions
- Add GPIO control functions for direct hardware access
- Add tuner IF frequency configuration
- Add error handling with sddc_get_last_error()
- Improved sample rate documentation

**Performance Benchmarks (Apple M4):**

FFT Backend Performance (Accelerate framework):
```
  Size  |   R2C (µs)   |  C2C Fwd (µs) |  C2C Bwd (µs)
-------------------------------------------------------
  4096  |       8.66   |       10.67   |        7.44
  2048  |       1.97   |        2.86   |        3.35
  1024  |       0.86   |        1.29   |        1.55
```

NEON Optimization Speedups:
- int16→float conversion: **7.7x faster**
- Complex multiplication: **3.4x faster**
- Complex copy/conjugate: **3.2x faster**
- Overall pipeline: **~4.5x faster** (enables 128 Msps real-time streaming)

## Upstream Project

For references, acknowledgments, and full project history, see the [upstream repository](https://github.com/ik1xpv/ExtIO_sddc).
