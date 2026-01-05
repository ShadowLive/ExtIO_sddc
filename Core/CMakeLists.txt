cmake_minimum_required(VERSION 3.13)

# FFT Backend Selection
# Options: FFTW (default), MKL
set(FFT_BACKEND "FFTW" CACHE STRING "FFT backend to use (FFTW, MKL)")
set_property(CACHE FFT_BACKEND PROPERTY STRINGS FFTW MKL)

message(STATUS "FFT Backend: ${FFT_BACKEND}")

if (MSVC)
    file(GLOB ARCH_SRC "arch/win32/*.cpp" "arch/win32/CyAPI/*.cpp")
else()
    file(GLOB ARCH_SRC "arch/linux/*.c" "arch/linux/*.cpp")
endif (MSVC)

file(GLOB SRC "*.cpp" "radio/*.cpp" "pffft/*.cpp" ${ARCH_SRC})

# Exclude FFT backend files - we'll add the selected one
list(FILTER SRC EXCLUDE REGEX "fft_backend_.*\\.cpp$")

# Include the FFT benchmark
list(APPEND SRC fft_benchmark.cpp)

# Add selected FFT backend
if(FFT_BACKEND STREQUAL "MKL")
    list(APPEND SRC fft_backend_mkl.cpp)
    message(STATUS "Using Intel MKL backend")
else()
    # Default: FFTW
    list(APPEND SRC fft_backend_fftw.cpp)
    message(STATUS "Using FFTW backend")
endif()

if (MSVC)
    # Assume Windows/x86 target ;)
    set_source_files_properties(fft_mt_r2iq_avx.cpp PROPERTIES COMPILE_FLAGS /arch:AVX)
    set_source_files_properties(fft_mt_r2iq_avx2.cpp PROPERTIES COMPILE_FLAGS /arch:AVX2)
    set_source_files_properties(fft_mt_r2iq_avx512.cpp PROPERTIES COMPILE_FLAGS /arch:AVX512)
else()
    # Assume GCC, check target CPU/FPU
  if ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
    message(STATUS "Compiling for x64")
    set_source_files_properties(fft_mt_r2iq_avx.cpp PROPERTIES COMPILE_FLAGS -mavx)
    set_source_files_properties(fft_mt_r2iq_avx2.cpp PROPERTIES COMPILE_FLAGS -mavx2)
    set_source_files_properties(fft_mt_r2iq_avx512.cpp PROPERTIES COMPILE_FLAGS -mavx512f)
  elseif("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "arm.*")
    # We may have Neon..
    message(STATUS "Compiling for Neon")
    list(FILTER SRC EXCLUDE REGEX "fft_mt_r2iq_avx.*")
    list(APPEND SRC fft_mt_r2iq_neon.cpp)
    # On Apple Silicon (arm64) clang does not accept '-mfpu='; NEON is enabled by default.
    if(APPLE)
      # Don't pass -mfpu to Apple clang; only enable PFFFT's NEON code path and relax strict-aliasing warnings
      set_source_files_properties(fft_mt_r2iq_neon.cpp PROPERTIES COMPILE_FLAGS "")
      set_source_files_properties(pffft/pf_mixer.cpp PROPERTIES COMPILE_FLAGS "-D PFFFT_ENABLE_NEON -Wno-strict-aliasing")
    else()
      set_source_files_properties(fft_mt_r2iq_neon.cpp PROPERTIES COMPILE_FLAGS -mfpu=neon-vfpv4)
      set_source_files_properties(pffft/pf_mixer.cpp PROPERTIES COMPILE_FLAGS "-D PFFFT_ENABLE_NEON -mfpu=neon-vfpv4 -Wno-strict-aliasing")
    endif()
  else()
    message(FATAL_ERROR "Unable to identify CPU: ${CMAKE_SYSTEM_PROCESSOR}")
  endif()
endif (MSVC)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")

# Generatge firmware
include(bin2h.cmake)
bin2h(SOURCE_FILE "../SDDC_FX3.img" HEADER_FILE "${CMAKE_BINARY_DIR}/firmware.h" VARIABLE_NAME "firmware")
include_directories(${CMAKE_BINARY_DIR})

add_library(SDDC_CORE STATIC ${SRC})
set_target_properties(SDDC_CORE PROPERTIES POSITION_INDEPENDENT_CODE True)
target_compile_definitions(SDDC_CORE PUBLIC _CRT_SECURE_NO_WARNINGS)

# FFT backend configuration
if(FFT_BACKEND STREQUAL "MKL")
    target_compile_definitions(SDDC_CORE PUBLIC FFT_BACKEND_MKL)
    # MKL configuration - use FFTW-compatible interface
    set(MKL_ROOT "/opt/intel/oneapi/mkl/latest" CACHE PATH "Intel MKL root directory")
    target_include_directories(SDDC_CORE PUBLIC "${MKL_ROOT}/include/fftw")
    target_link_directories(SDDC_CORE PUBLIC "${MKL_ROOT}/lib")
    # Link MKL runtime library
    target_link_libraries(SDDC_CORE PUBLIC mkl_rt)
else()
    # Default: FFTW
    target_compile_definitions(SDDC_CORE PUBLIC FFT_BACKEND_FFTW)
    target_include_directories(SDDC_CORE PUBLIC "${LIBFFTW_INCLUDE_DIRS}")
    target_link_directories(SDDC_CORE PUBLIC "${LIBFFTW_LIBRARY_DIRS}")
    target_link_libraries(SDDC_CORE PUBLIC ${LIBFFTW_LIBRARIES})
endif()

if (NOT USE_SIMD_OPTIMIZATIONS)
   target_compile_definitions(SDDC_CORE PRIVATE NO_SIMD_OPTIM)
endif()

if(MSVC)
  target_link_libraries(SDDC_CORE PUBLIC Setupapi.lib)
else(MSVC)
  target_include_directories(SDDC_CORE PUBLIC "${LIBUSB_INCLUDE_DIRS}")
  target_link_directories(SDDC_CORE PUBLIC "${LIBUSB_LIBRARY_DIRS}")
  target_link_libraries(SDDC_CORE PUBLIC ${LIBUSB_LIBRARIES})
endif(MSVC)

# Standalone FFT benchmark executable
add_executable(fft_benchmark fft_benchmark.cpp)
target_compile_definitions(fft_benchmark PRIVATE FFT_BENCHMARK_MAIN)
target_link_libraries(fft_benchmark PRIVATE SDDC_CORE)
if(FFT_BACKEND STREQUAL "MKL")
    target_sources(fft_benchmark PRIVATE fft_backend_mkl.cpp)
else()
    target_sources(fft_benchmark PRIVATE fft_backend_fftw.cpp)
endif()