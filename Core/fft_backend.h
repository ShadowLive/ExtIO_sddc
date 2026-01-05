#pragma once

// FFT Backend Abstraction Layer
// Allows compile-time selection of FFT library: FFTW, MKL, pocketfft

#include <complex>
#include <cstddef>

// Complex type used throughout (interoperable with fftwf_complex)
using fft_complex = std::complex<float>;

// Opaque plan handle (defined by each backend)
typedef void* FFTPlanHandle;

// FFT direction
enum class FFTDirection {
    Forward,
    Backward
};

// Abstract FFT backend interface
class FFTBackend {
public:
    virtual ~FFTBackend() = default;

    // Get backend name for logging
    virtual const char* name() const = 0;

    // Plan creation
    virtual FFTPlanHandle plan_r2c(int n, float* in, fft_complex* out) = 0;
    virtual FFTPlanHandle plan_c2c(int n, fft_complex* in, fft_complex* out, FFTDirection dir) = 0;

    // Plan execution (can use different arrays than planning if same size/alignment)
    virtual void execute_r2c(FFTPlanHandle plan, float* in, fft_complex* out) = 0;
    virtual void execute_c2c(FFTPlanHandle plan, fft_complex* in, fft_complex* out) = 0;

    // Plan destruction
    virtual void destroy_plan(FFTPlanHandle plan) = 0;

    // Optional: import/export wisdom (FFTW-specific, no-op for others)
    virtual void import_wisdom(const char* filename) { (void)filename; }
    virtual void export_wisdom(const char* filename) { (void)filename; }

    // Aligned memory allocation (some backends require specific alignment)
    virtual void* alloc(size_t bytes) = 0;
    virtual void free(void* ptr) = 0;
};

// Factory function to get the compiled-in backend
FFTBackend* getFFTBackend();

// Backend name macro for compile-time identification
#if defined(FFT_BACKEND_MKL)
    #define FFT_BACKEND_NAME "MKL"
#else
    #define FFT_BACKEND_NAME "FFTW"
#endif
