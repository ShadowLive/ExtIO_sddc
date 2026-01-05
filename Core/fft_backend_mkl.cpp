// Intel MKL Backend Implementation
// Uses MKL's FFTW-compatible interface for drop-in compatibility

#include "fft_backend.h"

// MKL provides FFTW3-compatible interface
#include "fftw3.h"  // MKL's FFTW wrapper (via MKL include path)

#include <cstdlib>

// Internal plan wrapper
struct MKLPlan {
    fftwf_plan plan;
    bool is_r2c;
};

class MKLBackend : public FFTBackend {
public:
    MKLBackend() = default;
    ~MKLBackend() override = default;

    const char* name() const override {
        return "MKL";
    }

    FFTPlanHandle plan_r2c(int n, float* in, fft_complex* out) override {
        auto* p = new MKLPlan();
        p->is_r2c = true;
        p->plan = fftwf_plan_dft_r2c_1d(n, in,
            reinterpret_cast<fftwf_complex*>(out), FFTW_MEASURE);
        return static_cast<FFTPlanHandle>(p);
    }

    FFTPlanHandle plan_c2c(int n, fft_complex* in, fft_complex* out, FFTDirection dir) override {
        auto* p = new MKLPlan();
        p->is_r2c = false;
        int fftw_dir = (dir == FFTDirection::Forward) ? FFTW_FORWARD : FFTW_BACKWARD;
        p->plan = fftwf_plan_dft_1d(n,
            reinterpret_cast<fftwf_complex*>(in),
            reinterpret_cast<fftwf_complex*>(out),
            fftw_dir, FFTW_MEASURE);
        return static_cast<FFTPlanHandle>(p);
    }

    void execute_r2c(FFTPlanHandle handle, float* in, fft_complex* out) override {
        auto* plan = static_cast<MKLPlan*>(handle);
        fftwf_execute_dft_r2c(plan->plan, in, reinterpret_cast<fftwf_complex*>(out));
    }

    void execute_c2c(FFTPlanHandle handle, fft_complex* in, fft_complex* out) override {
        auto* plan = static_cast<MKLPlan*>(handle);
        fftwf_execute_dft(plan->plan,
            reinterpret_cast<fftwf_complex*>(in),
            reinterpret_cast<fftwf_complex*>(out));
    }

    void destroy_plan(FFTPlanHandle handle) override {
        auto* plan = static_cast<MKLPlan*>(handle);
        if (plan) {
            fftwf_destroy_plan(plan->plan);
            delete plan;
        }
    }

    void* alloc(size_t bytes) override {
        return fftwf_malloc(bytes);
    }

    void free(void* ptr) override {
        fftwf_free(ptr);
    }
};

// Singleton instance
static MKLBackend g_mkl_backend;

FFTBackend* getFFTBackend() {
    return &g_mkl_backend;
}
