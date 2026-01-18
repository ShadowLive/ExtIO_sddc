// Apple Accelerate Backend Implementation
// Uses vDSP for FFT operations on macOS/iOS

#include "fft_backend.h"
#include <Accelerate/Accelerate.h>
#include <cstring>
#include <cstdlib>

// Internal plan wrapper for vDSP
struct AcceleratePlan {
    FFTSetup setup;
    int log2n;
    bool is_r2c;
    // Split-complex temporary buffers (vDSP uses split-complex format)
    DSPSplitComplex split_buffer;
    float* temp_real;
    float* temp_imag;
};

class AccelerateBackend : public FFTBackend {
public:
    AccelerateBackend() = default;
    ~AccelerateBackend() override = default;

    const char* name() const override {
        return "Accelerate";
    }

    FFTPlanHandle plan_r2c(int n, float* in, fft_complex* out) override {
        auto* p = new AcceleratePlan();
        p->is_r2c = true;

        // Calculate log2(n) for vDSP
        int log2n = 0;
        int temp = n;
        while (temp > 1) {
            temp >>= 1;
            log2n++;
        }
        p->log2n = log2n;

        // Create FFT setup (reusable for multiple executions)
        p->setup = vDSP_create_fftsetup(log2n, FFT_RADIX2);

        // Allocate split-complex temporary buffers
        int complex_size = n / 2 + 1; // R2C produces n/2+1 complex values
        p->temp_real = static_cast<float*>(malloc(complex_size * sizeof(float)));
        p->temp_imag = static_cast<float*>(malloc(complex_size * sizeof(float)));
        p->split_buffer.realp = p->temp_real;
        p->split_buffer.imagp = p->temp_imag;

        return static_cast<FFTPlanHandle>(p);
    }

    FFTPlanHandle plan_c2c(int n, fft_complex* in, fft_complex* out, FFTDirection dir) override {
        auto* p = new AcceleratePlan();
        p->is_r2c = false;

        // Calculate log2(n)
        int log2n = 0;
        int temp = n;
        while (temp > 1) {
            temp >>= 1;
            log2n++;
        }
        p->log2n = log2n;

        // Create FFT setup
        p->setup = vDSP_create_fftsetup(log2n, FFT_RADIX2);

        // Allocate split-complex temporary buffers
        p->temp_real = static_cast<float*>(malloc(n * sizeof(float)));
        p->temp_imag = static_cast<float*>(malloc(n * sizeof(float)));
        p->split_buffer.realp = p->temp_real;
        p->split_buffer.imagp = p->temp_imag;

        return static_cast<FFTPlanHandle>(p);
    }

    void execute_r2c(FFTPlanHandle handle, float* in, fft_complex* out) override {
        auto* plan = static_cast<AcceleratePlan*>(handle);

        // vDSP R2C FFT expects the input to be treated as split-complex
        // with even indices as real and odd indices as imaginary
        DSPSplitComplex input_split;
        input_split.realp = in;
        input_split.imagp = in + 1;

        // Perform R2C FFT (actually uses vDSP_fft_zrip which is optimized for real input)
        // Note: vDSP_fft_zrip expects data in a packed format
        // We need to use vDSP_ctoz to convert interleaved to split, then vDSP_fft_zop

        // Convert input (reinterpreted as interleaved complex) to split format
        vDSP_ctoz(reinterpret_cast<DSPComplex*>(in), 2, &plan->split_buffer, 1, 1 << (plan->log2n - 1));

        // Perform FFT
        vDSP_fft_zrip(plan->setup, &plan->split_buffer, 1, plan->log2n, FFT_FORWARD);

        // Scale the output (vDSP doesn't scale by default)
        float scale = 0.5f;
        vDSP_vsmul(plan->split_buffer.realp, 1, &scale, plan->split_buffer.realp, 1, 1 << (plan->log2n - 1));
        vDSP_vsmul(plan->split_buffer.imagp, 1, &scale, plan->split_buffer.imagp, 1, 1 << (plan->log2n - 1));

        // Convert split-complex back to interleaved format
        vDSP_ztoc(&plan->split_buffer, 1, reinterpret_cast<DSPComplex*>(out), 2, 1 << (plan->log2n - 1));

        // Handle DC and Nyquist components
        int n = 1 << plan->log2n;
        out[n/2][0] = plan->split_buffer.imagp[0]; // Nyquist
        out[n/2][1] = 0.0f;
    }

    void execute_c2c(FFTPlanHandle handle, fft_complex* in, fft_complex* out) override {
        auto* plan = static_cast<AcceleratePlan*>(handle);
        int n = 1 << plan->log2n;

        // Convert interleaved complex input to split-complex format
        vDSP_ctoz(reinterpret_cast<DSPComplex*>(in), 2, &plan->split_buffer, 1, n);

        // Perform C2C FFT (forward direction, vDSP doesn't distinguish in the API)
        vDSP_fft_zop(plan->setup, &plan->split_buffer, 1, &plan->split_buffer, 1,
                     plan->log2n, FFT_FORWARD);

        // Convert split-complex back to interleaved format
        vDSP_ztoc(&plan->split_buffer, 1, reinterpret_cast<DSPComplex*>(out), 2, n);
    }

    void destroy_plan(FFTPlanHandle handle) override {
        auto* plan = static_cast<AcceleratePlan*>(handle);
        if (plan) {
            vDSP_destroy_fftsetup(plan->setup);
            free(plan->temp_real);
            free(plan->temp_imag);
            delete plan;
        }
    }

    void* alloc(size_t bytes) override {
        // Use 64-byte alignment for Apple Silicon cache optimization
        void* ptr = nullptr;
        posix_memalign(&ptr, 64, bytes);
        return ptr;
    }

    void free(void* ptr) override {
        ::free(ptr);
    }
};

// Singleton instance
static AccelerateBackend g_accelerate_backend;

FFTBackend* getFFTBackend() {
    return &g_accelerate_backend;
}
