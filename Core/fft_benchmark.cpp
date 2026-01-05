// FFT Backend Benchmark
// Standalone benchmark to compare FFTW, pocketfft, and MKL performance

#include "fft_backend.h"
#include "config.h"
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cmath>

// Test sizes matching actual usage in fft_mt_r2iq
static const int TEST_SIZES[] = {4096, 2048, 1024, 512, 256, 128};
static const int NUM_SIZES = sizeof(TEST_SIZES) / sizeof(TEST_SIZES[0]);
static const int WARMUP_ITERS = 10;
static const int BENCH_ITERS = 1000;

struct BenchResult {
    int size;
    double r2c_time_us;
    double c2c_fwd_time_us;
    double c2c_bwd_time_us;
};

static void fill_random(float* data, int n) {
    for (int i = 0; i < n; i++) {
        data[i] = static_cast<float>(rand()) / RAND_MAX - 0.5f;
    }
}

static void fill_random_complex(fft_complex* data, int n) {
    for (int i = 0; i < n; i++) {
        data[i] = fft_complex(
            static_cast<float>(rand()) / RAND_MAX - 0.5f,
            static_cast<float>(rand()) / RAND_MAX - 0.5f
        );
    }
}

static BenchResult benchmark_size(FFTBackend* backend, int n) {
    BenchResult result;
    result.size = n;

    // Allocate aligned buffers
    float* real_in = static_cast<float*>(backend->alloc(sizeof(float) * n));
    fft_complex* complex_out = static_cast<fft_complex*>(backend->alloc(sizeof(fft_complex) * (n/2 + 1)));
    fft_complex* complex_in = static_cast<fft_complex*>(backend->alloc(sizeof(fft_complex) * n));
    fft_complex* complex_tmp = static_cast<fft_complex*>(backend->alloc(sizeof(fft_complex) * n));

    fill_random(real_in, n);
    fill_random_complex(complex_in, n);

    // Create plans
    FFTPlanHandle plan_r2c = backend->plan_r2c(n, real_in, complex_out);
    FFTPlanHandle plan_c2c_fwd = backend->plan_c2c(n, complex_in, complex_tmp, FFTDirection::Forward);
    FFTPlanHandle plan_c2c_bwd = backend->plan_c2c(n, complex_tmp, complex_in, FFTDirection::Backward);

    // Warmup
    for (int i = 0; i < WARMUP_ITERS; i++) {
        backend->execute_r2c(plan_r2c, real_in, complex_out);
        backend->execute_c2c(plan_c2c_fwd, complex_in, complex_tmp);
        backend->execute_c2c(plan_c2c_bwd, complex_tmp, complex_in);
    }

    // Benchmark R2C
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < BENCH_ITERS; i++) {
        backend->execute_r2c(plan_r2c, real_in, complex_out);
    }
    auto end = std::chrono::high_resolution_clock::now();
    result.r2c_time_us = std::chrono::duration<double, std::micro>(end - start).count() / BENCH_ITERS;

    // Benchmark C2C Forward
    start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < BENCH_ITERS; i++) {
        backend->execute_c2c(plan_c2c_fwd, complex_in, complex_tmp);
    }
    end = std::chrono::high_resolution_clock::now();
    result.c2c_fwd_time_us = std::chrono::duration<double, std::micro>(end - start).count() / BENCH_ITERS;

    // Benchmark C2C Backward
    start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < BENCH_ITERS; i++) {
        backend->execute_c2c(plan_c2c_bwd, complex_tmp, complex_in);
    }
    end = std::chrono::high_resolution_clock::now();
    result.c2c_bwd_time_us = std::chrono::duration<double, std::micro>(end - start).count() / BENCH_ITERS;

    // Cleanup
    backend->destroy_plan(plan_r2c);
    backend->destroy_plan(plan_c2c_fwd);
    backend->destroy_plan(plan_c2c_bwd);
    backend->free(real_in);
    backend->free(complex_out);
    backend->free(complex_in);
    backend->free(complex_tmp);

    return result;
}

void run_fft_backend_benchmark() {
    FFTBackend* backend = getFFTBackend();

    printf("\n");
    printf("=======================================================================\n");
    printf("                    FFT BACKEND BENCHMARK\n");
    printf("=======================================================================\n");
    printf("Backend: %s\n", backend->name());
    printf("Iterations: %d (warmup: %d)\n", BENCH_ITERS, WARMUP_ITERS);
    printf("-----------------------------------------------------------------------\n");
    printf("  Size  |   R2C (µs)   |  C2C Fwd (µs) |  C2C Bwd (µs) |  Total (µs)\n");
    printf("-----------------------------------------------------------------------\n");

    double total_time = 0;
    for (int i = 0; i < NUM_SIZES; i++) {
        BenchResult r = benchmark_size(backend, TEST_SIZES[i]);
        double row_total = r.r2c_time_us + r.c2c_fwd_time_us + r.c2c_bwd_time_us;
        total_time += row_total;
        printf("  %4d  |    %7.2f   |     %7.2f   |     %7.2f   |    %7.2f\n",
               r.size, r.r2c_time_us, r.c2c_fwd_time_us, r.c2c_bwd_time_us, row_total);
    }
    printf("-----------------------------------------------------------------------\n");
    printf("Total time per iteration: %.2f µs\n", total_time);
    printf("=======================================================================\n\n");
}

// Can be called from unit tests or standalone
extern "C" void fft_backend_benchmark() {
    run_fft_backend_benchmark();
}

#ifdef FFT_BENCHMARK_MAIN
int main() {
    run_fft_backend_benchmark();
    return 0;
}
#endif
