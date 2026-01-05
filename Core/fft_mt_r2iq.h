#pragma once

#include "r2iq.h"
#include "fftw3.h"
#include "config.h"
#include <algorithm>
#include <string.h>

// Enable SSE optimizations only when compiling with AVX or higher
// (AVX implies SSE3/SSE4 support and proper compiler flags)
#if defined(__AVX__) || defined(__AVX2__) || defined(__AVX512F__)
#include <emmintrin.h>  // SSE2
#include <pmmintrin.h>  // SSE3 (for _mm_addsub_ps)
#include <smmintrin.h>  // SSE4.1
#define USE_SSE_OPTIMIZATIONS 1
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define USE_NEON_OPTIMIZATIONS 1
#else
#define USE_SSE_OPTIMIZATIONS 0
#define USE_NEON_OPTIMIZATIONS 0
#endif

// use up to this many threads for FFT processing
#define N_MAX_R2IQ_THREADS 4
#define PRINT_INPUT_RANGE  0

static const int halfFft = FFTN_R_ADC / 2;    // half the size of the first fft at ADC 64Msps real rate (2048)
static const int fftPerBuf = transferSize / sizeof(short) / (3 * halfFft / 2) + 1; // number of ffts per buffer with 256|768 overlap

class fft_mt_r2iq : public r2iqControlClass
{
public:
    fft_mt_r2iq();
    virtual ~fft_mt_r2iq();

    float setFreqOffset(float offset);

    void Init(float gain, ringbuffer<int16_t>* buffers, ringbuffer<float>* obuffers);
    void TurnOn();
    void TurnOff(void);
    bool IsOn(void);

protected:

    template<bool rand> void convert_float(const int16_t *input, float* output, int size)
    {
#if USE_SSE_OPTIMIZATIONS
        int m = 0;
        if (!rand) {
            // Fast path: no randomization, process 8 samples at a time
            for (; m + 7 < size; m += 8) {
                // Load 8 int16 values
                __m128i v16 = _mm_loadu_si128((const __m128i*)&input[m]);
                // Convert lower 4 int16 to int32, then to float
                __m128i v32_lo = _mm_cvtepi16_epi32(v16);
                __m128 vf_lo = _mm_cvtepi32_ps(v32_lo);
                // Convert upper 4 int16 to int32, then to float
                __m128i v32_hi = _mm_cvtepi16_epi32(_mm_srli_si128(v16, 8));
                __m128 vf_hi = _mm_cvtepi32_ps(v32_hi);
                // Store results
                _mm_storeu_ps(&output[m], vf_lo);
                _mm_storeu_ps(&output[m + 4], vf_hi);
            }
        } else {
            // Randomization path: process 8 samples at a time with conditional XOR
            const __m128i ones = _mm_set1_epi16(1);
            const __m128i xor_mask = _mm_set1_epi16(-2);
            for (; m + 7 < size; m += 8) {
                __m128i v16 = _mm_loadu_si128((const __m128i*)&input[m]);
                // Check which values have LSB set
                __m128i lsb = _mm_and_si128(v16, ones);
                __m128i need_xor = _mm_cmpeq_epi16(lsb, ones);
                // XOR only where needed
                __m128i xored = _mm_xor_si128(v16, _mm_and_si128(need_xor, xor_mask));
                // Convert to float
                __m128i v32_lo = _mm_cvtepi16_epi32(xored);
                __m128 vf_lo = _mm_cvtepi32_ps(v32_lo);
                __m128i v32_hi = _mm_cvtepi16_epi32(_mm_srli_si128(xored, 8));
                __m128 vf_hi = _mm_cvtepi32_ps(v32_hi);
                _mm_storeu_ps(&output[m], vf_lo);
                _mm_storeu_ps(&output[m + 4], vf_hi);
            }
        }
        // Handle remaining samples
        for (; m < size; m++) {
            int16_t val = input[m];
            if (rand && (val & 1)) {
                val = val ^ (-2);
            }
            output[m] = float(val);
        }
#else
        // Scalar fallback
        for (int m = 0; m < size; m++) {
            int16_t val = input[m];
            if (rand && (val & 1)) {
                val = val ^ (-2);
            }
            output[m] = float(val);
        }
#endif
    }

    void shift_freq(fftwf_complex* dest, const fftwf_complex* source1, const fftwf_complex* source2, int start, int end)
    {
#if USE_SSE_OPTIMIZATIONS
        int m = start;
        // Process 2 complex numbers at a time (4 floats = 128 bits)
        for (; m + 1 < end; m += 2) {
            // Load 2 complex numbers from each source: [r0, i0, r1, i1]
            __m128 s1 = _mm_loadu_ps((const float*)&source1[m]);
            __m128 s2 = _mm_loadu_ps((const float*)&source2[m]);

            // Complex multiply: (a+bi)(c+di) = (ac-bd) + (ad+bc)i
            // s1 = [a0, b0, a1, b1], s2 = [c0, d0, c1, d1]

            // Shuffle to get: [a0, a0, a1, a1] and [b0, b0, b1, b1]
            __m128 s1_rr = _mm_shuffle_ps(s1, s1, _MM_SHUFFLE(2, 2, 0, 0));  // [a0, a0, a1, a1]
            __m128 s1_ii = _mm_shuffle_ps(s1, s1, _MM_SHUFFLE(3, 3, 1, 1));  // [b0, b0, b1, b1]

            // Multiply: [a*c, a*d, a*c, a*d] and [b*c, b*d, b*c, b*d]
            __m128 ac_ad = _mm_mul_ps(s1_rr, s2);  // [a0*c0, a0*d0, a1*c1, a1*d1]
            __m128 bc_bd = _mm_mul_ps(s1_ii, s2);  // [b0*c0, b0*d0, b1*c1, b1*d1]

            // Shuffle bc_bd to get [b*d, b*c, b*d, b*c]
            __m128 bd_bc = _mm_shuffle_ps(bc_bd, bc_bd, _MM_SHUFFLE(2, 3, 0, 1));

            // addsub: [ac-bd, ad+bc, ac-bd, ad+bc]
            __m128 result = _mm_addsub_ps(ac_ad, bd_bc);

            _mm_storeu_ps((float*)&dest[m], result);
        }
        // Handle remaining single complex number
        for (; m < end; m++) {
            dest[m][0] = source1[m][0] * source2[m][0] - source1[m][1] * source2[m][1];
            dest[m][1] = source1[m][1] * source2[m][0] + source1[m][0] * source2[m][1];
        }
#else
        for (int m = start; m < end; m++) {
            // besides circular shift, do complex multiplication with the lowpass filter's spectrum
            dest[m][0] = source1[m][0] * source2[m][0] - source1[m][1] * source2[m][1];
            dest[m][1] = source1[m][1] * source2[m][0] + source1[m][0] * source2[m][1];
        }
#endif
    }

    template<bool flip> void copy(fftwf_complex* dest, const fftwf_complex* source, int count)
    {
        if (!flip) {
            // Non-flip case: use memcpy for maximum efficiency
            memcpy(dest, source, count * sizeof(fftwf_complex));
        } else {
#if USE_SSE_OPTIMIZATIONS
            // Flip case: negate imaginary part using SSE
            // Sign mask: [+0, -0, +0, -0] to negate only Q components
            const __m128 sign_mask = _mm_set_ps(-0.0f, 0.0f, -0.0f, 0.0f);
            int i = 0;
            // Process 2 complex numbers at a time
            for (; i + 1 < count; i += 2) {
                __m128 v = _mm_loadu_ps((const float*)&source[i]);
                __m128 flipped = _mm_xor_ps(v, sign_mask);
                _mm_storeu_ps((float*)&dest[i], flipped);
            }
            // Handle remaining single complex number
            for (; i < count; i++) {
                dest[i][0] = source[i][0];
                dest[i][1] = -source[i][1];
            }
#else
            for (int i = 0; i < count; i++) {
                dest[i][0] = source[i][0];
                dest[i][1] = -source[i][1];
            }
#endif
        }
    }

private:
    ringbuffer<int16_t>* inputbuffer;    // pointer to input buffers
    ringbuffer<float>* outputbuffer;    // pointer to ouput buffers
    int bufIdx;         // index to next buffer to be processed
    r2iqThreadArg* lastThread;

    float GainScale;
    int mfftdim [NDECIDX]; // FFT N dimensions: mfftdim[k] = halfFft / 2^k
    int mtunebin;

    void *r2iqThreadf(r2iqThreadArg *th);   // thread function

    void * r2iqThreadf_def(r2iqThreadArg *th);
    void * r2iqThreadf_avx(r2iqThreadArg *th);
    void * r2iqThreadf_avx2(r2iqThreadArg *th);
    void * r2iqThreadf_avx512(r2iqThreadArg *th);
    void * r2iqThreadf_neon(r2iqThreadArg *th);

    fftwf_complex **filterHw;       // Hw complex to each decimation ratio

	fftwf_plan plan_t2f_r2c;          // fftw plan buffers Freq to Time complex to complex per decimation ratio
	fftwf_plan *plan_f2t_c2c;          // fftw plan buffers Time to Freq real to complex per buffer
	fftwf_plan plans_f2t_c2c[NDECIDX];

    uint32_t processor_count;
    r2iqThreadArg* threadArgs[N_MAX_R2IQ_THREADS];
    std::mutex mutexR2iqControl;                   // r2iq control lock
    std::thread r2iq_thread[N_MAX_R2IQ_THREADS]; // thread pointers
};

// assure, that ADC is not oversteered?
struct r2iqThreadArg {

	r2iqThreadArg()
	{
#if PRINT_INPUT_RANGE
		MinMaxBlockCount = 0;
		MinValue = 0;
		MaxValue = 0;
#endif
	}

	float *ADCinTime;                // point to each threads input buffers [nftt][n]
	fftwf_complex *ADCinFreq;         // buffers in frequency
	fftwf_complex *inFreqTmp;         // tmp decimation output buffers (after tune shift)
#if PRINT_INPUT_RANGE
	int MinMaxBlockCount;
	int16_t MinValue;
	int16_t MaxValue;
#endif
};
