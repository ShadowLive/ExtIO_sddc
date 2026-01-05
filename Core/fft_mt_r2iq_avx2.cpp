#include "fft_mt_r2iq.h"
#include "config.h"
#include "fftw3.h"
#include "RadioHandler.h"

// Include AVX2 intrinsics at file scope (not inside function)
#include <immintrin.h>

// AVX2-optimized convert_float: int16 to float conversion
// Processes 16 elements per iteration (2x AVX2 registers)
template<bool rand>
static inline void convert_float_avx2(const int16_t* __restrict input, float* __restrict output, int size)
{
    int m = 0;

    if constexpr (!rand) {
        // Fast path: no randomization, pure SIMD
        // Process 16 int16 -> 16 floats per iteration
        for (; m + 15 < size; m += 16)
        {
            // Load 16 int16 values (256 bits)
            __m256i in16 = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(input + m));

            // Split into low and high 128-bit halves
            __m128i lo16 = _mm256_castsi256_si128(in16);
            __m128i hi16 = _mm256_extracti128_si256(in16, 1);

            // Convert int16 to int32 (sign extend)
            __m256i lo32 = _mm256_cvtepi16_epi32(lo16);
            __m256i hi32 = _mm256_cvtepi16_epi32(hi16);

            // Convert int32 to float
            __m256 lof = _mm256_cvtepi32_ps(lo32);
            __m256 hif = _mm256_cvtepi32_ps(hi32);

            // Store 16 floats
            _mm256_storeu_ps(output + m, lof);
            _mm256_storeu_ps(output + m + 8, hif);
        }

        // Process remaining 8 elements if possible
        if (m + 7 < size)
        {
            __m128i in16 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(input + m));
            __m256i in32 = _mm256_cvtepi16_epi32(in16);
            __m256 outf = _mm256_cvtepi32_ps(in32);
            _mm256_storeu_ps(output + m, outf);
            m += 8;
        }
    }
    else {
        // Randomization path: need to check LSB and potentially flip bits
        // XOR mask for randomization: -2 = 0xFFFFFFFE
        __m256i xor_mask = _mm256_set1_epi16(-2);
        __m256i one_mask = _mm256_set1_epi16(1);

        for (; m + 15 < size; m += 16)
        {
            __m256i in16 = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(input + m));

            // Check which values have LSB set: (input & 1) != 0
            __m256i lsb = _mm256_and_si256(in16, one_mask);
            __m256i needs_xor = _mm256_cmpeq_epi16(lsb, one_mask);

            // Compute XOR result for values that need it
            __m256i xored = _mm256_xor_si256(in16, xor_mask);

            // Blend: use xored where needs_xor is true, else use original
            __m256i result = _mm256_blendv_epi8(in16, xored, needs_xor);

            // Convert to float
            __m128i lo16 = _mm256_castsi256_si128(result);
            __m128i hi16 = _mm256_extracti128_si256(result, 1);
            __m256i lo32 = _mm256_cvtepi16_epi32(lo16);
            __m256i hi32 = _mm256_cvtepi16_epi32(hi16);
            __m256 lof = _mm256_cvtepi32_ps(lo32);
            __m256 hif = _mm256_cvtepi32_ps(hi32);

            _mm256_storeu_ps(output + m, lof);
            _mm256_storeu_ps(output + m + 8, hif);
        }
    }

    // Scalar cleanup for remaining elements
    for (; m < size; m++)
    {
        int16_t val;
        if (rand && (input[m] & 1))
            val = input[m] ^ (-2);
        else
            val = input[m];
        output[m] = static_cast<float>(val);
    }
}

// AVX2-optimized shift_freq: complex multiplication
// dest[m] = source1[m] * source2[m] (complex multiply)
// Processes 4 complex values (8 floats) per iteration
static inline void shift_freq_avx2(fftwf_complex* __restrict dest,
                                   const fftwf_complex* __restrict source1,
                                   const fftwf_complex* __restrict source2,
                                   int start, int end)
{
    int m = start;

    // Process 4 complex numbers at a time (8 floats)
    for (; m + 3 < end; m += 4)
    {
        // Load 4 complex from source1: [r0,i0,r1,i1,r2,i2,r3,i3]
        __m256 s1 = _mm256_loadu_ps(reinterpret_cast<const float*>(&source1[m]));
        // Load 4 complex from source2
        __m256 s2 = _mm256_loadu_ps(reinterpret_cast<const float*>(&source2[m]));

        // Complex multiply: (a+bi)(c+di) = (ac-bd) + (ad+bc)i
        // s1 = [a0,b0,a1,b1,a2,b2,a3,b3]
        // s2 = [c0,d0,c1,d1,c2,d2,c3,d3]

        // Shuffle to get: s1_rr = [a0,a0,a1,a1,a2,a2,a3,a3]
        __m256 s1_rr = _mm256_moveldup_ps(s1);
        // Shuffle to get: s1_ii = [b0,b0,b1,b1,b2,b2,b3,b3]
        __m256 s1_ii = _mm256_movehdup_ps(s1);

        // s2_ir = [d0,c0,d1,c1,...] (swap pairs)
        __m256 s2_ir = _mm256_permute_ps(s2, 0xB1); // 0xB1 = 10 11 00 01

        // Compute: s1_rr * s2 = [a*c, a*d, ...]
        __m256 ac_ad = _mm256_mul_ps(s1_rr, s2);
        // Compute: s1_ii * s2_ir = [b*d, b*c, ...]
        __m256 bd_bc = _mm256_mul_ps(s1_ii, s2_ir);

        // Result: [ac-bd, ad+bc, ...] using addsub
        // addsub: subtract even indices, add odd indices
        __m256 result = _mm256_addsub_ps(ac_ad, bd_bc);

        _mm256_storeu_ps(reinterpret_cast<float*>(&dest[m]), result);
    }

    // Scalar cleanup
    for (; m < end; m++)
    {
        dest[m][0] = source1[m][0] * source2[m][0] - source1[m][1] * source2[m][1];
        dest[m][1] = source1[m][1] * source2[m][0] + source1[m][0] * source2[m][1];
    }
}

// AVX2-optimized copy: copy complex values with optional conjugate
// Processes 4 complex values (8 floats) per iteration
template<bool flip>
static inline void copy_avx2(fftwf_complex* __restrict dest,
                             const fftwf_complex* __restrict source,
                             int count)
{
    int i = 0;

    if constexpr (flip) {
        // Need to negate imaginary parts
        // Sign mask: negate odd elements (imaginary parts)
        // [+0, -0, +0, -0, +0, -0, +0, -0]
        __m256 sign_mask = _mm256_set_ps(-0.0f, 0.0f, -0.0f, 0.0f, -0.0f, 0.0f, -0.0f, 0.0f);

        // Process 8 complex at a time (2x unroll)
        for (; i + 7 < count; i += 8)
        {
            __m256 v0 = _mm256_loadu_ps(reinterpret_cast<const float*>(&source[i]));
            __m256 v1 = _mm256_loadu_ps(reinterpret_cast<const float*>(&source[i + 4]));

            // XOR with sign mask flips sign of odd elements
            v0 = _mm256_xor_ps(v0, sign_mask);
            v1 = _mm256_xor_ps(v1, sign_mask);

            _mm256_storeu_ps(reinterpret_cast<float*>(&dest[i]), v0);
            _mm256_storeu_ps(reinterpret_cast<float*>(&dest[i + 4]), v1);
        }

        // Process 4 complex
        if (i + 3 < count)
        {
            __m256 v = _mm256_loadu_ps(reinterpret_cast<const float*>(&source[i]));
            v = _mm256_xor_ps(v, sign_mask);
            _mm256_storeu_ps(reinterpret_cast<float*>(&dest[i]), v);
            i += 4;
        }

        // Scalar cleanup
        for (; i < count; i++)
        {
            dest[i][0] = source[i][0];
            dest[i][1] = -source[i][1];
        }
    }
    else {
        // Simple copy - use streaming stores for large copies
        // Process 8 complex at a time (2x unroll)
        for (; i + 7 < count; i += 8)
        {
            __m256 v0 = _mm256_loadu_ps(reinterpret_cast<const float*>(&source[i]));
            __m256 v1 = _mm256_loadu_ps(reinterpret_cast<const float*>(&source[i + 4]));

            _mm256_storeu_ps(reinterpret_cast<float*>(&dest[i]), v0);
            _mm256_storeu_ps(reinterpret_cast<float*>(&dest[i + 4]), v1);
        }

        // Process 4 complex
        if (i + 3 < count)
        {
            __m256 v = _mm256_loadu_ps(reinterpret_cast<const float*>(&source[i]));
            _mm256_storeu_ps(reinterpret_cast<float*>(&dest[i]), v);
            i += 4;
        }

        // Scalar cleanup
        for (; i < count; i++)
        {
            dest[i][0] = source[i][0];
            dest[i][1] = source[i][1];
        }
    }
}

// Now include the implementation that uses these AVX2 functions
void * fft_mt_r2iq::r2iqThreadf_avx2(r2iqThreadArg *th)
{
	const int decimate = this->mdecimation;
	const int mfft = this->mfftdim[decimate];
	const fftwf_complex* filter = filterHw[decimate];
	const bool lsb = this->getSideband();
	const auto filter2 = &filter[halfFft - mfft / 2];

	fftwf_plan local_plan_f2t_c2c = plans_f2t_c2c[decimate];
	const int decimateMask = (1 << decimate) - 1;

	while (r2iqOn) {
		const int16_t *dataADC;
		const int16_t *endloop;
		uint64_t mySeq;

		const int _mtunebin = this->mtunebin;

		{
			std::unique_lock<std::mutex> lk(mutexR2iqControl);
			dataADC = inputbuffer->getReadPtr();
			if (!r2iqOn) return 0;

			mySeq = inputSeq.fetch_add(1);
			this->bufIdx = (this->bufIdx + 1) % QUEUE_SIZE;
			endloop = inputbuffer->peekReadPtr(-1) + transferSamples - halfFft;
		}

		auto inloop = th->ADCinTime;

		// Use AVX2-optimized convert_float
		if (!this->getRand()) {
			convert_float_avx2<false>(endloop, inloop, halfFft);
			convert_float_avx2<false>(dataADC, inloop + halfFft, transferSamples);
		} else {
			convert_float_avx2<true>(endloop, inloop, halfFft);
			convert_float_avx2<true>(dataADC, inloop + halfFft, transferSamples);
		}

		dataADC = nullptr;
		inputbuffer->ReadDone();

		const auto count = std::min(mfft / 2, halfFft - _mtunebin);
		const auto source = &th->ADCinFreq[_mtunebin];
		const auto start = std::max(0, mfft / 2 - _mtunebin);
		const auto source2 = &th->ADCinFreq[_mtunebin - mfft / 2];
		const auto dest = &th->inFreqTmp[mfft / 2];

		for (int k = 0; k < fftPerBuf; k++)
		{
			fftwf_execute_dft_r2c(plan_t2f_r2c, th->ADCinTime + (3 * halfFft / 2) * k, th->ADCinFreq);

			// Use AVX2-optimized shift_freq
			shift_freq_avx2(th->inFreqTmp, source, filter, 0, count);
			if (mfft / 2 != count)
				memset(th->inFreqTmp[count], 0, sizeof(float) * 2 * (mfft / 2 - count));

			shift_freq_avx2(dest, source2, filter2, start, mfft / 2);
			if (start != 0)
				memset(th->inFreqTmp[mfft / 2], 0, sizeof(float) * 2 * start);

			fftwf_execute_dft(local_plan_f2t_c2c, th->inFreqTmp, th->inFreqTmp);

			{
				std::unique_lock<std::mutex> lk(outputMutex);

				if (k == 0) {
					outputCV.wait(lk, [&] { return outputWriteTurn.load() == mySeq || !r2iqOn; });
					if (!r2iqOn) return 0;

					int myDecimateCount = static_cast<int>(mySeq & decimateMask);
					if (myDecimateCount == 0) {
						sharedPout = (fftwf_complex*)outputbuffer->getWritePtr();
					}
				}

				int myDecimateCount = static_cast<int>(mySeq & decimateMask);
				fftwf_complex* pout = sharedPout;
				for (int i = 0; i < myDecimateCount; i++) {
					pout += mfft / 2 + (3 * mfft / 4) * (fftPerBuf - 1);
				}

				// Use AVX2-optimized copy
				if (lsb) {
					if (k == 0)
						copy_avx2<true>(pout, &th->inFreqTmp[mfft / 4], mfft / 2);
					else
						copy_avx2<true>(pout + mfft / 2 + (3 * mfft / 4) * (k - 1), &th->inFreqTmp[0], (3 * mfft / 4));
				} else {
					if (k == 0)
						copy_avx2<false>(pout, &th->inFreqTmp[mfft / 4], mfft / 2);
					else
						copy_avx2<false>(pout + mfft / 2 + (3 * mfft / 4) * (k - 1), &th->inFreqTmp[0], (3 * mfft / 4));
				}

				if (k == fftPerBuf - 1) {
					if (myDecimateCount == decimateMask) {
						outputbuffer->WriteDone();
						sharedPout = nullptr;
					}
					outputWriteTurn.fetch_add(1);
					outputCV.notify_all();
				}
			}
		}
	}
	return 0;
}
