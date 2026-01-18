#include "fft_mt_r2iq.h"
#include "config.h"
#include "fftw3.h"
#include "RadioHandler.h"

// Include ARM NEON intrinsics at file scope
#include <arm_neon.h>

// NEON-optimized convert_float: int16 to float conversion
// Processes 8 elements per iteration (NEON 128-bit register)
template<bool rand>
static inline void convert_float_neon(const int16_t* __restrict input, float* __restrict output, int size)
{
    int m = 0;

    if constexpr (!rand) {
        // Fast path: no randomization, pure SIMD
        // Process 8 int16 -> 8 floats per iteration
        for (; m + 7 < size; m += 8)
        {
            // Load 8 int16 values (128 bits)
            int16x8_t in16 = vld1q_s16(input + m);

            // Split into low and high halves for conversion
            int16x4_t lo16 = vget_low_s16(in16);
            int16x4_t hi16 = vget_high_s16(in16);

            // Convert int16 to int32 (sign extend)
            int32x4_t lo32 = vmovl_s16(lo16);
            int32x4_t hi32 = vmovl_s16(hi16);

            // Convert int32 to float
            float32x4_t lof = vcvtq_f32_s32(lo32);
            float32x4_t hif = vcvtq_f32_s32(hi32);

            // Store 8 floats
            vst1q_f32(output + m, lof);
            vst1q_f32(output + m + 4, hif);
        }
    }
    else {
        // Randomization path: need to check LSB and potentially flip bits
        // XOR mask for randomization: -2 = 0xFFFE
        uint16x8_t xor_mask = vdupq_n_u16(0xFFFE);
        uint16x8_t one_mask = vdupq_n_u16(1);

        for (; m + 7 < size; m += 8)
        {
            int16x8_t in16 = vld1q_s16(input + m);

            // Check which values have LSB set: (input & 1) != 0
            uint16x8_t in16_u = vreinterpretq_u16_s16(in16);
            uint16x8_t lsb = vandq_u16(in16_u, one_mask);
            uint16x8_t needs_xor = vceqq_u16(lsb, one_mask);

            // Compute XOR result for values that need it
            uint16x8_t xored = veorq_u16(in16_u, xor_mask);

            // Blend: use xored where needs_xor is true, else use original
            uint16x8_t result_u = vbslq_u16(needs_xor, xored, in16_u);
            int16x8_t result = vreinterpretq_s16_u16(result_u);

            // Convert to float
            int16x4_t lo16 = vget_low_s16(result);
            int16x4_t hi16 = vget_high_s16(result);
            int32x4_t lo32 = vmovl_s16(lo16);
            int32x4_t hi32 = vmovl_s16(hi16);
            float32x4_t lof = vcvtq_f32_s32(lo32);
            float32x4_t hif = vcvtq_f32_s32(hi32);

            vst1q_f32(output + m, lof);
            vst1q_f32(output + m + 4, hif);
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

// NEON-optimized shift_freq: complex multiplication
// dest[m] = source1[m] * source2[m] (complex multiply)
// Processes 2 complex values (4 floats) per iteration
static inline void shift_freq_neon(fftwf_complex* __restrict dest,
                                   const fftwf_complex* __restrict source1,
                                   const fftwf_complex* __restrict source2,
                                   int start, int end)
{
    int m = start;

    // Sign mask for complex multiplication: {-1.0, 1.0, -1.0, 1.0}
    // This gives us [ac-bd, ad+bc, ...] when combined properly
    const float sign_array[4] = {-1.0f, 1.0f, -1.0f, 1.0f};
    float32x4_t sign_mask = vld1q_f32(sign_array);

    // Process 2 complex numbers at a time (4 floats)
    for (; m + 1 < end; m += 2)
    {
        // Load 2 complex from source1: [r0,i0,r1,i1]
        float32x4_t s1 = vld1q_f32(reinterpret_cast<const float*>(&source1[m]));
        // Load 2 complex from source2
        float32x4_t s2 = vld1q_f32(reinterpret_cast<const float*>(&source2[m]));

        // Complex multiply: (a+bi)(c+di) = (ac-bd) + (ad+bc)i
        // s1 = [a0,b0,a1,b1]
        // s2 = [c0,d0,c1,d1]

        // Extract real and imaginary parts
        float32x2_t s1_lo = vget_low_f32(s1);   // [a0, b0]
        float32x2_t s1_hi = vget_high_f32(s1);  // [a1, b1]

        // Duplicate real parts: [a0,a0,a1,a1]
        float32x4_t s1_rr = vcombine_f32(vdup_lane_f32(s1_lo, 0), vdup_lane_f32(s1_hi, 0));

        // Duplicate imaginary parts: [b0,b0,b1,b1]
        float32x4_t s1_ii = vcombine_f32(vdup_lane_f32(s1_lo, 1), vdup_lane_f32(s1_hi, 1));

        // Swap pairs in s2: [d0,c0,d1,c1]
        float32x4_t s2_ir = vrev64q_f32(s2);

        // Compute: s1_rr * s2 = [a*c, a*d, ...]
        float32x4_t ac_ad = vmulq_f32(s1_rr, s2);

        // Compute: s1_ii * s2_ir = [b*d, b*c, ...]
        float32x4_t bd_bc = vmulq_f32(s1_ii, s2_ir);

        // Use FMA (fused multiply-add) for better performance and precision
        // Result: ac_ad + (bd_bc * sign_mask) = [ac-bd, ad+bc, ...]
        float32x4_t result = vfmaq_f32(ac_ad, bd_bc, sign_mask);

        vst1q_f32(reinterpret_cast<float*>(&dest[m]), result);
    }

    // Scalar cleanup
    for (; m < end; m++)
    {
        dest[m][0] = source1[m][0] * source2[m][0] - source1[m][1] * source2[m][1];
        dest[m][1] = source1[m][1] * source2[m][0] + source1[m][0] * source2[m][1];
    }
}

// NEON-optimized copy: copy complex values with optional conjugate
// Processes 2 complex values (4 floats) per iteration
template<bool flip>
static inline void copy_neon(fftwf_complex* __restrict dest,
                             const fftwf_complex* __restrict source,
                             int count)
{
    int i = 0;

    if constexpr (flip) {
        // Need to negate imaginary parts
        // Sign mask: 0x00000000, 0x80000000, 0x00000000, 0x80000000
        const uint32_t sign_array[4] = {0x00000000, 0x80000000, 0x00000000, 0x80000000};
        uint32x4_t sign_mask = vld1q_u32(sign_array);

        // Process 4 complex at a time (2x unroll)
        for (; i + 3 < count; i += 4)
        {
            float32x4_t v0 = vld1q_f32(reinterpret_cast<const float*>(&source[i]));
            float32x4_t v1 = vld1q_f32(reinterpret_cast<const float*>(&source[i + 2]));

            // XOR with sign mask flips sign of odd elements (imaginary parts)
            uint32x4_t v0_u = vreinterpretq_u32_f32(v0);
            uint32x4_t v1_u = vreinterpretq_u32_f32(v1);
            v0_u = veorq_u32(v0_u, sign_mask);
            v1_u = veorq_u32(v1_u, sign_mask);
            v0 = vreinterpretq_f32_u32(v0_u);
            v1 = vreinterpretq_f32_u32(v1_u);

            vst1q_f32(reinterpret_cast<float*>(&dest[i]), v0);
            vst1q_f32(reinterpret_cast<float*>(&dest[i + 2]), v1);
        }

        // Process 2 complex
        if (i + 1 < count)
        {
            float32x4_t v = vld1q_f32(reinterpret_cast<const float*>(&source[i]));
            uint32x4_t v_u = vreinterpretq_u32_f32(v);
            v_u = veorq_u32(v_u, sign_mask);
            v = vreinterpretq_f32_u32(v_u);
            vst1q_f32(reinterpret_cast<float*>(&dest[i]), v);
            i += 2;
        }

        // Scalar cleanup
        for (; i < count; i++)
        {
            dest[i][0] = source[i][0];
            dest[i][1] = -source[i][1];
        }
    }
    else {
        // Simple copy - for large copies, memcpy is already optimal on Apple Silicon
        if (count >= 8) {
            memcpy(dest, source, count * sizeof(fftwf_complex));
        } else {
            // For small counts, use SIMD
            for (; i + 3 < count; i += 4)
            {
                float32x4_t v0 = vld1q_f32(reinterpret_cast<const float*>(&source[i]));
                float32x4_t v1 = vld1q_f32(reinterpret_cast<const float*>(&source[i + 2]));

                vst1q_f32(reinterpret_cast<float*>(&dest[i]), v0);
                vst1q_f32(reinterpret_cast<float*>(&dest[i + 2]), v1);
            }

            // Process 2 complex
            if (i + 1 < count)
            {
                float32x4_t v = vld1q_f32(reinterpret_cast<const float*>(&source[i]));
                vst1q_f32(reinterpret_cast<float*>(&dest[i]), v);
                i += 2;
            }

            // Scalar cleanup
            for (; i < count; i++)
            {
                dest[i][0] = source[i][0];
                dest[i][1] = source[i][1];
            }
        }
    }
}

// NEON thread function that uses the optimized implementations
void * fft_mt_r2iq::r2iqThreadf_neon(r2iqThreadArg *th)
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

		// Use NEON-optimized convert_float
		if (!this->getRand()) {
			convert_float_neon<false>(endloop, inloop, halfFft);
			convert_float_neon<false>(dataADC, inloop + halfFft, transferSamples);
		} else {
			convert_float_neon<true>(endloop, inloop, halfFft);
			convert_float_neon<true>(dataADC, inloop + halfFft, transferSamples);
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

			// Use NEON-optimized shift_freq
			shift_freq_neon(th->inFreqTmp, source, filter, 0, count);
			if (mfft / 2 != count)
				memset(th->inFreqTmp[count], 0, sizeof(float) * 2 * (mfft / 2 - count));

			shift_freq_neon(dest, source2, filter2, start, mfft / 2);
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

				// Use NEON-optimized copy
				if (lsb) {
					if (k == 0)
						copy_neon<true>(pout, &th->inFreqTmp[mfft / 4], mfft / 2);
					else
						copy_neon<true>(pout + mfft / 2 + (3 * mfft / 4) * (k - 1), &th->inFreqTmp[0], (3 * mfft / 4));
				} else {
					if (k == 0)
						copy_neon<false>(pout, &th->inFreqTmp[mfft / 4], mfft / 2);
					else
						copy_neon<false>(pout + mfft / 2 + (3 * mfft / 4) * (k - 1), &th->inFreqTmp[0], (3 * mfft / 4));
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
