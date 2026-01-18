#include "CppUnitTestFramework.hpp"
#include <vector>
#include <cmath>
#include <cstring>

#ifdef __ARM_NEON
#include <arm_neon.h>

// Copy the NEON implementation functions for testing
// These are the same as in fft_mt_r2iq_neon.cpp

template<bool rand>
static inline void convert_float_neon(const int16_t* __restrict input, float* __restrict output, int size)
{
    int m = 0;

    if constexpr (!rand) {
        for (; m + 7 < size; m += 8)
        {
            int16x8_t in16 = vld1q_s16(input + m);
            int16x4_t lo16 = vget_low_s16(in16);
            int16x4_t hi16 = vget_high_s16(in16);
            int32x4_t lo32 = vmovl_s16(lo16);
            int32x4_t hi32 = vmovl_s16(hi16);
            float32x4_t lof = vcvtq_f32_s32(lo32);
            float32x4_t hif = vcvtq_f32_s32(hi32);
            vst1q_f32(output + m, lof);
            vst1q_f32(output + m + 4, hif);
        }
    }
    else {
        uint16x8_t xor_mask = vdupq_n_u16(0xFFFE);
        uint16x8_t one_mask = vdupq_n_u16(1);

        for (; m + 7 < size; m += 8)
        {
            int16x8_t in16 = vld1q_s16(input + m);
            uint16x8_t in16_u = vreinterpretq_u16_s16(in16);
            uint16x8_t lsb = vandq_u16(in16_u, one_mask);
            uint16x8_t needs_xor = vceqq_u16(lsb, one_mask);
            uint16x8_t xored = veorq_u16(in16_u, xor_mask);
            uint16x8_t result_u = vbslq_u16(needs_xor, xored, in16_u);
            int16x8_t result = vreinterpretq_s16_u16(result_u);

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

template<bool rand>
static inline void convert_float_scalar(const int16_t* __restrict input, float* __restrict output, int size)
{
    for (int m = 0; m < size; m++)
    {
        int16_t val;
        if (rand && (input[m] & 1))
            val = input[m] ^ (-2);
        else
            val = input[m];
        output[m] = static_cast<float>(val);
    }
}

// Complex type for testing (interleaved real/imag)
typedef float complex_pair[2];

static inline void shift_freq_neon(complex_pair* __restrict dest,
                                   const complex_pair* __restrict source1,
                                   const complex_pair* __restrict source2,
                                   int start, int end)
{
    int m = start;

    const float sign_array[4] = {1.0f, -1.0f, 1.0f, -1.0f};
    float32x4_t sign_mask = vld1q_f32(sign_array);

    for (; m + 1 < end; m += 2)
    {
        float32x4_t s1 = vld1q_f32(reinterpret_cast<const float*>(&source1[m]));
        float32x4_t s2 = vld1q_f32(reinterpret_cast<const float*>(&source2[m]));

        float32x4_t s1_rr = vuzp1q_f32(s1, s1);
        float32x4_t s1_ii = vuzp2q_f32(s1, s1);
        float32x4_t s2_ir = vrev64q_f32(s2);

        float32x4_t ac_ad = vmulq_f32(s1_rr, s2);
        float32x4_t bd_bc = vmulq_f32(s1_ii, s2_ir);

        bd_bc = vmulq_f32(bd_bc, sign_mask);
        float32x4_t result = vaddq_f32(ac_ad, bd_bc);

        vst1q_f32(reinterpret_cast<float*>(&dest[m]), result);
    }

    for (; m < end; m++)
    {
        dest[m][0] = source1[m][0] * source2[m][0] - source1[m][1] * source2[m][1];
        dest[m][1] = source1[m][1] * source2[m][0] + source1[m][0] * source2[m][1];
    }
}

static inline void shift_freq_scalar(complex_pair* __restrict dest,
                                     const complex_pair* __restrict source1,
                                     const complex_pair* __restrict source2,
                                     int start, int end)
{
    for (int m = start; m < end; m++)
    {
        dest[m][0] = source1[m][0] * source2[m][0] - source1[m][1] * source2[m][1];
        dest[m][1] = source1[m][1] * source2[m][0] + source1[m][0] * source2[m][1];
    }
}

template<bool flip>
static inline void copy_neon(complex_pair* __restrict dest,
                             const complex_pair* __restrict source,
                             int count)
{
    int i = 0;

    if constexpr (flip) {
        const uint32_t sign_array[4] = {0x00000000, 0x80000000, 0x00000000, 0x80000000};
        uint32x4_t sign_mask = vld1q_u32(sign_array);

        for (; i + 3 < count; i += 4)
        {
            float32x4_t v0 = vld1q_f32(reinterpret_cast<const float*>(&source[i]));
            float32x4_t v1 = vld1q_f32(reinterpret_cast<const float*>(&source[i + 2]));

            uint32x4_t v0_u = vreinterpretq_u32_f32(v0);
            uint32x4_t v1_u = vreinterpretq_u32_f32(v1);
            v0_u = veorq_u32(v0_u, sign_mask);
            v1_u = veorq_u32(v1_u, sign_mask);
            v0 = vreinterpretq_f32_u32(v0_u);
            v1 = vreinterpretq_f32_u32(v1_u);

            vst1q_f32(reinterpret_cast<float*>(&dest[i]), v0);
            vst1q_f32(reinterpret_cast<float*>(&dest[i + 2]), v1);
        }

        if (i + 1 < count)
        {
            float32x4_t v = vld1q_f32(reinterpret_cast<const float*>(&source[i]));
            uint32x4_t v_u = vreinterpretq_u32_f32(v);
            v_u = veorq_u32(v_u, sign_mask);
            v = vreinterpretq_f32_u32(v_u);
            vst1q_f32(reinterpret_cast<float*>(&dest[i]), v);
            i += 2;
        }

        for (; i < count; i++)
        {
            dest[i][0] = source[i][0];
            dest[i][1] = -source[i][1];
        }
    }
    else {
        if (count >= 8) {
            memcpy(dest, source, count * sizeof(complex_pair));
        } else {
            for (; i + 3 < count; i += 4)
            {
                float32x4_t v0 = vld1q_f32(reinterpret_cast<const float*>(&source[i]));
                float32x4_t v1 = vld1q_f32(reinterpret_cast<const float*>(&source[i + 2]));
                vst1q_f32(reinterpret_cast<float*>(&dest[i]), v0);
                vst1q_f32(reinterpret_cast<float*>(&dest[i + 2]), v1);
            }

            if (i + 1 < count)
            {
                float32x4_t v = vld1q_f32(reinterpret_cast<const float*>(&source[i]));
                vst1q_f32(reinterpret_cast<float*>(&dest[i]), v);
                i += 2;
            }

            for (; i < count; i++)
            {
                dest[i][0] = source[i][0];
                dest[i][1] = source[i][1];
            }
        }
    }
}

template<bool flip>
static inline void copy_scalar(complex_pair* __restrict dest,
                               const complex_pair* __restrict source,
                               int count)
{
    for (int i = 0; i < count; i++)
    {
        dest[i][0] = source[i][0];
        dest[i][1] = flip ? -source[i][1] : source[i][1];
    }
}

namespace {
    struct NeonOptimizations {};
}

TEST_CASE(NeonOptimizations, ConvertFloatCorrectness)
{
    const int size = 1024;
    std::vector<int16_t> input(size);
    std::vector<float> output_neon(size);
    std::vector<float> output_scalar(size);

    // Test with various input patterns
    for (int i = 0; i < size; i++) {
        input[i] = static_cast<int16_t>(i * 13 - 512); // Varied pattern
    }

    // Test without randomization
    convert_float_neon<false>(input.data(), output_neon.data(), size);
    convert_float_scalar<false>(input.data(), output_scalar.data(), size);

    for (int i = 0; i < size; i++) {
        REQUIRE_EQUAL(output_neon[i], output_scalar[i]);
    }

    // Test with randomization
    convert_float_neon<true>(input.data(), output_neon.data(), size);
    convert_float_scalar<true>(input.data(), output_scalar.data(), size);

    for (int i = 0; i < size; i++) {
        REQUIRE_EQUAL(output_neon[i], output_scalar[i]);
    }
}

TEST_CASE(NeonOptimizations, ShiftFreqCorrectness)
{
    const int size = 256;
    std::vector<complex_pair> source1(size);
    std::vector<complex_pair> source2(size);
    std::vector<complex_pair> dest_neon(size);
    std::vector<complex_pair> dest_scalar(size);

    // Initialize with test data
    for (int i = 0; i < size; i++) {
        source1[i][0] = static_cast<float>(i) * 0.1f;
        source1[i][1] = static_cast<float>(i) * 0.2f;
        source2[i][0] = std::cos(i * 0.01f);
        source2[i][1] = std::sin(i * 0.01f);
    }

    // Test complex multiplication
    shift_freq_neon(dest_neon.data(), source1.data(), source2.data(), 0, size);
    shift_freq_scalar(dest_scalar.data(), source1.data(), source2.data(), 0, size);

    const float tolerance = 1e-6f;
    for (int i = 0; i < size; i++) {
        float diff_real = std::abs(dest_neon[i][0] - dest_scalar[i][0]);
        float diff_imag = std::abs(dest_neon[i][1] - dest_scalar[i][1]);
        REQUIRE_TRUE(diff_real < tolerance);
        REQUIRE_TRUE(diff_imag < tolerance);
    }
}

TEST_CASE(NeonOptimizations, CopyCorrectness)
{
    const int size = 512;
    std::vector<complex_pair> source(size);
    std::vector<complex_pair> dest_neon(size);
    std::vector<complex_pair> dest_scalar(size);

    // Initialize with test data
    for (int i = 0; i < size; i++) {
        source[i][0] = static_cast<float>(i) * 1.5f;
        source[i][1] = static_cast<float>(i) * 2.5f;
    }

    // Test copy without flip
    copy_neon<false>(dest_neon.data(), source.data(), size);
    copy_scalar<false>(dest_scalar.data(), source.data(), size);

    for (int i = 0; i < size; i++) {
        REQUIRE_EQUAL(dest_neon[i][0], dest_scalar[i][0]);
        REQUIRE_EQUAL(dest_neon[i][1], dest_scalar[i][1]);
    }

    // Test copy with flip (conjugate)
    copy_neon<true>(dest_neon.data(), source.data(), size);
    copy_scalar<true>(dest_scalar.data(), source.data(), size);

    for (int i = 0; i < size; i++) {
        REQUIRE_EQUAL(dest_neon[i][0], dest_scalar[i][0]);
        REQUIRE_EQUAL(dest_neon[i][1], dest_scalar[i][1]);
    }
}

#else // !__ARM_NEON

// Placeholder tests for non-ARM platforms
namespace {
    struct NeonOptimizations {};
}

TEST_CASE(NeonOptimizations, SkipOnNonARM)
{
    // NEON tests only run on ARM platforms
    printf("NEON tests skipped (not ARM platform)\n");
}

#endif // __ARM_NEON
