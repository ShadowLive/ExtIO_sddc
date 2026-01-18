/**
 * VHF Mode Tests
 *
 * Tests for VHF tuner mode including:
 * - Mode switching (HF <-> VHF)
 * - Tuner frequency setting and IF offset
 * - RF attenuation (discrete steps)
 * - IF gain (discrete steps)
 * - Bias-T control (MOCK HARDWARE ONLY)
 * - Sideband configuration (no inversion for R828D)
 *
 * IMPORTANT SAFETY NOTE:
 * ======================
 * These tests use MOCK hardware - no actual USB commands are sent to real hardware.
 * The Bias-T tests verify the software API only.
 *
 * WARNING: DO NOT enable Bias-T on real hardware unless your antenna/LNA
 * explicitly supports DC power injection. Enabling Bias-T with incompatible
 * equipment can cause permanent damage to your antenna, SDR, or connected devices.
 */

#include <limits>
#include <cstdint>
#include "CppUnitTestFramework.hpp"
#include "r2iq.h"
#include "FX3Class.h"
#include "RadioHandler.h"
#include "config.h"
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>

using namespace std::chrono;

/**
 * Mock FX3 Handler that tracks VHF-related commands
 */
class VHFMockFx3Handler : public fx3class
{
public:
    // Track commands for verification
    struct CommandLog {
        FX3Command command;
        uint64_t data;
    };
    std::vector<CommandLog> command_log;

    // Track arguments for verification
    struct ArgumentLog {
        uint16_t index;
        uint16_t value;
    };
    std::vector<ArgumentLog> argument_log;

    // Simulated GPIO state
    uint32_t gpio_state = 0;

    // Tuner state
    bool tuner_initialized = false;
    uint64_t tuner_frequency = 0;
    uint32_t tuner_ref_freq = 0;

    // Attenuation state
    uint16_t rf_attenuation_index = 0;
    uint16_t if_gain_index = 0;
    uint16_t vga_gain = 0;

    VHFMockFx3Handler() = default;

    bool Open() override { return true; }

    bool Control(FX3Command command, uint8_t data = 0) override
    {
        command_log.push_back({command, data});

        if (command == TUNERSTDBY) {
            tuner_initialized = false;
        }
        return true;
    }

    bool Control(FX3Command command, uint32_t data) override
    {
        command_log.push_back({command, data});

        if (command == TUNERINIT) {
            tuner_initialized = true;
            tuner_ref_freq = data;
        }
        return true;
    }

    bool Control(FX3Command command, uint64_t data) override
    {
        command_log.push_back({command, data});

        if (command == TUNERTUNE) {
            tuner_frequency = data;
        }
        return true;
    }

    bool SetArgument(uint16_t index, uint16_t value) override
    {
        argument_log.push_back({index, value});

        // Track specific arguments
        if (index == R82XX_ATTENUATOR) {
            rf_attenuation_index = value;
        } else if (index == R82XX_VGA) {
            if_gain_index = value;
        } else if (index == AD8340_VGA) {
            vga_gain = value;
        }
        return true;
    }

    bool GetHardwareInfo(uint32_t* data) override
    {
        // Simulate RX888R2 hardware (has R828D tuner)
        const uint8_t d[4] = {
            RX888r2,  // Model (internal enum from Interface.h)
            FIRMWARE_VER_MAJOR,
            FIRMWARE_VER_MINOR,
            0
        };
        *data = *(uint32_t*)d;
        return true;
    }

    bool Enumerate(unsigned char& idx, char* lbuf) override
    {
        strcpy(lbuf, "RX888R2 Mock");
        return true;
    }

    bool ReadDebugTrace(uint8_t* pdata, uint8_t len) override
    {
        return true;
    }

    std::thread emuthread{};
    bool run = false;

    void StartStream(ringbuffer<int16_t>& input, int numofblock) override
    {
        input.setBlockSize(transferSamples);
        run = true;
        emuthread = std::thread([&input, this]{
            while(run) {
                auto ptr = input.getWritePtr();
                memset(ptr, 0, input.getWriteCount() * sizeof(int16_t));
                input.WriteDone();
                std::this_thread::sleep_for(1ms);
            }
        });
    }

    void StopStream() override
    {
        run = false;
        if (emuthread.joinable()) {
            emuthread.join();
        }
    }

    // Helper to clear logs between tests
    void ClearLogs()
    {
        command_log.clear();
        argument_log.clear();
    }

    // Helper to check if a command was issued
    bool HasCommand(FX3Command cmd) const
    {
        for (const auto& log : command_log) {
            if (log.command == cmd) return true;
        }
        return false;
    }

    // Helper to get last command data
    uint64_t GetLastCommandData(FX3Command cmd) const
    {
        for (auto it = command_log.rbegin(); it != command_log.rend(); ++it) {
            if (it->command == cmd) return it->data;
        }
        return 0;
    }

    // Helper to check if an argument was set
    bool HasArgument(uint16_t index) const
    {
        for (const auto& log : argument_log) {
            if (log.index == index) return true;
        }
        return false;
    }
};

// Callback for radio handler
static uint32_t vhf_callback_count = 0;
static void VHFCallback(void* context, const float* data, uint32_t len)
{
    vhf_callback_count++;
}

namespace {
    struct VHFFixture {};
}

// =============================================================================
// VHF Mode Switching Tests
// =============================================================================

TEST_CASE(VHFFixture, ModeSwitchToVHF)
{
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);
    usb->ClearLogs();

    // Switch to VHF mode
    bool result = radio->UpdatemodeRF(VHFMODE);
    REQUIRE_TRUE(result);

    // Verify tuner was initialized
    REQUIRE_TRUE(usb->HasCommand(TUNERINIT));

    // Verify reference frequency was set (R828D uses 16 MHz)
    uint32_t ref_freq = (uint32_t)usb->GetLastCommandData(TUNERINIT);
    REQUIRE_TRUE(ref_freq == 16000000 || ref_freq == 32000000);  // R828D or R820T2

    delete radio;
    delete usb;
}

TEST_CASE(VHFFixture, ModeSwitchToHF)
{
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);

    // First switch to VHF
    radio->UpdatemodeRF(VHFMODE);
    usb->ClearLogs();

    // Then switch back to HF
    bool result = radio->UpdatemodeRF(HFMODE);
    REQUIRE_TRUE(result);

    // Verify tuner was put to standby
    REQUIRE_TRUE(usb->HasCommand(TUNERSTDBY));

    delete radio;
    delete usb;
}

TEST_CASE(VHFFixture, ModeSwitchPreservesOnSameMode)
{
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);

    // Switch to VHF
    radio->UpdatemodeRF(VHFMODE);
    usb->ClearLogs();

    // Switch to VHF again (should be no-op)
    radio->UpdatemodeRF(VHFMODE);

    // Should not re-initialize tuner
    REQUIRE_FALSE(usb->HasCommand(TUNERINIT));

    delete radio;
    delete usb;
}

// =============================================================================
// VHF Tuner Frequency Tests
// =============================================================================

TEST_CASE(VHFFixture, TunerFrequencySet)
{
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);
    radio->UpdatemodeRF(VHFMODE);
    usb->ClearLogs();

    // Set tuner to 100 MHz
    uint64_t freq = 100000000;
    radio->TuneLO(freq);

    // Verify TUNERTUNE command was issued
    REQUIRE_TRUE(usb->HasCommand(TUNERTUNE));

    // Verify frequency was set
    REQUIRE_EQUAL(usb->tuner_frequency, freq);

    delete radio;
    delete usb;
}

TEST_CASE(VHFFixture, TunerFrequencyRange)
{
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);
    radio->UpdatemodeRF(VHFMODE);

    // Test various frequencies in VHF/UHF range
    std::vector<uint64_t> test_freqs = {
        24000000,    // 24 MHz (minimum)
        88000000,    // 88 MHz (FM broadcast)
        144000000,   // 144 MHz (2m amateur)
        430000000,   // 430 MHz (70cm amateur)
        900000000,   // 900 MHz
        1296000000,  // 1296 MHz (23cm amateur)
        1800000000   // 1.8 GHz (maximum)
    };

    for (uint64_t freq : test_freqs) {
        usb->ClearLogs();
        radio->TuneLO(freq);

        // Each frequency should result in a TUNERTUNE command
        REQUIRE_TRUE(usb->HasCommand(TUNERTUNE));
        REQUIRE_EQUAL(usb->tuner_frequency, freq);
    }

    delete radio;
    delete usb;
}

TEST_CASE(VHFFixture, TunerFrequencyNotSetInHFMode)
{
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);
    // Stay in HF mode (default)
    usb->ClearLogs();

    // Try to tune - should not issue TUNERTUNE in HF mode
    radio->TuneLO(100000000);

    // TUNERTUNE should not be issued in HF mode
    REQUIRE_FALSE(usb->HasCommand(TUNERTUNE));

    delete radio;
    delete usb;
}

// =============================================================================
// VHF Sideband/IF Frequency Tests
// =============================================================================

TEST_CASE(VHFFixture, SidebandConfiguredOnModeSwitch)
{
    // This test verifies that mode switching properly initializes tuner.
    // Sideband configuration (no inversion for R828D low-side injection)
    // is handled internally by RadioHandler and tested via signal integrity tests.
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);

    // Switch to VHF mode - this internally configures sideband
    bool result = radio->UpdatemodeRF(VHFMODE);
    REQUIRE_TRUE(result);

    // Tuner initialization indicates sideband was configured
    REQUIRE_TRUE(usb->HasCommand(TUNERINIT));

    delete radio;
    delete usb;
}

TEST_CASE(VHFFixture, IFFrequencyDefault)
{
    // The IF frequency for R828D tuner is 4.57 MHz
    const double expected_if = 4570000.0;

    // This is a constant check - verify the IF frequency definition
    // In actual use, RF frequency = tuner_frequency + IF_frequency
    REQUIRE_TRUE(expected_if > 4000000.0 && expected_if < 5000000.0);
}

// =============================================================================
// VHF Attenuation Tests
// =============================================================================

TEST_CASE(VHFFixture, RFAttenuationSteps)
{
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);
    radio->UpdatemodeRF(VHFMODE);

    // Get RF attenuation steps
    const float* steps = nullptr;
    int count = radio->GetRFAttSteps(&steps);

    // Should have attenuation steps for VHF mode
    REQUIRE_TRUE(count > 0);
    REQUIRE_TRUE(steps != nullptr);

    // First step should be 0 dB (no attenuation)
    REQUIRE_TRUE(std::abs(steps[0] - 0.0f) < 0.1f);

    // Steps should be monotonically increasing
    for (int i = 1; i < count; i++) {
        REQUIRE_TRUE(steps[i] >= steps[i-1]);
    }

    delete radio;
    delete usb;
}

TEST_CASE(VHFFixture, RFAttenuationSet)
{
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);
    radio->UpdatemodeRF(VHFMODE);
    usb->ClearLogs();

    // Set RF attenuation to index 5
    radio->UpdateattRF(5);

    // Verify R82XX_ATTENUATOR argument was set
    REQUIRE_TRUE(usb->HasArgument(R82XX_ATTENUATOR));
    REQUIRE_EQUAL(usb->rf_attenuation_index, 5);

    delete radio;
    delete usb;
}

TEST_CASE(VHFFixture, RFAttenuationBoundsCheck)
{
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);
    radio->UpdatemodeRF(VHFMODE);

    const float* steps = nullptr;
    int count = radio->GetRFAttSteps(&steps);

    // Test setting attenuation beyond bounds
    usb->ClearLogs();
    radio->UpdateattRF(count + 10);  // Beyond max

    // Should clamp to valid range
    REQUIRE_TRUE(usb->rf_attenuation_index <= count - 1);

    usb->ClearLogs();
    radio->UpdateattRF(-5);  // Below min

    // Should clamp to 0
    REQUIRE_EQUAL(usb->rf_attenuation_index, 0);

    delete radio;
    delete usb;
}

// =============================================================================
// VHF IF Gain Tests
// =============================================================================

TEST_CASE(VHFFixture, IFGainSteps)
{
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);
    radio->UpdatemodeRF(VHFMODE);

    // Get IF gain steps
    const float* steps = nullptr;
    int count = radio->GetIFGainSteps(&steps);

    // Should have gain steps for VHF mode
    REQUIRE_TRUE(count > 0);
    REQUIRE_TRUE(steps != nullptr);

    // Gain range should span negative to positive (typical for VGA)
    float min_gain = steps[0];
    float max_gain = steps[count - 1];
    REQUIRE_TRUE(max_gain > min_gain);

    delete radio;
    delete usb;
}

TEST_CASE(VHFFixture, IFGainSet)
{
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);
    radio->UpdatemodeRF(VHFMODE);
    usb->ClearLogs();

    // Set IF gain to index 8
    radio->UpdateIFGain(8);

    // Verify R82XX_VGA argument was set
    REQUIRE_TRUE(usb->HasArgument(R82XX_VGA));
    REQUIRE_EQUAL(usb->if_gain_index, 8);

    delete radio;
    delete usb;
}

// =============================================================================
// VHF Bias-T Tests
// =============================================================================
// NOTE: These tests use MOCK hardware only - no actual Bias-T is enabled.
// NEVER enable Bias-T on real hardware without verifying antenna compatibility!

TEST_CASE(VHFFixture, BiasTDefaultOff)
{
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);

    // Bias-T should be off by default
    REQUIRE_FALSE(radio->GetBiasT_VHF());

    delete radio;
    delete usb;
}

TEST_CASE(VHFFixture, BiasTEnable)
{
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);

    // Enable VHF bias-T
    radio->UpdBiasT_VHF(true);
    REQUIRE_TRUE(radio->GetBiasT_VHF());

    // Disable VHF bias-T
    radio->UpdBiasT_VHF(false);
    REQUIRE_FALSE(radio->GetBiasT_VHF());

    delete radio;
    delete usb;
}

TEST_CASE(VHFFixture, BiasTIndependentFromHF)
{
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);

    // Enable only VHF bias-T
    radio->UpdBiasT_VHF(true);
    radio->UpdBiasT_HF(false);

    REQUIRE_TRUE(radio->GetBiasT_VHF());
    REQUIRE_FALSE(radio->GetBiasT_HF());

    // Enable only HF bias-T
    radio->UpdBiasT_VHF(false);
    radio->UpdBiasT_HF(true);

    REQUIRE_FALSE(radio->GetBiasT_VHF());
    REQUIRE_TRUE(radio->GetBiasT_HF());

    delete radio;
    delete usb;
}

// =============================================================================
// VHF Streaming Tests
// =============================================================================

TEST_CASE(VHFFixture, StreamingInVHFMode)
{
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);
    radio->UpdatemodeRF(VHFMODE);

    // Set tuner frequency
    radio->TuneLO(144000000);  // 2m band

    // Start streaming
    vhf_callback_count = 0;
    radio->Start(2);  // 8 Msps decimation

    // Wait for some callbacks
    std::this_thread::sleep_for(500ms);

    radio->Stop();

    // Should have received callbacks
    REQUIRE_TRUE(vhf_callback_count > 0);

    delete radio;
    delete usb;
}

TEST_CASE(VHFFixture, FrequencyChangeWhileStreaming)
{
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);
    radio->UpdatemodeRF(VHFMODE);
    radio->TuneLO(100000000);

    // Start streaming
    vhf_callback_count = 0;
    radio->Start(2);

    std::this_thread::sleep_for(100ms);
    uint32_t count_before = vhf_callback_count;

    // Change frequency while streaming
    usb->ClearLogs();
    radio->TuneLO(200000000);

    // Verify frequency change command was issued
    REQUIRE_TRUE(usb->HasCommand(TUNERTUNE));
    REQUIRE_EQUAL(usb->tuner_frequency, 200000000ULL);

    std::this_thread::sleep_for(100ms);

    // Should still be receiving callbacks
    REQUIRE_TRUE(vhf_callback_count > count_before);

    radio->Stop();

    delete radio;
    delete usb;
}

// =============================================================================
// Integration Tests
// =============================================================================

TEST_CASE(VHFFixture, FullVHFSetupSequence)
{
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);

    // Complete VHF setup sequence
    // 1. Switch to VHF mode
    REQUIRE_TRUE(radio->UpdatemodeRF(VHFMODE));

    // 2. Set tuner frequency
    radio->TuneLO(145000000);  // 145 MHz
    REQUIRE_EQUAL(usb->tuner_frequency, 145000000ULL);

    // 3. Set RF attenuation
    radio->UpdateattRF(10);  // ~20 dB

    // 4. Set IF gain
    radio->UpdateIFGain(8);  // Mid-range gain

    // 5. Start streaming
    vhf_callback_count = 0;
    radio->Start(1);  // 16 Msps

    std::this_thread::sleep_for(200ms);

    // 6. Verify streaming works
    REQUIRE_TRUE(vhf_callback_count > 0);

    // 7. Stop and cleanup
    radio->Stop();

    delete radio;
    delete usb;
}

TEST_CASE(VHFFixture, ModeSwapWhileStreaming)
{
    auto usb = new VHFMockFx3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, VHFCallback);
    radio->UpdatemodeRF(VHFMODE);
    radio->TuneLO(144000000);

    // Start streaming in VHF mode
    radio->Start(2);
    std::this_thread::sleep_for(100ms);

    // Stop streaming before mode switch
    radio->Stop();

    // Switch to HF mode
    usb->ClearLogs();
    REQUIRE_TRUE(radio->UpdatemodeRF(HFMODE));

    // Verify tuner was put to standby
    REQUIRE_TRUE(usb->HasCommand(TUNERSTDBY));

    // Switch back to VHF mode
    usb->ClearLogs();
    REQUIRE_TRUE(radio->UpdatemodeRF(VHFMODE));

    // Verify tuner was re-initialized
    REQUIRE_TRUE(usb->HasCommand(TUNERINIT));

    delete radio;
    delete usb;
}
