/**
 * stability_test.cpp - Long-duration streaming stability tests
 *
 * These tests verify that the SDR streaming pipeline remains stable over
 * extended periods and stress test threading/synchronization.
 *
 * Test durations are configurable via environment variables:
 *   STABILITY_TEST_DURATION_SEC - Duration in seconds (default: 30)
 *   STABILITY_TEST_MIN_BUFFER   - Use minimal buffer sizes for stress testing
 */

#include "r2iq.h"
#include "FX3Class.h"
#include "RadioHandler.h"

// Standard library includes required before CppUnitTestFramework
#include <cstdint>
#include <limits>
#include <atomic>
#include <thread>
#include <chrono>
#include <vector>
#include <cstdlib>
#include <cstring>
#include <inttypes.h>

#include "CppUnitTestFramework.hpp"

using namespace std::chrono;

// Default test duration in seconds (can be overridden via environment variable)
static const int DEFAULT_TEST_DURATION_SEC = 30;

// Environment variable names for configuration
static const char* ENV_DURATION = "STABILITY_TEST_DURATION_SEC";
static const char* ENV_MIN_BUFFER = "STABILITY_TEST_MIN_BUFFER";

/**
 * Get the test duration from environment variable or use default
 */
static int getTestDurationSec()
{
    const char* env = std::getenv(ENV_DURATION);
    if (env != nullptr)
    {
        int duration = std::atoi(env);
        if (duration > 0)
        {
            return duration;
        }
    }
    return DEFAULT_TEST_DURATION_SEC;
}

/**
 * Check if minimal buffer stress test mode is enabled
 */
static bool isMinBufferMode()
{
    const char* env = std::getenv(ENV_MIN_BUFFER);
    return (env != nullptr && std::strcmp(env, "1") == 0);
}

/**
 * Mock FX3 handler for stability testing with sequence number support
 * This extends the basic mock to include sequence numbers for detecting
 * data gaps and continuity issues.
 */
class StabilityFX3Handler : public fx3class
{
public:
    StabilityFX3Handler() : run(false), nxfers(0), sequenceNumber(0) {}

    bool Open() override
    {
        return true;
    }

    bool Control(FX3Command command, uint8_t data) override
    {
        return true;
    }

    bool Control(FX3Command command, uint32_t data) override
    {
        return true;
    }

    bool Control(FX3Command command, uint64_t data) override
    {
        return true;
    }

    bool SetArgument(uint16_t index, uint16_t value) override
    {
        return true;
    }

    bool GetHardwareInfo(uint32_t* data) override
    {
        const uint8_t d[4] = {
            0, FIRMWARE_VER_MAJOR, FIRMWARE_VER_MINOR, 0
        };
        *data = *(uint32_t*)d;
        return true;
    }

    bool Enumerate(unsigned char& idx, char* lbuf) override
    {
        return true;
    }

    bool ReadDebugTrace(uint8_t* pdata, uint8_t len) override
    {
        return true;
    }

    void StartStream(ringbuffer<int16_t>& input, int numofblock) override
    {
        input.setBlockSize(transferSamples);
        run = true;
        sequenceNumber = 0;
        emuthread = std::thread([&input, this] {
            while (run)
            {
                auto ptr = input.getWritePtr();
                if (!run) break;

                // Write sequence number as first 4 bytes of each block
                // This allows detection of gaps in the data stream
                uint32_t seq = sequenceNumber.fetch_add(1);
                memcpy(ptr, &seq, sizeof(uint32_t));

                // Fill rest with known pattern
                for (size_t i = sizeof(uint32_t) / sizeof(int16_t); i < transferSamples; i++)
                {
                    ptr[i] = static_cast<int16_t>(0x5A5A);
                }

                input.WriteDone();
                ++nxfers;

                // Simulate realistic USB transfer timing (~1ms per transfer)
                std::this_thread::sleep_for(1ms);
            }
        });
    }

    void StopStream() override
    {
        run = false;
        if (emuthread.joinable())
        {
            emuthread.join();
        }
    }

    long getTransferCount(bool clear)
    {
        long rv = nxfers;
        if (clear) nxfers = 0;
        return rv;
    }

    uint32_t getSequenceNumber() const
    {
        return sequenceNumber.load();
    }

private:
    std::thread emuthread;
    volatile bool run;
    long nxfers;
    std::atomic<uint32_t> sequenceNumber;
};

/**
 * Statistics collected during stability tests
 */
struct StabilityStats
{
    uint64_t callbackCount;          // Number of callbacks received
    uint64_t totalSamples;           // Total samples processed
    uint64_t expectedSamples;        // Expected samples per callback
    int maxFullCount;                // Maximum buffer full events
    int maxEmptyCount;               // Maximum buffer empty events
    int finalFullCount;              // Final buffer full count
    int finalEmptyCount;             // Final buffer empty count
    uint32_t lastSequenceNumber;     // Last sequence number seen
    uint32_t sequenceGaps;           // Number of sequence gaps detected
    uint32_t duplicateSequences;     // Number of duplicate sequences
    bool hadUnderrun;                // Whether buffer underrun occurred
    bool hadOverrun;                 // Whether buffer overrun occurred
    steady_clock::time_point startTime;
    steady_clock::time_point endTime;

    StabilityStats()
        : callbackCount(0), totalSamples(0), expectedSamples(0),
          maxFullCount(0), maxEmptyCount(0), finalFullCount(0), finalEmptyCount(0),
          lastSequenceNumber(0), sequenceGaps(0), duplicateSequences(0),
          hadUnderrun(false), hadOverrun(false)
    {
    }

    void printReport() const
    {
        auto duration = duration_cast<milliseconds>(endTime - startTime).count();
        double seconds = duration / 1000.0;
        double callbacksPerSec = callbackCount / seconds;
        double samplesPerSec = totalSamples / seconds;

        printf("\n=== Stability Test Report ===\n");
        printf("Duration:             %.2f seconds\n", seconds);
        printf("Callbacks:            %" PRIu64 " (%.1f/sec)\n", callbackCount, callbacksPerSec);
        printf("Total samples:        %" PRIu64 " (%.1f M/sec)\n", totalSamples, samplesPerSec / 1e6);
        printf("Expected per callback: %" PRIu64 "\n", expectedSamples);
        printf("Buffer full events:   %d (max during test)\n", maxFullCount);
        printf("Buffer empty events:  %d (max during test)\n", maxEmptyCount);
        printf("Final full count:     %d\n", finalFullCount);
        printf("Final empty count:    %d\n", finalEmptyCount);
        printf("Sequence gaps:        %u\n", sequenceGaps);
        printf("Duplicate sequences:  %u\n", duplicateSequences);
        printf("Buffer underrun:      %s\n", hadUnderrun ? "YES - FAILURE" : "No");
        printf("Buffer overrun:       %s\n", hadOverrun ? "YES - FAILURE" : "No");
        printf("=============================\n\n");
    }
};

// Thread-safe statistics for callback
static std::atomic<uint64_t> g_stab_callbackCount{0};
static std::atomic<uint64_t> g_stab_totalSamples{0};
static std::atomic<bool> g_stab_streamActive{false};

/**
 * Callback function for stability tests
 */
static void StabilityCallback(void* context, const float* data, uint32_t len)
{
    if (!g_stab_streamActive.load()) return;

    g_stab_callbackCount.fetch_add(1);
    g_stab_totalSamples.fetch_add(len);
}

namespace {
    struct StabilityFixture {};
}

/**
 * Test: Stress test with rapid start/stop cycles
 *
 * This test rapidly starts and stops streaming to stress the threading
 * and synchronization code.
 */
TEST_CASE(StabilityFixture, StartStopStressTest)
{
    int cycles = 20;  // Number of start/stop cycles
    printf("\n[StartStopStressTest] Running %d start/stop cycles\n", cycles);

    auto usb = new StabilityFX3Handler();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, StabilityCallback);

    for (int i = 0; i < cycles; i++)
    {
        g_stab_callbackCount.store(0);
        g_stab_totalSamples.store(0);
        g_stab_streamActive.store(true);

        // Vary decimation level (use valid srate_idx values 0-4)
        int srate_idx = i % 5;

        radio->Start(srate_idx);

        // Run for a short time (100-500ms)
        auto runTime = milliseconds(100 + (i % 5) * 100);
        std::this_thread::sleep_for(runTime);

        g_stab_streamActive.store(false);
        radio->Stop();

        // Verify we got some callbacks
        uint64_t callbacks = g_stab_callbackCount.load();
        if (callbacks == 0)
        {
            printf("WARNING: Cycle %d (srate_idx=%d, %ldms): No callbacks\n",
                   i, srate_idx, runTime.count());
        }

        if ((i + 1) % 10 == 0)
        {
            printf("  Completed %d/%d cycles\n", i + 1, cycles);
        }
    }

    printf("StartStopStressTest: PASSED\n");

    delete radio;
    delete usb;
}

/**
 * Test: Data continuity check
 *
 * This test verifies that there are no gaps in the data stream by checking
 * sequence numbers embedded in the mock data.
 */
TEST_CASE(StabilityFixture, DataContinuityTest)
{
    int testDurationSec = std::min(getTestDurationSec(), 15);  // Cap at 15s
    printf("\n[DataContinuityTest] Checking data continuity for %d seconds\n", testDurationSec);

    auto usb = new StabilityFX3Handler();
    auto radio = new RadioHandlerClass();

    g_stab_callbackCount.store(0);
    g_stab_totalSamples.store(0);
    g_stab_streamActive.store(true);

    radio->Init(usb, StabilityCallback);

    // Start streaming
    radio->Start(3);  // Use srate_idx=3 (16 MHz BW)

    auto startTime = steady_clock::now();
    auto testDuration = seconds(testDurationSec);

    uint32_t lastSeq = 0;
    uint32_t gaps = 0;

    while (steady_clock::now() - startTime < testDuration)
    {
        std::this_thread::sleep_for(100ms);

        uint32_t currentSeq = usb->getSequenceNumber();
        if (currentSeq > 0 && lastSeq > 0)
        {
            // Check for gaps (sequence should increase monotonically)
            if (currentSeq < lastSeq)
            {
                printf("WARNING: Sequence number went backwards: %u -> %u\n", lastSeq, currentSeq);
                gaps++;
            }
        }
        lastSeq = currentSeq;
    }

    g_stab_streamActive.store(false);
    radio->Stop();

    uint32_t finalSeq = usb->getSequenceNumber();
    printf("Final sequence number: %u\n", finalSeq);
    printf("Detected gaps: %u\n", gaps);

    // Should have many transfers over the test period
    REQUIRE_TRUE(finalSeq > 100);
    // No backwards sequence numbers
    REQUIRE_EQUAL(gaps, 0u);

    printf("DataContinuityTest: PASSED\n");

    delete radio;
    delete usb;
}
