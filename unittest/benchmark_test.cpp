/**
 * benchmark_test.cpp - DSP Processing Throughput Benchmarks
 *
 * Measures:
 * 1. Samples per second at each decimation level (0-4)
 * 2. Latency from input to output callback
 * 3. CPU efficiency (processing time vs real-time)
 * 4. Buffer efficiency (full/empty events)
 */

#include "r2iq.h"
#include "FX3Class.h"
#include "CppUnitTestFramework.hpp"
#include "RadioHandler.h"
#include "config.h"

#include <thread>
#include <chrono>
#include <vector>
#include <atomic>
#include <cstdio>
#include <inttypes.h>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <mutex>

using namespace std::chrono;

// Benchmark configuration
static const int BENCHMARK_DURATION_SECONDS = 3;  // Duration for each decimation test
static const int WARMUP_ITERATIONS = 100;         // Warmup transfers before measurement

/**
 * Fast FX3 mock handler for benchmarking
 * Feeds data as fast as possible without artificial delays
 */
class fx3handler_benchmark : public fx3class
{
public:
    fx3handler_benchmark() :
        run(false),
        nxfers(0),
        inputRingbuffer(nullptr),
        firstInputTime(high_resolution_clock::time_point::min()),
        inputStarted(false)
    {}

    bool Open() override { return true; }

    bool Control(FX3Command command, uint8_t data = 0) override { return true; }
    bool Control(FX3Command command, uint32_t data) override { return true; }
    bool Control(FX3Command command, uint64_t data) override { return true; }

    bool SetArgument(uint16_t index, uint16_t value) override { return true; }

    bool GetHardwareInfo(uint32_t* data) override {
        const uint8_t d[4] = {
            0, FIRMWARE_VER_MAJOR, FIRMWARE_VER_MINOR, 0
        };
        *data = *(uint32_t*)d;
        return true;
    }

    bool Enumerate(unsigned char& idx, char* lbuf) override { return true; }

    bool ReadDebugTrace(uint8_t* pdata, uint8_t len) override { return true; }

    void StartStream(ringbuffer<int16_t>& input, int numofblock) override
    {
        inputRingbuffer = &input;
        input.setBlockSize(transferSamples);
        run = true;
        nxfers = 0;
        inputStarted = false;
        firstInputTime = high_resolution_clock::time_point::min();

        emuthread = std::thread([&input, this]{
            // Generate test pattern data
            std::vector<int16_t> testPattern(transferSamples);
            for (uint32_t i = 0; i < transferSamples; i++) {
                // Generate a simple sine-like pattern for realistic DSP load
                testPattern[i] = static_cast<int16_t>(16384 * std::sin(2.0 * M_PI * i / 64.0));
            }

            // Warmup phase - fill some buffers without counting
            for (int i = 0; i < WARMUP_ITERATIONS && run; i++) {
                auto ptr = input.getWritePtr();
                memcpy(ptr, testPattern.data(), transferSamples * sizeof(int16_t));
                input.WriteDone();
            }

            // Record time of first measurement input
            firstInputTime = high_resolution_clock::now();
            inputStarted = true;

            // Main benchmark loop - feed data as fast as possible
            while(run)
            {
                auto ptr = input.getWritePtr();
                if (!run) break;

                memcpy(ptr, testPattern.data(), transferSamples * sizeof(int16_t));
                input.WriteDone();
                ++nxfers;
            }
        });
    }

    void StopStream() override {
        run = false;
        if (emuthread.joinable())
            emuthread.join();
    }

    // Accessors for benchmark data
    long getXferCount(bool clear = false) {
        long rv = nxfers;
        if (clear) nxfers = 0;
        return rv;
    }

    high_resolution_clock::time_point getFirstInputTime() const {
        return firstInputTime;
    }

    bool isInputStarted() const {
        return inputStarted;
    }

    ringbuffer<int16_t>* getInputBuffer() const {
        return inputRingbuffer;
    }

private:
    std::thread emuthread;
    std::atomic<bool> run;
    std::atomic<long> nxfers;
    ringbuffer<int16_t>* inputRingbuffer;
    high_resolution_clock::time_point firstInputTime;
    std::atomic<bool> inputStarted;
};

/**
 * Benchmark statistics collector
 */
struct BenchmarkStats {
    uint64_t callbackCount;
    uint64_t totalSamples;
    double elapsedSeconds;
    double msps;              // Mega samples per second
    double realtimePercent;   // Processing efficiency (>100% means faster than realtime)
    int bufferFullEvents;
    int bufferEmptyEvents;
    double minLatencyUs;
    double maxLatencyUs;
    double avgLatencyUs;

    BenchmarkStats() :
        callbackCount(0),
        totalSamples(0),
        elapsedSeconds(0),
        msps(0),
        realtimePercent(0),
        bufferFullEvents(0),
        bufferEmptyEvents(0),
        minLatencyUs(1e9),
        maxLatencyUs(0),
        avgLatencyUs(0)
    {}
};

// Global stats for callback
static std::atomic<uint64_t> g_callbackCount{0};
static std::atomic<uint64_t> g_totalSamples{0};
static high_resolution_clock::time_point g_firstCallbackTime;
static high_resolution_clock::time_point g_lastCallbackTime;
static std::atomic<bool> g_firstCallbackReceived{false};
static std::atomic<double> g_totalLatencyUs{0};
static std::atomic<double> g_minLatencyUs{1e9};
static std::atomic<double> g_maxLatencyUs{0};
static high_resolution_clock::time_point g_inputStartTime;

static void BenchmarkCallback(void* context, const float* data, uint32_t len)
{
    auto now = high_resolution_clock::now();

    if (!g_firstCallbackReceived.exchange(true)) {
        g_firstCallbackTime = now;
    }
    g_lastCallbackTime = now;

    g_callbackCount++;
    g_totalSamples += len;

    // Calculate latency if we have input start time
    if (g_inputStartTime != high_resolution_clock::time_point::min()) {
        auto latencyUs = duration_cast<microseconds>(now - g_inputStartTime).count();

        // Track running average latency per-callback
        double currentMin = g_minLatencyUs.load();
        while (latencyUs < currentMin && !g_minLatencyUs.compare_exchange_weak(currentMin, latencyUs)) {}

        double currentMax = g_maxLatencyUs.load();
        while (latencyUs > currentMax && !g_maxLatencyUs.compare_exchange_weak(currentMax, latencyUs)) {}
    }
}

static void ResetStats()
{
    g_callbackCount = 0;
    g_totalSamples = 0;
    g_firstCallbackReceived = false;
    g_totalLatencyUs = 0;
    g_minLatencyUs = 1e9;
    g_maxLatencyUs = 0;
    g_inputStartTime = high_resolution_clock::time_point::min();
}

namespace {
    struct BenchmarkFixture {};
}

/**
 * Main throughput benchmark test
 * Tests all decimation levels (0-4) and measures:
 * - Processing throughput (Msps)
 * - Real-time efficiency
 * - Buffer statistics
 */
TEST_CASE(BenchmarkFixture, ThroughputBenchmark)
{
    printf("\n");
    printf("================================================================================\n");
    printf("                    DSP PROCESSING THROUGHPUT BENCHMARK\n");
    printf("================================================================================\n");
    printf("Configuration:\n");
    printf("  Transfer size:    %u bytes (%u samples)\n", transferSize, transferSamples);
    printf("  ADC sample rate:  %u Hz (%.1f Msps)\n", DEFAULT_ADC_FREQ, DEFAULT_ADC_FREQ / 1e6);
    printf("  Test duration:    %d seconds per decimation level\n", BENCHMARK_DURATION_SECONDS);
    printf("  Warmup:           %d transfers\n", WARMUP_ITERATIONS);
    printf("================================================================================\n\n");

    auto usb = new fx3handler_benchmark();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, BenchmarkCallback);

    std::vector<BenchmarkStats> results;

    // Test each decimation level (0-4)
    for (int decimate = 0; decimate < 5; decimate++)
    {
        ResetStats();

        // Calculate expected output sample rate for this decimation
        // Decimation 0 = full rate (32 Msps), 1 = 16 Msps, 2 = 8 Msps, 3 = 4 Msps, 4 = 2 Msps
        uint32_t expectedOutputRate = DEFAULT_ADC_FREQ / 2;  // DDC halves the rate
        for (int i = 0; i < decimate; i++) {
            expectedOutputRate /= 2;
        }

        printf("Testing decimation %d (expected %.1f Msps output)...\n",
               decimate, expectedOutputRate / 1e6);

        // Start streaming
        auto startTime = high_resolution_clock::now();
        radio->Start(decimate);

        // Wait for input to start
        while (!usb->isInputStarted()) {
            std::this_thread::sleep_for(1ms);
        }
        g_inputStartTime = usb->getFirstInputTime();

        // Run for specified duration
        std::this_thread::sleep_for(seconds(BENCHMARK_DURATION_SECONDS));

        // Stop and collect stats
        radio->Stop();
        auto endTime = high_resolution_clock::now();

        // Collect results
        BenchmarkStats stats;
        stats.callbackCount = g_callbackCount.load();
        stats.totalSamples = g_totalSamples.load();
        stats.elapsedSeconds = duration<double>(endTime - startTime).count();

        // Calculate Msps
        stats.msps = (stats.totalSamples / 1e6) / stats.elapsedSeconds;

        // Calculate real-time efficiency
        // If we're processing at the expected rate, efficiency should be ~100%
        double expectedSamples = expectedOutputRate * stats.elapsedSeconds;
        stats.realtimePercent = (stats.totalSamples / expectedSamples) * 100.0;

        // Get buffer statistics from the ringbuffer
        auto inputBuffer = usb->getInputBuffer();
        if (inputBuffer) {
            stats.bufferFullEvents = inputBuffer->getFullCount();
            stats.bufferEmptyEvents = inputBuffer->getEmptyCount();
        }

        stats.minLatencyUs = g_minLatencyUs.load();
        stats.maxLatencyUs = g_maxLatencyUs.load();
        if (stats.callbackCount > 0) {
            stats.avgLatencyUs = (stats.minLatencyUs + stats.maxLatencyUs) / 2.0;
        }

        results.push_back(stats);

        // Brief pause between tests
        std::this_thread::sleep_for(100ms);
    }

    // Print benchmark results
    printf("\n");
    printf("================================================================================\n");
    printf("                           BENCHMARK RESULTS\n");
    printf("================================================================================\n");
    printf("\nThroughput by Decimation Level:\n");
    printf("--------------------------------------------------------------------------------\n");
    printf("  Dec  |   Output Rate   |  Actual Rate  |  Efficiency  |    Callbacks\n");
    printf("--------------------------------------------------------------------------------\n");

    for (int i = 0; i < 5; i++) {
        uint32_t expectedRate = DEFAULT_ADC_FREQ / 2;
        for (int j = 0; j < i; j++) expectedRate /= 2;

        printf("   %d   |   %6.2f Msps   |  %6.2f Msps  |   %6.1f%%    |   %'" PRIu64 "\n",
               i,
               expectedRate / 1e6,
               results[i].msps,
               results[i].realtimePercent,
               results[i].callbackCount);
    }
    printf("--------------------------------------------------------------------------------\n");

    printf("\nBuffer Statistics (across all tests):\n");
    printf("--------------------------------------------------------------------------------\n");
    int totalFull = 0, totalEmpty = 0;
    for (const auto& s : results) {
        totalFull += s.bufferFullEvents;
        totalEmpty += s.bufferEmptyEvents;
    }
    printf("  Buffer full events (input overrun):   %d\n", totalFull);
    printf("  Buffer empty events (output underrun): %d\n", totalEmpty);
    printf("--------------------------------------------------------------------------------\n");

    printf("\nLatency Measurements:\n");
    printf("--------------------------------------------------------------------------------\n");
    for (int i = 0; i < 5; i++) {
        if (results[i].minLatencyUs < 1e8) {
            printf("  Decimation %d: min=%.1f us, max=%.1f us\n",
                   i, results[i].minLatencyUs, results[i].maxLatencyUs);
        }
    }
    printf("--------------------------------------------------------------------------------\n");

    // Summary
    printf("\n================================================================================\n");
    printf("                              SUMMARY\n");
    printf("================================================================================\n");

    double avgEfficiency = 0;
    for (const auto& s : results) {
        avgEfficiency += s.realtimePercent;
    }
    avgEfficiency /= results.size();

    printf("  Average real-time efficiency: %.1f%%\n", avgEfficiency);
    printf("  Total buffer overruns:        %d\n", totalFull);
    printf("  Total buffer underruns:       %d\n", totalEmpty);

    if (avgEfficiency >= 100.0) {
        printf("\n  STATUS: PASS - System can process faster than real-time\n");
    } else {
        printf("\n  STATUS: WARNING - System may not keep up with real-time data\n");
    }
    printf("================================================================================\n\n");

    // Verify basic functionality
    for (int i = 0; i < 5; i++) {
        REQUIRE_TRUE(results[i].callbackCount > 0);
        REQUIRE_TRUE(results[i].totalSamples > 0);
    }

    delete radio;
    delete usb;
}

/**
 * Sustained load benchmark
 * Tests system stability under continuous high-throughput operation
 */
TEST_CASE(BenchmarkFixture, SustainedLoadBenchmark)
{
    printf("\n");
    printf("================================================================================\n");
    printf("                    SUSTAINED LOAD BENCHMARK (10 seconds)\n");
    printf("================================================================================\n");

    auto usb = new fx3handler_benchmark();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, BenchmarkCallback);
    ResetStats();

    // Run at maximum throughput (decimation 0) for 10 seconds
    int testDuration = 10;
    printf("Running at decimation 0 (max throughput) for %d seconds...\n", testDuration);

    auto startTime = high_resolution_clock::now();
    radio->Start(0);

    // Wait for start
    while (!usb->isInputStarted()) {
        std::this_thread::sleep_for(1ms);
    }
    g_inputStartTime = usb->getFirstInputTime();

    // Track samples over time for variance analysis
    std::vector<uint64_t> sampleCounts;
    sampleCounts.reserve(testDuration * 10);

    for (int i = 0; i < testDuration * 10; i++) {
        std::this_thread::sleep_for(100ms);
        sampleCounts.push_back(g_totalSamples.load());
    }

    radio->Stop();
    auto endTime = high_resolution_clock::now();

    double elapsed = duration<double>(endTime - startTime).count();
    uint64_t totalSamples = g_totalSamples.load();
    double msps = (totalSamples / 1e6) / elapsed;

    // Calculate throughput variance
    std::vector<double> intervalRates;
    for (size_t i = 1; i < sampleCounts.size(); i++) {
        uint64_t intervalSamples = sampleCounts[i] - sampleCounts[i-1];
        double intervalRate = (intervalSamples / 1e6) / 0.1;  // Msps over 100ms
        intervalRates.push_back(intervalRate);
    }

    double avgRate = 0, minRate = 1e9, maxRate = 0;
    for (double rate : intervalRates) {
        avgRate += rate;
        if (rate < minRate) minRate = rate;
        if (rate > maxRate) maxRate = rate;
    }
    avgRate /= intervalRates.size();

    // Calculate standard deviation
    double variance = 0;
    for (double rate : intervalRates) {
        variance += (rate - avgRate) * (rate - avgRate);
    }
    double stddev = std::sqrt(variance / intervalRates.size());

    printf("\n");
    printf("Results:\n");
    printf("--------------------------------------------------------------------------------\n");
    printf("  Total samples processed: %" PRIu64 "\n", totalSamples);
    printf("  Total callbacks:         %" PRIu64 "\n", g_callbackCount.load());
    printf("  Elapsed time:            %.2f seconds\n", elapsed);
    printf("  Average throughput:      %.2f Msps\n", msps);
    printf("--------------------------------------------------------------------------------\n");
    printf("  Throughput variance:\n");
    printf("    Min rate:   %.2f Msps\n", minRate);
    printf("    Max rate:   %.2f Msps\n", maxRate);
    printf("    Avg rate:   %.2f Msps\n", avgRate);
    printf("    Std dev:    %.2f Msps (%.1f%% of mean)\n", stddev, (stddev/avgRate)*100);
    printf("--------------------------------------------------------------------------------\n");

    auto inputBuffer = usb->getInputBuffer();
    if (inputBuffer) {
        printf("  Buffer statistics:\n");
        printf("    Full events:  %d\n", inputBuffer->getFullCount());
        printf("    Empty events: %d\n", inputBuffer->getEmptyCount());
    }
    printf("================================================================================\n\n");

    // Verify stability
    REQUIRE_TRUE(totalSamples > 0);
    REQUIRE_TRUE(g_callbackCount.load() > 0);

    // Throughput should be relatively stable (stddev < 20% of mean)
    CHECK_TRUE(stddev < avgRate * 0.2);

    delete radio;
    delete usb;
}

/**
 * Per-callback latency benchmark
 * Measures time between consecutive callbacks to assess jitter
 */
TEST_CASE(BenchmarkFixture, CallbackJitterBenchmark)
{
    printf("\n");
    printf("================================================================================\n");
    printf("                      CALLBACK JITTER BENCHMARK\n");
    printf("================================================================================\n");

    // Track inter-callback timing
    static std::vector<double> callbackIntervals;
    static high_resolution_clock::time_point lastCallbackTime;
    static std::atomic<bool> firstCallback{true};
    static std::mutex intervalMutex;

    callbackIntervals.clear();
    callbackIntervals.reserve(100000);
    firstCallback = true;

    auto jitterCallback = [](void* context, const float* data, uint32_t len) {
        auto now = high_resolution_clock::now();

        if (firstCallback.exchange(false)) {
            lastCallbackTime = now;
            return;
        }

        double intervalUs = duration_cast<nanoseconds>(now - lastCallbackTime).count() / 1000.0;
        lastCallbackTime = now;

        std::lock_guard<std::mutex> lock(intervalMutex);
        if (callbackIntervals.size() < 100000) {
            callbackIntervals.push_back(intervalUs);
        }
    };

    auto usb = new fx3handler_benchmark();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, jitterCallback);

    // Test at decimation 2 (moderate rate)
    printf("Measuring callback jitter at decimation 2...\n");

    radio->Start(2);
    std::this_thread::sleep_for(seconds(3));
    radio->Stop();

    // Analyze jitter
    if (callbackIntervals.size() > 100) {
        double sum = 0, minInterval = 1e9, maxInterval = 0;
        for (double interval : callbackIntervals) {
            sum += interval;
            if (interval < minInterval) minInterval = interval;
            if (interval > maxInterval) maxInterval = interval;
        }
        double avgInterval = sum / callbackIntervals.size();

        // Calculate standard deviation
        double variance = 0;
        for (double interval : callbackIntervals) {
            variance += (interval - avgInterval) * (interval - avgInterval);
        }
        double stddev = std::sqrt(variance / callbackIntervals.size());

        // Calculate percentiles
        std::vector<double> sorted = callbackIntervals;
        std::sort(sorted.begin(), sorted.end());
        double p50 = sorted[sorted.size() / 2];
        double p95 = sorted[static_cast<size_t>(sorted.size() * 0.95)];
        double p99 = sorted[static_cast<size_t>(sorted.size() * 0.99)];

        printf("\n");
        printf("Callback Interval Statistics (microseconds):\n");
        printf("--------------------------------------------------------------------------------\n");
        printf("  Samples collected: %zu\n", callbackIntervals.size());
        printf("  Min interval:      %.1f us\n", minInterval);
        printf("  Max interval:      %.1f us\n", maxInterval);
        printf("  Avg interval:      %.1f us\n", avgInterval);
        printf("  Std deviation:     %.1f us\n", stddev);
        printf("--------------------------------------------------------------------------------\n");
        printf("  Percentiles:\n");
        printf("    50th (median):   %.1f us\n", p50);
        printf("    95th:            %.1f us\n", p95);
        printf("    99th:            %.1f us\n", p99);
        printf("--------------------------------------------------------------------------------\n");
        printf("  Jitter (max-min):  %.1f us\n", maxInterval - minInterval);
        printf("  Coeff of Var:      %.1f%%\n", (stddev / avgInterval) * 100);
        printf("================================================================================\n\n");

        REQUIRE_TRUE(callbackIntervals.size() > 100);
    } else {
        printf("  ERROR: Insufficient callback data collected\n");
        REQUIRE_TRUE(false);
    }

    delete radio;
    delete usb;
}

/**
 * Multi-rate switching benchmark
 * Tests how quickly the system can switch between decimation rates
 */
TEST_CASE(BenchmarkFixture, RateSwitchingBenchmark)
{
    printf("\n");
    printf("================================================================================\n");
    printf("                      RATE SWITCHING BENCHMARK\n");
    printf("================================================================================\n");

    auto usb = new fx3handler_benchmark();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, BenchmarkCallback);

    const int switchCycles = 10;
    std::vector<double> switchTimes;
    switchTimes.reserve(switchCycles * 5);

    printf("Testing rapid rate switching (%d cycles through all decimation levels)...\n", switchCycles);

    for (int cycle = 0; cycle < switchCycles; cycle++) {
        for (int decimate = 0; decimate < 5; decimate++) {
            ResetStats();

            auto startSwitch = high_resolution_clock::now();
            radio->Start(decimate);

            // Wait for data to flow
            while (!g_firstCallbackReceived.load()) {
                std::this_thread::sleep_for(100us);
            }

            auto dataFlowing = high_resolution_clock::now();
            double switchTimeMs = duration_cast<microseconds>(dataFlowing - startSwitch).count() / 1000.0;
            switchTimes.push_back(switchTimeMs);

            // Brief operation
            std::this_thread::sleep_for(50ms);

            radio->Stop();
        }
    }

    // Analyze switching times
    double sum = 0, minTime = 1e9, maxTime = 0;
    for (double t : switchTimes) {
        sum += t;
        if (t < minTime) minTime = t;
        if (t > maxTime) maxTime = t;
    }
    double avgTime = sum / switchTimes.size();

    double variance = 0;
    for (double t : switchTimes) {
        variance += (t - avgTime) * (t - avgTime);
    }
    double stddev = std::sqrt(variance / switchTimes.size());

    printf("\n");
    printf("Rate Switch Time Statistics (milliseconds):\n");
    printf("--------------------------------------------------------------------------------\n");
    printf("  Total switches:    %zu\n", switchTimes.size());
    printf("  Min switch time:   %.2f ms\n", minTime);
    printf("  Max switch time:   %.2f ms\n", maxTime);
    printf("  Avg switch time:   %.2f ms\n", avgTime);
    printf("  Std deviation:     %.2f ms\n", stddev);
    printf("================================================================================\n\n");

    REQUIRE_TRUE(switchTimes.size() == static_cast<size_t>(switchCycles * 5));

    delete radio;
    delete usb;
}

/**
 * High-rate throughput benchmark (128 Msps)
 * Tests the system at 128 MHz ADC rate with 6 decimation levels
 */
TEST_CASE(BenchmarkFixture, HighRateThroughputBenchmark)
{
    const uint32_t HIGH_ADC_FREQ = 128000000;  // 128 MHz

    printf("\n");
    printf("================================================================================\n");
    printf("               HIGH-RATE DSP THROUGHPUT BENCHMARK (128 Msps)\n");
    printf("================================================================================\n");
    printf("Configuration:\n");
    printf("  Transfer size:    %u bytes (%u samples)\n", transferSize, transferSamples);
    printf("  ADC sample rate:  %u Hz (%.1f Msps)\n", HIGH_ADC_FREQ, HIGH_ADC_FREQ / 1e6);
    printf("  Test duration:    %d seconds per decimation level\n", BENCHMARK_DURATION_SECONDS);
    printf("  Decimation levels: 6 (above 80 MHz threshold)\n");
    printf("================================================================================\n\n");

    // Save original ADC frequency and set to 128 MHz
    uint32_t originalAdcFreq = adcnominalfreq;
    adcnominalfreq = HIGH_ADC_FREQ;

    auto usb = new fx3handler_benchmark();
    auto radio = new RadioHandlerClass();

    radio->Init(usb, BenchmarkCallback);
    radio->UpdateSampleRate(HIGH_ADC_FREQ);

    std::vector<BenchmarkStats> results;

    // Test each decimation level (0-5 at 128 MHz - 6 levels)
    // At 128 MHz: dec=0 -> 64 Msps, dec=1 -> 32 Msps, dec=2 -> 16 Msps,
    //             dec=3 -> 8 Msps, dec=4 -> 4 Msps, dec=5 -> 2 Msps
    for (int srate_idx = 0; srate_idx < 6; srate_idx++)
    {
        ResetStats();

        // Calculate expected output sample rate for this decimation
        // decimate = 5 - srate_idx (since adcnominalfreq > N2_BANDSWITCH)
        int decimate = 5 - srate_idx;
        uint32_t expectedOutputRate = HIGH_ADC_FREQ / 2;  // DDC halves the rate
        for (int i = 0; i < decimate; i++) {
            expectedOutputRate /= 2;
        }

        printf("Testing srate_idx %d (decimate=%d, expected %.1f Msps output)...\n",
               srate_idx, decimate, expectedOutputRate / 1e6);

        // Start streaming
        auto startTime = high_resolution_clock::now();
        radio->Start(srate_idx);

        // Wait for input to start
        while (!usb->isInputStarted()) {
            std::this_thread::sleep_for(1ms);
        }
        g_inputStartTime = usb->getFirstInputTime();

        // Run for specified duration
        std::this_thread::sleep_for(seconds(BENCHMARK_DURATION_SECONDS));

        // Stop and collect stats
        radio->Stop();
        auto endTime = high_resolution_clock::now();

        // Collect results
        BenchmarkStats stats;
        stats.callbackCount = g_callbackCount.load();
        stats.totalSamples = g_totalSamples.load();
        stats.elapsedSeconds = duration<double>(endTime - startTime).count();

        // Calculate Msps
        stats.msps = (stats.totalSamples / 1e6) / stats.elapsedSeconds;

        // Calculate real-time efficiency
        double expectedSamples = expectedOutputRate * stats.elapsedSeconds;
        stats.realtimePercent = (stats.totalSamples / expectedSamples) * 100.0;

        // Get buffer statistics from the ringbuffer
        auto inputBuffer = usb->getInputBuffer();
        if (inputBuffer) {
            stats.bufferFullEvents = inputBuffer->getFullCount();
            stats.bufferEmptyEvents = inputBuffer->getEmptyCount();
        }

        results.push_back(stats);

        // Brief pause between tests
        std::this_thread::sleep_for(100ms);
    }

    // Print benchmark results
    printf("\n");
    printf("================================================================================\n");
    printf("                     128 Msps BENCHMARK RESULTS\n");
    printf("================================================================================\n");
    printf("\nThroughput by Sample Rate Index:\n");
    printf("--------------------------------------------------------------------------------\n");
    printf("  Idx | Dec |  Expected Rate  |  Actual Rate  |  Efficiency  |  Callbacks\n");
    printf("--------------------------------------------------------------------------------\n");

    for (int i = 0; i < 6; i++) {
        int decimate = 5 - i;
        uint32_t expectedRate = HIGH_ADC_FREQ / 2;
        for (int j = 0; j < decimate; j++) expectedRate /= 2;

        printf("   %d  |  %d  |   %6.2f Msps   |  %6.2f Msps  |   %6.1f%%    |  %" PRIu64 "\n",
               i, decimate,
               expectedRate / 1e6,
               results[i].msps,
               results[i].realtimePercent,
               results[i].callbackCount);
    }
    printf("--------------------------------------------------------------------------------\n");

    printf("\nBuffer Statistics:\n");
    printf("--------------------------------------------------------------------------------\n");
    int totalFull = 0, totalEmpty = 0;
    for (const auto& s : results) {
        totalFull += s.bufferFullEvents;
        totalEmpty += s.bufferEmptyEvents;
    }
    printf("  Buffer full events (input overrun):   %d\n", totalFull);
    printf("  Buffer empty events (output underrun): %d\n", totalEmpty);
    printf("--------------------------------------------------------------------------------\n");

    // Summary
    printf("\n================================================================================\n");
    printf("                            128 Msps SUMMARY\n");
    printf("================================================================================\n");

    double avgEfficiency = 0;
    for (const auto& s : results) {
        avgEfficiency += s.realtimePercent;
    }
    avgEfficiency /= results.size();

    printf("  Average real-time efficiency: %.1f%%\n", avgEfficiency);
    printf("  Total buffer overruns:        %d\n", totalFull);
    printf("  Total buffer underruns:       %d\n", totalEmpty);

    // Check critical bottleneck - srate_idx 0 (full 64 Msps bandwidth)
    printf("\n  Critical bottleneck check:\n");
    printf("    srate_idx 0 (64 Msps): %.1f%% efficiency %s\n",
           results[0].realtimePercent,
           results[0].realtimePercent >= 100.0 ? "(PASS)" : "(BOTTLENECK)");

    if (avgEfficiency >= 100.0) {
        printf("\n  STATUS: PASS - System can process 128 Msps faster than real-time\n");
    } else {
        printf("\n  STATUS: WARNING - System may not keep up with 128 Msps data\n");
    }
    printf("================================================================================\n\n");

    // Verify basic functionality
    for (int i = 0; i < 6; i++) {
        REQUIRE_TRUE(results[i].callbackCount > 0);
        REQUIRE_TRUE(results[i].totalSamples > 0);
    }

    // Restore original ADC frequency
    adcnominalfreq = originalAdcFreq;

    delete radio;
    delete usb;
}
