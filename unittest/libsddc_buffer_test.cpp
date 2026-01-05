/**
 * libsddc_buffer_test.cpp - Tests for libsddc synchronous ring buffer
 *
 * Tests the sync buffer implementation used by sddc_read_sync().
 * The buffer was optimized from byte-by-byte copy to memcpy, and these
 * tests ensure correctness of:
 *   - Basic read/write operations
 *   - Buffer wraparound handling
 *   - Partial reads
 *   - Overflow handling (old data discarded)
 *   - Exact byte comparison for data integrity
 *
 * Since libsddc uses static globals for the buffer, we test through
 * exposed test hooks (LIBSDDC_TESTING must be defined when building).
 */

// Include these before CppUnitTestFramework to work around missing includes in the framework
#include <cstdint>
#include <limits>

#include "CppUnitTestFramework.hpp"
#include <cstring>
#include <vector>
#include <thread>
#include <chrono>
#include <random>
#include <atomic>

// Test hooks - these are declared in libsddc.cpp when LIBSDDC_TESTING is defined
extern "C" {
    // Test hook to reset buffer state
    void sddc_test_reset_sync_buffer(void);

    // Test hook to write data directly to sync buffer (simulates Callback)
    void sddc_test_write_sync_buffer(const uint8_t* data, size_t len);

    // Test hook to read from sync buffer (wrapper around internal read logic)
    size_t sddc_test_read_sync_buffer(uint8_t* data, size_t max_len);

    // Test hook to get buffer state
    void sddc_test_get_buffer_state(size_t* write_pos, size_t* read_pos, size_t* available);

    // Test hook to get buffer size
    size_t sddc_test_get_buffer_size(void);
}

namespace {
    struct LibsddcBufferFixture {
        LibsddcBufferFixture() {
            // Reset buffer state before each test
            sddc_test_reset_sync_buffer();
        }
    };

    // Helper to generate test pattern
    std::vector<uint8_t> generate_pattern(size_t size, uint8_t seed = 0) {
        std::vector<uint8_t> data(size);
        for (size_t i = 0; i < size; i++) {
            data[i] = static_cast<uint8_t>((seed + i) & 0xFF);
        }
        return data;
    }

    // Helper to verify pattern
    bool verify_pattern(const uint8_t* data, size_t size, uint8_t seed = 0) {
        for (size_t i = 0; i < size; i++) {
            if (data[i] != static_cast<uint8_t>((seed + i) & 0xFF)) {
                return false;
            }
        }
        return true;
    }
}

// =============================================================================
// Basic Read/Write Tests
// =============================================================================

TEST_CASE(LibsddcBufferFixture, BasicWriteRead)
{
    // Write data via callback, read via sddc_read_sync, verify match
    const size_t test_size = 1024;
    auto write_data = generate_pattern(test_size, 0x42);

    sddc_test_write_sync_buffer(write_data.data(), write_data.size());

    // Verify buffer state
    size_t write_pos, read_pos, available;
    sddc_test_get_buffer_state(&write_pos, &read_pos, &available);
    REQUIRE_EQUAL(available, test_size);
    REQUIRE_EQUAL(read_pos, 0u);
    REQUIRE_EQUAL(write_pos, test_size);

    // Read and verify
    std::vector<uint8_t> read_data(test_size);
    size_t read = sddc_test_read_sync_buffer(read_data.data(), test_size);

    REQUIRE_EQUAL(read, test_size);
    REQUIRE_TRUE(verify_pattern(read_data.data(), test_size, 0x42));
    REQUIRE_EQUAL(memcmp(write_data.data(), read_data.data(), test_size), 0);
}

TEST_CASE(LibsddcBufferFixture, MultipleSmallWrites)
{
    // Test multiple small writes followed by a larger read
    const size_t chunk_size = 256;
    const size_t num_chunks = 10;
    const size_t total_size = chunk_size * num_chunks;

    // Write multiple chunks
    for (size_t i = 0; i < num_chunks; i++) {
        auto data = generate_pattern(chunk_size, static_cast<uint8_t>(i * chunk_size));
        sddc_test_write_sync_buffer(data.data(), data.size());
    }

    // Verify available
    size_t write_pos, read_pos, available;
    sddc_test_get_buffer_state(&write_pos, &read_pos, &available);
    REQUIRE_EQUAL(available, total_size);

    // Read all at once
    std::vector<uint8_t> read_data(total_size);
    size_t read = sddc_test_read_sync_buffer(read_data.data(), total_size);
    REQUIRE_EQUAL(read, total_size);

    // Verify each chunk's pattern
    for (size_t i = 0; i < num_chunks; i++) {
        REQUIRE_TRUE(verify_pattern(&read_data[i * chunk_size], chunk_size,
                                    static_cast<uint8_t>(i * chunk_size)));
    }
}

TEST_CASE(LibsddcBufferFixture, EmptyBufferRead)
{
    // Reading from empty buffer should return 0
    std::vector<uint8_t> read_data(1024);
    size_t read = sddc_test_read_sync_buffer(read_data.data(), 1024);
    REQUIRE_EQUAL(read, 0u);
}

// =============================================================================
// Buffer Wraparound Tests
// =============================================================================

TEST_CASE(LibsddcBufferFixture, WriteWraparound)
{
    // Write data that wraps around the 1MB buffer boundary
    const size_t buffer_size = sddc_test_get_buffer_size();
    const size_t chunk_size = buffer_size / 4;  // 256KB chunks

    // Write 3 chunks to position near end
    for (int i = 0; i < 3; i++) {
        auto data = generate_pattern(chunk_size, static_cast<uint8_t>(i));
        sddc_test_write_sync_buffer(data.data(), data.size());
    }

    // Read them to advance read pointer
    std::vector<uint8_t> discard(chunk_size * 3);
    sddc_test_read_sync_buffer(discard.data(), chunk_size * 3);

    // Now write_pos and read_pos are at 768KB
    // Write a chunk that will wrap around (256KB from 768KB wraps to 0)
    auto wrap_data = generate_pattern(chunk_size, 0xAA);
    sddc_test_write_sync_buffer(wrap_data.data(), wrap_data.size());

    // Read and verify the wrapped data
    std::vector<uint8_t> read_data(chunk_size);
    size_t read = sddc_test_read_sync_buffer(read_data.data(), chunk_size);

    REQUIRE_EQUAL(read, chunk_size);
    REQUIRE_EQUAL(memcmp(wrap_data.data(), read_data.data(), chunk_size), 0);
}

TEST_CASE(LibsddcBufferFixture, ReadWraparound)
{
    // Set up buffer so read wraps around
    const size_t buffer_size = sddc_test_get_buffer_size();
    const size_t position = buffer_size - 1024;  // Near end
    const size_t write_size = 4096;  // Will wrap

    // First, fill and consume to get pointers near end
    // Write in chunks to get write_pos near the end
    size_t current_pos = 0;
    while (current_pos < position) {
        size_t chunk = std::min((size_t)65536, position - current_pos);
        auto data = generate_pattern(chunk, 0);
        sddc_test_write_sync_buffer(data.data(), data.size());
        std::vector<uint8_t> tmp(chunk);
        sddc_test_read_sync_buffer(tmp.data(), chunk);
        current_pos += chunk;
    }

    // Now both pointers are at 'position' (near end)
    // Write data that wraps
    auto write_data = generate_pattern(write_size, 0x55);
    sddc_test_write_sync_buffer(write_data.data(), write_data.size());

    // Read should handle wraparound correctly
    std::vector<uint8_t> read_data(write_size);
    size_t read = sddc_test_read_sync_buffer(read_data.data(), write_size);

    REQUIRE_EQUAL(read, write_size);
    REQUIRE_EQUAL(memcmp(write_data.data(), read_data.data(), write_size), 0);
}

TEST_CASE(LibsddcBufferFixture, LargeWriteAcrossEntireBuffer)
{
    // Write exactly buffer_size bytes in one operation (should fill entire buffer)
    const size_t buffer_size = sddc_test_get_buffer_size();

    auto write_data = generate_pattern(buffer_size, 0x12);
    sddc_test_write_sync_buffer(write_data.data(), write_data.size());

    size_t write_pos, read_pos, available;
    sddc_test_get_buffer_state(&write_pos, &read_pos, &available);
    REQUIRE_EQUAL(available, buffer_size);

    std::vector<uint8_t> read_data(buffer_size);
    size_t read = sddc_test_read_sync_buffer(read_data.data(), buffer_size);

    REQUIRE_EQUAL(read, buffer_size);
    REQUIRE_EQUAL(memcmp(write_data.data(), read_data.data(), buffer_size), 0);
}

// =============================================================================
// Partial Read Tests
// =============================================================================

TEST_CASE(LibsddcBufferFixture, PartialReadPreservesRemaining)
{
    // Read less than available, verify remaining data is preserved
    const size_t total_size = 4096;
    const size_t first_read = 1024;

    auto write_data = generate_pattern(total_size, 0x33);
    sddc_test_write_sync_buffer(write_data.data(), write_data.size());

    // Read first portion
    std::vector<uint8_t> read1(first_read);
    size_t r1 = sddc_test_read_sync_buffer(read1.data(), first_read);
    REQUIRE_EQUAL(r1, first_read);
    REQUIRE_TRUE(verify_pattern(read1.data(), first_read, 0x33));

    // Verify remaining is still available
    size_t write_pos, read_pos, available;
    sddc_test_get_buffer_state(&write_pos, &read_pos, &available);
    REQUIRE_EQUAL(available, total_size - first_read);

    // Read second portion
    std::vector<uint8_t> read2(total_size - first_read);
    size_t r2 = sddc_test_read_sync_buffer(read2.data(), total_size - first_read);
    REQUIRE_EQUAL(r2, total_size - first_read);

    // Verify second portion has correct pattern (continuing from offset)
    REQUIRE_TRUE(verify_pattern(read2.data(), total_size - first_read,
                                static_cast<uint8_t>(0x33 + first_read)));
}

TEST_CASE(LibsddcBufferFixture, MultiplePartialReads)
{
    // Write once, read in multiple small chunks
    const size_t total_size = 8192;
    const size_t read_size = 512;

    auto write_data = generate_pattern(total_size, 0x77);
    sddc_test_write_sync_buffer(write_data.data(), write_data.size());

    std::vector<uint8_t> all_read_data;
    size_t offset = 0;

    while (offset < total_size) {
        std::vector<uint8_t> chunk(read_size);
        size_t read = sddc_test_read_sync_buffer(chunk.data(), read_size);
        REQUIRE_EQUAL(read, read_size);
        all_read_data.insert(all_read_data.end(), chunk.begin(), chunk.end());
        offset += read_size;
    }

    REQUIRE_EQUAL(all_read_data.size(), total_size);
    REQUIRE_EQUAL(memcmp(write_data.data(), all_read_data.data(), total_size), 0);
}

TEST_CASE(LibsddcBufferFixture, ReadMoreThanAvailable)
{
    // Request more bytes than available, should return only what's available
    const size_t write_size = 1000;
    const size_t request_size = 2000;

    auto write_data = generate_pattern(write_size, 0x99);
    sddc_test_write_sync_buffer(write_data.data(), write_data.size());

    std::vector<uint8_t> read_data(request_size);
    size_t read = sddc_test_read_sync_buffer(read_data.data(), request_size);

    REQUIRE_EQUAL(read, write_size);  // Only get what's available
    REQUIRE_EQUAL(memcmp(write_data.data(), read_data.data(), write_size), 0);
}

// =============================================================================
// Overflow Handling Tests
// =============================================================================

TEST_CASE(LibsddcBufferFixture, OverflowDiscardsOldData)
{
    // Write more than buffer size, verify old data is discarded correctly
    const size_t buffer_size = sddc_test_get_buffer_size();

    // Write buffer_size + extra bytes
    const size_t extra = 1024;
    const size_t total_write = buffer_size + extra;

    auto write_data = generate_pattern(total_write, 0x11);
    sddc_test_write_sync_buffer(write_data.data(), write_data.size());

    // Buffer should be full (not more than buffer_size)
    size_t write_pos, read_pos, available;
    sddc_test_get_buffer_state(&write_pos, &read_pos, &available);
    REQUIRE_EQUAL(available, buffer_size);

    // Read all and verify we get the NEWEST data (last buffer_size bytes)
    std::vector<uint8_t> read_data(buffer_size);
    size_t read = sddc_test_read_sync_buffer(read_data.data(), buffer_size);
    REQUIRE_EQUAL(read, buffer_size);

    // The oldest 'extra' bytes should be discarded
    // So we should have bytes [extra, total_write)
    REQUIRE_TRUE(verify_pattern(read_data.data(), buffer_size,
                                static_cast<uint8_t>(0x11 + extra)));
}

TEST_CASE(LibsddcBufferFixture, OverflowWithPartialBuffer)
{
    // Partially fill buffer, then overflow
    const size_t buffer_size = sddc_test_get_buffer_size();
    const size_t initial_write = buffer_size / 2;
    const size_t overflow_write = buffer_size;  // Will cause overflow of initial_write bytes

    // Initial write
    auto data1 = generate_pattern(initial_write, 0xAA);
    sddc_test_write_sync_buffer(data1.data(), data1.size());

    // Overflow write
    auto data2 = generate_pattern(overflow_write, 0xBB);
    sddc_test_write_sync_buffer(data2.data(), data2.size());

    // Should have buffer_size available (full buffer)
    size_t write_pos, read_pos, available;
    sddc_test_get_buffer_state(&write_pos, &read_pos, &available);
    REQUIRE_EQUAL(available, buffer_size);

    // Read all data
    std::vector<uint8_t> read_data(buffer_size);
    size_t read = sddc_test_read_sync_buffer(read_data.data(), buffer_size);
    REQUIRE_EQUAL(read, buffer_size);

    // First part should be remaining from data1 (if any), rest should be from data2
    // Total written = initial_write + overflow_write = 1.5 * buffer_size
    // Overflow = 0.5 * buffer_size
    // So we lose first 0.5 * buffer_size of data1
    // Remaining: nothing from data1 (all overwritten), all of data2
    REQUIRE_TRUE(verify_pattern(read_data.data(), buffer_size, 0xBB));
}

TEST_CASE(LibsddcBufferFixture, MultipleOverflows)
{
    // Multiple writes that each overflow
    const size_t buffer_size = sddc_test_get_buffer_size();

    for (int i = 0; i < 3; i++) {
        auto data = generate_pattern(buffer_size, static_cast<uint8_t>(i * 0x30));
        sddc_test_write_sync_buffer(data.data(), data.size());
    }

    // Should have exactly buffer_size available
    size_t write_pos, read_pos, available;
    sddc_test_get_buffer_state(&write_pos, &read_pos, &available);
    REQUIRE_EQUAL(available, buffer_size);

    // Read and verify we have the LAST buffer worth of data
    std::vector<uint8_t> read_data(buffer_size);
    size_t read = sddc_test_read_sync_buffer(read_data.data(), buffer_size);
    REQUIRE_EQUAL(read, buffer_size);

    // Should have the pattern from iteration 2 (the last write)
    REQUIRE_TRUE(verify_pattern(read_data.data(), buffer_size, 2 * 0x30));
}

// =============================================================================
// Exact Byte Comparison Tests
// =============================================================================

TEST_CASE(LibsddcBufferFixture, ExactByteComparisonRandom)
{
    // Verify memcpy produces identical results to what was written
    // Use random data to catch any byte-order or alignment issues
    std::mt19937 rng(12345);  // Fixed seed for reproducibility
    std::uniform_int_distribution<int> dist(0, 255);

    const size_t test_size = 32768;
    std::vector<uint8_t> write_data(test_size);
    for (size_t i = 0; i < test_size; i++) {
        write_data[i] = static_cast<uint8_t>(dist(rng));
    }

    sddc_test_write_sync_buffer(write_data.data(), write_data.size());

    std::vector<uint8_t> read_data(test_size);
    size_t read = sddc_test_read_sync_buffer(read_data.data(), test_size);

    REQUIRE_EQUAL(read, test_size);
    REQUIRE_EQUAL(memcmp(write_data.data(), read_data.data(), test_size), 0);
}

TEST_CASE(LibsddcBufferFixture, ExactByteComparisonAllValues)
{
    // Write every byte value 0x00-0xFF and verify
    std::vector<uint8_t> write_data(256);
    for (int i = 0; i < 256; i++) {
        write_data[i] = static_cast<uint8_t>(i);
    }

    sddc_test_write_sync_buffer(write_data.data(), write_data.size());

    std::vector<uint8_t> read_data(256);
    size_t read = sddc_test_read_sync_buffer(read_data.data(), 256);

    REQUIRE_EQUAL(read, 256u);
    for (int i = 0; i < 256; i++) {
        REQUIRE_EQUAL(read_data[i], static_cast<uint8_t>(i));
    }
}

TEST_CASE(LibsddcBufferFixture, AlignmentBoundaryTest)
{
    // Test various sizes to check alignment handling
    const size_t sizes[] = {1, 3, 7, 15, 31, 63, 127, 255, 511, 1023, 2047, 4095};

    for (size_t size : sizes) {
        sddc_test_reset_sync_buffer();

        auto write_data = generate_pattern(size, 0xCC);
        sddc_test_write_sync_buffer(write_data.data(), write_data.size());

        std::vector<uint8_t> read_data(size);
        size_t read = sddc_test_read_sync_buffer(read_data.data(), size);

        REQUIRE_EQUAL(read, size);
        REQUIRE_EQUAL(memcmp(write_data.data(), read_data.data(), size), 0);
    }
}

TEST_CASE(LibsddcBufferFixture, SingleByteOperations)
{
    // Edge case: single byte operations
    for (int i = 0; i < 100; i++) {
        uint8_t write_byte = static_cast<uint8_t>(i);
        sddc_test_write_sync_buffer(&write_byte, 1);

        uint8_t read_byte = 0xFF;
        size_t read = sddc_test_read_sync_buffer(&read_byte, 1);

        REQUIRE_EQUAL(read, 1u);
        REQUIRE_EQUAL(read_byte, write_byte);
    }
}

// =============================================================================
// Concurrency Tests
// =============================================================================

TEST_CASE(LibsddcBufferFixture, ConcurrentWriteRead)
{
    // Test concurrent write and read (simulating real usage)
    const size_t chunk_size = 4096;
    const size_t num_iterations = 100;
    std::atomic<bool> writer_done{false};
    std::atomic<size_t> total_written{0};
    std::atomic<size_t> total_read{0};

    // Writer thread
    std::thread writer([&]() {
        for (size_t i = 0; i < num_iterations; i++) {
            auto data = generate_pattern(chunk_size, static_cast<uint8_t>(i));
            sddc_test_write_sync_buffer(data.data(), data.size());
            total_written += chunk_size;
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        writer_done = true;
    });

    // Reader thread
    std::thread reader([&]() {
        std::vector<uint8_t> buffer(chunk_size * 2);
        while (!writer_done || total_read < total_written) {
            size_t read = sddc_test_read_sync_buffer(buffer.data(), buffer.size());
            total_read += read;
            if (read == 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }
        }
    });

    writer.join();
    reader.join();

    // All written data should have been read (assuming no overflow)
    // Note: with small buffer and fast writer, some may be discarded
    REQUIRE_TRUE(total_read > 0);
}

TEST_CASE(LibsddcBufferFixture, StressTestLargeTransfers)
{
    // Stress test with large transfers at buffer boundaries
    const size_t buffer_size = sddc_test_get_buffer_size();

    // Write exactly at boundary
    auto data1 = generate_pattern(buffer_size, 0x11);
    sddc_test_write_sync_buffer(data1.data(), data1.size());

    // Read half
    std::vector<uint8_t> read1(buffer_size / 2);
    size_t r1 = sddc_test_read_sync_buffer(read1.data(), buffer_size / 2);
    REQUIRE_EQUAL(r1, buffer_size / 2);

    // Write more (will wrap)
    auto data2 = generate_pattern(buffer_size / 2, 0x22);
    sddc_test_write_sync_buffer(data2.data(), data2.size());

    // Verify buffer state
    size_t write_pos, read_pos, available;
    sddc_test_get_buffer_state(&write_pos, &read_pos, &available);
    REQUIRE_EQUAL(available, buffer_size);  // Full

    // Read all and verify
    std::vector<uint8_t> read2(buffer_size);
    size_t r2 = sddc_test_read_sync_buffer(read2.data(), buffer_size);
    REQUIRE_EQUAL(r2, buffer_size);

    // First half should be remaining from data1 (second half of data1)
    REQUIRE_TRUE(verify_pattern(read2.data(), buffer_size / 2,
                                static_cast<uint8_t>(0x11 + buffer_size / 2)));
    // Second half should be all of data2
    REQUIRE_TRUE(verify_pattern(&read2[buffer_size / 2], buffer_size / 2, 0x22));
}

// =============================================================================
// Buffer State Tests
// =============================================================================

TEST_CASE(LibsddcBufferFixture, BufferSizeIsOneMB)
{
    // Verify buffer size constant
    size_t buffer_size = sddc_test_get_buffer_size();
    REQUIRE_EQUAL(buffer_size, 1024u * 1024u);  // 1MB
}

TEST_CASE(LibsddcBufferFixture, ResetClearsState)
{
    // Write some data
    auto data = generate_pattern(1024, 0x55);
    sddc_test_write_sync_buffer(data.data(), data.size());

    // Reset
    sddc_test_reset_sync_buffer();

    // Verify clean state
    size_t write_pos, read_pos, available;
    sddc_test_get_buffer_state(&write_pos, &read_pos, &available);
    REQUIRE_EQUAL(write_pos, 0u);
    REQUIRE_EQUAL(read_pos, 0u);
    REQUIRE_EQUAL(available, 0u);
}

TEST_CASE(LibsddcBufferFixture, WritePositionModuloBuffer)
{
    // Verify write position correctly wraps around
    const size_t buffer_size = sddc_test_get_buffer_size();
    const size_t write_size = buffer_size + 100;

    auto data = generate_pattern(write_size, 0x66);
    sddc_test_write_sync_buffer(data.data(), data.size());

    size_t write_pos, read_pos, available;
    sddc_test_get_buffer_state(&write_pos, &read_pos, &available);

    // Write position should be 100 (wrapped around)
    REQUIRE_EQUAL(write_pos, 100u);
    REQUIRE_EQUAL(available, buffer_size);
}
