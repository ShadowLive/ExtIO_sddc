#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

const int default_count = 64;
const int spin_count = 100;
#define ALIGN (8)

class ringbufferbase {
public:
    ringbufferbase(int count) :
        max_count(count),
        read_index(0),
        write_index(0),
        emptyCount(0),
        fullCount(0),
        writeCount(0),
        stopped(false)
    {
    }

    int getFullCount() const { return fullCount; }

    int getEmptyCount() const { return emptyCount; }

    int getWriteCount() const { return writeCount; }

    void ReadDone()
    {
        int cur_write = write_index.load(std::memory_order_acquire);
        int cur_read = read_index.load(std::memory_order_relaxed);
        bool was_full = ((cur_write + 1) % max_count == cur_read);

        read_index.store((cur_read + 1) % max_count, std::memory_order_release);

        if (was_full) {
            std::lock_guard<std::mutex> lk(mutex);
            nonfullCV.notify_one();
        }
    }

    void WriteDone()
    {
        int cur_read = read_index.load(std::memory_order_acquire);
        int cur_write = write_index.load(std::memory_order_relaxed);
        bool was_empty = (cur_read == cur_write);

        write_index.store((cur_write + 1) % max_count, std::memory_order_release);
        writeCount++;

        if (was_empty) {
            std::lock_guard<std::mutex> lk(mutex);
            nonemptyCV.notify_one();
        }
    }

    void Start()
    {
        std::lock_guard<std::mutex> lk(mutex);
        write_index.store(0, std::memory_order_release);
        read_index.store(0, std::memory_order_release);
        stopped.store(false, std::memory_order_release);
    }

    void Stop()
    {
        {
            std::lock_guard<std::mutex> lk(mutex);
            read_index.store(0, std::memory_order_release);
            stopped.store(true, std::memory_order_release);
            write_index.store(max_count / 2, std::memory_order_release);
        }
        nonfullCV.notify_all();
        nonemptyCV.notify_all();
    }

protected:

    void WaitUntilNotEmpty()
    {
        if (stopped.load(std::memory_order_acquire)) return;

        // Spin-wait phase with relaxed loads for efficiency
        for (int i = 0; i < spin_count; i++) {
            if (read_index.load(std::memory_order_acquire) !=
                write_index.load(std::memory_order_acquire))
                return;
        }

        // Fall back to condition variable
        std::unique_lock<std::mutex> lk(mutex);
        emptyCount++;
        nonemptyCV.wait(lk, [this] {
            return stopped.load(std::memory_order_acquire) ||
                   read_index.load(std::memory_order_acquire) !=
                   write_index.load(std::memory_order_acquire);
        });
    }

    void WaitUntilNotFull()
    {
        if (stopped.load(std::memory_order_acquire)) return;

        // Spin-wait phase
        for (int i = 0; i < spin_count; i++) {
            int r = read_index.load(std::memory_order_acquire);
            int w = write_index.load(std::memory_order_acquire);
            if ((w + 1) % max_count != r)
                return;
        }

        // Fall back to condition variable
        std::unique_lock<std::mutex> lk(mutex);
        fullCount++;
        nonfullCV.wait(lk, [this] {
            if (stopped.load(std::memory_order_acquire)) return true;
            int r = read_index.load(std::memory_order_acquire);
            int w = write_index.load(std::memory_order_acquire);
            return (w + 1) % max_count != r;
        });
    }

    int max_count;

    std::atomic<int> read_index;
    std::atomic<int> write_index;

private:
    int emptyCount;
    int fullCount;
    int writeCount;

    std::mutex mutex;
    std::atomic<bool> stopped;
    std::condition_variable nonemptyCV;
    std::condition_variable nonfullCV;
};

template<typename T> class ringbuffer : public ringbufferbase {
    typedef T* TPtr;

public:
    ringbuffer(int count = default_count) :
        ringbufferbase(count), block_size(0)
    {
        buffers = new TPtr[max_count];
        buffers[0] = nullptr;
    }

    ~ringbuffer()
    {
        if (buffers[0])
            delete[] buffers[0];

        delete[] buffers;
    }

    void setBlockSize(int size)
    {
        if (block_size != size)
        {
            block_size = size;

            if (buffers[0])
                delete[] buffers[0];

            int aligned_block_size = (block_size + ALIGN - 1) & (~(ALIGN - 1));

            auto data = new T[max_count * aligned_block_size];

            for (int i = 0; i < max_count; ++i)
            {
                buffers[i] = &data[i * aligned_block_size];
            }
        }
    }

    T* peekWritePtr(int offset)
    {
        int w = write_index.load(std::memory_order_acquire);
        return buffers[(w + max_count + offset) % max_count];
    }

    T* peekReadPtr(int offset)
    {
        int r = read_index.load(std::memory_order_acquire);
        return buffers[(r + max_count + offset) % max_count];
    }

    T* getWritePtr()
    {
        // if there is still space
        WaitUntilNotFull();
        int w = write_index.load(std::memory_order_acquire);
        return buffers[w % max_count];
    }

    const T* getReadPtr()
    {
        WaitUntilNotEmpty();
        int r = read_index.load(std::memory_order_acquire);
        return buffers[r];
    }

    int getBlockSize() const { return block_size; }

private:
    int block_size;

    TPtr* buffers;
};