/*
 DYNAMICENGINE3D
 AI-Assisted 3D Physics Engine
 By: Elitmers
*/

#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>
#include <vector>
#include <memory>
#include <cstring>   // memcpy for atomic-float helper
#include <algorithm> // std::min
#include <cstdint>

#ifdef _WIN32
#include <windows.h>
#endif

// Physical core detection.

inline int DetectPhysicalCoreCount()
{
#ifdef _WIN32
    DWORD len = 0;
    GetLogicalProcessorInformation(nullptr, &len);
    if (len == 0)
        return (int)std::thread::hardware_concurrency();

    std::vector<char> buffer(len);
    auto* info = reinterpret_cast<SYSTEM_LOGICAL_PROCESSOR_INFORMATION*>(buffer.data());
    if (!GetLogicalProcessorInformation(info, &len))
        return (int)std::thread::hardware_concurrency();

    int physicalCores = 0;
    int entryCount = (int)(len / sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION));
    for (int i = 0; i < entryCount; ++i)
        if (info[i].Relationship == RelationProcessorCore)
            ++physicalCores;

    return physicalCores > 0 ? physicalCores : (int)std::thread::hardware_concurrency();
#else
    // Non-Windows fallback: halve logical count as a rough HT heuristic.
    int logical = (int)std::thread::hardware_concurrency();
    return logical > 1 ? logical / 2 : logical;
#endif
}

// Background worker count: physical cores minus one
inline int RecommendedWorkerThreadCount()
{
    int physical = DetectPhysicalCoreCount();
    int workers = physical - 1;
    return workers > 0 ? workers : 1;
}

// Atomic float add helper (CAS loop).

inline void AtomicAddFloat(float* dest, float value)
{
    static_assert(sizeof(float) == sizeof(uint32_t), "float must be 32 bits");
    auto* target = reinterpret_cast<std::atomic<uint32_t>*>(dest);

    uint32_t oldBits = target->load(std::memory_order_relaxed);
    for (;;)
    {
        float oldVal;
        std::memcpy(&oldVal, &oldBits, sizeof(float));
        float newVal = oldVal + value;
        uint32_t newBits;
        std::memcpy(&newBits, &newVal, sizeof(uint32_t));
        if (target->compare_exchange_weak(oldBits, newBits,
            std::memory_order_relaxed, std::memory_order_relaxed))
            break;
    }
}

inline void AtomicAddInt(int* dest, int value)
{
    reinterpret_cast<std::atomic<int>*>(dest)->fetch_add(value, std::memory_order_relaxed);
}


// Per-thread dispatch telemetry.

struct alignas(64) ThreadDispatchStats
{
    std::atomic<int> serialDispatches{ 0 };

    void bumpSerial()
    {
        serialDispatches.fetch_add(1, std::memory_order_relaxed);
    }
};

inline std::mutex& DispatchStatsMutex()
{
    static std::mutex m;
    return m;
}

// Blocks outlive the worker threads that own them; freed when the DLL unloads.
inline std::vector<std::unique_ptr<ThreadDispatchStats>>& DispatchStatsBlocks()
{
    static std::vector<std::unique_ptr<ThreadDispatchStats>> blocks;
    return blocks;
}

inline ThreadDispatchStats& LocalDispatchStats()
{
    static thread_local ThreadDispatchStats* local = nullptr;
    if (!local)
    {
        auto block = std::make_unique<ThreadDispatchStats>();
        local = block.get();
        std::lock_guard<std::mutex> lock(DispatchStatsMutex());
        DispatchStatsBlocks().push_back(std::move(block));
    }
    return *local;
}

// ThreadPool

class ThreadPool
{
public:
    explicit ThreadPool(int numThreads = 0)
    {
        spawn(numThreads > 0 ? numThreads : RecommendedWorkerThreadCount());
    }

    ~ThreadPool() { shutdown(); }

    // Non-copyable, non-movable
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    // Resize the pool (destroys old workers, spawns new ones).
    void resize(int numThreads)
    {
        shutdown();
        spawn(numThreads > 0 ? numThreads : 1);
    }

    int workerCount() const { return m_workerCount; }

    // Debug telemetry.
    int pooledThreads() const { return m_workerCount; }
    int peakThreadsUsedThisFrame() const { return m_peakDispatchThreadsUsed.load(std::memory_order_relaxed); }
    int parallelDispatchesThisFrame() const { return m_parallelDispatches.load(std::memory_order_relaxed); }

    // Summed from the per-thread blocks; the serial path touches no shared state.
    int serialDispatchesThisFrame() const
    {
        int total = 0;
        std::lock_guard<std::mutex> lock(DispatchStatsMutex());
        for (const auto& s : DispatchStatsBlocks())
            total += s->serialDispatches.load(std::memory_order_relaxed);
        return total;
    }

    void resetFrameStats()
    {
        m_peakDispatchThreadsUsed.store(0, std::memory_order_relaxed);
        m_parallelDispatches.store(0, std::memory_order_relaxed);

        std::lock_guard<std::mutex> lock(DispatchStatsMutex());
        for (const auto& s : DispatchStatsBlocks())
            s->serialDispatches.store(0, std::memory_order_relaxed);
    }

    // True while this thread runs a chunk from parallel_for, so nested dispatches go serial.
    static inline thread_local bool t_dispatching = false;

    template <typename Func>
    void parallel_for(int count, Func&& func, int minBatch = 1)
    {
        if (count <= 0) return;

        if (count <= minBatch || m_workerCount == 0 || t_dispatching)
        {
            // No workers involved — ran on the calling thread; counted into its own block.
            LocalDispatchStats().bumpSerial();
            for (int i = 0; i < count; ++i) func(i);
            return;
        }

        // Mutual exclusion so multiple concurrent callers do not corrupt dispatch state
        std::unique_lock<std::mutex> dispatchLock(m_dispatchMutex);

        if (m_workerCount == 0 || t_dispatching)
        {
            LocalDispatchStats().bumpSerial();
            for (int i = 0; i < count; ++i) func(i);
            return;
        }

        // Use only as many worker chunks as can do useful work. The caller owns
        // the final chunk, so count - 1 is the maximum useful worker count.
        const int workerChunks = std::min(m_workerCount, count - 1);
        const int totalThreads = workerChunks + 1;
        const int chunkSize = (count + totalThreads - 1) / totalThreads;

        m_parallelDispatches.fetch_add(1, std::memory_order_relaxed);
        foldMaxThreadsUsed(workerChunks);

        // Shared completion counter (workers + caller chunk).
        std::atomic<int> remaining(totalThreads);

        // Enqueue chunks for worker threads.
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_taskFunc = [&func, chunkSize, count](int threadIdx)
                {
                    int lo = threadIdx * chunkSize;
                    int hi = lo + chunkSize;
                    if (hi > count) hi = count;
                    for (int i = lo; i < hi; ++i) func(i);
                };
            m_taskRemaining = &remaining;
            m_taskThreads = workerChunks;
            m_generation.fetch_add(1, std::memory_order_release);
        }
        m_cv.notify_all();

        // Caller executes the last chunk.
        {
            int callerIdx = totalThreads - 1;
            int lo = callerIdx * chunkSize;
            int hi = lo + chunkSize;
            if (hi > count) hi = count;
            if (lo < hi)
            {
                t_dispatching = true;
                for (int i = lo; i < hi; ++i) func(i);
                t_dispatching = false;
            }
        }
        if (remaining.fetch_sub(1, std::memory_order_acq_rel) == 1)
            remaining.notify_all();

        // Bounded spin, then a real block.
        constexpr int kMaxSpins = 2000;
        int spins = 0;
        int cur;
        while ((cur = remaining.load(std::memory_order_acquire)) > 0)
        {
            if (spins < kMaxSpins)
            {
                ++spins;
                std::this_thread::yield();
            }
            else
            {
                remaining.wait(cur, std::memory_order_acquire);
            }
        }

        // Clear task under lock before releasing dispatchLock
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_taskFunc = nullptr;
            m_taskRemaining = nullptr;
            m_taskThreads = 0;
        }
    }

private:
    void foldMaxThreadsUsed(int used)
    {
        int cur = m_peakDispatchThreadsUsed.load(std::memory_order_relaxed);
        while (used > cur && !m_peakDispatchThreadsUsed.compare_exchange_weak(
            cur, used, std::memory_order_relaxed, std::memory_order_relaxed)) {
        }
    }

    void spawn(int n)
    {
        m_workerCount = n;
        m_running.store(true, std::memory_order_relaxed);
        m_generation.store(0, std::memory_order_relaxed);
        m_taskRemaining = nullptr;
        m_taskThreads = 0;
        m_peakDispatchThreadsUsed.store(0, std::memory_order_relaxed);

        m_workers.reserve(n);
        for (int id = 0; id < n; ++id)
            m_workers.emplace_back(&ThreadPool::workerLoop, this, id);
    }

    void shutdown()
    {
        std::lock_guard<std::mutex> dispatchLock(m_dispatchMutex);
        if (!m_running.load(std::memory_order_relaxed)) return;
        m_running.store(false, std::memory_order_release);
        m_cv.notify_all();
        for (auto& t : m_workers)
            if (t.joinable()) t.join();
        m_workers.clear();
        m_workerCount = 0;
    }

    void workerLoop(int workerId)
    {
        uint64_t localGen = 0;
        while (m_running.load(std::memory_order_acquire))
        {
            std::function<void(int)> task;
            std::atomic<int>* pRemaining = nullptr;
            bool hasWork = false;

            // Wait for new work under lock.
            {
                std::unique_lock<std::mutex> lock(m_mutex);
                m_cv.wait(lock, [&]
                    {
                        return !m_running.load(std::memory_order_relaxed) ||
                            m_generation.load(std::memory_order_relaxed) != localGen;
                    });

                if (!m_running.load(std::memory_order_relaxed)) break;

                localGen = m_generation.load(std::memory_order_relaxed);

                if (workerId < m_taskThreads && m_taskFunc)
                {
                    task = m_taskFunc;
                    pRemaining = m_taskRemaining;
                    hasWork = true;
                }
            }

            // Execute this worker's chunk outside the lock.
            if (hasWork && task)
            {
                t_dispatching = true;
                task(workerId);
                t_dispatching = false;
            }

            // Signal completion only if this worker owned a real chunk.
            if (hasWork && pRemaining)
            {
                if (pRemaining->fetch_sub(1, std::memory_order_acq_rel) == 1)
                    pRemaining->notify_all();
            }
        }
    }

    std::mutex                  m_dispatchMutex;
    std::vector<std::thread>    m_workers;
    int                         m_workerCount = 0;
    std::atomic<bool>           m_running{ false };
    std::mutex                  m_mutex;
    std::condition_variable     m_cv;

    // Per-dispatch state (protected by m_mutex for writes, generation for reads).
    std::function<void(int)>    m_taskFunc;
    std::atomic<int>*           m_taskRemaining = nullptr;
    int                         m_taskThreads = 0;
    std::atomic<uint64_t>       m_generation{ 0 };
    std::atomic<int>            m_peakDispatchThreadsUsed{ 0 };
    std::atomic<int>            m_parallelDispatches{ 0 };
};

// Global singleton — lives for the DLL lifetime.

inline ThreadPool& GetGlobalPool()
{
    static ThreadPool pool;
    return pool;
}