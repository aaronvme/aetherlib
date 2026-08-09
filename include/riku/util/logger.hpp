#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <string>
#include <vector>

#include "pros/rtos.hpp"

namespace riku {

// Buffered CSV logger for the microSD card, for offline gain fitting.
//
// row() is non-blocking: it copies into a ring buffer and returns. A
// background task does the actual SD writes, which take milliseconds and would
// otherwise wreck a 10 ms control loop.
//
// Everything degrades to a no-op when no card is inserted, so logging calls
// can be left in place on a competition robot.
class Logger {
public:
    static constexpr std::size_t kMaxColumns = 15;   // + the auto t_ms column

    /// @param name     base filename. Kept short - the V5 filesystem is
    ///                 unforgiving about long names. Becomes /usd/<name><n>.csv
    ///                 where <n> is the first free index, so runs never clobber
    ///                 each other.
    /// @param columns  column headers, excluding t_ms which is prepended.
    /// @param capacityRows  ring buffer depth. At 100 Hz, 256 rows is ~2.5 s of
    ///                 slack before the flush task has to keep up.
    Logger(std::string name, std::vector<std::string> columns,
           std::size_t capacityRows = 256);
    ~Logger();

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    /// Open the file, write the header, start the flush task.
    /// Returns false if there is no SD card or the file could not be opened.
    bool start();

    /// Drain whatever is buffered, then close. Safe to call twice.
    void stop();

    /// Queue one row. Extra values past kMaxColumns are dropped; missing ones
    /// are left empty. Does nothing if the logger never started.
    void row(std::initializer_list<float> values);

    bool isRunning() const { return mRunning; }

    /// Rows discarded because the buffer was full. Non-zero means the flush
    /// task could not keep up - log less often or raise capacityRows.
    uint32_t droppedRows() const { return mDropped; }

    const std::string& path() const { return mPath; }

    static bool sdInserted();

private:
    struct Entry {
        uint32_t t;
        uint8_t n;
        std::array<float, kMaxColumns> v;
    };

    void flushLoop();
    std::size_t drain(std::vector<Entry>& out);

    std::string mName;
    std::vector<std::string> mColumns;
    std::string mPath;

    std::vector<Entry> mRing;
    std::size_t mHead{0};       // next write slot
    std::size_t mTail{0};       // next read slot
    std::size_t mCount{0};
    mutable pros::Mutex mMutex;

    std::FILE* mFile{nullptr};
    pros::Task* mTask{nullptr};
    std::atomic<bool> mRunning{false};
    std::atomic<uint32_t> mDropped{0};
    uint32_t mStartMs{0};
};

}  // namespace riku
