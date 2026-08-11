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

// Buffered CSV logger for the microSD card. row() never touches the card; a
// background task does the writes.
class Logger {
public:
    static constexpr std::size_t kMaxColumns = 15;   // t_ms is prepended on top

    /// Writes /usd/<name><n>.csv, first free n. Keep `name` short - the V5
    /// filesystem dislikes long names.
    Logger(std::string name, std::vector<std::string> columns,
           std::size_t capacityRows = 256);
    ~Logger();

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    /// False if there is no card or the file could not be opened.
    bool start();
    void stop();

    /// Extra values past kMaxColumns are dropped. No-op if start() failed.
    void row(std::initializer_list<float> values);

    bool isRunning() const { return mRunning; }

    /// Non-zero means the flush task could not keep up.
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
    std::size_t mHead{0};
    std::size_t mTail{0};
    std::size_t mCount{0};
    mutable pros::Mutex mMutex;

    std::FILE* mFile{nullptr};
    pros::Task* mTask{nullptr};
    std::atomic<bool> mRunning{false};
    std::atomic<uint32_t> mDropped{0};
    uint32_t mStartMs{0};
};

}  // namespace riku
