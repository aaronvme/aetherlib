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

namespace chad {
class Logger {
public:
    static constexpr std::size_t kMaxColumns = 15;

    Logger(std::string name, std::vector<std::string> columns,
           std::size_t capacityRows = 256);
    ~Logger();

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    bool start();
    void stop();

    void row(std::initializer_list<float> values);

    bool isRunning() const { return mRunning; }

    uint32_t droppedRows() const { return mDropped; }

    const std::string& path() const { return mPath; }

    static bool sdInserted();

private:
    struct Entry {
        uint32_t t{0};
        uint8_t n{0};
        std::array<float, kMaxColumns> v{};
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
    std::atomic<uint32_t> mStartMs{0};
};
}
