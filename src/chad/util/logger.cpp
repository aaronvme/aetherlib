#include "chad/util/logger.hpp"

#include <mutex>

#include "pros/misc.hpp"

namespace chad {
namespace {
constexpr int kFlushPeriodMs = 50;
constexpr std::size_t kBatchRows = 64;
}

Logger::Logger(std::string name, std::vector<std::string> columns,
               std::size_t capacityRows)
    : mName(std::move(name)),
      mColumns(std::move(columns)),
      mRing(capacityRows == 0 ? 1 : capacityRows) {
    if (mColumns.size() > kMaxColumns) mColumns.resize(kMaxColumns);
}

Logger::~Logger() {
    stop();
}

bool Logger::sdInserted() {
    return pros::usd::is_installed() == 1;
}

bool Logger::start() {
    if (mRunning) return true;
    if (!sdInserted()) return false;

    for (int i = 0; i < 100; ++i) {
        char candidate[64];
        std::snprintf(candidate, sizeof(candidate), "/usd/%s%02d.csv", mName.c_str(), i);
        std::FILE* probe = std::fopen(candidate, "r");
        if (probe == nullptr) {
            mPath = candidate;
            break;
        }
        std::fclose(probe);
    }
    if (mPath.empty()) return false;

    mFile = std::fopen(mPath.c_str(), "w");
    if (mFile == nullptr) return false;

    std::fputs("t_ms", mFile);
    for (const std::string& c : mColumns) {
        std::fputc(',', mFile);
        std::fputs(c.c_str(), mFile);
    }
    std::fputc('\n', mFile);
    std::fflush(mFile);

    mHead = mTail = mCount = 0;
    mDropped = 0;
    mStartMs = pros::millis();
    mRunning = true;
    mTask = new pros::Task([this] { flushLoop(); }, "chadLog");
    return true;
}

void Logger::stop() {
    if (!mRunning) return;
    mRunning = false;

    if (mTask != nullptr) {
        mTask->join();
        delete mTask;
        mTask = nullptr;
    }

    std::vector<Entry> batch;
    while (drain(batch) > 0) {
        for (const Entry& e : batch) {
            std::fprintf(mFile, "%lu", static_cast<unsigned long>(e.t));
            for (uint8_t i = 0; i < e.n; ++i) std::fprintf(mFile, ",%.4f", e.v[i]);
            std::fputc('\n', mFile);
        }
    }

    if (mFile != nullptr) {
        std::fflush(mFile);
        std::fclose(mFile);
        mFile = nullptr;
    }
}

void Logger::row(std::initializer_list<float> values) {
    if (!mRunning) return;

    Entry e;
    e.t = pros::millis() - mStartMs;
    e.n = 0;
    for (float v : values) {
        if (e.n >= kMaxColumns) break;
        e.v[e.n++] = v;
    }

    std::lock_guard<pros::Mutex> lock(mMutex);
    if (mCount == mRing.size()) {
        mDropped++;
        return;
    }
    mRing[mHead] = e;
    mHead = (mHead + 1) % mRing.size();
    mCount++;
}

std::size_t Logger::drain(std::vector<Entry>& out) {
    out.clear();
    std::lock_guard<pros::Mutex> lock(mMutex);
    const std::size_t n = (mCount < kBatchRows) ? mCount : kBatchRows;
    for (std::size_t i = 0; i < n; ++i) {
        out.push_back(mRing[mTail]);
        mTail = (mTail + 1) % mRing.size();
    }
    mCount -= n;
    return n;
}

void Logger::flushLoop() {
    std::vector<Entry> batch;
    batch.reserve(kBatchRows);

    while (mRunning) {
        while (drain(batch) > 0) {
            for (const Entry& e : batch) {
                std::fprintf(mFile, "%lu", static_cast<unsigned long>(e.t));
                for (uint8_t i = 0; i < e.n; ++i) std::fprintf(mFile, ",%.4f", e.v[i]);
                std::fputc('\n', mFile);
            }
        }
        std::fflush(mFile);
        pros::delay(kFlushPeriodMs);
    }
}
}
