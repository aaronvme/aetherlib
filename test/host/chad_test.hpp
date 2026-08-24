#pragma once

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace chadtest {

struct Case {
    const char* suite;
    const char* name;
    void (*fn)();
};

std::vector<Case>& registry();
int registerCase(const char* suite, const char* name, void (*fn)());

struct Failure {
    std::string where;
    std::string what;
};

std::vector<Failure>& failures();
void fail(const char* file, int line, const std::string& what);

inline bool closeTo(double a, double b, double tol) {
    if (std::isnan(a) || std::isnan(b)) return false;
    return std::fabs(a - b) <= tol;
}

int run(int argc, char** argv);

}  // namespace chadtest

#define CHAD_TEST(suite, name)                                              \
    static void chadTest_##suite##_##name();                                \
    static const int chadTestReg_##suite##_##name =                         \
        ::chadtest::registerCase(#suite, #name, chadTest_##suite##_##name); \
    static void chadTest_##suite##_##name()

// Wrap an argument containing template commas in its own parentheses --
// CHECK((f<2, 2>(x))) -- or the preprocessor splits it into two arguments.
#define CHAD_FAIL(msg) ::chadtest::fail(__FILE__, __LINE__, (msg))

#define CHECK(cond)                                                    \
    do {                                                               \
        if (!(cond)) CHAD_FAIL(std::string("CHECK failed: " #cond));    \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                        \
    do {                                                                             \
        const double chadA = (a), chadB = (b), chadT = (tol);                        \
        if (!::chadtest::closeTo(chadA, chadB, chadT)) {                             \
            char chadMsg[256];                                                            \
            std::snprintf(chadMsg, sizeof chadMsg, "%s (%.9g) != %s (%.9g), tol %.3g", #a,   \
                          chadA, #b, chadB, chadT);                                   \
            CHAD_FAIL(chadMsg);                                                           \
        }                                                                             \
    } while (0)

#define CHECK_EQ(a, b)                                                            \
    do {                                                                          \
        const auto chadA = (a);                                                   \
        const auto chadB = (b);                                                   \
        if (!(chadA == chadB)) {                                                  \
            char chadMsg[256];                                                         \
            std::snprintf(chadMsg, sizeof chadMsg, "%s != %s", #a, #b);                   \
            CHAD_FAIL(chadMsg);                                                        \
        }                                                                          \
    } while (0)

#define CHECK_FINITE(x)                                                     \
    do {                                                                    \
        const double chadX = (x);                                           \
        if (!std::isfinite(chadX)) CHAD_FAIL(std::string(#x " is not finite")); \
    } while (0)
