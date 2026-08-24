#include "chad_test.hpp"

#include <algorithm>
#include <cstring>

namespace chadtest {

std::vector<Case>& registry() {
    static std::vector<Case> cases;
    return cases;
}

int registerCase(const char* suite, const char* name, void (*fn)()) {
    registry().push_back({suite, name, fn});
    return 0;
}

std::vector<Failure>& failures() {
    static std::vector<Failure> f;
    return f;
}

void fail(const char* file, int line, const std::string& what) {
    const char* slash = std::strrchr(file, '\\');
    if (slash == nullptr) slash = std::strrchr(file, '/');
    const std::string where =
        std::string(slash != nullptr ? slash + 1 : file) + ":" + std::to_string(line);
    failures().push_back({where, what});
}

int run(int argc, char** argv) {
    const char* filter = (argc > 1) ? argv[1] : nullptr;

    auto& cases = registry();
    std::stable_sort(cases.begin(), cases.end(), [](const Case& a, const Case& b) {
        const int s = std::strcmp(a.suite, b.suite);
        return s != 0 ? s < 0 : std::strcmp(a.name, b.name) < 0;
    });

    int ran = 0, failed = 0;
    const char* suite = nullptr;

    for (const Case& c : cases) {
        if (filter != nullptr && std::strstr(c.suite, filter) == nullptr &&
            std::strstr(c.name, filter) == nullptr) {
            continue;
        }
        if (suite == nullptr || std::strcmp(suite, c.suite) != 0) {
            suite = c.suite;
            std::printf("\n%s\n", suite);
        }

        const size_t before = failures().size();
        c.fn();
        ran++;

        const size_t added = failures().size() - before;
        if (added == 0) {
            std::printf("  [ ok ] %s\n", c.name);
        } else {
            failed++;
            std::printf("  [FAIL] %s\n", c.name);
            for (size_t i = before; i < failures().size(); i++) {
                std::printf("         %s  %s\n", failures()[i].where.c_str(),
                            failures()[i].what.c_str());
            }
        }
    }

    std::printf("\n%d case%s, %d failed, %zu assertion failure%s\n", ran,
                ran == 1 ? "" : "s", failed, failures().size(),
                failures().size() == 1 ? "" : "s");
    std::printf(failed == 0 ? "HOST SUITE PASSED\n" : "HOST SUITE FAILED\n");
    return failed == 0 ? 0 : 1;
}

}  // namespace chadtest

int main(int argc, char** argv) { return chadtest::run(argc, argv); }
