// Tiny assertion helpers for the host tests - deliberately dependency-free so
// the tests build with nothing but a C++17 compiler.
#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>

inline int g_failures = 0;
inline int g_checks = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        ++g_checks;                                                              \
        if (!(cond)) {                                                           \
            ++g_failures;                                                        \
            std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
        }                                                                        \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                    \
    do {                                                                         \
        ++g_checks;                                                              \
        const double va_ = (a), vb_ = (b);                                       \
        if (!(std::fabs(va_ - vb_) <= (tol))) {                                  \
            ++g_failures;                                                        \
            std::fprintf(stderr, "%s:%d: CHECK_NEAR failed: %s = %g, %s = %g, tol %g\n", \
                         __FILE__, __LINE__, #a, va_, #b, vb_, (double)(tol));   \
        }                                                                        \
    } while (0)

inline int summarize(const char *name) {
    std::printf("%-16s %d checks, %d failures\n", name, g_checks, g_failures);
    return g_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
