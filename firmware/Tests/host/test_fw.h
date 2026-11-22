/**
 * @file test_fw.h
 * @brief Minimal host test framework (no external dependency).
 *
 * Discipline: assertions MUST be exact equations; every suite should contain
 * at least one negative case (a past project shipped four defective verdicts
 * because criteria passed even when the behavior failed).
 */
#ifndef TEST_FW_H
#define TEST_FW_H

#include <stdio.h>

extern int g_test_pass;
extern int g_test_fail;

#define TEST_ASSERT_EQ_MSG(actual, expected, what)                        \
    do {                                                                  \
        const long long _a = (long long)(actual);                          \
        const long long _e = (long long)(expected);                        \
        if (_a == _e) { g_test_pass++; }                                   \
        else {                                                             \
            g_test_fail++;                                                 \
            printf("FAIL %s:%d %s: actual=%lld expected=%lld (%s)\r\n",    \
                   __FILE__, __LINE__, #actual, _a, _e, (what));           \
        }                                                                 \
    } while (0)

#define TEST_ASSERT_EQ(a, e) TEST_ASSERT_EQ_MSG((a), (e), "")

#define TEST_ASSERT_TRUE_MSG(cond, what)                                  \
    do {                                                                  \
        if ((cond) != 0) { g_test_pass++; }                               \
        else {                                                            \
            g_test_fail++;                                                \
            printf("FAIL %s:%d %s: condition false (%s)\r\n",             \
                   __FILE__, __LINE__, #cond, (what));                    \
        }                                                                 \
    } while (0)

#define TEST_ASSERT_TRUE(c) TEST_ASSERT_TRUE_MSG((c), "")

#define TEST_ASSERT_FALSE_MSG(cond, what) TEST_ASSERT_TRUE_MSG(!(cond), (what))
#define TEST_ASSERT_FALSE(c) TEST_ASSERT_FALSE_MSG((c), "")

#define TEST_RUN(name, fn)                                                \
    do {                                                                  \
        const int _p0 = g_test_pass, _f0 = g_test_fail;                    \
        fn();                                                              \
        printf("suite %-24s +%d -%d\r\n", (name),                          \
               g_test_pass - _p0, g_test_fail - _f0);                      \
    } while (0)

#define TEST_REPORT()                                                     \
    do {                                                                  \
        printf("TOTAL %d/%d\r\n", g_test_pass, g_test_pass + g_test_fail);\
        return g_test_fail == 0 ? 0 : 1;                                   \
    } while (0)

#endif /* TEST_FW_H */
