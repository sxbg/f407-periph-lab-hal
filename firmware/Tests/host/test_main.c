/**
 * @file test_main.c
 * @brief Host test entry: registers suites and prints the TOTAL line that
 * hosttest.py parses.
 */
#include "test_fw.h"

#include <stdio.h>

int g_test_pass = 0;
int g_test_fail = 0;

/* Forward declarations of suite functions (one per test_*.c). */
void suite_blink_state(void);

int main(void)
{
    TEST_RUN("blink_state", suite_blink_state);
    TEST_REPORT();
}
