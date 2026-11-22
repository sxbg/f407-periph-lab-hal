/**
 * @file test_blink_state.c
 * @brief Host tests for the domain blink state machine (template example).
 *
 * Includes exact-equation assertions AND a negative case: verifying that an
 * invalid period is clamped, and that tick with dt=0 never toggles.
 */
#include "test_fw.h"
#include "blink_state.h"

void suite_blink_state(void)
{
    blink_state_t s;

    /* Exact equations. */
    blink_init(&s, 100u);
    TEST_ASSERT_EQ(s.period_ms, 100u);
    TEST_ASSERT_EQ(s.on, 0);

    TEST_ASSERT_FALSE(blink_tick(&s, 50u));  /* half period: no toggle */
    TEST_ASSERT_TRUE(blink_tick(&s, 50u));   /* full period: toggle to on */
    TEST_ASSERT_EQ(s.on, 1);

    /* Overshoot: extra ms carry into the next period. */
    blink_init(&s, 100u);
    (void)blink_tick(&s, 130u);
    TEST_ASSERT_EQ(s.on, 1);
    TEST_ASSERT_EQ(s.elapsed_ms, 30u);

    /* Negative case: dt=0 must never toggle. */
    blink_init(&s, 10u);
    for (int i = 0; i < 1000; i++) {
        TEST_ASSERT_FALSE(blink_tick(&s, 0u));
        if (g_test_fail > 0) break;
    }

    /* Negative case: zero period is clamped to 1 (no division by zero). */
    blink_init(&s, 0u);
    TEST_ASSERT_EQ(s.period_ms, 1u);
}
