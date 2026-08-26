#include "pid.h"
#include "unity.h"

/* Unity calls these before/after every test; nothing to do here since each
 * test builds its own pid_controller_t on the stack. */
void setUp(void) {}
void tearDown(void) {}

/* Pure proportional gain on a fixed error should give exactly kp * error,
 * with the integrator and derivative both starting at zero. */
void test_proportional_only_gives_kp_times_error(void) {
    pid_controller_t pid;
    pid_init(&pid, 2.0f, 0.0f, 0.0f, 0.01f, -100.0f, 100.0f, 0.02f);

    float output = pid_update(&pid, 10.0f, 4.0f);

    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 12.0f, output);
}

/* With a steady error and no proportional or derivative term, the
 * integrator should keep accumulating and eventually drive the output past
 * a small target, showing it removes steady-state error over time. */
void test_integral_accumulates_and_removes_steady_state_error(void) {
    pid_controller_t pid;
    pid_init(&pid, 0.0f, 5.0f, 0.0f, 0.1f, -100.0f, 100.0f, 0.02f);

    float output = 0.0f;
    for (int i = 0; i < 20; i++) {
        output = pid_update(&pid, 1.0f, 0.0f);
    }

    /* error is 1.0 for every step: integrator = ki * error * dt * steps
     * = 5.0 * 1.0 * 0.1 * 20 = 10.0 */
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, output);
}

/* A large enough error must not push the output past the configured
 * limits, regardless of how big the gain is. */
void test_output_clamps_to_upper_and_lower_limits(void) {
    pid_controller_t pid;
    pid_init(&pid, 100.0f, 0.0f, 0.0f, 0.01f, -5.0f, 5.0f, 0.02f);

    float high = pid_update(&pid, 1000.0f, 0.0f);
    pid_reset(&pid);
    float low = pid_update(&pid, -1000.0f, 0.0f);

    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.0f, high);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -5.0f, low);
}

/* While the output sits at a limit and the error keeps pushing it further
 * that way, the integrator must not keep growing. Once the error reverses
 * (setpoint back in range), the output should come off the limit within a
 * step or two instead of staying pinned by an oversized integrator. */
void test_integrator_does_not_wind_up_while_saturated(void) {
    pid_controller_t pid;
    pid_init(&pid, 0.0f, 2.0f, 0.0f, 0.01f, -1.0f, 1.0f, 0.02f);

    /* Drive it hard into positive saturation. It takes a handful of steps
     * for the output estimate to cross the limit and the freeze to kick
     * in, so sample the integrator once early and once much later. */
    for (int i = 0; i < 10; i++) {
        pid_update(&pid, 10.0f, 0.0f);
    }
    float integrator_after_10_steps = pid.integrator;

    for (int i = 0; i < 40; i++) {
        pid_update(&pid, 10.0f, 0.0f);
    }
    float integrator_after_50_steps = pid.integrator;

    /* Once saturated, the integrator should stop growing even though the
     * error stays large, instead of accumulating for the whole 50 steps. */
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, integrator_after_10_steps, integrator_after_50_steps);

    /* Reverse the error hard and confirm the output leaves the limit
     * within a handful of steps rather than staying pinned while an
     * oversized integrator winds back down. */
    float output = 0.0f;
    for (int i = 0; i < 5; i++) {
        output = pid_update(&pid, -10.0f, 0.0f);
    }
    TEST_ASSERT_TRUE(output < 1.0f);
}

/* Derivative on measurement: a step change in setpoint changes the error
 * instantly but the measurement has not moved, so the derivative term
 * (and therefore the output) should not spike. */
void test_step_setpoint_does_not_cause_derivative_kick(void) {
    pid_controller_t pid;
    pid_init(&pid, 1.0f, 0.0f, 50.0f, 0.01f, -1000.0f, 1000.0f, 0.02f);

    /* Settle the derivative history at a constant measurement first. */
    pid_update(&pid, 0.0f, 0.0f);
    float before_step = pid_update(&pid, 0.0f, 0.0f);

    /* Setpoint jumps from 0 to 100 while measurement stays at 0. */
    float after_step = pid_update(&pid, 100.0f, 0.0f);

    /* Only the proportional term should move (kp * error = 1 * 100 = 100);
     * the derivative contribution stays at whatever it already was. */
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, before_step + 100.0f, after_step);
}

/* Feed a measurement that alternates by a fixed amount every step (a stand
 * in for sensor noise) and check the filtered derivative settles to a much
 * smaller magnitude than the raw sample-to-sample derivative would be. */
void test_derivative_filter_smooths_noisy_measurement(void) {
    pid_controller_t pid;
    pid_init(&pid, 0.0f, 0.0f, 1.0f, 0.01f, -1000.0f, 1000.0f, 0.1f);

    float raw_derivative_magnitude = 10.0f / 0.01f; /* a 10 unit jump over dt */
    float last_output = 0.0f;
    float measurement = 0.0f;
    for (int i = 0; i < 30; i++) {
        measurement = (i % 2 == 0) ? 10.0f : 0.0f;
        last_output = pid_update(&pid, 0.0f, measurement);
    }

    float filtered_magnitude = last_output < 0.0f ? -last_output : last_output;
    TEST_ASSERT_TRUE(filtered_magnitude < raw_derivative_magnitude * 0.5f);
}

/* pid_reset must zero the integrator and forget the previous measurement,
 * so a controller can be reused for a fresh run without a derivative kick
 * from stale history. */
void test_reset_clears_integrator_and_derivative_history(void) {
    pid_controller_t pid;
    pid_init(&pid, 1.0f, 5.0f, 1.0f, 0.1f, -100.0f, 100.0f, 0.02f);

    pid_update(&pid, 10.0f, 0.0f);
    pid_update(&pid, 10.0f, 2.0f);
    TEST_ASSERT_TRUE(pid.integrator != 0.0f);
    TEST_ASSERT_TRUE(pid.has_prev_measurement);

    pid_reset(&pid);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integrator);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.prev_filtered_derivative);
    TEST_ASSERT_FALSE(pid.has_prev_measurement);
}

/* A zero or negative sample time would divide by zero or run the
 * integrator backwards, so pid_update should refuse to compute anything
 * and return 0.0f instead. */
void test_zero_and_negative_sample_time_return_zero(void) {
    pid_controller_t pid;
    pid_init(&pid, 1.0f, 1.0f, 1.0f, 0.01f, -100.0f, 100.0f, 0.02f);

    pid.sample_time = 0.0f;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid_update(&pid, 10.0f, 0.0f));

    pid.sample_time = -0.5f;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid_update(&pid, 10.0f, 0.0f));
}

/* A NULL pid pointer must be handled gracefully rather than crashing, for
 * every entry point that takes one. */
void test_null_pointer_is_handled(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid_update(NULL, 1.0f, 0.0f));

    /* These must simply not crash. */
    pid_init(NULL, 1.0f, 1.0f, 1.0f, 0.01f, -1.0f, 1.0f, 0.02f);
    pid_set_gains(NULL, 1.0f, 1.0f, 1.0f);
    pid_set_output_limits(NULL, -1.0f, 1.0f);
    pid_reset(NULL);
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_proportional_only_gives_kp_times_error);
    RUN_TEST(test_integral_accumulates_and_removes_steady_state_error);
    RUN_TEST(test_output_clamps_to_upper_and_lower_limits);
    RUN_TEST(test_integrator_does_not_wind_up_while_saturated);
    RUN_TEST(test_step_setpoint_does_not_cause_derivative_kick);
    RUN_TEST(test_derivative_filter_smooths_noisy_measurement);
    RUN_TEST(test_reset_clears_integrator_and_derivative_history);
    RUN_TEST(test_zero_and_negative_sample_time_return_zero);
    RUN_TEST(test_null_pointer_is_handled);

    return UNITY_END();
}
