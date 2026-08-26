#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * A small PID controller with conditional-integration anti-windup and a
 * filtered derivative on measurement. Written in C99 with no dynamic
 * allocation and no calls that would not link on a bare-metal target, so it
 * can run inside a timer interrupt on a microcontroller as easily as it runs
 * in a unit test on a laptop.
 */

typedef struct {
    float kp;
    float ki;
    float kd;

    float sample_time; /* seconds between calls to pid_update, must be > 0 */
    float tau;          /* derivative low-pass filter time constant, seconds */

    float out_min;
    float out_max;

    float integrator;
    float prev_measurement;
    float prev_filtered_derivative;
    int has_prev_measurement; /* 0 until the first pid_update call */
} pid_controller_t;

/*
 * Sets the gains, sample time, output limits and derivative filter time
 * constant, and clears all internal state. Call this once before the first
 * pid_update. tau controls how aggressively the derivative term is
 * smoothed: larger values filter out more noise but add more lag.
 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float sample_time,
              float out_min, float out_max, float tau);

/*
 * Changes the gains without touching the integrator or derivative history,
 * so a controller can be retuned live without a bump in its output.
 */
void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd);

/*
 * Changes the output clamp without touching the gains or sample time.
 */
void pid_set_output_limits(pid_controller_t *pid, float out_min, float out_max);

/*
 * Clears the integrator and the derivative history. Use this when a
 * controller is being handed a setpoint it has never seen before, or after
 * a mode change, so old state does not leak into the next run.
 */
void pid_reset(pid_controller_t *pid);

/*
 * Runs one control step and returns the clamped output. setpoint and
 * measurement are in whatever units the caller is working in (rpm, degrees,
 * volts, ...); the controller does not care. Returns 0.0f if pid is NULL.
 */
float pid_update(pid_controller_t *pid, float setpoint, float measurement);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
