#include "pid.h"

#include <stddef.h>

void pid_init(pid_t *pid, float kp, float ki, float kd, float sample_time, float out_min,
              float out_max, float tau) {
    if (pid == NULL) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->sample_time = sample_time;
    pid->tau = tau;
    pid->out_min = out_min;
    pid->out_max = out_max;

    pid_reset(pid);
}

void pid_set_gains(pid_t *pid, float kp, float ki, float kd) {
    if (pid == NULL) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_set_output_limits(pid_t *pid, float out_min, float out_max) {
    if (pid == NULL) {
        return;
    }

    pid->out_min = out_min;
    pid->out_max = out_max;
}

void pid_reset(pid_t *pid) {
    if (pid == NULL) {
        return;
    }

    pid->integrator = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->prev_filtered_derivative = 0.0f;
    pid->has_prev_measurement = 0;
}

float pid_update(pid_t *pid, float setpoint, float measurement) {
    if (pid == NULL) {
        return 0.0f;
    }

    /* A zero or negative sample time makes every dt-based term meaningless
     * (division by zero, or an integrator that runs backwards). Rather than
     * crash or return garbage, hold the output at zero so a bad caller is
     * obvious instead of silently unstable. */
    if (pid->sample_time <= 0.0f) {
        return 0.0f;
    }

    float error = setpoint - measurement;
    float proportional = pid->kp * error;

    /* Derivative on measurement, not on error. If the derivative were taken
     * on the error, a step change in setpoint would make the error jump
     * instantly, and dividing that jump by a small sample time produces a
     * huge derivative spike (a "derivative kick") even though the plant has
     * not moved at all. Measurement changes smoothly in the real world, so
     * taking the derivative of the measurement instead gives the same
     * damping effect on disturbances without reacting to setpoint changes. */
    float raw_derivative = 0.0f;
    if (pid->has_prev_measurement) {
        raw_derivative = (measurement - pid->prev_measurement) / pid->sample_time;
    }

    /* Raw derivatives amplify high-frequency measurement noise, since noise
     * is mostly fast changes and a derivative is exactly the operator that
     * emphasises fast changes. Passing it through a first-order low-pass
     * filter trades a bit of lag for a lot less noise. The filter is
     * expressed in its discrete, one-line form: alpha is the fraction of
     * the raw sample that gets through on this step, derived from the
     * usual RC time constant tau by alpha = dt / (tau + dt). alpha -> 1
     * as tau -> 0 (no filtering), and alpha -> 0 as tau grows (heavy
     * filtering, more lag). */
    float alpha = pid->sample_time / (pid->tau + pid->sample_time);
    float filtered_derivative =
        pid->prev_filtered_derivative + alpha * (raw_derivative - pid->prev_filtered_derivative);

    /* The term is negative because it is built from d(measurement)/dt
     * rather than d(error)/dt = d(setpoint)/dt - d(measurement)/dt: with a
     * constant setpoint the two differ only by that sign. */
    float derivative_term = -pid->kd * filtered_derivative;

    /* Conditional integration anti-windup. We first estimate what the
     * output would be if we integrated this step's error as normal. If
     * that estimate is already past a limit, and the sign of the error
     * would push it further past that same limit, we skip the integrator
     * update for this step. Clamping the final output is not enough on its
     * own: if the integrator kept accumulating while the output sits at a
     * limit, it would grow far beyond what is needed, and once the error
     * finally reverses the controller would keep the output pinned at the
     * limit until that oversized integrator winds back down, causing a
     * large, slow overshoot. Freezing the integrator while saturated (but
     * still letting it run if the error would pull the output back toward
     * the valid range) keeps it sized to what the plant actually needs. */
    float unsaturated_output = proportional + pid->ki * pid->integrator + derivative_term;
    int saturated_high = unsaturated_output > pid->out_max;
    int saturated_low = unsaturated_output < pid->out_min;
    int would_deepen_saturation = (saturated_high && error > 0.0f) || (saturated_low && error < 0.0f);

    if (!would_deepen_saturation) {
        pid->integrator += error * pid->sample_time;
    }

    float output = proportional + pid->ki * pid->integrator + derivative_term;

    if (output > pid->out_max) {
        output = pid->out_max;
    } else if (output < pid->out_min) {
        output = pid->out_min;
    }

    pid->prev_measurement = measurement;
    pid->prev_filtered_derivative = filtered_derivative;
    pid->has_prev_measurement = 1;

    return output;
}
