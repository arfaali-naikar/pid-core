#include "motor.h"

#include <stdlib.h>

void motor_init(motor_t *motor, float gain, float tau, float load_torque, float noise_amplitude) {
    motor->speed = 0.0f;
    motor->gain = gain;
    motor->tau = tau;
    motor->load_torque = load_torque;
    motor->noise_amplitude = noise_amplitude;
}

void motor_step(motor_t *motor, float voltage, float dt) {
    /* tau * d(speed)/dt = -speed + gain * voltage - load_torque
     *
     * This is the standard first order lag: the same equation describes an
     * RC circuit charging or a room heating up. Left alone, speed decays to
     * zero (the -speed term); the drive voltage pulls it toward
     * gain * voltage, its steady-state speed once acceleration has died
     * out; load_torque subtracts a constant amount to represent friction or
     * a mechanical load fighting the motor. tau lumps together the rotor's
     * inertia and its friction, and sets how quickly speed can change:
     * larger tau means a sluggish motor. Integrated with forward Euler,
     * which is accurate enough here because the 1 ms step used by the
     * simulation is much smaller than tau. */
    float dspeed = (-motor->speed + motor->gain * voltage - motor->load_torque) / motor->tau;
    motor->speed += dspeed * dt;
}

float motor_measure(const motor_t *motor) {
    float noise = 0.0f;
    if (motor->noise_amplitude > 0.0f) {
        /* Uniform noise on [-amplitude, +amplitude], enough to make the
         * derivative filter earn its keep without swamping the signal. */
        noise = ((float)rand() / (float)RAND_MAX * 2.0f - 1.0f) * motor->noise_amplitude;
    }
    return motor->speed + noise;
}
