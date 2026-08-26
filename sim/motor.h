#ifndef MOTOR_H
#define MOTOR_H

/*
 * A first order model of a brushed DC motor's speed loop, just enough to
 * give the PID controller something realistic to fight against: a lag
 * between drive voltage and speed, a constant load, and a bit of sensor
 * noise. This lives in sim/ rather than src/ because it is a test rig, not
 * part of the portable controller.
 */
typedef struct {
    float speed;           /* current true speed, arbitrary units */
    float gain;            /* steady-state speed produced per volt of drive */
    float tau;             /* mechanical time constant, seconds */
    float load_torque;     /* constant disturbance, expressed as a speed offset */
    float noise_amplitude; /* peak amplitude of the noise added to measured speed */
} motor_t;

/* Sets up the motor's parameters and zeroes its speed. */
void motor_init(motor_t *motor, float gain, float tau, float load_torque, float noise_amplitude);

/* Advances the true speed by one step of dt seconds under the given drive voltage. */
void motor_step(motor_t *motor, float voltage, float dt);

/* Returns the true speed plus noise, standing in for what a real speed sensor would report. */
float motor_measure(const motor_t *motor);

#endif /* MOTOR_H */
