#include "motor.h"
#include "pid.h"

#include <stdio.h>
#include <stdlib.h>

#define SIM_STEP 0.001f /* 1 ms, the rate you would drive pid_update from a timer isr */
#define SIM_DURATION 4.5f

/*
 * The demand profile the controller is asked to track: a step up from rest,
 * a step down through zero into reverse, and a stretch demanding far more
 * speed than the motor can produce at its voltage limit, so the anti-windup
 * behaviour actually gets exercised before easing back to something
 * reachable.
 */
static float setpoint_at(float t) {
    if (t < 0.2f) {
        return 0.0f;
    } else if (t < 1.5f) {
        return 100.0f;
    } else if (t < 2.5f) {
        return -50.0f;
    } else if (t < 3.5f) {
        return 500.0f; /* unreachable at +/-12 V, drives the output into saturation */
    }
    return 50.0f;
}

int main(int argc, char **argv) {
    const char *filename = (argc > 1) ? argv[1] : "step_response.csv";
    float kp = (argc > 2) ? (float)atof(argv[2]) : 0.15f;
    float ki = (argc > 3) ? (float)atof(argv[3]) : 0.6f;
    float kd = (argc > 4) ? (float)atof(argv[4]) : 0.0015f;

    const float out_min = -12.0f;
    const float out_max = 12.0f;
    const float derivative_tau = 0.01f;

    pid_controller_t pid;
    pid_init(&pid, kp, ki, kd, SIM_STEP, out_min, out_max, derivative_tau);

    motor_t motor;
    motor_init(&motor, 20.0f, 0.5f, 2.0f, 0.5f);

    srand(42); /* fixed seed so the CSV and plots are the same on every run */

    FILE *csv = fopen(filename, "w");
    if (csv == NULL) {
        fprintf(stderr, "could not open %s for writing\n", filename);
        return 1;
    }
    fprintf(csv, "time,setpoint,speed,output\n");

    int steps = (int)(SIM_DURATION / SIM_STEP);
    for (int i = 0; i < steps; i++) {
        float t = (float)i * SIM_STEP;
        float setpoint = setpoint_at(t);

        float measured_speed = motor_measure(&motor);
        float output = pid_update(&pid, setpoint, measured_speed);
        motor_step(&motor, output, SIM_STEP);

        fprintf(csv, "%.4f,%.4f,%.4f,%.4f\n", t, setpoint, motor.speed, output);
    }

    fclose(csv);
    printf("wrote %d samples to %s (kp=%.4f ki=%.4f kd=%.4f)\n", steps, filename, kp, ki, kd);
    return 0;
}
