#ifndef APP_DSHOT_TEST_H
#define APP_DSHOT_TEST_H

#include "../../Motor/Inc/motor.h"

#define APP_DSHOT_TEST_ZERO_MS 15000U
#define APP_DSHOT_TEST_THROTTLE 0.30f

typedef struct
{
    uint32_t start_ms;
    bool started;
    bool failed;
    uint32_t fault;
    uint32_t dma_error;
    uint32_t last_value;
    uint32_t frames_completed;
    uint32_t timer_clock_hz;
} AppDshotTest;

/* Exclusive bench owner of the output; call from the 1 ms flight task.
 * No ARM, radio failsafe or PID in this mode. Power off to stop the bench.
 * Header-only so CubeMX-generated source lists need no modification. */
static inline void AppDshotTest_Step(AppDshotTest *test,
                                     MotorOutput *output,
                                     uint32_t now_ms)
{
    MotorStatus status;
    float throttle[MOTOR_OUTPUT_MAX_MOTORS];
    uint8_t motor;

    if (test->failed)
    {
        return;
    }
    if (!MotorOutput_IsReady(output) ||
        (MotorOutput_GetProtocol(output) != MOTOR_PROTOCOL_DSHOT))
    {
        test->failed = true;
        return;
    }
    if (!test->started)
    {
        status = MotorOutput_Start(output);
        if (status != MOTOR_OK)
        {
            /* Start has no previous frame to wait for; even BUSY is a fault. */
            test->failed = true;
            test->fault = 1U;
            test->dma_error = 1U;
            (void)MotorOutput_Stop(output);
            return;
        }
        test->start_ms = now_ms;
        test->started = true;
        return;
    }

    for (motor = 0U; motor < MOTOR_OUTPUT_MAX_MOTORS; ++motor)
    {
        throttle[motor] = ((uint32_t)(now_ms - test->start_ms) <
                           APP_DSHOT_TEST_ZERO_MS)
                              ? 0.0f
                              : APP_DSHOT_TEST_THROTTLE;
    }
    status = MotorOutput_SetAllThrottle(output, throttle, MOTOR_OUTPUT_MAX_MOTORS);
    if (status == MOTOR_OK)
    {
        status = MotorOutput_Update(output);
        if (status == MOTOR_BUSY)
        {
            return; /* The active DMA frame must not be overwritten. */
        }
        if (status == MOTOR_OK)
        {
            ++test->frames_completed;
            test->last_value = MotorOutput_GetRawOutput(output, 0U);
            test->dma_error = 0U;
            test->fault = 0U;
            return;
        }
    }
    if (status != MOTOR_OK)
    {
        test->failed = true;
        test->fault = 1U;
        test->dma_error = 1U;
        (void)MotorOutput_Stop(output);
    }
}

#endif /* APP_DSHOT_TEST_H */
