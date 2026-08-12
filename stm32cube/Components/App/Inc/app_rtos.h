#ifndef APP_RTOS_H
#define APP_RTOS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Set to 0 only when isolating scheduler/USB/LED behavior from the IMU and
 * rate-control path.
 */
#ifndef APP_RTOS_FLIGHT_ENABLE
#define APP_RTOS_FLIGHT_ENABLE 1U
#endif

/*
 * Called by the CubeMX default task. It creates the hand-owned application
 * tasks once, then keeps that task alive as the housekeeping/supervisor task.
 */
void AppRtos_Bootstrap(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_RTOS_H */
