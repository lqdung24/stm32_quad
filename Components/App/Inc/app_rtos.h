#ifndef APP_RTOS_H
#define APP_RTOS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Called by the CubeMX default task. It creates the hand-owned application
 * tasks once, then terminates the CubeMX bootstrap task.
 */
void AppRtos_Bootstrap(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_RTOS_H */
