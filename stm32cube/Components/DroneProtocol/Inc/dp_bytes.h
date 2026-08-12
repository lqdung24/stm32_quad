#ifndef DP_BYTES_H
#define DP_BYTES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint16_t DroneProtocol_ReadU16Le(const uint8_t *data);
int16_t DroneProtocol_ReadI16Le(const uint8_t *data);
uint32_t DroneProtocol_ReadU32Le(const uint8_t *data);
void DroneProtocol_WriteU16Le(uint8_t *data, uint16_t value);
void DroneProtocol_WriteI16Le(uint8_t *data, int16_t value);
void DroneProtocol_WriteU32Le(uint8_t *data, uint32_t value);

#ifdef __cplusplus
}
#endif

#endif /* DP_BYTES_H */
