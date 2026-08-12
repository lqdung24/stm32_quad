#include "dp_bytes.h"

uint16_t DroneProtocol_ReadU16Le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

int16_t DroneProtocol_ReadI16Le(const uint8_t *data)
{
    return (int16_t)DroneProtocol_ReadU16Le(data);
}

uint32_t DroneProtocol_ReadU32Le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

void DroneProtocol_WriteU16Le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

void DroneProtocol_WriteI16Le(uint8_t *data, int16_t value)
{
    DroneProtocol_WriteU16Le(data, (uint16_t)value);
}

void DroneProtocol_WriteU32Le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}
