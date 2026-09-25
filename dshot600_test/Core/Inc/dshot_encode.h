#ifndef DSHOT_ENCODE_H
#define DSHOT_ENCODE_H
#include <stdint.h>
#define DSHOT_WORDS 72U
static inline uint16_t Dshot_PacketWithTelemetry(uint16_t value, uint8_t telemetry)
{
  uint16_t p = (uint16_t)((value << 1U) | (telemetry ? 1U : 0U));
  return (uint16_t)((p << 4U) | ((p ^ (p >> 4U) ^ (p >> 8U)) & 15U));
}
static inline void Dshot_EncodeWithTelemetry(uint32_t *buffer, uint16_t value, uint8_t telemetry,
                                uint32_t zero, uint32_t one)
{
  uint16_t packet = Dshot_PacketWithTelemetry(value, telemetry);
  for (unsigned slot = 0; slot < 18; ++slot) {
    uint32_t duty = slot < 16 ?
      ((packet & (0x8000U >> slot)) ? one : zero) : 0U;
    for (unsigned motor = 0; motor < 4; ++motor)
      buffer[slot * 4 + motor] = duty;
  }
}
static inline uint16_t Dshot_Packet(uint16_t value)
{
  return Dshot_PacketWithTelemetry(value, 0);
}
static inline void Dshot_Encode(uint32_t *buffer, uint16_t value,
                                uint32_t zero, uint32_t one)
{
  Dshot_EncodeWithTelemetry(buffer, value, 0, zero, one);
}
#endif
