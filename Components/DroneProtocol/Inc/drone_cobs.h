#ifndef DRONE_COBS_H
#define DRONE_COBS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

/*
 * Return encoded/decoded size, or 0 on invalid input/capacity.
 * The encoded result does not include the UART 0x00 frame delimiter.
 */
size_t DroneCobs_Encode(const uint8_t *input, size_t input_length,
                        uint8_t *output, size_t output_capacity);
size_t DroneCobs_Decode(const uint8_t *input, size_t input_length,
                        uint8_t *output, size_t output_capacity);

#ifdef __cplusplus
}
#endif

#endif
