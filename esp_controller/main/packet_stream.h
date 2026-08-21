#pragma once

#include <stddef.h>
#include <stdint.h>

#include "dp_protocol.h"

typedef void (*packet_stream_callback_t)(const uint8_t *packet, size_t length,
                                         void *context);

typedef struct {
    uint8_t encoded[DRONE_PROTOCOL_MAX_PACKET_SIZE + 1U];
    size_t encoded_length;
    packet_stream_callback_t callback;
    void *context;
} packet_stream_t;

/* COBS/0x00 framing for byte streams such as USB CDC and UART. */
void packet_stream_init(packet_stream_t *stream, packet_stream_callback_t callback,
                        void *context);
void packet_stream_feed(packet_stream_t *stream, const uint8_t *bytes,
                        size_t length);

/* Writes a leading and trailing delimiter, so console text cannot corrupt sync. */
size_t packet_stream_encode(const uint8_t *packet, size_t packet_length,
                            uint8_t *frame, size_t frame_capacity);
