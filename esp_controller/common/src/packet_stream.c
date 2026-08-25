#include "packet_stream.h" /* shared by the USB and UART transports */

#include <string.h>

#include "dp_cobs.h"

void packet_stream_init(packet_stream_t *stream, packet_stream_callback_t callback,
                        void *context)
{
    if (stream == NULL) {
        return;
    }
    memset(stream, 0, sizeof(*stream));
    stream->callback = callback;
    stream->context = context;
}

void packet_stream_feed(packet_stream_t *stream, const uint8_t *bytes,
                        size_t length)
{
    if ((stream == NULL) || (bytes == NULL)) {
        return;
    }

    for (size_t i = 0; i < length; ++i) {
        if (bytes[i] == 0U) {
            uint8_t packet[DRONE_PROTOCOL_MAX_PACKET_SIZE];
            const size_t packet_length = DroneCobs_Decode(
                stream->encoded, stream->encoded_length, packet, sizeof(packet));
            if ((packet_length != 0U) && (stream->callback != NULL)) {
                stream->callback(packet, packet_length, stream->context);
            }
            stream->encoded_length = 0U;
        } else if (stream->encoded_length < sizeof(stream->encoded)) {
            stream->encoded[stream->encoded_length++] = bytes[i];
        } else {
            /* Ignore the rest of an oversized frame until its delimiter. */
            stream->encoded_length = 0U;
        }
    }
}

size_t packet_stream_encode(const uint8_t *packet, size_t packet_length,
                            uint8_t *frame, size_t frame_capacity)
{
    if ((packet == NULL) || (packet_length == 0U) ||
        (packet_length > DRONE_PROTOCOL_MAX_PACKET_SIZE) || (frame == NULL) ||
        (frame_capacity < packet_length + 3U)) {
        return 0U;
    }

    frame[0] = 0U;
    const size_t encoded_length = DroneCobs_Encode(packet, packet_length,
                                                    &frame[1], frame_capacity - 2U);
    if (encoded_length == 0U) {
        return 0U;
    }
    frame[encoded_length + 1U] = 0U;
    return encoded_length + 2U;
}
