#include "dp_cobs.h"
#include "dp_bytes.h"
#include "packet_stream.h"
#include "dp_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_crc_known_vector(void)
{
    static const uint8_t vector[] = "123456789";
    assert(DroneProtocol_Crc16CcittFalse(vector, 9U) == 0x29B1U);
}

static void test_control_round_trip(void)
{
    DroneControlCommand input = {
        .header = {
            .sequence = 65535U,
            .session_id = 0x1234U,
            .flags = DRONE_CONTROL_FLAG_ARM_REQUEST,
            .sender_time_ms = 0x78563412U,
        },
        .throttle = 200U,
        .roll = -1000,
        .pitch = 1000,
        .yaw = -1,
        .aux1 = DRONE_CONTROL_MOTOR_SELECT_ALL,
        .aux2 = 0U,
    };
    DroneControlCommand output;
    uint8_t packet[DRONE_CONTROL_PACKET_SIZE];

    assert(DroneProtocol_EncodeControl(&input, packet) == DRONE_PROTOCOL_OK);
    assert(packet[0] == 0x5AU && packet[1] == 0xA5U);
    assert(packet[10] == DRONE_CONTROL_PAYLOAD_SIZE);
    assert(DroneProtocol_DecodeControl(packet, sizeof(packet), &output) ==
           DRONE_PROTOCOL_OK);
    assert(output.header.sequence == input.header.sequence);
    assert(output.header.session_id == input.header.session_id);
    assert(output.header.sender_time_ms == input.header.sender_time_ms);
    assert(output.throttle == input.throttle);
    assert(output.roll == input.roll);
    assert(output.pitch == input.pitch);
    assert(output.yaw == input.yaw);
    assert(output.aux1 == DRONE_CONTROL_MOTOR_SELECT_ALL);
    assert(output.aux2 == 0U);

    packet[16] ^= 1U;
    assert(DroneProtocol_DecodeControl(packet, sizeof(packet), &output) ==
           DRONE_PROTOCOL_CRC_ERROR);
}

static void test_control_modes(void)
{
    DroneControlCommand input = {.header = {.session_id = 1U}};
    DroneControlCommand output;
    uint8_t packet[DRONE_CONTROL_PACKET_SIZE];
    const uint16_t modes[] = {0U, DRONE_CONTROL_FLAG_ANGLE_MODE,
                              DRONE_CONTROL_FLAG_ACRO_MODE};
    for (size_t i = 0U; i < sizeof(modes) / sizeof(modes[0]); ++i)
    {
        input.header.flags = modes[i] | DRONE_CONTROL_FLAG_ARM_REQUEST;
        assert(DroneProtocol_EncodeControl(&input, packet) == DRONE_PROTOCOL_OK);
        assert(DroneProtocol_DecodeControl(packet, sizeof(packet), &output) ==
               DRONE_PROTOCOL_OK);
        assert(output.header.flags == input.header.flags);
    }
    input.header.flags = DRONE_CONTROL_FLAG_MOTOR_TEST | DRONE_CONTROL_FLAG_ARM_REQUEST;
    input.aux1 = DRONE_CONTROL_MOTOR_SELECT_M3;
    assert(DroneProtocol_EncodeControl(&input, packet) == DRONE_PROTOCOL_OK);
    assert(DroneProtocol_DecodeControl(packet, sizeof(packet), &output) ==
           DRONE_PROTOCOL_OK);
    assert(output.aux1 == DRONE_CONTROL_MOTOR_SELECT_M3);
    input.header.flags |= DRONE_CONTROL_FLAG_ANGLE_MODE;
    assert(DroneProtocol_EncodeControl(&input, packet) == DRONE_PROTOCOL_FLAGS_ERROR);
    input.header.flags = DRONE_CONTROL_FLAG_MOTOR_TEST;
    input.roll = 1;
    assert(DroneProtocol_EncodeControl(&input, packet) == DRONE_PROTOCOL_RANGE_ERROR);
    input.roll = 0;
    input.aux1 = DRONE_CONTROL_MOTOR_SELECT_ALL;
    input.header.flags = DRONE_CONTROL_FLAG_MODE_MASK;
    assert(DroneProtocol_EncodeControl(&input, packet) == DRONE_PROTOCOL_FLAGS_ERROR);

    /* Corrupt only flags, then repair CRC to exercise the decoder's mode gate. */
    packet[8] = (uint8_t)DRONE_CONTROL_FLAG_MODE_MASK;
    const uint16_t crc = DroneProtocol_Crc16CcittFalse(packet, sizeof(packet) - 2U);
    packet[28] = (uint8_t)crc;
    packet[29] = (uint8_t)(crc >> 8);
    assert(DroneProtocol_DecodeControl(packet, sizeof(packet), &output) ==
           DRONE_PROTOCOL_FLAGS_ERROR);
}

static void test_status_round_trip(void)
{
    DroneSystemStatus input = {
        .header = {.sequence = 9U, .session_id = 7U, .sender_time_ms = 123U},
        .last_control_sequence = 8U,
        .requested_throttle = 300U,
        .applied_throttle = 200U,
        .pwm_pulse_us = {1200U, 1210U, 1220U, 1230U},
        .state = DRONE_STATE_ARMED,
        .error_flags = DRONE_ERROR_THROTTLE_CLAMPED,
        .uart_rx_rate = 50U,
    };
    DroneSystemStatus output;
    uint8_t packet[DRONE_STATUS_PACKET_SIZE];

    assert(DroneProtocol_EncodeStatus(&input, packet) == DRONE_PROTOCOL_OK);
    assert(DroneProtocol_DecodeStatus(packet, sizeof(packet), &output) ==
           DRONE_PROTOCOL_OK);
    assert(output.applied_throttle == 200U);
    assert(output.pwm_pulse_us[0] == 1200U);
    assert(output.pwm_pulse_us[1] == 1210U);
    assert(output.pwm_pulse_us[2] == 1220U);
    assert(output.pwm_pulse_us[3] == 1230U);
    assert(output.error_flags == DRONE_ERROR_THROTTLE_CLAMPED);
}

static void test_flight_telemetry_round_trip(void)
{
    DroneFlightTelemetry input = {
        .header = {.sequence = 99U, .session_id = 0x2A2BU,
                   .sender_time_ms = 0x12345678U},
        .attitude_cdeg = {-1234, 567, 890},
        .gyro_mrad_s = {-1200, 2300, -3400},
        .rate_setpoint_mrad_s = {100, -200, 300},
        .pid_command_centi = {456, -789, 1234},
        .motor_pwm_us = {1000U, 1200U, 1500U, 2000U},
        .state = DRONE_STATE_ARMED,
        .actuators_active = true,
        .attitude_valid = true,
        .sample_id = UINT32_MAX,
        .motor_commit_time_ms = 0x1234567AU,
        .output_sample_matched = true,
    };
    DroneFlightTelemetry output;
    uint8_t packet[DRONE_FLIGHT_TELEMETRY_PACKET_SIZE];

    assert(DroneProtocol_EncodeFlightTelemetry(&input, packet) ==
           DRONE_PROTOCOL_OK);
    assert(packet[3] == DRONE_PACKET_FLIGHT_TELEMETRY_SYNC);
    assert(packet[10] == DRONE_FLIGHT_TELEMETRY_PAYLOAD_SIZE);
    assert(packet[8] == (DRONE_STATE_ARMED |
                         DRONE_FLIGHT_TELEMETRY_FLAG_ACTUATORS_ACTIVE |
                         DRONE_FLIGHT_TELEMETRY_FLAG_ATTITUDE_VALID |
                         DRONE_FLIGHT_TELEMETRY_FLAG_OUTPUT_SAMPLE_MATCHED));
    assert(DroneProtocol_DecodeFlightTelemetry(packet, sizeof(packet),
                                                &output) == DRONE_PROTOCOL_OK);
    assert(output.header.sequence == input.header.sequence);
    assert(output.header.session_id == input.header.session_id);
    assert(output.header.sender_time_ms == input.header.sender_time_ms);
    assert(output.attitude_cdeg[0] == -1234);
    assert(output.gyro_mrad_s[2] == -3400);
    assert(output.rate_setpoint_mrad_s[1] == -200);
    assert(output.pid_command_centi[1] == -789);
    assert(output.motor_pwm_us[3] == 2000U);
    assert(output.state == DRONE_STATE_ARMED);
    assert(output.actuators_active);
    assert(output.attitude_valid);
    assert(output.sample_id == UINT32_MAX);
    assert(output.motor_commit_time_ms == input.motor_commit_time_ms);
    assert(output.output_sample_matched);

    /* The new packet remains inside the existing 64-byte STM32 UART frame. */
    {
        uint8_t framed[64];
        uint8_t decoded[DRONE_PROTOCOL_MAX_PACKET_SIZE];
        const size_t n = DroneCobs_Encode(packet, sizeof(packet), framed, sizeof(framed) - 1U);
        assert(n > 0U && n + 1U <= sizeof(framed));
        assert(DroneCobs_Decode(framed, n, decoded, sizeof(decoded)) == sizeof(packet));
        assert(memcmp(decoded, packet, sizeof(packet)) == 0);
    }

    /* Legacy packets decode without inventing sample IDs/commit association. */
    {
        uint8_t legacy[DRONE_FLIGHT_TELEMETRY_LEGACY_PACKET_SIZE];
        memcpy(legacy, packet, 48U);
        legacy[3] = DRONE_PACKET_FLIGHT_TELEMETRY;
        legacy[10] = DRONE_FLIGHT_TELEMETRY_LEGACY_PAYLOAD_SIZE;
        legacy[8] &= (uint8_t)~DRONE_FLIGHT_TELEMETRY_FLAG_OUTPUT_SAMPLE_MATCHED;
        DroneProtocol_WriteU16Le(legacy + 48U, DroneProtocol_Crc16CcittFalse(legacy, 48U));
        assert(DroneProtocol_DecodeFlightTelemetry(legacy, sizeof(legacy), &output) == DRONE_PROTOCOL_OK);
        assert(output.gyro_mrad_s[2] == input.gyro_mrad_s[2]);
        assert(output.sample_id == 0U && output.motor_commit_time_ms == 0U);
        assert(!output.output_sample_matched);
        legacy[8] |= DRONE_FLIGHT_TELEMETRY_FLAG_OUTPUT_SAMPLE_MATCHED;
        DroneProtocol_WriteU16Le(legacy + 48U, DroneProtocol_Crc16CcittFalse(legacy, 48U));
        assert(DroneProtocol_DecodeFlightTelemetry(legacy, sizeof(legacy), &output) == DRONE_PROTOCOL_FLAGS_ERROR);
    }

    packet[42] ^= 1U;
    assert(DroneProtocol_DecodeFlightTelemetry(packet, sizeof(packet),
                                                &output) ==
           DRONE_PROTOCOL_CRC_ERROR);
}

static size_t stream_packet_count;
static void assert_stream_packet(const uint8_t *packet, size_t length, void *context)
{
    assert(length == DRONE_FLIGHT_TELEMETRY_PACKET_SIZE);
    assert(memcmp(packet, context, length) == 0);
    ++stream_packet_count;
}

static void test_esp_stream_capacity_and_fragmentation(void)
{
    packet_stream_t stream;
    uint8_t packet[DRONE_FLIGHT_TELEMETRY_PACKET_SIZE];
    uint8_t frame[DRONE_PROTOCOL_MAX_PACKET_SIZE + 3U];
    memset(packet, 0xFF, sizeof(packet));
    stream_packet_count = 0U;
    packet_stream_init(&stream, assert_stream_packet, packet);
    const size_t n = packet_stream_encode(packet, sizeof(packet), frame, sizeof(frame));
    assert(n == sizeof(frame)); /* Worst case: no zero bytes in the raw packet. */
    for (size_t i = 0U; i < n; ++i)
    {
        packet_stream_feed(&stream, frame + i, 1U);
    }
    assert(stream_packet_count == 1U);
    memset(packet, 0, sizeof(packet));
    const size_t m = packet_stream_encode(packet, sizeof(packet), frame, sizeof(frame));
    assert(m > 0U);
    packet_stream_feed(&stream, frame, m);
    assert(stream_packet_count == 2U);
}

static void test_control_validation(void)
{
    DroneControlCommand input = {
        .header = {.sequence = 1U, .session_id = 2U},
        .throttle = 0U,
    };
    DroneControlCommand output;
    uint8_t packet[DRONE_CONTROL_PACKET_SIZE];

    assert(DroneProtocol_EncodeControl(&input, packet) == DRONE_PROTOCOL_OK);
    packet[0] = 0U;
    assert(DroneProtocol_DecodeControl(packet, sizeof(packet), &output) ==
           DRONE_PROTOCOL_MAGIC_ERROR);

    assert(DroneProtocol_EncodeControl(&input, packet) == DRONE_PROTOCOL_OK);
    packet[2] = 2U;
    assert(DroneProtocol_DecodeControl(packet, sizeof(packet), &output) ==
           DRONE_PROTOCOL_VERSION_ERROR);

    assert(DroneProtocol_EncodeControl(&input, packet) == DRONE_PROTOCOL_OK);
    assert(DroneProtocol_DecodeControl(packet, sizeof(packet) - 1U, &output) ==
           DRONE_PROTOCOL_SIZE_ERROR);

    input.throttle = 1001U;
    assert(DroneProtocol_EncodeControl(&input, packet) ==
           DRONE_PROTOCOL_RANGE_ERROR);

    input.throttle = 0U;
    input.aux1 = DRONE_CONTROL_MOTOR_SELECT_MAX + 1U;
    assert(DroneProtocol_EncodeControl(&input, packet) ==
           DRONE_PROTOCOL_RANGE_ERROR);

    input.aux1 = DRONE_CONTROL_MOTOR_SELECT_ALL;
    input.aux2 = 1U;
    assert(DroneProtocol_EncodeControl(&input, packet) ==
           DRONE_PROTOCOL_RANGE_ERROR);
}

static void test_cobs_and_sequence(void)
{
    static const uint8_t source[] = {0U, 1U, 2U, 0U, 3U, 0U};
    static const uint8_t malformed[] = {3U, 1U};
    uint8_t encoded[16];
    uint8_t decoded[16];
    const size_t encoded_length =
        DroneCobs_Encode(source, sizeof(source), encoded, sizeof(encoded));
    const size_t decoded_length =
        DroneCobs_Decode(encoded, encoded_length, decoded, sizeof(decoded));

    assert(encoded_length != 0U);
    assert(decoded_length == sizeof(source));
    assert(memcmp(source, decoded, sizeof(source)) == 0);
    assert(DroneCobs_Decode(malformed, sizeof(malformed),
                            decoded, sizeof(decoded)) == 0U);
    assert(DroneCobs_Decode(encoded, encoded_length,
                            decoded, sizeof(decoded)) == sizeof(source));
    assert(DroneProtocol_IsSequenceNewer(1U, 0U));
    assert(DroneProtocol_IsSequenceNewer(0U, 65535U));
    assert(!DroneProtocol_IsSequenceNewer(65535U, 0U));
    assert(!DroneProtocol_IsSequenceNewer(42U, 42U));
}

int main(void)
{
    test_crc_known_vector();
    test_control_round_trip();
    test_control_modes();
    test_status_round_trip();
    test_flight_telemetry_round_trip();
    test_esp_stream_capacity_and_fragmentation();
    test_control_validation();
    test_cobs_and_sequence();
    puts("drone protocol tests: PASS");
    return 0;
}
