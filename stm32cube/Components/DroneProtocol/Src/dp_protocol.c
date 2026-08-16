#include "dp_protocol.h"
#include "dp_bytes.h"

#include <string.h>

static void encode_header(const DronePacketHeader *header, uint8_t *output)
{
    DroneProtocol_WriteU16Le(output, DRONE_PROTOCOL_MAGIC);
    output[2] = DRONE_PROTOCOL_VERSION;
    output[3] = header->type;
    DroneProtocol_WriteU16Le(output + 4, header->sequence);
    DroneProtocol_WriteU16Le(output + 6, header->session_id);
    DroneProtocol_WriteU16Le(output + 8, header->flags);
    output[10] = header->payload_length;
    output[11] = 0U;
    DroneProtocol_WriteU32Le(output + 12, header->sender_time_ms);
}

static DroneProtocolResult decode_header(const uint8_t *packet,
                                         size_t packet_length,
                                         uint8_t required_type,
                                         uint8_t required_payload_length,
                                         DronePacketHeader *header)
{
    uint16_t expected_crc;
    uint16_t packet_crc;

    if ((packet == NULL) || (header == NULL))
    {
        return DRONE_PROTOCOL_NULL_ARGUMENT;
    }
    if (packet_length !=
        (DRONE_PROTOCOL_HEADER_SIZE + required_payload_length +
         DRONE_PROTOCOL_CRC_SIZE))
    {
        return DRONE_PROTOCOL_SIZE_ERROR;
    }
    if (DroneProtocol_ReadU16Le(packet) != DRONE_PROTOCOL_MAGIC)
    {
        return DRONE_PROTOCOL_MAGIC_ERROR;
    }
    if (packet[2] != DRONE_PROTOCOL_VERSION)
    {
        return DRONE_PROTOCOL_VERSION_ERROR;
    }
    if (packet[3] != required_type)
    {
        return DRONE_PROTOCOL_TYPE_ERROR;
    }
    if (packet[10] != required_payload_length)
    {
        return DRONE_PROTOCOL_LENGTH_ERROR;
    }
    if (packet[11] != 0U)
    {
        return DRONE_PROTOCOL_RESERVED_ERROR;
    }

    packet_crc = DroneProtocol_ReadU16Le(
        packet + packet_length - DRONE_PROTOCOL_CRC_SIZE);
    expected_crc = DroneProtocol_Crc16CcittFalse(
        packet, packet_length - DRONE_PROTOCOL_CRC_SIZE);
    if (packet_crc != expected_crc)
    {
        return DRONE_PROTOCOL_CRC_ERROR;
    }

    header->type = packet[3];
    header->sequence = DroneProtocol_ReadU16Le(packet + 4);
    header->session_id = DroneProtocol_ReadU16Le(packet + 6);
    header->flags = DroneProtocol_ReadU16Le(packet + 8);
    header->payload_length = packet[10];
    header->sender_time_ms = DroneProtocol_ReadU32Le(packet + 12);
    return DRONE_PROTOCOL_OK;
}

static void append_crc(uint8_t *packet, size_t packet_length)
{
    const uint16_t crc = DroneProtocol_Crc16CcittFalse(
        packet, packet_length - DRONE_PROTOCOL_CRC_SIZE);
    DroneProtocol_WriteU16Le(packet + packet_length - DRONE_PROTOCOL_CRC_SIZE,
                             crc);
}

uint16_t DroneProtocol_Crc16CcittFalse(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t i;
    uint8_t bit;

    if ((data == NULL) && (length != 0U))
    {
        return 0U;
    }

    for (i = 0U; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = ((crc & 0x8000U) != 0U)
                      ? (uint16_t)((crc << 1) ^ 0x1021U)
                      : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

DroneProtocolResult DroneProtocol_EncodeControl(
    const DroneControlCommand *command,
    uint8_t output[DRONE_CONTROL_PACKET_SIZE])
{
    DronePacketHeader header;

    if ((command == NULL) || (output == NULL))
    {
        return DRONE_PROTOCOL_NULL_ARGUMENT;
    }
    if ((command->header.flags & ~DRONE_CONTROL_FLAG_ALLOWED_MASK) != 0U)
    {
        return DRONE_PROTOCOL_FLAGS_ERROR;
    }
    if ((command->throttle > 1000U) ||
        (command->roll < -1000) || (command->roll > 1000) ||
        (command->pitch < -1000) || (command->pitch > 1000) ||
        (command->yaw < -1000) || (command->yaw > 1000) ||
        (command->aux1 > DRONE_CONTROL_MOTOR_SELECT_MAX) ||
        (command->aux2 != 0U))
    {
        return DRONE_PROTOCOL_RANGE_ERROR;
    }

    memset(output, 0, DRONE_CONTROL_PACKET_SIZE);
    header = command->header;
    header.type = DRONE_PACKET_CONTROL_COMMAND;
    header.payload_length = DRONE_CONTROL_PAYLOAD_SIZE;
    encode_header(&header, output);
    DroneProtocol_WriteU16Le(output + 16, command->throttle);
    DroneProtocol_WriteI16Le(output + 18, command->roll);
    DroneProtocol_WriteI16Le(output + 20, command->pitch);
    DroneProtocol_WriteI16Le(output + 22, command->yaw);
    DroneProtocol_WriteU16Le(output + 24, command->aux1);
    DroneProtocol_WriteU16Le(output + 26, command->aux2);
    append_crc(output, DRONE_CONTROL_PACKET_SIZE);
    return DRONE_PROTOCOL_OK;
}

DroneProtocolResult DroneProtocol_DecodeControl(
    const uint8_t *packet,
    size_t packet_length,
    DroneControlCommand *command)
{
    DroneProtocolResult result;

    if (command == NULL)
    {
        return DRONE_PROTOCOL_NULL_ARGUMENT;
    }
    result = decode_header(packet, packet_length,
                           DRONE_PACKET_CONTROL_COMMAND,
                           DRONE_CONTROL_PAYLOAD_SIZE,
                           &command->header);
    if (result != DRONE_PROTOCOL_OK)
    {
        return result;
    }
    if ((command->header.flags & ~DRONE_CONTROL_FLAG_ALLOWED_MASK) != 0U)
    {
        return DRONE_PROTOCOL_FLAGS_ERROR;
    }

    command->throttle = DroneProtocol_ReadU16Le(packet + 16);
    command->roll = DroneProtocol_ReadI16Le(packet + 18);
    command->pitch = DroneProtocol_ReadI16Le(packet + 20);
    command->yaw = DroneProtocol_ReadI16Le(packet + 22);
    command->aux1 = DroneProtocol_ReadU16Le(packet + 24);
    command->aux2 = DroneProtocol_ReadU16Le(packet + 26);
    if ((command->throttle > 1000U) ||
        (command->roll < -1000) || (command->roll > 1000) ||
        (command->pitch < -1000) || (command->pitch > 1000) ||
        (command->yaw < -1000) || (command->yaw > 1000) ||
        (command->aux1 > DRONE_CONTROL_MOTOR_SELECT_MAX) ||
        (command->aux2 != 0U))
    {
        return DRONE_PROTOCOL_RANGE_ERROR;
    }
    return DRONE_PROTOCOL_OK;
}

DroneProtocolResult DroneProtocol_EncodeStatus(
    const DroneSystemStatus *status,
    uint8_t output[DRONE_STATUS_PACKET_SIZE])
{
    DronePacketHeader header;

    if ((status == NULL) || (output == NULL))
    {
        return DRONE_PROTOCOL_NULL_ARGUMENT;
    }
    if ((status->requested_throttle > 1000U) ||
        (status->applied_throttle > 1000U) ||
        (status->state > DRONE_STATE_ERROR))
    {
        return DRONE_PROTOCOL_RANGE_ERROR;
    }

    memset(output, 0, DRONE_STATUS_PACKET_SIZE);
    header = status->header;
    header.type = DRONE_PACKET_SYSTEM_STATUS;
    header.flags = 0U;
    header.payload_length = DRONE_STATUS_PAYLOAD_SIZE;
    encode_header(&header, output);
    DroneProtocol_WriteU16Le(output + 16, status->last_control_sequence);
    DroneProtocol_WriteU16Le(output + 18, status->requested_throttle);
    DroneProtocol_WriteU16Le(output + 20, status->applied_throttle);
    DroneProtocol_WriteU16Le(output + 22, status->pwm_pulse_us[0]);
    DroneProtocol_WriteU16Le(output + 24, status->pwm_pulse_us[1]);
    DroneProtocol_WriteU16Le(output + 26, status->pwm_pulse_us[2]);
    DroneProtocol_WriteU16Le(output + 28, status->pwm_pulse_us[3]);
    output[30] = status->state;
    DroneProtocol_WriteU16Le(output + 31, status->error_flags);
    DroneProtocol_WriteU16Le(output + 33, status->uart_rx_rate);
    output[35] = 0U;
    append_crc(output, DRONE_STATUS_PACKET_SIZE);
    return DRONE_PROTOCOL_OK;
}

DroneProtocolResult DroneProtocol_DecodeStatus(
    const uint8_t *packet,
    size_t packet_length,
    DroneSystemStatus *status)
{
    DroneProtocolResult result;

    if (status == NULL)
    {
        return DRONE_PROTOCOL_NULL_ARGUMENT;
    }
    result = decode_header(packet, packet_length,
                           DRONE_PACKET_SYSTEM_STATUS,
                           DRONE_STATUS_PAYLOAD_SIZE,
                           &status->header);
    if (result != DRONE_PROTOCOL_OK)
    {
        return result;
    }
    if ((status->header.flags != 0U) || (packet[35] != 0U))
    {
        return DRONE_PROTOCOL_RESERVED_ERROR;
    }

    status->last_control_sequence = DroneProtocol_ReadU16Le(packet + 16);
    status->requested_throttle = DroneProtocol_ReadU16Le(packet + 18);
    status->applied_throttle = DroneProtocol_ReadU16Le(packet + 20);
    status->pwm_pulse_us[0] = DroneProtocol_ReadU16Le(packet + 22);
    status->pwm_pulse_us[1] = DroneProtocol_ReadU16Le(packet + 24);
    status->pwm_pulse_us[2] = DroneProtocol_ReadU16Le(packet + 26);
    status->pwm_pulse_us[3] = DroneProtocol_ReadU16Le(packet + 28);
    status->state = packet[30];
    status->error_flags = DroneProtocol_ReadU16Le(packet + 31);
    status->uart_rx_rate = DroneProtocol_ReadU16Le(packet + 33);
    if ((status->requested_throttle > 1000U) ||
        (status->applied_throttle > 1000U) ||
        (status->state > DRONE_STATE_ERROR))
    {
        return DRONE_PROTOCOL_RANGE_ERROR;
    }
    return DRONE_PROTOCOL_OK;
}

DroneProtocolResult DroneProtocol_EncodeFlightTelemetry(
    const DroneFlightTelemetry *telemetry,
    uint8_t output[DRONE_FLIGHT_TELEMETRY_PACKET_SIZE])
{
    DronePacketHeader header;
    uint8_t axis;
    uint8_t motor;

    if ((telemetry == NULL) || (output == NULL))
    {
        return DRONE_PROTOCOL_NULL_ARGUMENT;
    }
    if ((telemetry->state > DRONE_STATE_ERROR) ||
        (telemetry->header.flags &
         ~DRONE_FLIGHT_TELEMETRY_FLAG_ALLOWED_MASK) != 0U)
    {
        return DRONE_PROTOCOL_RANGE_ERROR;
    }
    for (motor = 0U; motor < DRONE_PROTOCOL_MOTOR_COUNT; ++motor)
    {
        if ((telemetry->motor_pwm_us[motor] < 1000U) ||
            (telemetry->motor_pwm_us[motor] > 2000U))
        {
            return DRONE_PROTOCOL_RANGE_ERROR;
        }
    }

    memset(output, 0, DRONE_FLIGHT_TELEMETRY_PACKET_SIZE);
    header = telemetry->header;
    header.type = DRONE_PACKET_FLIGHT_TELEMETRY;
    header.flags = (uint16_t)(telemetry->state &
                              DRONE_FLIGHT_TELEMETRY_FLAG_STATE_MASK);
    if (telemetry->actuators_active)
    {
        header.flags |= DRONE_FLIGHT_TELEMETRY_FLAG_ACTUATORS_ACTIVE;
    }
    if (telemetry->attitude_valid)
    {
        header.flags |= DRONE_FLIGHT_TELEMETRY_FLAG_ATTITUDE_VALID;
    }
    header.payload_length = DRONE_FLIGHT_TELEMETRY_PAYLOAD_SIZE;
    encode_header(&header, output);

    for (axis = 0U; axis < 3U; ++axis)
    {
        DroneProtocol_WriteI16Le(output + 16U + (2U * axis),
                                 telemetry->attitude_cdeg[axis]);
        DroneProtocol_WriteI16Le(output + 22U + (2U * axis),
                                 telemetry->gyro_mrad_s[axis]);
        DroneProtocol_WriteI16Le(output + 28U + (2U * axis),
                                 telemetry->rate_setpoint_mrad_s[axis]);
        DroneProtocol_WriteI16Le(output + 34U + (2U * axis),
                                 telemetry->pid_command_centi[axis]);
    }
    for (motor = 0U; motor < DRONE_PROTOCOL_MOTOR_COUNT; ++motor)
    {
        DroneProtocol_WriteU16Le(output + 40U + (2U * motor),
                                 telemetry->motor_pwm_us[motor]);
    }
    append_crc(output, DRONE_FLIGHT_TELEMETRY_PACKET_SIZE);
    return DRONE_PROTOCOL_OK;
}

DroneProtocolResult DroneProtocol_DecodeFlightTelemetry(
    const uint8_t *packet,
    size_t packet_length,
    DroneFlightTelemetry *telemetry)
{
    DroneProtocolResult result;
    uint8_t axis;
    uint8_t motor;

    if (telemetry == NULL)
    {
        return DRONE_PROTOCOL_NULL_ARGUMENT;
    }
    result = decode_header(packet, packet_length,
                           DRONE_PACKET_FLIGHT_TELEMETRY,
                           DRONE_FLIGHT_TELEMETRY_PAYLOAD_SIZE,
                           &telemetry->header);
    if (result != DRONE_PROTOCOL_OK)
    {
        return result;
    }
    if ((telemetry->header.flags &
         ~DRONE_FLIGHT_TELEMETRY_FLAG_ALLOWED_MASK) != 0U)
    {
        return DRONE_PROTOCOL_FLAGS_ERROR;
    }

    telemetry->state = (uint8_t)(telemetry->header.flags &
                                 DRONE_FLIGHT_TELEMETRY_FLAG_STATE_MASK);
    telemetry->actuators_active =
        (telemetry->header.flags &
         DRONE_FLIGHT_TELEMETRY_FLAG_ACTUATORS_ACTIVE) != 0U;
    telemetry->attitude_valid =
        (telemetry->header.flags &
         DRONE_FLIGHT_TELEMETRY_FLAG_ATTITUDE_VALID) != 0U;
    if (telemetry->state > DRONE_STATE_ERROR)
    {
        return DRONE_PROTOCOL_RANGE_ERROR;
    }
    for (axis = 0U; axis < 3U; ++axis)
    {
        telemetry->attitude_cdeg[axis] =
            DroneProtocol_ReadI16Le(packet + 16U + (2U * axis));
        telemetry->gyro_mrad_s[axis] =
            DroneProtocol_ReadI16Le(packet + 22U + (2U * axis));
        telemetry->rate_setpoint_mrad_s[axis] =
            DroneProtocol_ReadI16Le(packet + 28U + (2U * axis));
        telemetry->pid_command_centi[axis] =
            DroneProtocol_ReadI16Le(packet + 34U + (2U * axis));
    }
    for (motor = 0U; motor < DRONE_PROTOCOL_MOTOR_COUNT; ++motor)
    {
        telemetry->motor_pwm_us[motor] =
            DroneProtocol_ReadU16Le(packet + 40U + (2U * motor));
        if ((telemetry->motor_pwm_us[motor] < 1000U) ||
            (telemetry->motor_pwm_us[motor] > 2000U))
        {
            return DRONE_PROTOCOL_RANGE_ERROR;
        }
    }
    return DRONE_PROTOCOL_OK;
}

bool DroneProtocol_IsSequenceNewer(uint16_t candidate, uint16_t reference)
{
    return (candidate != reference) &&
           ((uint16_t)(candidate - reference) < 0x8000U);
}
