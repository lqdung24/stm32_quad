#include "drone_protocol.h"

#include <string.h>

static uint16_t read_u16_le(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t read_i16_le(const uint8_t *data)
{
  return (int16_t)read_u16_le(data);
}

static uint32_t read_u32_le(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
}

static void write_i16_le(uint8_t *data, int16_t value)
{
  write_u16_le(data, (uint16_t)value);
}

static void write_u32_le(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
  data[2] = (uint8_t)(value >> 16);
  data[3] = (uint8_t)(value >> 24);
}

static void encode_header(const DronePacketHeader *header, uint8_t *output)
{
  write_u16_le(output, DRONE_PROTOCOL_MAGIC);
  output[2] = DRONE_PROTOCOL_VERSION;
  output[3] = header->type;
  write_u16_le(output + 4, header->sequence);
  write_u16_le(output + 6, header->session_id);
  write_u16_le(output + 8, header->flags);
  output[10] = header->payload_length;
  output[11] = 0U;
  write_u32_le(output + 12, header->sender_time_ms);
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
  if (read_u16_le(packet) != DRONE_PROTOCOL_MAGIC)
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

  packet_crc = read_u16_le(packet + packet_length - DRONE_PROTOCOL_CRC_SIZE);
  expected_crc = DroneProtocol_Crc16CcittFalse(
      packet, packet_length - DRONE_PROTOCOL_CRC_SIZE);
  if (packet_crc != expected_crc)
  {
    return DRONE_PROTOCOL_CRC_ERROR;
  }

  header->type = packet[3];
  header->sequence = read_u16_le(packet + 4);
  header->session_id = read_u16_le(packet + 6);
  header->flags = read_u16_le(packet + 8);
  header->payload_length = packet[10];
  header->sender_time_ms = read_u32_le(packet + 12);
  return DRONE_PROTOCOL_OK;
}

static void append_crc(uint8_t *packet, size_t packet_length)
{
  const uint16_t crc = DroneProtocol_Crc16CcittFalse(
      packet, packet_length - DRONE_PROTOCOL_CRC_SIZE);
  write_u16_le(packet + packet_length - DRONE_PROTOCOL_CRC_SIZE, crc);
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
      (command->aux1 > 1000U) || (command->aux2 > 1000U))
  {
    return DRONE_PROTOCOL_RANGE_ERROR;
  }

  memset(output, 0, DRONE_CONTROL_PACKET_SIZE);
  header = command->header;
  header.type = DRONE_PACKET_CONTROL_COMMAND;
  header.payload_length = DRONE_CONTROL_PAYLOAD_SIZE;
  encode_header(&header, output);
  write_u16_le(output + 16, command->throttle);
  write_i16_le(output + 18, command->roll);
  write_i16_le(output + 20, command->pitch);
  write_i16_le(output + 22, command->yaw);
  write_u16_le(output + 24, command->aux1);
  write_u16_le(output + 26, command->aux2);
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

  command->throttle = read_u16_le(packet + 16);
  command->roll = read_i16_le(packet + 18);
  command->pitch = read_i16_le(packet + 20);
  command->yaw = read_i16_le(packet + 22);
  command->aux1 = read_u16_le(packet + 24);
  command->aux2 = read_u16_le(packet + 26);
  if ((command->throttle > 1000U) ||
      (command->roll < -1000) || (command->roll > 1000) ||
      (command->pitch < -1000) || (command->pitch > 1000) ||
      (command->yaw < -1000) || (command->yaw > 1000) ||
      (command->aux1 > 1000U) || (command->aux2 > 1000U))
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
  write_u16_le(output + 16, status->last_control_sequence);
  write_u16_le(output + 18, status->requested_throttle);
  write_u16_le(output + 20, status->applied_throttle);
  write_u16_le(output + 22, status->pwm_pulse_us);
  output[24] = status->state;
  write_u16_le(output + 25, status->error_flags);
  write_u16_le(output + 27, status->uart_rx_rate);
  output[29] = 0U;
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
  if ((status->header.flags != 0U) || (packet[29] != 0U))
  {
    return DRONE_PROTOCOL_RESERVED_ERROR;
  }

  status->last_control_sequence = read_u16_le(packet + 16);
  status->requested_throttle = read_u16_le(packet + 18);
  status->applied_throttle = read_u16_le(packet + 20);
  status->pwm_pulse_us = read_u16_le(packet + 22);
  status->state = packet[24];
  status->error_flags = read_u16_le(packet + 25);
  status->uart_rx_rate = read_u16_le(packet + 27);
  if ((status->requested_throttle > 1000U) ||
      (status->applied_throttle > 1000U) ||
      (status->state > DRONE_STATE_ERROR))
  {
    return DRONE_PROTOCOL_RANGE_ERROR;
  }
  return DRONE_PROTOCOL_OK;
}

bool DroneProtocol_IsSequenceNewer(uint16_t candidate, uint16_t reference)
{
  return (candidate != reference) &&
         ((uint16_t)(candidate - reference) < 0x8000U);
}
