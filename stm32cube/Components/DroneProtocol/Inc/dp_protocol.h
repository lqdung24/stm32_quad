#ifndef DRONE_PROTOCOL_H
#define DRONE_PROTOCOL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DRONE_PROTOCOL_MAGIC 0xA55AU
#define DRONE_PROTOCOL_VERSION 1U
#define DRONE_PROTOCOL_HEADER_SIZE 16U
#define DRONE_PROTOCOL_CRC_SIZE 2U
#define DRONE_PROTOCOL_MAX_PAYLOAD_SIZE 32U
#define DRONE_PROTOCOL_MAX_PACKET_SIZE 50U
#define DRONE_CONTROL_PAYLOAD_SIZE 12U
#define DRONE_CONTROL_PACKET_SIZE 30U
#define DRONE_STATUS_PAYLOAD_SIZE 20U
#define DRONE_STATUS_PACKET_SIZE 38U
#define DRONE_FLIGHT_TELEMETRY_PAYLOAD_SIZE 32U
#define DRONE_FLIGHT_TELEMETRY_PACKET_SIZE 50U
#define DRONE_PROTOCOL_MOTOR_COUNT 4U

#define DRONE_CONTROL_FLAG_ARM_REQUEST (1U << 0)
#define DRONE_CONTROL_FLAG_EMERGENCY_STOP (1U << 1)
#define DRONE_CONTROL_FLAG_ANGLE_MODE (1U << 2)
#define DRONE_CONTROL_FLAG_ACRO_MODE (1U << 3)
#define DRONE_CONTROL_FLAG_FAILSAFE_TEST (1U << 4)
#define DRONE_CONTROL_FLAG_ALLOWED_MASK 0x001FU

/* Flight telemetry flags live in the packet header, not in its payload. */
#define DRONE_FLIGHT_TELEMETRY_FLAG_STATE_MASK 0x0007U
#define DRONE_FLIGHT_TELEMETRY_FLAG_ACTUATORS_ACTIVE (1U << 3)
#define DRONE_FLIGHT_TELEMETRY_FLAG_ATTITUDE_VALID (1U << 4)
#define DRONE_FLIGHT_TELEMETRY_FLAG_ALLOWED_MASK 0x001FU

/* AUX1 motor selection used only by the no-prop threshold-test mode. */
#define DRONE_CONTROL_MOTOR_SELECT_ALL 0U
#define DRONE_CONTROL_MOTOR_SELECT_M1 1U
#define DRONE_CONTROL_MOTOR_SELECT_M2 2U
#define DRONE_CONTROL_MOTOR_SELECT_M3 3U
#define DRONE_CONTROL_MOTOR_SELECT_M4 4U
#define DRONE_CONTROL_MOTOR_SELECT_MAX DRONE_CONTROL_MOTOR_SELECT_M4

#define DRONE_ERROR_PHONE_LINK_LOST (1U << 0)
#define DRONE_ERROR_UART_LINK_LOST (1U << 1)
#define DRONE_ERROR_CRC (1U << 2)
#define DRONE_ERROR_INVALID_PACKET (1U << 3)
#define DRONE_ERROR_SEQUENCE (1U << 4)
#define DRONE_ERROR_SESSION_CHANGED (1U << 5)
#define DRONE_ERROR_THROTTLE_CLAMPED (1U << 6)
#define DRONE_ERROR_ARM_REJECTED (1U << 7)
#define DRONE_ERROR_PWM_INIT (1U << 8)
#define DRONE_ERROR_FAILSAFE_ACTIVE (1U << 9)

    typedef enum
    {
        DRONE_PACKET_CONTROL_COMMAND = 0x01,
        DRONE_PACKET_SYSTEM_STATUS = 0x02,
        DRONE_PACKET_HEARTBEAT = 0x03,
        DRONE_PACKET_CONFIG_REQUEST = 0x04,
        DRONE_PACKET_CONFIG_RESPONSE = 0x05,
        DRONE_PACKET_ERROR_REPORT = 0x06,
        DRONE_PACKET_FLIGHT_TELEMETRY = 0x07
    } DronePacketType;

    typedef enum
    {
        DRONE_STATE_BOOT = 0,
        DRONE_STATE_DISARMED = 1,
        DRONE_STATE_ARMED = 2,
        DRONE_STATE_FAILSAFE = 3,
        DRONE_STATE_ERROR = 4
    } DroneSystemState;

    typedef enum
    {
        DRONE_PROTOCOL_OK = 0,
        DRONE_PROTOCOL_NULL_ARGUMENT,
        DRONE_PROTOCOL_SIZE_ERROR,
        DRONE_PROTOCOL_MAGIC_ERROR,
        DRONE_PROTOCOL_VERSION_ERROR,
        DRONE_PROTOCOL_TYPE_ERROR,
        DRONE_PROTOCOL_LENGTH_ERROR,
        DRONE_PROTOCOL_RESERVED_ERROR,
        DRONE_PROTOCOL_FLAGS_ERROR,
        DRONE_PROTOCOL_RANGE_ERROR,
        DRONE_PROTOCOL_CRC_ERROR
    } DroneProtocolResult;

    typedef struct
    {
        uint8_t type;
        uint16_t sequence;
        uint16_t session_id;
        uint16_t flags;
        uint8_t payload_length;
        uint32_t sender_time_ms;
    } DronePacketHeader;

    typedef struct
    {
        DronePacketHeader header;
        uint16_t throttle;
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
        uint16_t aux1;
        uint16_t aux2;
    } DroneControlCommand;

    typedef struct
    {
        DronePacketHeader header;
        uint16_t last_control_sequence;
        uint16_t requested_throttle;
        uint16_t applied_throttle;
        uint16_t pwm_pulse_us[DRONE_PROTOCOL_MOTOR_COUNT];
        uint8_t state;
        uint16_t error_flags;
        uint16_t uart_rx_rate;
    } DroneSystemStatus;

    /*
     * Packed payload layout (all values are little-endian):
     * attitude_cdeg[3], gyro_mrad_s[3], rate_setpoint_mrad_s[3],
     * pid_command_centi[3], motor_pwm_us[4].
     * The state and validity bits are carried in header.flags.
     */
    typedef struct
    {
        DronePacketHeader header;
        int16_t attitude_cdeg[3];
        int16_t gyro_mrad_s[3];
        int16_t rate_setpoint_mrad_s[3];
        int16_t pid_command_centi[3];
        uint16_t motor_pwm_us[DRONE_PROTOCOL_MOTOR_COUNT];
        uint8_t state;
        bool actuators_active;
        bool attitude_valid;
    } DroneFlightTelemetry;

    uint16_t DroneProtocol_Crc16CcittFalse(const uint8_t *data, size_t length);

    DroneProtocolResult DroneProtocol_EncodeControl(
        const DroneControlCommand *command,
        uint8_t output[DRONE_CONTROL_PACKET_SIZE]);

    DroneProtocolResult DroneProtocol_DecodeControl(
        const uint8_t *packet,
        size_t packet_length,
        DroneControlCommand *command);

    DroneProtocolResult DroneProtocol_EncodeStatus(
        const DroneSystemStatus *status,
        uint8_t output[DRONE_STATUS_PACKET_SIZE]);

    DroneProtocolResult DroneProtocol_DecodeStatus(
        const uint8_t *packet,
        size_t packet_length,
        DroneSystemStatus *status);

    DroneProtocolResult DroneProtocol_EncodeFlightTelemetry(
        const DroneFlightTelemetry *telemetry,
        uint8_t output[DRONE_FLIGHT_TELEMETRY_PACKET_SIZE]);

    DroneProtocolResult DroneProtocol_DecodeFlightTelemetry(
        const uint8_t *packet,
        size_t packet_length,
        DroneFlightTelemetry *telemetry);

    bool DroneProtocol_IsSequenceNewer(uint16_t candidate, uint16_t reference);

#ifdef __cplusplus
}
#endif

#endif
