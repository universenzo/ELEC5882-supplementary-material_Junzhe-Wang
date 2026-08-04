#ifndef CAN_IV_PROTOCOL_H
#define CAN_IV_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_IV_PROTOCOL_VERSION 1U
#define CAN_IV_CLASSIC_DLC       8U

/* PC -> STM32, 11-bit standard identifiers. */
#define CAN_IV_ID_COMMAND        0x100U
#define CAN_IV_ID_SWEEP_CONFIG_A 0x101U
#define CAN_IV_ID_SWEEP_CONFIG_B 0x102U
#define CAN_IV_ID_PID_CONFIG     0x103U

/* STM32 -> PC, 11-bit standard identifiers. */
#define CAN_IV_ID_ACK            0x180U
#define CAN_IV_ID_STATUS         0x181U
#define CAN_IV_ID_INA_SAMPLE     0x182U
#define CAN_IV_ID_POINT_A        0x200U
#define CAN_IV_ID_POINT_B        0x201U
#define CAN_IV_ID_EVENT          0x280U

typedef enum {
    CAN_IV_CMD_PING = 0x01,
    CAN_IV_CMD_GET_STATUS = 0x02,
    CAN_IV_CMD_START_SWEEP = 0x03,
    CAN_IV_CMD_ABORT = 0x04,
    CAN_IV_CMD_GET_POINT = 0x05,
    CAN_IV_CMD_GET_INA_SAMPLE = 0x06,
    CAN_IV_CMD_CLEAR_DATA = 0x07,
    CAN_IV_CMD_START_PID_FF = 0x08,
    CAN_IV_CMD_STOP_OUTPUT = 0x09
} can_iv_opcode_t;

typedef enum {
    CAN_IV_RESULT_OK = 0,
    CAN_IV_RESULT_BAD_DLC = 1,
    CAN_IV_RESULT_BAD_COMMAND = 2,
    CAN_IV_RESULT_BAD_CONFIG = 3,
    CAN_IV_RESULT_NOT_ARMED = 4,
    CAN_IV_RESULT_BUSY = 5,
    CAN_IV_RESULT_BAD_INDEX = 6,
    CAN_IV_RESULT_SENSOR_INVALID = 7,
    CAN_IV_RESULT_FAULT_LATCHED = 8,
    CAN_IV_RESULT_NOT_IMPLEMENTED = 9
} can_iv_result_t;

typedef enum {
    CAN_IV_STATE_IDLE = 0,
    CAN_IV_STATE_PHYSICALLY_ARMED = 1,
    CAN_IV_STATE_SWEEP_SETTLING = 2,
    CAN_IV_STATE_SWEEP_SAMPLING = 3,
    CAN_IV_STATE_DATA_READY = 4,
    CAN_IV_STATE_PID_FF_ACTIVE = 5,
    CAN_IV_STATE_FAULT = 6
} can_iv_state_t;

typedef enum {
    CAN_IV_EVENT_ARMED = 1,
    CAN_IV_EVENT_SWEEP_STARTED = 2,
    CAN_IV_EVENT_SWEEP_COMPLETE = 3,
    CAN_IV_EVENT_SOFT_CURRENT_STOP = 4,
    CAN_IV_EVENT_SOFT_TEMPERATURE_STOP = 5,
    CAN_IV_EVENT_ABORTED = 6,
    CAN_IV_EVENT_FAULT = 7,
    CAN_IV_EVENT_PID_STARTED = 8,
    CAN_IV_EVENT_OUTPUT_STOPPED = 9
} can_iv_event_t;

/* Bits 0..2 deliberately preserve the existing project fault meanings. */
typedef enum {
    CAN_IV_FAULT_OVERCURRENT = (1U << 0),
    CAN_IV_FAULT_OVERTEMPERATURE = (1U << 1),
    CAN_IV_FAULT_NTC_RAIL = (1U << 2),
    CAN_IV_FAULT_CONFIGURATION = (1U << 3),
    CAN_IV_FAULT_SENSOR_IO = (1U << 4),
    CAN_IV_FAULT_POINT_BUFFER = (1U << 5)
} can_iv_fault_t;

typedef enum {
    CAN_IV_POINT_FLAG_INA_VALID = (1U << 0),
    CAN_IV_POINT_FLAG_SOFT_CURRENT_STOP = (1U << 1),
    CAN_IV_POINT_FLAG_SOFT_TEMPERATURE_STOP = (1U << 2)
} can_iv_point_flag_t;

typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[CAN_IV_CLASSIC_DLC];
} can_iv_frame_t;

typedef struct {
    uint16_t start_dac_mV;
    uint16_t stop_dac_mV;
    uint16_t step_dac_mV;
    uint16_t settle_ms;
    uint8_t sample_count;
    uint8_t sample_interval_ms;
    uint16_t soft_current_stop_mA_x10;
    int16_t soft_temperature_stop_C_x10;
} can_iv_sweep_config_t;

typedef struct {
    uint8_t profile;
    uint16_t target_current_mA_x10;
    uint16_t base_dac_mV;
    uint16_t temp_ff_mV_per_C_x100;
} can_iv_pid_config_t;

typedef struct {
    uint16_t index;
    uint16_t dac_mV;
    uint16_t current_adc_mA_x10;
    int16_t temperature_C_x10;
    uint16_t ina_bus_mV;
    int16_t ina_current_mA_x10;
    uint16_t flags;
} can_iv_point_t;

uint16_t can_iv_get_u16_le(const uint8_t *bytes);
int16_t can_iv_get_i16_le(const uint8_t *bytes);
void can_iv_put_u16_le(uint8_t *bytes, uint16_t value);
void can_iv_put_i16_le(uint8_t *bytes, int16_t value);

bool can_iv_decode_config_a(const can_iv_frame_t *frame,
                            uint8_t *token,
                            can_iv_sweep_config_t *config);
bool can_iv_decode_config_b(const can_iv_frame_t *frame,
                            uint8_t *token,
                            can_iv_sweep_config_t *config);
bool can_iv_decode_pid_config(const can_iv_frame_t *frame,
                              uint8_t *token,
                              can_iv_pid_config_t *config);

void can_iv_build_ack(can_iv_frame_t *frame,
                      uint8_t opcode,
                      uint8_t token,
                      can_iv_result_t result,
                      can_iv_state_t state,
                      uint8_t fault_flags,
                      uint8_t detail);
void can_iv_build_status(can_iv_frame_t *frame,
                         can_iv_state_t state,
                         uint8_t fault_flags,
                         uint16_t dac_mV,
                         uint16_t current_mA_x10,
                         int16_t temperature_C_x10);
void can_iv_build_ina_sample(can_iv_frame_t *frame,
                             bool valid,
                             uint8_t diagnostic_flags,
                             uint16_t bus_mV,
                             int16_t current_mA_x10,
                             uint16_t power_mW);
void can_iv_build_point_a(can_iv_frame_t *frame, const can_iv_point_t *point);
void can_iv_build_point_b(can_iv_frame_t *frame, const can_iv_point_t *point);
void can_iv_build_event(can_iv_frame_t *frame,
                        can_iv_event_t event,
                        can_iv_state_t state,
                        uint8_t fault_flags,
                        uint16_t point_count,
                        uint16_t elapsed_100ms);

#ifdef __cplusplus
}
#endif

#endif
