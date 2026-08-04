#include "can_iv_protocol.h"

#include <string.h>

uint16_t can_iv_get_u16_le(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

int16_t can_iv_get_i16_le(const uint8_t *bytes)
{
    return (int16_t)can_iv_get_u16_le(bytes);
}

void can_iv_put_u16_le(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)(value & 0xFFU);
    bytes[1] = (uint8_t)(value >> 8);
}

void can_iv_put_i16_le(uint8_t *bytes, int16_t value)
{
    can_iv_put_u16_le(bytes, (uint16_t)value);
}

static void initialise_frame(can_iv_frame_t *frame, uint16_t identifier)
{
    frame->id = identifier;
    frame->dlc = CAN_IV_CLASSIC_DLC;
    (void)memset(frame->data, 0, sizeof(frame->data));
}

bool can_iv_decode_config_a(const can_iv_frame_t *frame,
                            uint8_t *token,
                            can_iv_sweep_config_t *config)
{
    if ((frame == NULL) || (token == NULL) || (config == NULL) ||
        (frame->id != CAN_IV_ID_SWEEP_CONFIG_A) ||
        (frame->dlc != CAN_IV_CLASSIC_DLC)) {
        return false;
    }

    *token = frame->data[0];
    config->start_dac_mV = can_iv_get_u16_le(&frame->data[2]);
    config->stop_dac_mV = can_iv_get_u16_le(&frame->data[4]);
    config->step_dac_mV = can_iv_get_u16_le(&frame->data[6]);
    return true;
}

bool can_iv_decode_config_b(const can_iv_frame_t *frame,
                            uint8_t *token,
                            can_iv_sweep_config_t *config)
{
    if ((frame == NULL) || (token == NULL) || (config == NULL) ||
        (frame->id != CAN_IV_ID_SWEEP_CONFIG_B) ||
        (frame->dlc != CAN_IV_CLASSIC_DLC)) {
        return false;
    }

    *token = frame->data[0];
    config->sample_count = frame->data[1];
    config->settle_ms = can_iv_get_u16_le(&frame->data[2]);
    config->sample_interval_ms = frame->data[4];
    config->soft_temperature_stop_C_x10 = (int16_t)((uint16_t)frame->data[5] * 10U);
    config->soft_current_stop_mA_x10 = can_iv_get_u16_le(&frame->data[6]);
    return true;
}

bool can_iv_decode_pid_config(const can_iv_frame_t *frame,
                              uint8_t *token,
                              can_iv_pid_config_t *config)
{
    if ((frame == NULL) || (token == NULL) || (config == NULL) ||
        (frame->id != CAN_IV_ID_PID_CONFIG) ||
        (frame->dlc != CAN_IV_CLASSIC_DLC)) {
        return false;
    }

    *token = frame->data[0];
    config->profile = frame->data[1];
    config->target_current_mA_x10 = can_iv_get_u16_le(&frame->data[2]);
    config->base_dac_mV = can_iv_get_u16_le(&frame->data[4]);
    config->temp_ff_mV_per_C_x100 = can_iv_get_u16_le(&frame->data[6]);
    return true;
}

void can_iv_build_ack(can_iv_frame_t *frame,
                      uint8_t opcode,
                      uint8_t token,
                      can_iv_result_t result,
                      can_iv_state_t state,
                      uint8_t fault_flags,
                      uint8_t detail)
{
    initialise_frame(frame, CAN_IV_ID_ACK);
    frame->data[0] = opcode;
    frame->data[1] = token;
    frame->data[2] = (uint8_t)result;
    frame->data[3] = (uint8_t)state;
    frame->data[4] = fault_flags;
    frame->data[5] = detail;
    frame->data[6] = CAN_IV_PROTOCOL_VERSION;
}

void can_iv_build_status(can_iv_frame_t *frame,
                         can_iv_state_t state,
                         uint8_t fault_flags,
                         uint16_t dac_mV,
                         uint16_t current_mA_x10,
                         int16_t temperature_C_x10)
{
    initialise_frame(frame, CAN_IV_ID_STATUS);
    frame->data[0] = (uint8_t)state;
    frame->data[1] = fault_flags;
    can_iv_put_u16_le(&frame->data[2], dac_mV);
    can_iv_put_u16_le(&frame->data[4], current_mA_x10);
    can_iv_put_i16_le(&frame->data[6], temperature_C_x10);
}

void can_iv_build_ina_sample(can_iv_frame_t *frame,
                             bool valid,
                             uint8_t diagnostic_flags,
                             uint16_t bus_mV,
                             int16_t current_mA_x10,
                             uint16_t power_mW)
{
    initialise_frame(frame, CAN_IV_ID_INA_SAMPLE);
    frame->data[0] = valid ? 1U : 0U;
    frame->data[1] = diagnostic_flags;
    can_iv_put_u16_le(&frame->data[2], bus_mV);
    can_iv_put_i16_le(&frame->data[4], current_mA_x10);
    can_iv_put_u16_le(&frame->data[6], power_mW);
}

void can_iv_build_point_a(can_iv_frame_t *frame, const can_iv_point_t *point)
{
    initialise_frame(frame, CAN_IV_ID_POINT_A);
    can_iv_put_u16_le(&frame->data[0], point->index);
    can_iv_put_u16_le(&frame->data[2], point->dac_mV);
    can_iv_put_u16_le(&frame->data[4], point->current_adc_mA_x10);
    can_iv_put_i16_le(&frame->data[6], point->temperature_C_x10);
}

void can_iv_build_point_b(can_iv_frame_t *frame, const can_iv_point_t *point)
{
    initialise_frame(frame, CAN_IV_ID_POINT_B);
    can_iv_put_u16_le(&frame->data[0], point->index);
    can_iv_put_u16_le(&frame->data[2], point->ina_bus_mV);
    can_iv_put_i16_le(&frame->data[4], point->ina_current_mA_x10);
    can_iv_put_u16_le(&frame->data[6], point->flags);
}

void can_iv_build_event(can_iv_frame_t *frame,
                        can_iv_event_t event,
                        can_iv_state_t state,
                        uint8_t fault_flags,
                        uint16_t point_count,
                        uint16_t elapsed_100ms)
{
    initialise_frame(frame, CAN_IV_ID_EVENT);
    frame->data[0] = (uint8_t)event;
    frame->data[1] = (uint8_t)state;
    frame->data[2] = fault_flags;
    can_iv_put_u16_le(&frame->data[3], point_count);
    can_iv_put_u16_le(&frame->data[5], elapsed_100ms);
    frame->data[7] = CAN_IV_PROTOCOL_VERSION;
}
