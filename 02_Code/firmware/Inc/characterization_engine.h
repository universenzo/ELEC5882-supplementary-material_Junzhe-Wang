#ifndef CHARACTERIZATION_ENGINE_H
#define CHARACTERIZATION_ENGINE_H

#include "can_iv_protocol.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_IV_MAX_POINTS 64U

typedef enum {
    CAN_IV_CURRENT_SOURCE_ADC = 0,
    CAN_IV_CURRENT_SOURCE_INA226 = 1
} can_iv_current_source_t;

typedef struct {
    uint16_t current_adc_mA_x10;
    int16_t temperature_C_x10;
    uint16_t current_adc_raw;
    uint16_t ntc_adc_raw;
    bool ina_valid;
    uint16_t ina_bus_mV;
    int16_t ina_current_mA_x10;
    uint16_t ina_power_mW;
} can_iv_measurement_t;

typedef struct {
    can_iv_current_source_t current_source;
    uint16_t ina_bus_min_mV;
    uint16_t ina_bus_max_mV;
    uint16_t dac_min_mV;
    uint16_t dac_max_mV;
    uint16_t hard_current_trip_mA_x10;
    int16_t hard_temperature_trip_C_x10;
    uint16_t ntc_rail_low_raw;
    uint16_t ntc_rail_high_raw;
} can_iv_hard_limits_t;

typedef bool (*can_iv_set_dac_fn)(void *context, uint16_t dac_mV);
typedef bool (*can_iv_read_measurement_fn)(void *context,
                                           can_iv_measurement_t *measurement);

typedef struct {
    can_iv_state_t state;
    uint8_t fault_flags;
    can_iv_event_t pending_event;
    bool event_pending;
    bool config_valid;
    can_iv_sweep_config_t config;
    can_iv_hard_limits_t hard_limits;
    can_iv_point_t points[CAN_IV_MAX_POINTS];
    uint16_t point_count;
    uint16_t current_dac_mV;
    uint32_t start_ms;
    uint32_t deadline_ms;
    uint32_t next_sample_ms;
    uint32_t next_safety_check_ms;
    uint8_t samples_accumulated;
    int64_t selected_current_sum;
    int64_t temperature_sum;
    int64_t ina_bus_sum;
    int64_t ina_current_sum;
    uint8_t ina_samples_accumulated;
    can_iv_measurement_t latest;
    can_iv_set_dac_fn set_dac;
    can_iv_read_measurement_fn read_measurement;
    void *io_context;
} can_iv_engine_t;

bool can_iv_measurement_selected_current(
    const can_iv_hard_limits_t *hard_limits,
    const can_iv_measurement_t *measurement,
    uint16_t *current_mA_x10);

void can_iv_engine_init(can_iv_engine_t *engine,
                        const can_iv_hard_limits_t *hard_limits,
                        can_iv_set_dac_fn set_dac,
                        can_iv_read_measurement_fn read_measurement,
                        void *io_context);
can_iv_result_t can_iv_engine_configure(can_iv_engine_t *engine,
                                        const can_iv_sweep_config_t *config);
can_iv_result_t can_iv_engine_physical_arm(can_iv_engine_t *engine);
can_iv_result_t can_iv_engine_start(can_iv_engine_t *engine, uint32_t now_ms);
void can_iv_engine_abort(can_iv_engine_t *engine);
can_iv_result_t can_iv_engine_clear_data(can_iv_engine_t *engine);
void can_iv_engine_tick(can_iv_engine_t *engine, uint32_t now_ms);
bool can_iv_engine_get_point(const can_iv_engine_t *engine,
                             uint16_t index,
                             can_iv_point_t *point);
bool can_iv_engine_take_event(can_iv_engine_t *engine, can_iv_event_t *event);

#ifdef __cplusplus
}
#endif

#endif
