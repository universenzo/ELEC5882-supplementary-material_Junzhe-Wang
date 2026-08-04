#include "characterization_engine.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static bool time_reached(uint32_t now_ms, uint32_t target_ms)
{
    return ((int32_t)(now_ms - target_ms) >= 0);
}

static void publish_event(can_iv_engine_t *engine, can_iv_event_t event)
{
    engine->pending_event = event;
    engine->event_pending = true;
}

static void force_dac_off(can_iv_engine_t *engine)
{
    engine->current_dac_mV = 0U;
    if (engine->set_dac != NULL) {
        (void)engine->set_dac(engine->io_context, 0U);
    }
}

static void latch_fault(can_iv_engine_t *engine, uint8_t fault)
{
    engine->fault_flags |= fault;
    force_dac_off(engine);
    engine->state = CAN_IV_STATE_FAULT;
    publish_event(engine, CAN_IV_EVENT_FAULT);
}

bool can_iv_measurement_selected_current(
    const can_iv_hard_limits_t *hard_limits,
    const can_iv_measurement_t *measurement,
    uint16_t *current_mA_x10)
{
    if ((hard_limits == NULL) || (measurement == NULL) ||
        (current_mA_x10 == NULL)) {
        return false;
    }

    if (hard_limits->current_source == CAN_IV_CURRENT_SOURCE_INA226) {
        if (!measurement->ina_valid) {
            return false;
        }
        *current_mA_x10 = (measurement->ina_current_mA_x10 > 0)
                             ? (uint16_t)measurement->ina_current_mA_x10
                             : 0U;
        return true;
    }

    if (hard_limits->current_source == CAN_IV_CURRENT_SOURCE_ADC) {
        *current_mA_x10 = measurement->current_adc_mA_x10;
        return true;
    }

    return false;
}

static bool measurement_hits_hard_limit(const can_iv_engine_t *engine,
                                        const can_iv_measurement_t *measurement,
                                        uint8_t *fault)
{
    uint16_t selected_current_mA_x10 = 0U;

    *fault = 0U;
    if (!can_iv_measurement_selected_current(&engine->hard_limits,
                                             measurement,
                                             &selected_current_mA_x10)) {
        *fault |= CAN_IV_FAULT_SENSOR_IO;
    } else if ((engine->hard_limits.current_source ==
                CAN_IV_CURRENT_SOURCE_INA226) &&
               (((engine->hard_limits.ina_bus_min_mV != 0U) &&
                 (measurement->ina_bus_mV <
                  engine->hard_limits.ina_bus_min_mV)) ||
                ((engine->hard_limits.ina_bus_max_mV != 0U) &&
                 (measurement->ina_bus_mV >
                  engine->hard_limits.ina_bus_max_mV)))) {
        *fault |= CAN_IV_FAULT_SENSOR_IO;
    } else if (selected_current_mA_x10 >=
        engine->hard_limits.hard_current_trip_mA_x10) {
        *fault |= CAN_IV_FAULT_OVERCURRENT;
    }
    if (measurement->temperature_C_x10 >=
        engine->hard_limits.hard_temperature_trip_C_x10) {
        *fault |= CAN_IV_FAULT_OVERTEMPERATURE;
    }
    if ((measurement->ntc_adc_raw <= engine->hard_limits.ntc_rail_low_raw) ||
        (measurement->ntc_adc_raw >= engine->hard_limits.ntc_rail_high_raw)) {
        *fault |= CAN_IV_FAULT_NTC_RAIL;
    }
    return (*fault != 0U);
}

static bool configuration_is_valid(const can_iv_engine_t *engine,
                                   const can_iv_sweep_config_t *config)
{
    uint32_t point_count;

    if ((config->start_dac_mV < engine->hard_limits.dac_min_mV) ||
        (config->stop_dac_mV > engine->hard_limits.dac_max_mV) ||
        (config->start_dac_mV > config->stop_dac_mV) ||
        (config->step_dac_mV == 0U)) {
        return false;
    }

    point_count =
        ((uint32_t)config->stop_dac_mV - config->start_dac_mV) /
        config->step_dac_mV + 1U;
    if ((point_count == 0U) || (point_count > CAN_IV_MAX_POINTS)) {
        return false;
    }

    if ((config->settle_ms < 50U) || (config->settle_ms > 5000U) ||
        (config->sample_count == 0U) || (config->sample_count > 32U) ||
        (config->sample_interval_ms < 10U) ||
        (config->sample_interval_ms > 250U)) {
        return false;
    }

    if ((config->soft_current_stop_mA_x10 == 0U) ||
        (config->soft_current_stop_mA_x10 >=
         engine->hard_limits.hard_current_trip_mA_x10) ||
        (config->soft_temperature_stop_C_x10 <= 0) ||
        (config->soft_temperature_stop_C_x10 >=
         engine->hard_limits.hard_temperature_trip_C_x10)) {
        return false;
    }

    return true;
}

static int16_t clamp_i64_to_i16(int64_t value)
{
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)value;
}

static uint16_t clamp_i64_to_u16(int64_t value)
{
    if (value <= 0) {
        return 0U;
    }
    if (value > UINT16_MAX) {
        return UINT16_MAX;
    }
    return (uint16_t)value;
}

static void reset_accumulators(can_iv_engine_t *engine)
{
    engine->samples_accumulated = 0U;
    engine->ina_samples_accumulated = 0U;
    engine->selected_current_sum = 0;
    engine->temperature_sum = 0;
    engine->ina_bus_sum = 0;
    engine->ina_current_sum = 0;
}

static void finish_sweep(can_iv_engine_t *engine, can_iv_event_t event)
{
    force_dac_off(engine);
    engine->state = CAN_IV_STATE_DATA_READY;
    publish_event(engine, event);
}

static void store_completed_point(can_iv_engine_t *engine, uint32_t now_ms)
{
    can_iv_point_t *point;
    uint16_t selected_current_mA_x10;
    int16_t ina_current;

    if (engine->point_count >= CAN_IV_MAX_POINTS) {
        latch_fault(engine, CAN_IV_FAULT_POINT_BUFFER);
        return;
    }

    point = &engine->points[engine->point_count];
    (void)memset(point, 0, sizeof(*point));
    point->index = engine->point_count;
    point->dac_mV = engine->current_dac_mV;
    selected_current_mA_x10 = clamp_i64_to_u16(
        engine->selected_current_sum / engine->samples_accumulated);
    point->current_adc_mA_x10 =
        (engine->hard_limits.current_source == CAN_IV_CURRENT_SOURCE_ADC)
            ? selected_current_mA_x10
            : 0U;
    point->temperature_C_x10 = clamp_i64_to_i16(
        engine->temperature_sum / engine->samples_accumulated);

    if (engine->ina_samples_accumulated > 0U) {
        point->ina_bus_mV = clamp_i64_to_u16(
            engine->ina_bus_sum / engine->ina_samples_accumulated);
        ina_current = clamp_i64_to_i16(
            engine->ina_current_sum / engine->ina_samples_accumulated);
        point->ina_current_mA_x10 = ina_current;
        point->flags |= CAN_IV_POINT_FLAG_INA_VALID;
    }

    ++engine->point_count;
    if (selected_current_mA_x10 >= engine->config.soft_current_stop_mA_x10) {
        point->flags |= CAN_IV_POINT_FLAG_SOFT_CURRENT_STOP;
        finish_sweep(engine, CAN_IV_EVENT_SOFT_CURRENT_STOP);
        return;
    }
    if (point->temperature_C_x10 >=
        engine->config.soft_temperature_stop_C_x10) {
        point->flags |= CAN_IV_POINT_FLAG_SOFT_TEMPERATURE_STOP;
        finish_sweep(engine, CAN_IV_EVENT_SOFT_TEMPERATURE_STOP);
        return;
    }

    if (((uint32_t)engine->current_dac_mV + engine->config.step_dac_mV) >
        engine->config.stop_dac_mV) {
        finish_sweep(engine, CAN_IV_EVENT_SWEEP_COMPLETE);
        return;
    }

    engine->current_dac_mV =
        (uint16_t)(engine->current_dac_mV + engine->config.step_dac_mV);
    if (!engine->set_dac(engine->io_context, engine->current_dac_mV)) {
        latch_fault(engine, CAN_IV_FAULT_SENSOR_IO);
        return;
    }
    reset_accumulators(engine);
    engine->deadline_ms = now_ms + engine->config.settle_ms;
    engine->state = CAN_IV_STATE_SWEEP_SETTLING;
}

void can_iv_engine_init(can_iv_engine_t *engine,
                        const can_iv_hard_limits_t *hard_limits,
                        can_iv_set_dac_fn set_dac,
                        can_iv_read_measurement_fn read_measurement,
                        void *io_context)
{
    if ((engine == NULL) || (hard_limits == NULL)) {
        return;
    }

    (void)memset(engine, 0, sizeof(*engine));
    engine->hard_limits = *hard_limits;
    engine->set_dac = set_dac;
    engine->read_measurement = read_measurement;
    engine->io_context = io_context;
    engine->state = CAN_IV_STATE_IDLE;
    force_dac_off(engine);
}

can_iv_result_t can_iv_engine_configure(can_iv_engine_t *engine,
                                        const can_iv_sweep_config_t *config)
{
    if ((engine == NULL) || (config == NULL)) {
        return CAN_IV_RESULT_BAD_CONFIG;
    }
    if (engine->state != CAN_IV_STATE_IDLE) {
        return CAN_IV_RESULT_BUSY;
    }
    if (!configuration_is_valid(engine, config)) {
        engine->config_valid = false;
        return CAN_IV_RESULT_BAD_CONFIG;
    }

    engine->config = *config;
    engine->config_valid = true;
    return CAN_IV_RESULT_OK;
}

can_iv_result_t can_iv_engine_physical_arm(can_iv_engine_t *engine)
{
    if (engine == NULL) {
        return CAN_IV_RESULT_BAD_COMMAND;
    }
    if (engine->fault_flags != 0U) {
        return CAN_IV_RESULT_FAULT_LATCHED;
    }
    if (engine->state != CAN_IV_STATE_IDLE) {
        return CAN_IV_RESULT_BUSY;
    }

    force_dac_off(engine);
    engine->state = CAN_IV_STATE_PHYSICALLY_ARMED;
    publish_event(engine, CAN_IV_EVENT_ARMED);
    return CAN_IV_RESULT_OK;
}

can_iv_result_t can_iv_engine_start(can_iv_engine_t *engine, uint32_t now_ms)
{
    uint8_t fault = 0U;

    if (engine == NULL) {
        return CAN_IV_RESULT_BAD_COMMAND;
    }
    if (engine->fault_flags != 0U) {
        return CAN_IV_RESULT_FAULT_LATCHED;
    }
    if (engine->state != CAN_IV_STATE_PHYSICALLY_ARMED) {
        return CAN_IV_RESULT_NOT_ARMED;
    }
    if (!engine->config_valid) {
        return CAN_IV_RESULT_BAD_CONFIG;
    }
    if ((engine->read_measurement == NULL) ||
        !engine->read_measurement(engine->io_context, &engine->latest)) {
        latch_fault(engine, CAN_IV_FAULT_SENSOR_IO);
        return CAN_IV_RESULT_SENSOR_INVALID;
    }
    if (measurement_hits_hard_limit(engine, &engine->latest, &fault)) {
        latch_fault(engine, fault);
        return CAN_IV_RESULT_FAULT_LATCHED;
    }

    engine->point_count = 0U;
    reset_accumulators(engine);
    engine->current_dac_mV = engine->config.start_dac_mV;
    if ((engine->set_dac == NULL) ||
        !engine->set_dac(engine->io_context, engine->current_dac_mV)) {
        latch_fault(engine, CAN_IV_FAULT_SENSOR_IO);
        return CAN_IV_RESULT_SENSOR_INVALID;
    }

    engine->start_ms = now_ms;
    engine->deadline_ms = now_ms + engine->config.settle_ms;
    engine->next_safety_check_ms = now_ms;
    engine->state = CAN_IV_STATE_SWEEP_SETTLING;
    publish_event(engine, CAN_IV_EVENT_SWEEP_STARTED);
    return CAN_IV_RESULT_OK;
}

void can_iv_engine_abort(can_iv_engine_t *engine)
{
    if ((engine == NULL) || (engine->state == CAN_IV_STATE_FAULT)) {
        return;
    }
    force_dac_off(engine);
    engine->state = CAN_IV_STATE_IDLE;
    publish_event(engine, CAN_IV_EVENT_ABORTED);
}

can_iv_result_t can_iv_engine_clear_data(can_iv_engine_t *engine)
{
    if (engine == NULL) {
        return CAN_IV_RESULT_BAD_COMMAND;
    }
    if ((engine->state != CAN_IV_STATE_IDLE) &&
        (engine->state != CAN_IV_STATE_DATA_READY)) {
        return CAN_IV_RESULT_BUSY;
    }
    force_dac_off(engine);
    engine->point_count = 0U;
    engine->state = CAN_IV_STATE_IDLE;
    return CAN_IV_RESULT_OK;
}

void can_iv_engine_tick(can_iv_engine_t *engine, uint32_t now_ms)
{
    uint8_t fault = 0U;
    uint16_t selected_current_mA_x10 = 0U;

    if ((engine == NULL) ||
        ((engine->state != CAN_IV_STATE_SWEEP_SETTLING) &&
         (engine->state != CAN_IV_STATE_SWEEP_SAMPLING))) {
        return;
    }

    if (!time_reached(now_ms, engine->next_safety_check_ms)) {
        return;
    }
    engine->next_safety_check_ms = now_ms + 10U;

    if ((engine->read_measurement == NULL) ||
        !engine->read_measurement(engine->io_context, &engine->latest)) {
        latch_fault(engine, CAN_IV_FAULT_SENSOR_IO);
        return;
    }
    if (measurement_hits_hard_limit(engine, &engine->latest, &fault)) {
        latch_fault(engine, fault);
        return;
    }

    if (engine->state == CAN_IV_STATE_SWEEP_SETTLING) {
        if (!time_reached(now_ms, engine->deadline_ms)) {
            return;
        }
        reset_accumulators(engine);
        engine->next_sample_ms = now_ms;
        engine->state = CAN_IV_STATE_SWEEP_SAMPLING;
    }

    if (!time_reached(now_ms, engine->next_sample_ms)) {
        return;
    }

    if (!can_iv_measurement_selected_current(&engine->hard_limits,
                                             &engine->latest,
                                             &selected_current_mA_x10)) {
        latch_fault(engine, CAN_IV_FAULT_SENSOR_IO);
        return;
    }
    engine->selected_current_sum += selected_current_mA_x10;
    engine->temperature_sum += engine->latest.temperature_C_x10;
    ++engine->samples_accumulated;
    if (engine->latest.ina_valid) {
        engine->ina_bus_sum += engine->latest.ina_bus_mV;
        engine->ina_current_sum += engine->latest.ina_current_mA_x10;
        ++engine->ina_samples_accumulated;
    }

    engine->next_sample_ms = now_ms + engine->config.sample_interval_ms;
    if (engine->samples_accumulated >= engine->config.sample_count) {
        store_completed_point(engine, now_ms);
    }
}

bool can_iv_engine_get_point(const can_iv_engine_t *engine,
                             uint16_t index,
                             can_iv_point_t *point)
{
    if ((engine == NULL) || (point == NULL) ||
        (index >= engine->point_count)) {
        return false;
    }
    *point = engine->points[index];
    return true;
}

bool can_iv_engine_take_event(can_iv_engine_t *engine, can_iv_event_t *event)
{
    if ((engine == NULL) || (event == NULL) || !engine->event_pending) {
        return false;
    }
    *event = engine->pending_event;
    engine->event_pending = false;
    return true;
}
