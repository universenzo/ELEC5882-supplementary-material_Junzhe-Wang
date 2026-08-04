#include "smartbias_can_iv_app.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define STATUS_PERIOD_MS 200U
#define INA_PERIOD_MS    1000U
#define MAX_RX_PER_POLL  4U

static bool time_reached(uint32_t now_ms, uint32_t target_ms)
{
    return ((int32_t)(now_ms - target_ms) >= 0);
}

static void transmit(smartbias_can_iv_app_t *app, const can_iv_frame_t *frame)
{
    if (app->hooks.can_transmit != NULL) {
        (void)app->hooks.can_transmit(app->hooks.io_context, frame);
    }
}

static bool refresh_latest_measurement(smartbias_can_iv_app_t *app)
{
    can_iv_measurement_t measurement;
    if ((app->hooks.read_measurement == NULL) ||
        !app->hooks.read_measurement(app->hooks.io_context, &measurement)) {
        app->engine.latest.ina_valid = false;
        return false;
    }
    app->engine.latest = measurement;
    return true;
}

static uint16_t elapsed_100ms(const smartbias_can_iv_app_t *app,
                              uint32_t now_ms)
{
    uint32_t elapsed = (now_ms - app->engine.start_ms) / 100U;
    return (elapsed > UINT16_MAX) ? UINT16_MAX : (uint16_t)elapsed;
}

static void transmit_ack(smartbias_can_iv_app_t *app,
                         uint8_t opcode,
                         uint8_t token,
                         can_iv_result_t result,
                         uint8_t detail)
{
    can_iv_frame_t response;
    can_iv_build_ack(&response,
                     opcode,
                     token,
                     result,
                     app->engine.state,
                     app->engine.fault_flags,
                     detail);
    transmit(app, &response);
}

static void transmit_status(smartbias_can_iv_app_t *app)
{
    can_iv_frame_t response;
    uint16_t selected_current_mA_x10 = 0U;

    (void)refresh_latest_measurement(app);
    (void)can_iv_measurement_selected_current(&app->engine.hard_limits,
                                              &app->engine.latest,
                                              &selected_current_mA_x10);
    can_iv_build_status(&response,
                        app->engine.state,
                        app->engine.fault_flags,
                        app->engine.current_dac_mV,
                        selected_current_mA_x10,
                        app->engine.latest.temperature_C_x10);
    transmit(app, &response);
}

static void transmit_ina(smartbias_can_iv_app_t *app)
{
    can_iv_frame_t response;
    (void)refresh_latest_measurement(app);
    can_iv_build_ina_sample(&response,
                            app->engine.latest.ina_valid,
                            0U,
                            app->engine.latest.ina_bus_mV,
                            app->engine.latest.ina_current_mA_x10,
                            app->engine.latest.ina_power_mW);
    transmit(app, &response);
}

static void transmit_pending_event(smartbias_can_iv_app_t *app,
                                   uint32_t now_ms)
{
    can_iv_event_t event;
    can_iv_frame_t response;

    if (!can_iv_engine_take_event(&app->engine, &event)) {
        return;
    }
    can_iv_build_event(&response,
                       event,
                       app->engine.state,
                       app->engine.fault_flags,
                       app->engine.point_count,
                       elapsed_100ms(app, now_ms));
    transmit(app, &response);
}

static void apply_staged_config_if_complete(smartbias_can_iv_app_t *app,
                                             uint8_t opcode,
                                             uint8_t token)
{
    can_iv_result_t result;

    if (!app->config_a_received || !app->config_b_received) {
        transmit_ack(app, opcode, token, CAN_IV_RESULT_OK, 1U);
        return;
    }
    if (app->config_a_token != app->config_b_token) {
        transmit_ack(app, opcode, token, CAN_IV_RESULT_BAD_CONFIG, 2U);
        return;
    }

    result = can_iv_engine_configure(&app->engine, &app->staged_config);
    transmit_ack(app, opcode, token, result, (result == CAN_IV_RESULT_OK) ? 0U : 3U);
    if (result == CAN_IV_RESULT_OK) {
        app->config_a_received = false;
        app->config_b_received = false;
    }
}

static void handle_config_a(smartbias_can_iv_app_t *app,
                            const can_iv_frame_t *frame)
{
    uint8_t token = 0U;
    if (!can_iv_decode_config_a(frame, &token, &app->staged_config)) {
        transmit_ack(app, 0xA1U, token, CAN_IV_RESULT_BAD_DLC, 0U);
        return;
    }
    app->config_a_token = token;
    app->config_a_received = true;
    apply_staged_config_if_complete(app, 0xA1U, token);
}

static void handle_config_b(smartbias_can_iv_app_t *app,
                            const can_iv_frame_t *frame)
{
    uint8_t token = 0U;
    if (!can_iv_decode_config_b(frame, &token, &app->staged_config)) {
        transmit_ack(app, 0xA2U, token, CAN_IV_RESULT_BAD_DLC, 0U);
        return;
    }
    app->config_b_token = token;
    app->config_b_received = true;
    apply_staged_config_if_complete(app, 0xA2U, token);
}

static void handle_pid_config(smartbias_can_iv_app_t *app,
                              const can_iv_frame_t *frame)
{
    uint8_t token = 0U;
    can_iv_result_t result;

    if (!can_iv_decode_pid_config(frame, &token, &app->pid_config)) {
        transmit_ack(app, 0xA3U, token, CAN_IV_RESULT_BAD_DLC, 0U);
        return;
    }
    if (app->engine.state != CAN_IV_STATE_IDLE) {
        result = CAN_IV_RESULT_BUSY;
    } else if (app->hooks.configure_pid_ff == NULL) {
        result = CAN_IV_RESULT_NOT_IMPLEMENTED;
    } else if (!app->hooks.configure_pid_ff(app->hooks.io_context,
                                             &app->pid_config)) {
        result = CAN_IV_RESULT_BAD_CONFIG;
    } else {
        result = CAN_IV_RESULT_OK;
    }
    transmit_ack(app, 0xA3U, token, result, 0U);
}

static void stop_pid_if_active(smartbias_can_iv_app_t *app)
{
    if ((app->engine.state == CAN_IV_STATE_PID_FF_ACTIVE) &&
        (app->hooks.stop_pid_ff != NULL)) {
        (void)app->hooks.stop_pid_ff(app->hooks.io_context);
    }
}

static void handle_command(smartbias_can_iv_app_t *app,
                           const can_iv_frame_t *frame,
                           uint32_t now_ms)
{
    uint8_t opcode;
    uint8_t token;
    uint16_t index;
    can_iv_result_t result = CAN_IV_RESULT_OK;
    can_iv_frame_t response;
    can_iv_point_t point;

    if (frame->dlc != CAN_IV_CLASSIC_DLC) {
        transmit_ack(app, 0U, 0U, CAN_IV_RESULT_BAD_DLC, frame->dlc);
        return;
    }

    opcode = frame->data[0];
    token = frame->data[1];
    switch ((can_iv_opcode_t)opcode) {
    case CAN_IV_CMD_PING:
        transmit_ack(app, opcode, token, CAN_IV_RESULT_OK,
                     CAN_IV_PROTOCOL_VERSION);
        break;

    case CAN_IV_CMD_GET_STATUS:
        transmit_ack(app, opcode, token, CAN_IV_RESULT_OK, 0U);
        transmit_status(app);
        break;

    case CAN_IV_CMD_START_SWEEP:
        result = can_iv_engine_start(&app->engine, now_ms);
        transmit_ack(app, opcode, token, result, 0U);
        break;

    case CAN_IV_CMD_ABORT:
        stop_pid_if_active(app);
        can_iv_engine_abort(&app->engine);
        transmit_ack(app, opcode, token, CAN_IV_RESULT_OK, 0U);
        break;

    case CAN_IV_CMD_GET_POINT:
        index = can_iv_get_u16_le(&frame->data[2]);
        if (!can_iv_engine_get_point(&app->engine, index, &point)) {
            transmit_ack(app, opcode, token, CAN_IV_RESULT_BAD_INDEX, 0U);
            break;
        }
        transmit_ack(app, opcode, token, CAN_IV_RESULT_OK, 0U);
        can_iv_build_point_a(&response, &point);
        transmit(app, &response);
        can_iv_build_point_b(&response, &point);
        transmit(app, &response);
        break;

    case CAN_IV_CMD_GET_INA_SAMPLE:
        (void)refresh_latest_measurement(app);
        result = app->engine.latest.ina_valid
                     ? CAN_IV_RESULT_OK
                     : CAN_IV_RESULT_SENSOR_INVALID;
        transmit_ack(app, opcode, token, result, 0U);
        transmit_ina(app);
        break;

    case CAN_IV_CMD_CLEAR_DATA:
        result = can_iv_engine_clear_data(&app->engine);
        transmit_ack(app, opcode, token, result, 0U);
        break;

    case CAN_IV_CMD_START_PID_FF:
        if (app->engine.state != CAN_IV_STATE_PHYSICALLY_ARMED) {
            result = CAN_IV_RESULT_NOT_ARMED;
        } else if (app->hooks.start_pid_ff == NULL) {
            result = CAN_IV_RESULT_NOT_IMPLEMENTED;
        } else if (!app->hooks.start_pid_ff(app->hooks.io_context)) {
            result = CAN_IV_RESULT_SENSOR_INVALID;
        } else {
            app->engine.state = CAN_IV_STATE_PID_FF_ACTIVE;
            app->engine.start_ms = now_ms;
            app->engine.pending_event = CAN_IV_EVENT_PID_STARTED;
            app->engine.event_pending = true;
        }
        transmit_ack(app, opcode, token, result, 0U);
        break;

    case CAN_IV_CMD_STOP_OUTPUT:
        stop_pid_if_active(app);
        can_iv_engine_abort(&app->engine);
        transmit_ack(app, opcode, token, CAN_IV_RESULT_OK, 0U);
        break;

    default:
        transmit_ack(app, opcode, token, CAN_IV_RESULT_BAD_COMMAND, 0U);
        break;
    }
}

static void service_receive_queue(smartbias_can_iv_app_t *app, uint32_t now_ms)
{
    can_iv_frame_t frame;
    uint8_t processed = 0U;

    if (app->hooks.can_receive == NULL) {
        return;
    }
    while ((processed < MAX_RX_PER_POLL) &&
           app->hooks.can_receive(app->hooks.io_context, &frame)) {
        ++processed;
        if (frame.id == CAN_IV_ID_COMMAND) {
            handle_command(app, &frame, now_ms);
        } else if (frame.id == CAN_IV_ID_SWEEP_CONFIG_A) {
            handle_config_a(app, &frame);
        } else if (frame.id == CAN_IV_ID_SWEEP_CONFIG_B) {
            handle_config_b(app, &frame);
        } else if (frame.id == CAN_IV_ID_PID_CONFIG) {
            handle_pid_config(app, &frame);
        }
    }
}

static void monitor_pid_safety(smartbias_can_iv_app_t *app, uint32_t now_ms)
{
    can_iv_measurement_t measurement;
    uint16_t selected_current_mA_x10 = 0U;
    uint8_t fault = 0U;

    if ((app->engine.state != CAN_IV_STATE_PID_FF_ACTIVE) ||
        (app->hooks.read_measurement == NULL)) {
        return;
    }
    if (!time_reached(now_ms, app->next_pid_safety_ms)) {
        return;
    }
    app->next_pid_safety_ms = now_ms + 10U;
    if (!app->hooks.read_measurement(app->hooks.io_context, &measurement)) {
        fault = CAN_IV_FAULT_SENSOR_IO;
    } else {
        app->engine.latest = measurement;
        if (!can_iv_measurement_selected_current(&app->engine.hard_limits,
                                                 &measurement,
                                                 &selected_current_mA_x10)) {
            fault |= CAN_IV_FAULT_SENSOR_IO;
        } else if ((app->engine.hard_limits.current_source ==
                    CAN_IV_CURRENT_SOURCE_INA226) &&
                   (((app->engine.hard_limits.ina_bus_min_mV != 0U) &&
                     (measurement.ina_bus_mV <
                      app->engine.hard_limits.ina_bus_min_mV)) ||
                    ((app->engine.hard_limits.ina_bus_max_mV != 0U) &&
                     (measurement.ina_bus_mV >
                      app->engine.hard_limits.ina_bus_max_mV)))) {
            fault |= CAN_IV_FAULT_SENSOR_IO;
        } else if (selected_current_mA_x10 >=
            app->engine.hard_limits.hard_current_trip_mA_x10) {
            fault |= CAN_IV_FAULT_OVERCURRENT;
        }
        if (measurement.temperature_C_x10 >=
            app->engine.hard_limits.hard_temperature_trip_C_x10) {
            fault |= CAN_IV_FAULT_OVERTEMPERATURE;
        }
        if ((measurement.ntc_adc_raw <=
             app->engine.hard_limits.ntc_rail_low_raw) ||
            (measurement.ntc_adc_raw >=
             app->engine.hard_limits.ntc_rail_high_raw)) {
            fault |= CAN_IV_FAULT_NTC_RAIL;
        }
    }

    if (fault != 0U) {
        stop_pid_if_active(app);
        app->engine.fault_flags |= fault;
        app->engine.current_dac_mV = 0U;
        if (app->hooks.set_dac != NULL) {
            (void)app->hooks.set_dac(app->hooks.io_context, 0U);
        }
        app->engine.state = CAN_IV_STATE_FAULT;
        app->engine.pending_event = CAN_IV_EVENT_FAULT;
        app->engine.event_pending = true;
    }
}

void smartbias_can_iv_app_init(smartbias_can_iv_app_t *app,
                               const smartbias_can_iv_hooks_t *hooks,
                               const can_iv_hard_limits_t *hard_limits,
                               const can_iv_sweep_config_t *default_sweep)
{
    uint32_t now_ms;

    if ((app == NULL) || (hooks == NULL) || (hard_limits == NULL)) {
        return;
    }
    (void)memset(app, 0, sizeof(*app));
    app->hooks = *hooks;
    can_iv_engine_init(&app->engine,
                       hard_limits,
                       hooks->set_dac,
                       hooks->read_measurement,
                       hooks->io_context);
    if (default_sweep != NULL) {
        app->staged_config = *default_sweep;
        (void)can_iv_engine_configure(&app->engine, default_sweep);
    }

    now_ms = (hooks->time_ms != NULL) ? hooks->time_ms(hooks->io_context) : 0U;
    app->next_status_ms = now_ms + STATUS_PERIOD_MS;
    app->next_ina_ms = now_ms + INA_PERIOD_MS;
    app->next_pid_safety_ms = now_ms;
}

can_iv_result_t smartbias_can_iv_app_user_button(smartbias_can_iv_app_t *app)
{
    if (app == NULL) {
        return CAN_IV_RESULT_BAD_COMMAND;
    }
    return can_iv_engine_physical_arm(&app->engine);
}

void smartbias_can_iv_app_poll(smartbias_can_iv_app_t *app)
{
    uint32_t now_ms;

    if ((app == NULL) || (app->hooks.time_ms == NULL)) {
        return;
    }
    now_ms = app->hooks.time_ms(app->hooks.io_context);
    service_receive_queue(app, now_ms);
    can_iv_engine_tick(&app->engine, now_ms);
    monitor_pid_safety(app, now_ms);
    transmit_pending_event(app, now_ms);

    if (time_reached(now_ms, app->next_status_ms)) {
        transmit_status(app);
        app->next_status_ms = now_ms + STATUS_PERIOD_MS;
    }
    if (time_reached(now_ms, app->next_ina_ms)) {
        transmit_ina(app);
        app->next_ina_ms = now_ms + INA_PERIOD_MS;
    }
}

void smartbias_can_iv_app_set_reported_dac(smartbias_can_iv_app_t *app,
                                            uint16_t dac_mV)
{
    if (app != NULL) {
        app->engine.current_dac_mV = dac_mV;
    }
}

bool smartbias_can_iv_app_pid_should_run(const smartbias_can_iv_app_t *app)
{
    return (app != NULL) &&
           (app->engine.state == CAN_IV_STATE_PID_FF_ACTIVE);
}
