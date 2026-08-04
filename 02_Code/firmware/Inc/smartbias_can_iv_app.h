#ifndef SMARTBIAS_CAN_IV_APP_H
#define SMARTBIAS_CAN_IV_APP_H

#include "characterization_engine.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t (*smartbias_time_ms_fn)(void *context);
typedef bool (*smartbias_can_transmit_fn)(void *context,
                                          const can_iv_frame_t *frame);
typedef bool (*smartbias_can_receive_fn)(void *context, can_iv_frame_t *frame);
typedef bool (*smartbias_pid_control_fn)(void *context);
typedef bool (*smartbias_pid_configure_fn)(void *context,
                                           const can_iv_pid_config_t *config);

typedef struct {
    smartbias_time_ms_fn time_ms;
    smartbias_can_transmit_fn can_transmit;
    smartbias_can_receive_fn can_receive;
    can_iv_set_dac_fn set_dac;
    can_iv_read_measurement_fn read_measurement;
    smartbias_pid_control_fn start_pid_ff;
    smartbias_pid_control_fn stop_pid_ff;
    smartbias_pid_configure_fn configure_pid_ff;
    void *io_context;
} smartbias_can_iv_hooks_t;

typedef struct {
    can_iv_engine_t engine;
    smartbias_can_iv_hooks_t hooks;
    can_iv_sweep_config_t staged_config;
    can_iv_pid_config_t pid_config;
    uint8_t config_a_token;
    uint8_t config_b_token;
    bool config_a_received;
    bool config_b_received;
    uint32_t next_status_ms;
    uint32_t next_ina_ms;
    uint32_t next_pid_safety_ms;
} smartbias_can_iv_app_t;

void smartbias_can_iv_app_init(smartbias_can_iv_app_t *app,
                               const smartbias_can_iv_hooks_t *hooks,
                               const can_iv_hard_limits_t *hard_limits,
                               const can_iv_sweep_config_t *default_sweep);
can_iv_result_t smartbias_can_iv_app_user_button(smartbias_can_iv_app_t *app);
void smartbias_can_iv_app_poll(smartbias_can_iv_app_t *app);
void smartbias_can_iv_app_set_reported_dac(smartbias_can_iv_app_t *app,
                                            uint16_t dac_mV);
bool smartbias_can_iv_app_pid_should_run(const smartbias_can_iv_app_t *app);

#ifdef __cplusplus
}
#endif

#endif
