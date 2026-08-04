/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ina226_portable.h"
#include "smartbias_firmware_profile.h"
#include "smartbias_can_iv_app.h"
#include "stm32_can_iv_port.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
    EXPERIMENT_PID_NO_NTC_FF = 0U,
    EXPERIMENT_TEMP_LIMITER = 1U,
    EXPERIMENT_PID_WITH_NTC_FF = 2U,
    EXPERIMENT_OPEN_LOOP = 3U
} experiment_mode_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

static smartbias_can_iv_app_t can_iv_app;
static ina226_device_t ina226;
static ina226_sample_t ina_latest;
static bool ina_i2c_init_ok;
static bool ina_probe_ok;
static bool ina_sample_valid;
static volatile bool user_arm_requested;

/* CAN may select one locked experiment mode; it cannot alter baselines. */
static float runtime_target_mA = SB_TARGET_CURRENT_MA;
static uint16_t runtime_base_dac_mV = SB_COMPILED_BASE_DAC_MV;
static float runtime_ff_mV_per_C = SB_TEMP_FF_MV_PER_C;
static experiment_mode_t runtime_experiment_mode = EXPERIMENT_PID_WITH_NTC_FF;
static bool pid_ff_enabled;
static uint16_t pid_applied_dac_mV;
static float pid_integral;
static float pid_previous_error;
static float pid_temp_reference_C = 25.0f;
static bool pid_previous_valid;
static uint32_t next_pid_control_ms;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define R_SENSE_EXTERNAL_OHM 1.2f
#define CURRENT_LIMIT_MA SB_CURRENT_LIMIT_MA
#define TEMP_LIMIT_C SB_TEMP_LIMIT_C
#define STARTUP_OFF_MS       5000U
#define DAC_VREF_MV          3300U
#define DAC_MAX_CODE         4095U
#define CONTROL_PERIOD_MS    500U
#define TARGET_CURRENT_MA SB_TARGET_CURRENT_MA
#define CALIBRATED_BASE_DAC_MV SB_COMPILED_BASE_DAC_MV
#define DAC_START_MV CALIBRATED_BASE_DAC_MV
#define DAC_MIN_MV SB_DAC_MIN_MV
#define DAC_MAX_MV SB_DAC_MAX_MV
#define LIVE_SWEEP_DAC_MIN_MV SB_DAC_MIN_MV
#define LIVE_SWEEP_DAC_MAX_MV SB_DAC_MAX_MV
#define LIVE_SWEEP_BUS_MIN_MV SB_BUS_MIN_MV
#define LIVE_SWEEP_BUS_MAX_MV SB_BUS_MAX_MV
#define PID_BASE_DAC_MV ((float)CALIBRATED_BASE_DAC_MV)
#define PID_KP_MV_PER_MA SB_PID_KP_MV_PER_MA
#define PID_KI_MV_PER_MA_S SB_PID_KI_MV_PER_MA_S
#define PID_KD_MV_PER_MA_S SB_PID_KD_MV_PER_MA_S
#define PID_INTEGRAL_LIMIT SB_PID_INTEGRAL_LIMIT
#define PID_DAC_SLEW_MV      5U
#define TEMP_REF_C 25.0f
#define TEMP_FF_MV_PER_C SB_TEMP_FF_MV_PER_C
#define TEMP_LIMITER_SET_C SB_TEMP_LIMITER_SET_C
#define TEMP_LIMITER_KP_MV_PER_C 10.0f
#define TEMP_LIMITER_KI_MV_PER_C_S 0.05f
#define TEMP_LIMITER_KD_MV_PER_C_S 0.0f
#define TEMP_LIMITER_INTEGRAL_LIMIT 800.0f
#define FEEDBACK_AVG_SAMPLES 16U
#define NTC_RAW_MIN_VALID 50U
#define NTC_RAW_MAX_VALID 4045U

#define ADC_MAX_COUNTS       4095.0f
#define ADC_VREF_V           3.3f

#define NTC_R25_OHM          10000.0f
#define NTC_T25_K            298.15f
#define NTC_BETA_K           3950.0f
#define R_NTC_TOP_OHM        9880.0f

#define FIRMWARE_VERSION SB_FIRMWARE_VERSION
#define GATE_MODE_TEXT SB_GATE_MODE_TEXT

#define FAULT_NONE           0x00000000UL
#define FAULT_OVERCURRENT    0x00000001UL
#define FAULT_OVERTEMP       0x00000002UL
#define FAULT_NTC_SENSOR     0x00000004UL

_Static_assert(SB_BUS_MIN_MV < SB_BUS_MAX_MV,
               "invalid INA bus window");
_Static_assert(SB_DAC_MIN_MV <= SB_SWEEP_START_MV,
               "sweep start below DAC minimum");
_Static_assert(SB_SWEEP_START_MV <= SB_SWEEP_STOP_MV,
               "sweep direction invalid");
_Static_assert(SB_SWEEP_STOP_MV <= SB_DAC_MAX_MV,
               "sweep stop above DAC maximum");
_Static_assert((((SB_SWEEP_STOP_MV - SB_SWEEP_START_MV) / 5U) + 1U) <=
               CAN_IV_MAX_POINTS,
               "default sweep exceeds point buffer");
_Static_assert(SB_SWEEP_SOFT_CURRENT_MA_X10 <
               SB_HARD_CURRENT_TRIP_MA_X10,
               "soft current stop must be below hard trip");
_Static_assert(SB_SWEEP_SOFT_TEMP_C_X10 < SB_HARD_TEMP_TRIP_C_X10,
               "soft temperature stop must be below hard trip");

int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

uint32_t Read_ADC1(void)
{
    uint32_t value = 0;

    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
    {
        value = HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);

    return value;
}

uint32_t Read_ADC1_Average(uint8_t samples)
{
    uint32_t sum = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        sum += Read_ADC1();
        HAL_Delay(2);
    }

    return sum / samples;
}

void Gate_DAC_Set_mV(uint32_t mv)
{
    if (mv > DAC_VREF_MV)
    {
        mv = DAC_VREF_MV;
    }

    uint32_t dac_code = (mv * DAC_MAX_CODE) / DAC_VREF_MV;
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_code);
}

void Gate_DAC_Off(void)
{
    Gate_DAC_Set_mV(0U);
}

float SenseVoltage_To_Current_mA(uint32_t isense_mv)
{
    return ((float)isense_mv) / R_SENSE_EXTERNAL_OHM;
}

uint32_t ADC_Raw_To_mV(uint32_t raw)
{
    return (raw * DAC_VREF_MV) / 4095U;
}

float Clamp_Float(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

uint32_t Clamp_DAC_mV(float mv)
{
    float clamped_mv = Clamp_Float(mv, (float)DAC_MIN_MV, (float)DAC_MAX_MV);
    return (uint32_t)(clamped_mv + 0.5f);
}

uint32_t Slew_DAC_mV(uint32_t current_mv, uint32_t target_mv)
{
    if (target_mv > current_mv + PID_DAC_SLEW_MV)
    {
        return current_mv + PID_DAC_SLEW_MV;
    }

    if (current_mv > target_mv + PID_DAC_SLEW_MV)
    {
        return current_mv - PID_DAC_SLEW_MV;
    }

    return target_mv;
}

float ADC_To_Voltage(uint32_t raw)
{
    return ADC_VREF_V * ((float)raw / ADC_MAX_COUNTS);
}

float Voltage_To_NTC_Resistance(float vout)
{
    if (vout <= 0.0f)
    {
        return 0.0f;
    }

    if (vout >= ADC_VREF_V)
    {
        return 999999.0f;
    }

    return R_NTC_TOP_OHM * vout / (ADC_VREF_V - vout);
}

float NTC_Resistance_To_Temperature(float r_ntc)
{
    if (r_ntc <= 0.0f)
    {
        return -273.15f;
    }

    float temp_k;

    temp_k = 1.0f / ((1.0f / NTC_T25_K) +
             (1.0f / NTC_BETA_K) * logf(r_ntc / NTC_R25_OHM));

    return temp_k - 273.15f;
}

void Read_ADC1_TwoChannels(uint32_t *ntc_raw, uint32_t *current_raw)
{
    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
    {
        *ntc_raw = HAL_ADC_GetValue(&hadc1);
    }

    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
    {
        *current_raw = HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);
}

void Read_Feedback_Average(uint32_t *ntc_raw_avg, uint32_t *current_raw_avg, uint32_t *isense_mv_avg)
{
    uint32_t ntc_raw = 0;
    uint32_t current_raw = 0;
    uint32_t ntc_sum = 0;
    uint32_t raw_sum = 0;

    for (uint8_t i = 0; i < FEEDBACK_AVG_SAMPLES; i++)
    {
        Read_ADC1_TwoChannels(&ntc_raw, &current_raw);
        ntc_sum += ntc_raw;
        raw_sum += current_raw;
    }

    *ntc_raw_avg = ntc_sum / FEEDBACK_AVG_SAMPLES;
    *current_raw_avg = raw_sum / FEEDBACK_AVG_SAMPLES;
    *isense_mv_avg = ADC_Raw_To_mV(*current_raw_avg);
}

void Print_Status_Row(const char *mode,
                      uint32_t dac_target_mv,
                      float target_mA,
                      float current_mA,
                      float error_mA,
                      uint32_t isense_mv,
                      uint32_t current_raw,
                      uint32_t ntc_raw,
                      uint32_t ntc_mv,
                      float temperature_C,
                      uint32_t fault_flags)
{
    int32_t target_x10 = (int32_t)(target_mA * 10.0f);
    int32_t current_x10 = (int32_t)(current_mA * 10.0f);
    int32_t error_x10 = (int32_t)(error_mA * 10.0f);
    int32_t temp_x10 = (int32_t)(temperature_C * 10.0f);
    uint32_t rs_milliohm = (uint32_t)((R_SENSE_EXTERNAL_OHM * 1000.0f) + 0.5f);
    uint32_t ntc_top_ohm = (uint32_t)(R_NTC_TOP_OHM + 0.5f);

    printf("t_ms=%lu,mode=%s,gate_mode=%s,target_mA_x10=%ld,current_mA_x10=%ld,error_mA_x10=%ld,dac_target_mV=%lu,isense_mV=%lu,current_raw=%lu,ntc_raw=%lu,ntc_mV=%lu,temp_C_x10=%ld,rs_milliohm=%lu,ntc_top_ohm=%lu,fault_flags=%lu\r\n",
           HAL_GetTick(),
           mode,
           GATE_MODE_TEXT,
           target_x10,
           current_x10,
           error_x10,
           dac_target_mv,
           isense_mv,
           current_raw,
           ntc_raw,
           ntc_mv,
           temp_x10,
           rs_milliohm,
           ntc_top_ohm,
           fault_flags);
}

static uint32_t App_TimeMs(void *context)
{
    (void)context;
    return HAL_GetTick();
}

static bool App_SetDacLiveSweep(void *context, uint16_t dac_mV)
{
    (void)context;

    if (dac_mV == 0U) {
        Gate_DAC_Off();
        return true;
    }
    if ((dac_mV < LIVE_SWEEP_DAC_MIN_MV) ||
        (dac_mV > LIVE_SWEEP_DAC_MAX_MV)) {
        Gate_DAC_Off();
        return false;
    }

    Gate_DAC_Set_mV(dac_mV);
    return true;
}

static int16_t Clamp_Float_x10_To_i16(float value)
{
    float scaled = value * 10.0f;

    if (scaled > 32767.0f) {
        return 32767;
    }
    if (scaled < -32768.0f) {
        return -32768;
    }
    return (int16_t)lroundf(scaled);
}

static bool App_ReadMeasurement(void *context, can_iv_measurement_t *measurement)
{
    uint32_t ntc_raw = 0U;
    uint32_t current_raw = 0U;
    uint32_t isense_mV = 0U;
    uint8_t ina_attempt;
    float temperature_C;
    (void)context;

    if (measurement == NULL) {
        return false;
    }

    Read_Feedback_Average(&ntc_raw, &current_raw, &isense_mV);
    temperature_C = NTC_Resistance_To_Temperature(
        Voltage_To_NTC_Resistance(ADC_To_Voltage(ntc_raw)));

    ina_sample_valid = false;
    if (ina_probe_ok) {
        for (ina_attempt = 0U; ina_attempt < 3U; ++ina_attempt) {
            if (ina226_read_sample(&ina226, &ina_latest)) {
                ina_sample_valid = true;
                break;
            }
            HAL_Delay(1U);
        }
    }

    measurement->ntc_adc_raw = (uint16_t)ntc_raw;
    /* PA1 is not in the selected INA226 current path in this build. */
    measurement->current_adc_raw = 0U;
    measurement->temperature_C_x10 =
        Clamp_Float_x10_To_i16(temperature_C);
    measurement->current_adc_mA_x10 = 0U;
    measurement->ina_valid = ina_sample_valid;
    measurement->ina_bus_mV = ina_sample_valid ? ina_latest.bus_mV : 0U;
    measurement->ina_current_mA_x10 =
        ina_sample_valid ? ina_latest.current_mA_x10 : 0;
    measurement->ina_power_mW =
        ina_sample_valid ? ina_latest.power_mW : 0U;
    return true;
}

static const char *Experiment_Mode_Text(experiment_mode_t mode)
{
    switch (mode) {
    case EXPERIMENT_PID_NO_NTC_FF:
        return "PID_NO_NTC_FF";
    case EXPERIMENT_TEMP_LIMITER:
        return "TEMP_LIMITER";
    case EXPERIMENT_PID_WITH_NTC_FF:
        return "PID_WITH_NTC_FF";
    case EXPERIMENT_OPEN_LOOP:
        return "OPEN_LOOP_FIXED_BASE";
    default:
        return "INVALID";
    }
}

#if !SB_CALIBRATION_ONLY
static bool App_ConfigurePidFfWithAllowList(
    void *context, const can_iv_pid_config_t *config)
{
    (void)context;

#if SB_CALIBRATION_ONLY
    (void)config;
    printf("EXPERIMENT_CONFIG_REJECT,reason=CALIBRATION_ONLY\r\n");
    return false;
#else
    uint16_t expected_ff_x100;

    if ((config == NULL) ||
        (config->profile > (uint8_t)EXPERIMENT_OPEN_LOOP)) {
        return false;
    }
    expected_ff_x100 =
        (config->profile == (uint8_t)EXPERIMENT_PID_WITH_NTC_FF) ?
        SB_TEMP_FF_MV_PER_C_X100 : 0U;

    /* Exact values preserve the compiled experiment basis. */
    if ((config->target_current_mA_x10 != SB_TARGET_CURRENT_MA_X10) ||
        (config->base_dac_mV != SB_COMPILED_BASE_DAC_MV) ||
        (config->temp_ff_mV_per_C_x100 != expected_ff_x100)) {
        return false;
    }

    runtime_experiment_mode = (experiment_mode_t)config->profile;
    runtime_target_mA = config->target_current_mA_x10 / 10.0f;
    runtime_base_dac_mV = config->base_dac_mV;
    runtime_ff_mV_per_C = config->temp_ff_mV_per_C_x100 / 100.0f;
    printf("EXPERIMENT_CONFIG_ACCEPTED,mode=%s,profile=%u,target_mA_x10=%u,base_mV=%u,ff_x100=%u\r\n",
           Experiment_Mode_Text(runtime_experiment_mode),
           config->profile,
           config->target_current_mA_x10,
           config->base_dac_mV,
           config->temp_ff_mV_per_C_x100);
    return true;
#endif
}

static bool App_StartPidFf(void *context)
{
    (void)context;

#if SB_CALIBRATION_ONLY
    printf("EXPERIMENT_START_REJECT,reason=CALIBRATION_ONLY\r\n");
    Gate_DAC_Off();
    return false;
#else
    can_iv_measurement_t measurement;
    uint16_t selected_current_mA_x10 = 0U;

    if (!App_ReadMeasurement(NULL, &measurement)) {
        printf("PID_START_REJECT,reason=MEASUREMENT_IO\r\n");
        Gate_DAC_Off();
        return false;
    }
    if (!measurement.ina_valid ||
        !can_iv_measurement_selected_current(&can_iv_app.engine.hard_limits,
                                             &measurement,
                                             &selected_current_mA_x10)) {
        printf("PID_START_REJECT,reason=INA_INVALID,probe=%u\r\n",
               ina_probe_ok ? 1U : 0U);
        Gate_DAC_Off();
        return false;
    }
    if ((measurement.ina_bus_mV < LIVE_SWEEP_BUS_MIN_MV) ||
        (measurement.ina_bus_mV > LIVE_SWEEP_BUS_MAX_MV)) {
        printf("PID_START_REJECT,reason=BUS_WINDOW,bus_mV=%u\r\n",
               measurement.ina_bus_mV);
        Gate_DAC_Off();
        return false;
    }
    if (selected_current_mA_x10 >= SB_START_MAX_CURRENT_MA_X10) {
        printf("PID_START_REJECT,reason=PRECURRENT,current_x10=%u\r\n",
               selected_current_mA_x10);
        Gate_DAC_Off();
        return false;
    }
    if (measurement.temperature_C_x10 >= SB_START_MAX_TEMP_C_X10) {
        printf("PID_START_REJECT,reason=OVERTEMP,temp_x10=%d\r\n",
               measurement.temperature_C_x10);
        Gate_DAC_Off();
        return false;
    }
    if ((measurement.ntc_adc_raw <= NTC_RAW_MIN_VALID) ||
        (measurement.ntc_adc_raw >= NTC_RAW_MAX_VALID)) {
        printf("PID_START_REJECT,reason=NTC_RAIL,ntc_raw=%u\r\n",
               measurement.ntc_adc_raw);
        Gate_DAC_Off();
        return false;
    }

    can_iv_app.engine.latest = measurement;
    pid_integral = 0.0f;
    pid_previous_error = 0.0f;
    pid_previous_valid = false;
    pid_temp_reference_C = measurement.temperature_C_x10 / 10.0f;
    pid_applied_dac_mV = runtime_base_dac_mV;
    next_pid_control_ms = HAL_GetTick() + CONTROL_PERIOD_MS;
    if (!App_SetDacLiveSweep(NULL, pid_applied_dac_mV)) {
        printf("PID_START_REJECT,reason=DAC_RANGE,base_mV=%u\r\n",
               pid_applied_dac_mV);
        Gate_DAC_Off();
        return false;
    }
    smartbias_can_iv_app_set_reported_dac(&can_iv_app, pid_applied_dac_mV);
    pid_ff_enabled = true;
    printf("EXPERIMENT_START,mode=%s,profile=%u,target_x10=%u,base_mV=%u,ff_x100=%u,temp_ref_x10=%d\r\n",
           Experiment_Mode_Text(runtime_experiment_mode),
           (unsigned int)runtime_experiment_mode,
           (unsigned int)lroundf(runtime_target_mA * 10.0f),
           runtime_base_dac_mV,
           (unsigned int)lroundf(runtime_ff_mV_per_C * 100.0f),
           measurement.temperature_C_x10);
    return true;
#endif
}
#endif /* !SB_CALIBRATION_ONLY */

static bool App_StopPidFf(void *context)
{
    (void)context;
    pid_ff_enabled = false;
    pid_applied_dac_mV = 0U;
    Gate_DAC_Off();
    smartbias_can_iv_app_set_reported_dac(&can_iv_app, 0U);
    return true;
}

static void App_RunPidFfStep(void)
{
    uint16_t selected_current_mA_x10 = 0U;
    float current_mA;
    float temperature_C;
    float error_mA;
    float error_derivative = 0.0f;
    float temp_delta_C;
    float temp_feedforward_mV;
    float temp_over_set_C;
    float thermal_cut_mV;
    float pid_output_mV;
    uint32_t pid_target_mV;
    bool integral_pushes_high;
    bool integral_pushes_low;
    uint32_t now_ms = HAL_GetTick();

    if (!pid_ff_enabled ||
        !smartbias_can_iv_app_pid_should_run(&can_iv_app) ||
        ((int32_t)(now_ms - next_pid_control_ms) < 0)) {
        return;
    }
    next_pid_control_ms = now_ms + CONTROL_PERIOD_MS;

    if (!can_iv_measurement_selected_current(&can_iv_app.engine.hard_limits,
                                             &can_iv_app.engine.latest,
                                             &selected_current_mA_x10)) {
        return;
    }
    current_mA = selected_current_mA_x10 / 10.0f;
    temperature_C = can_iv_app.engine.latest.temperature_C_x10 / 10.0f;
    error_mA = runtime_target_mA - current_mA;

    if (runtime_experiment_mode == EXPERIMENT_OPEN_LOOP) {
        pid_applied_dac_mV = runtime_base_dac_mV;
        if (!App_SetDacLiveSweep(NULL, pid_applied_dac_mV)) {
            (void)App_StopPidFf(NULL);
            return;
        }
        smartbias_can_iv_app_set_reported_dac(&can_iv_app,
                                               pid_applied_dac_mV);
        printf("EXPERIMENT_CTRL,t_ms=%lu,mode=%s,current_x10=%u,dac_mV=%u,temp_x10=%d\r\n",
               (unsigned long)now_ms,
               Experiment_Mode_Text(runtime_experiment_mode),
               selected_current_mA_x10,
               pid_applied_dac_mV,
               can_iv_app.engine.latest.temperature_C_x10);
        return;
    }

    if (runtime_experiment_mode == EXPERIMENT_TEMP_LIMITER) {
        temp_over_set_C = temperature_C - TEMP_LIMITER_SET_C;
        if (temp_over_set_C < 0.0f) {
            temp_over_set_C = 0.0f;
            pid_integral *= 0.98f;
        }
        pid_integral += temp_over_set_C *
                        (CONTROL_PERIOD_MS / 1000.0f);
        pid_integral = Clamp_Float(pid_integral,
                                   0.0f,
                                   TEMP_LIMITER_INTEGRAL_LIMIT);
        if (pid_previous_valid) {
            error_derivative =
                (temp_over_set_C - pid_previous_error) /
                (CONTROL_PERIOD_MS / 1000.0f);
        }
        pid_previous_error = temp_over_set_C;
        pid_previous_valid = true;
        thermal_cut_mV =
            (TEMP_LIMITER_KP_MV_PER_C * temp_over_set_C) +
            (TEMP_LIMITER_KI_MV_PER_C_S * pid_integral) +
            (TEMP_LIMITER_KD_MV_PER_C_S * error_derivative);
        pid_target_mV = Clamp_DAC_mV(runtime_base_dac_mV - thermal_cut_mV);
        pid_applied_dac_mV = Slew_DAC_mV(pid_applied_dac_mV,
                                         pid_target_mV);
        if (!App_SetDacLiveSweep(NULL, pid_applied_dac_mV)) {
            (void)App_StopPidFf(NULL);
            return;
        }
        smartbias_can_iv_app_set_reported_dac(&can_iv_app,
                                               pid_applied_dac_mV);
        printf("EXPERIMENT_CTRL,t_ms=%lu,mode=%s,current_x10=%u,dac_mV=%u,temp_x10=%d,temp_over_x10=%ld,thermal_cut_x10=%ld\r\n",
               (unsigned long)now_ms,
               Experiment_Mode_Text(runtime_experiment_mode),
               selected_current_mA_x10,
               pid_applied_dac_mV,
               can_iv_app.engine.latest.temperature_C_x10,
               (long)lroundf(temp_over_set_C * 10.0f),
               (long)lroundf(thermal_cut_mV * 10.0f));
        return;
    }

    temp_delta_C = temperature_C - pid_temp_reference_C;
    if (temp_delta_C < 0.0f) {
        temp_delta_C = 0.0f;
    }

    integral_pushes_high =
        (pid_applied_dac_mV >= DAC_MAX_MV) && (error_mA > 0.0f);
    integral_pushes_low =
        (pid_applied_dac_mV <= DAC_MIN_MV) && (error_mA < 0.0f);
    if (!integral_pushes_high && !integral_pushes_low) {
        pid_integral += error_mA * (CONTROL_PERIOD_MS / 1000.0f);
        pid_integral = Clamp_Float(pid_integral,
                                   -PID_INTEGRAL_LIMIT,
                                   PID_INTEGRAL_LIMIT);
    }
    if (pid_previous_valid) {
        error_derivative =
            (error_mA - pid_previous_error) /
            (CONTROL_PERIOD_MS / 1000.0f);
    }
    pid_previous_error = error_mA;
    pid_previous_valid = true;

    pid_output_mV = runtime_base_dac_mV +
                    (PID_KP_MV_PER_MA * error_mA) +
                    (PID_KI_MV_PER_MA_S * pid_integral) +
                    (PID_KD_MV_PER_MA_S * error_derivative);
    temp_feedforward_mV =
        (runtime_experiment_mode == EXPERIMENT_PID_WITH_NTC_FF) ?
        (runtime_ff_mV_per_C * temp_delta_C) : 0.0f;
    pid_target_mV = Clamp_DAC_mV(pid_output_mV - temp_feedforward_mV);
    pid_applied_dac_mV = Slew_DAC_mV(pid_applied_dac_mV, pid_target_mV);
    if (!App_SetDacLiveSweep(NULL, pid_applied_dac_mV)) {
        (void)App_StopPidFf(NULL);
        return;
    }
    smartbias_can_iv_app_set_reported_dac(&can_iv_app, pid_applied_dac_mV);
    printf("EXPERIMENT_CTRL,t_ms=%lu,mode=%s,target_x10=%u,current_x10=%u,error_x10=%ld,dac_mV=%u,temp_x10=%d,temp_ref_x10=%ld,ff_mV_x10=%ld\r\n",
           (unsigned long)now_ms,
           Experiment_Mode_Text(runtime_experiment_mode),
           (unsigned int)lroundf(runtime_target_mA * 10.0f),
           selected_current_mA_x10,
           (long)lroundf(error_mA * 10.0f),
           pid_applied_dac_mV,
           can_iv_app.engine.latest.temperature_C_x10,
           (long)lroundf(pid_temp_reference_C * 10.0f),
           (long)lroundf(temp_feedforward_mV * 10.0f));
}

static const can_iv_hard_limits_t hard_limits_active = {
    .current_source = CAN_IV_CURRENT_SOURCE_INA226,
    .ina_bus_min_mV = LIVE_SWEEP_BUS_MIN_MV,
    .ina_bus_max_mV = LIVE_SWEEP_BUS_MAX_MV,
    .dac_min_mV = LIVE_SWEEP_DAC_MIN_MV,
    .dac_max_mV = LIVE_SWEEP_DAC_MAX_MV,
    .hard_current_trip_mA_x10 = SB_HARD_CURRENT_TRIP_MA_X10,
    .hard_temperature_trip_C_x10 = SB_HARD_TEMP_TRIP_C_X10,
    .ntc_rail_low_raw = 50U,
    .ntc_rail_high_raw = 4045U
};

static const can_iv_sweep_config_t initial_sweep_active = {
    .start_dac_mV = SB_SWEEP_START_MV,
    .stop_dac_mV = SB_SWEEP_STOP_MV,
    .step_dac_mV = 5U,
    .settle_ms = 100U,
    .sample_count = 3U,
    .sample_interval_ms = 40U,
    .soft_current_stop_mA_x10 = SB_SWEEP_SOFT_CURRENT_MA_X10,
    .soft_temperature_stop_C_x10 = SB_SWEEP_SOFT_TEMP_C_X10
};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  Gate_DAC_Off();

  ina_i2c_init_ok = stm32_ina226_i2c_init();
  ina226_device_init(&ina226,
                     stm32_ina226_read_register,
                     stm32_ina226_write_register,
                     NULL);
  ina_probe_ok = ina_i2c_init_ok && ina226_probe_and_configure(&ina226);

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  smartbias_can_iv_hooks_t hooks = {
      .time_ms = App_TimeMs,
      .can_transmit = stm32_can_iv_port_transmit,
      .can_receive = stm32_can_iv_port_receive,
      .set_dac = App_SetDacLiveSweep,
      .read_measurement = App_ReadMeasurement,
#if SB_CALIBRATION_ONLY
      .start_pid_ff = NULL,
      .stop_pid_ff = NULL,
      .configure_pid_ff = NULL,
#else
      .start_pid_ff = App_StartPidFf,
      .stop_pid_ff = App_StopPidFf,
      .configure_pid_ff = App_ConfigurePidFfWithAllowList,
#endif
      .io_context = NULL
  };

  Gate_DAC_Off();
  if (!stm32_can_iv_port_start())
  {
      printf("CAN_IV_PORT_INIT_FAIL\r\n");
      Gate_DAC_Off();
      Error_Handler();
  }
  smartbias_can_iv_app_init(&can_iv_app,
                            &hooks,
                            &hard_limits_active,
                            &initial_sweep_active);

  printf("\r\n%s,500kbps,STD_ID,PROTOCOL_V%u\r\n",
         FIRMWARE_VERSION,
         CAN_IV_PROTOCOL_VERSION);
#if SB_CALIBRATION_ONLY
  printf("CALIBRATION_ONLY_%s: bounded one-point pulses and automated sweep only.\r\n",
         SB_SUPPLY_TEXT);
  printf("LONG_RUN_DISABLED: CAN experiment configuration/start is not implemented in this build.\r\n");
#else
  printf("FAIR_MATRIX_%s: four locked modes plus CAN telemetry and automated I-V sweep.\r\n",
         SB_SUPPLY_TEXT);
#endif
#if SB_STRESS_BUILD
  printf("STRESS_BUILD: firmware hard trips=%umA/%dC; external PSU limit must remain <=%umA.\r\n",
         SB_HARD_CURRENT_TRIP_MA_X10 / 10U,
         SB_HARD_TEMP_TRIP_C_X10 / 10,
         SB_PSU_LIMIT_MA);
#endif
  printf("RL fitted, INA in path, common ground, PSU physical limit=%umA maximum.\r\n",
         SB_PSU_LIMIT_MA);
  printf("USER button physically arms one sweep or experiment run; CAN cannot bypass arming.\r\n");
  printf("CURRENT_SOURCE=INA226; PA1 current feedback is disabled.\r\n");
  printf("SWEEP_PROFILE: start=%umV,stop=%umV,step=5mV,settle=100ms,samples=3@40ms.\r\n",
         SB_SWEEP_START_MV,
         SB_SWEEP_STOP_MV);
  printf("LIVE_LIMITS: bus=%u-%umV,dac=%u-%umV,soft=%umA/%dC,hard=%umA/%dC.\r\n",
         SB_BUS_MIN_MV,
         SB_BUS_MAX_MV,
         SB_DAC_MIN_MV,
         SB_DAC_MAX_MV,
         SB_SWEEP_SOFT_CURRENT_MA_X10 / 10U,
         SB_SWEEP_SOFT_TEMP_C_X10 / 10,
         SB_HARD_CURRENT_TRIP_MA_X10 / 10U,
         SB_HARD_TEMP_TRIP_C_X10 / 10);
  printf("DAC_CAL_NOTE: measured PA4 was about command-50mV; command scale is retained.\r\n");
  printf("I2C1_HAL: PA15=SCL,PB9=SDA,100kHz,address=0x41,init=%u,lines_high=%u,probe=%u\r\n",
         ina_i2c_init_ok ? 1U : 0U,
         stm32_ina226_i2c_lines_high() ? 1U : 0U,
         ina_probe_ok ? 1U : 0U);
  printf("CAN IDs: command=100,configA=101,configB=102,pid=103,ack=180,status=181,ina=182\r\n");
#if SB_CALIBRATION_ONLY
  printf("CAL_TARGET: target_x10=%u; compiled base is not a final calibrated claim.\r\n",
         SB_TARGET_CURRENT_MA_X10);
  printf("Use start=stop CAN sweeps for independent cold pulses; record B%s before finalising matrix firmware.\r\n",
         (SB_TARGET_CURRENT_MA_X10 == 500U) ? "10" : "15");
#else
  printf("LOCKED_BASIS: target_x10=%u,base=%umV,PID=%ld/%ld/%ld_x1000,FF=%u_x100mV/C.\r\n",
         SB_TARGET_CURRENT_MA_X10,
         SB_COMPILED_BASE_DAC_MV,
         (long)lroundf(SB_PID_KP_MV_PER_MA * 1000.0f),
         (long)lroundf(SB_PID_KI_MV_PER_MA_S * 1000.0f),
         (long)lroundf(SB_PID_KD_MV_PER_MA_S * 1000.0f),
         SB_TEMP_FF_MV_PER_C_X100);
  printf("MODES: profile0=PID_NO_FF,1=" SB_TEMP_LIMITER_MODE_TEXT
         ",2=PID_FF,3=OPEN_LOOP_FIXED_BASE.\r\n");
  printf("CAN allow-list is exact: target_x10=%u,base=%u,ff_x100=%u only for profile2.\r\n",
         SB_TARGET_CURRENT_MA_X10,
         SB_COMPILED_BASE_DAC_MV,
         SB_TEMP_FF_MV_PER_C_X100);
#endif

  uint32_t next_report_ms = HAL_GetTick();
  uint32_t next_ina_probe_ms = HAL_GetTick() + 2000U;

  while (1)
  {
      if (user_arm_requested)
      {
          can_iv_result_t arm_result;
          user_arm_requested = false;
          arm_result = smartbias_can_iv_app_user_button(&can_iv_app);
          printf("PHYSICAL_ARM,result=%u,state=%u\r\n",
                 (unsigned int)arm_result,
                 (unsigned int)can_iv_app.engine.state);
      }
      smartbias_can_iv_app_poll(&can_iv_app);
      App_RunPidFfStep();

      if (!ina_probe_ok &&
          ((int32_t)(HAL_GetTick() - next_ina_probe_ms) >= 0))
      {
          ina_probe_ok = ina_i2c_init_ok &&
                         ina226_probe_and_configure(&ina226);
          next_ina_probe_ms = HAL_GetTick() + 2000U;
      }

      if ((int32_t)(HAL_GetTick() - next_report_ms) >= 0)
      {
          uint16_t selected_current_mA_x10 = 0U;
          bool selected_current_valid =
              can_iv_measurement_selected_current(
                  &can_iv_app.engine.hard_limits,
                  &can_iv_app.engine.latest,
                  &selected_current_mA_x10);

#if SB_CALIBRATION_ONLY
          printf("CAN_IV,rx=%lu,tx=%lu,rx_drop=%lu,tx_drop=%lu,state=%u,dac=%u,mode=CALIBRATION_ONLY,profile=255,current_source=INA226,current_valid=%u,current_mA_x10=%u,temp_C_x10=%d,ina_valid=%u,ina_mV=%u,ina_mA_x10=%d,cal_target_x10=%u\r\n",
                 (unsigned long)stm32_can_iv_port_rx_count(),
                 (unsigned long)stm32_can_iv_port_tx_count(),
                 (unsigned long)stm32_can_iv_port_rx_drop_count(),
                 (unsigned long)stm32_can_iv_port_tx_drop_count(),
                 (unsigned int)can_iv_app.engine.state,
                 can_iv_app.engine.current_dac_mV,
                 selected_current_valid ? 1U : 0U,
                 selected_current_mA_x10,
                 can_iv_app.engine.latest.temperature_C_x10,
                 can_iv_app.engine.latest.ina_valid ? 1U : 0U,
                 can_iv_app.engine.latest.ina_bus_mV,
                 can_iv_app.engine.latest.ina_current_mA_x10,
                 SB_TARGET_CURRENT_MA_X10);
#else
          printf("CAN_IV,rx=%lu,tx=%lu,rx_drop=%lu,tx_drop=%lu,state=%u,dac=%u,mode=%s,profile=%u,current_source=INA226,current_valid=%u,current_mA_x10=%u,temp_C_x10=%d,ina_valid=%u,ina_mV=%u,ina_mA_x10=%d,target_x10=%u,base=%u,ff_x100=%u\r\n",
                 (unsigned long)stm32_can_iv_port_rx_count(),
                 (unsigned long)stm32_can_iv_port_tx_count(),
                 (unsigned long)stm32_can_iv_port_rx_drop_count(),
                 (unsigned long)stm32_can_iv_port_tx_drop_count(),
                 (unsigned int)can_iv_app.engine.state,
                 can_iv_app.engine.current_dac_mV,
                 Experiment_Mode_Text(runtime_experiment_mode),
                 (unsigned int)runtime_experiment_mode,
                 selected_current_valid ? 1U : 0U,
                 selected_current_mA_x10,
                 can_iv_app.engine.latest.temperature_C_x10,
                 can_iv_app.engine.latest.ina_valid ? 1U : 0U,
                 can_iv_app.engine.latest.ina_bus_mV,
                 can_iv_app.engine.latest.ina_current_mA_x10,
                 (unsigned int)lroundf(runtime_target_mA * 10.0f),
                 runtime_base_dac_mV,
                 (unsigned int)lroundf(runtime_ff_mV_per_C * 100.0f));
#endif
          next_report_ms = HAL_GetTick() + 1000U;
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 20;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void BSP_PB_Callback(Button_TypeDef Button)
{
    if (Button == BUTTON_USER)
    {
        user_arm_requested = true;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
