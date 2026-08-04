#ifndef SMARTBIAS_FIRMWARE_PROFILE_H
#define SMARTBIAS_FIRMWARE_PROFILE_H

/*
 * Select exactly one build profile. Calibration profiles deliberately expose
 * only the physically armed bounded-sweep path; long-run experiment start and
 * CAN PID configuration are disabled until a measured baseline is compiled
 * into a final matrix profile.
 */
#define SMARTBIAS_PROFILE_CAL_10V          1
#define SMARTBIAS_PROFILE_CAL_15V          2
#define SMARTBIAS_PROFILE_MATRIX_15V_2015  3
#define SMARTBIAS_PROFILE_MATRIX_10V_1934  4
#define SMARTBIAS_PROFILE_MATRIX_15V_2040  5
#define SMARTBIAS_PROFILE_MATRIX_15V_STRESS_150MA_100C 6

#ifndef SMARTBIAS_ACTIVE_PROFILE
#define SMARTBIAS_ACTIVE_PROFILE SMARTBIAS_PROFILE_MATRIX_15V_STRESS_150MA_100C
#endif

#if SMARTBIAS_ACTIVE_PROFILE == SMARTBIAS_PROFILE_CAL_10V

#define SB_FIRMWARE_VERSION              "FW_CAN_INA_CAL_10V_011"
#define SB_GATE_MODE_TEXT                "GATE_MODE_CALIBRATION_SWEEP_ONLY"
#define SB_SUPPLY_TEXT                   "10V"
#define SB_CALIBRATION_ONLY              1
#define SB_TARGET_CURRENT_MA             50.0f
#define SB_TARGET_CURRENT_MA_X10         500U
#define SB_COMPILED_BASE_DAC_MV          2000U
#define SB_TEMP_FF_MV_PER_C              3.0f
#define SB_TEMP_FF_MV_PER_C_X100         300U
#define SB_TEMP_LIMITER_SET_C             40.0f
#define SB_TEMP_LIMITER_MODE_TEXT         "TEMP_LIMITER_40C"
#define SB_STRESS_BUILD                    0
#define SB_PID_KP_MV_PER_MA              0.82f
#define SB_PID_KI_MV_PER_MA_S            0.035f
#define SB_PID_KD_MV_PER_MA_S            0.08f
#define SB_PID_INTEGRAL_LIMIT            2800.0f
#define SB_CURRENT_LIMIT_MA              80.0f
#define SB_HARD_CURRENT_TRIP_MA_X10      800U
#define SB_TEMP_LIMIT_C                  70.0f
#define SB_HARD_TEMP_TRIP_C_X10          700
#define SB_START_MAX_CURRENT_MA_X10      800U
#define SB_START_MAX_TEMP_C_X10          700
#define SB_BUS_MIN_MV                    8000U
#define SB_BUS_MAX_MV                    11000U
#define SB_DAC_MIN_MV                    1400U
#define SB_DAC_MAX_MV                    2100U
#define SB_SWEEP_START_MV                1800U
#define SB_SWEEP_STOP_MV                 2050U
#define SB_SWEEP_SOFT_CURRENT_MA_X10     700U
#define SB_SWEEP_SOFT_TEMP_C_X10         450
#define SB_PSU_LIMIT_MA                  90U

#elif SMARTBIAS_ACTIVE_PROFILE == SMARTBIAS_PROFILE_CAL_15V

#define SB_FIRMWARE_VERSION              "FW_CAN_INA_CAL_15V_012"
#define SB_GATE_MODE_TEXT                "GATE_MODE_CALIBRATION_SWEEP_ONLY"
#define SB_SUPPLY_TEXT                   "15V"
#define SB_CALIBRATION_ONLY              1
#define SB_TARGET_CURRENT_MA             100.0f
#define SB_TARGET_CURRENT_MA_X10         1000U
#define SB_COMPILED_BASE_DAC_MV          2015U
#define SB_TEMP_FF_MV_PER_C              4.0f
#define SB_TEMP_FF_MV_PER_C_X100         400U
#define SB_TEMP_LIMITER_SET_C             40.0f
#define SB_TEMP_LIMITER_MODE_TEXT         "TEMP_LIMITER_40C"
#define SB_STRESS_BUILD                    0
#define SB_PID_KP_MV_PER_MA              0.82f
#define SB_PID_KI_MV_PER_MA_S            0.04f
#define SB_PID_KD_MV_PER_MA_S            0.10f
#define SB_PID_INTEGRAL_LIMIT            3000.0f
#define SB_CURRENT_LIMIT_MA              130.0f
#define SB_HARD_CURRENT_TRIP_MA_X10      1300U
#define SB_TEMP_LIMIT_C                  90.0f
#define SB_HARD_TEMP_TRIP_C_X10          900
#define SB_START_MAX_CURRENT_MA_X10      1300U
#define SB_START_MAX_TEMP_C_X10          700
#define SB_BUS_MIN_MV                    13000U
#define SB_BUS_MAX_MV                    17000U
#define SB_DAC_MIN_MV                    1400U
#define SB_DAC_MAX_MV                    2100U
#define SB_SWEEP_START_MV                1800U
#define SB_SWEEP_STOP_MV                 2050U
#define SB_SWEEP_SOFT_CURRENT_MA_X10     1200U
#define SB_SWEEP_SOFT_TEMP_C_X10         450
#define SB_PSU_LIMIT_MA                  140U

#elif SMARTBIAS_ACTIVE_PROFILE == SMARTBIAS_PROFILE_MATRIX_15V_2015

#define SB_FIRMWARE_VERSION              "FW_CAN_INA_15V_FAIR_MATRIX_010"
#define SB_GATE_MODE_TEXT                "GATE_MODE_CAN_SELECTED_15V_MATRIX"
#define SB_SUPPLY_TEXT                   "15V"
#define SB_CALIBRATION_ONLY              0
#define SB_TARGET_CURRENT_MA             100.0f
#define SB_TARGET_CURRENT_MA_X10         1000U
#define SB_COMPILED_BASE_DAC_MV          2015U
#define SB_TEMP_FF_MV_PER_C              4.0f
#define SB_TEMP_FF_MV_PER_C_X100         400U
#define SB_TEMP_LIMITER_SET_C             40.0f
#define SB_TEMP_LIMITER_MODE_TEXT         "TEMP_LIMITER_40C"
#define SB_STRESS_BUILD                    0
#define SB_PID_KP_MV_PER_MA              0.82f
#define SB_PID_KI_MV_PER_MA_S            0.04f
#define SB_PID_KD_MV_PER_MA_S            0.10f
#define SB_PID_INTEGRAL_LIMIT            3000.0f
#define SB_CURRENT_LIMIT_MA              130.0f
#define SB_HARD_CURRENT_TRIP_MA_X10      1300U
#define SB_TEMP_LIMIT_C                  90.0f
#define SB_HARD_TEMP_TRIP_C_X10          900
#define SB_START_MAX_CURRENT_MA_X10      1300U
#define SB_START_MAX_TEMP_C_X10          700
#define SB_BUS_MIN_MV                    13000U
#define SB_BUS_MAX_MV                    17000U
#define SB_DAC_MIN_MV                    1400U
#define SB_DAC_MAX_MV                    2100U
#define SB_SWEEP_START_MV                1800U
#define SB_SWEEP_STOP_MV                 2050U
#define SB_SWEEP_SOFT_CURRENT_MA_X10     1200U
#define SB_SWEEP_SOFT_TEMP_C_X10         450
#define SB_PSU_LIMIT_MA                  140U

#elif SMARTBIAS_ACTIVE_PROFILE == SMARTBIAS_PROFILE_MATRIX_10V_1934

#define SB_FIRMWARE_VERSION              "FW_CAN_INA_10V_FAIR_MATRIX_013"
#define SB_GATE_MODE_TEXT                "GATE_MODE_CAN_SELECTED_10V_MATRIX"
#define SB_SUPPLY_TEXT                   "10V"
#define SB_CALIBRATION_ONLY              0
#define SB_TARGET_CURRENT_MA             50.0f
#define SB_TARGET_CURRENT_MA_X10         500U
#define SB_COMPILED_BASE_DAC_MV          1934U
#define SB_TEMP_FF_MV_PER_C              3.0f
#define SB_TEMP_FF_MV_PER_C_X100         300U
#define SB_TEMP_LIMITER_SET_C             40.0f
#define SB_TEMP_LIMITER_MODE_TEXT         "TEMP_LIMITER_40C"
#define SB_STRESS_BUILD                    0
#define SB_PID_KP_MV_PER_MA              0.82f
#define SB_PID_KI_MV_PER_MA_S            0.035f
#define SB_PID_KD_MV_PER_MA_S            0.08f
#define SB_PID_INTEGRAL_LIMIT            2800.0f
#define SB_CURRENT_LIMIT_MA              80.0f
#define SB_HARD_CURRENT_TRIP_MA_X10      800U
#define SB_TEMP_LIMIT_C                  70.0f
#define SB_HARD_TEMP_TRIP_C_X10          700
#define SB_START_MAX_CURRENT_MA_X10      800U
#define SB_START_MAX_TEMP_C_X10          700
#define SB_BUS_MIN_MV                    8000U
#define SB_BUS_MAX_MV                    11000U
#define SB_DAC_MIN_MV                    1400U
#define SB_DAC_MAX_MV                    2100U
#define SB_SWEEP_START_MV                1800U
#define SB_SWEEP_STOP_MV                 2050U
#define SB_SWEEP_SOFT_CURRENT_MA_X10     700U
#define SB_SWEEP_SOFT_TEMP_C_X10         450
#define SB_PSU_LIMIT_MA                  90U

#elif SMARTBIAS_ACTIVE_PROFILE == SMARTBIAS_PROFILE_MATRIX_15V_2040

#define SB_FIRMWARE_VERSION              "FW_CAN_INA_15V_FAIR_MATRIX_016"
#define SB_GATE_MODE_TEXT                "GATE_MODE_CAN_SELECTED_15V_MATRIX"
#define SB_SUPPLY_TEXT                   "15V"
#define SB_CALIBRATION_ONLY              0
#define SB_TARGET_CURRENT_MA             100.0f
#define SB_TARGET_CURRENT_MA_X10         1000U
#define SB_COMPILED_BASE_DAC_MV          2040U
#define SB_TEMP_FF_MV_PER_C              4.0f
#define SB_TEMP_FF_MV_PER_C_X100         400U
#define SB_TEMP_LIMITER_SET_C             30.0f
#define SB_TEMP_LIMITER_MODE_TEXT         "TEMP_LIMITER_30C"
#define SB_STRESS_BUILD                    0
#define SB_PID_KP_MV_PER_MA              0.82f
#define SB_PID_KI_MV_PER_MA_S            0.04f
#define SB_PID_KD_MV_PER_MA_S            0.10f
#define SB_PID_INTEGRAL_LIMIT            3000.0f
#define SB_CURRENT_LIMIT_MA              130.0f
#define SB_HARD_CURRENT_TRIP_MA_X10      1300U
#define SB_TEMP_LIMIT_C                  90.0f
#define SB_HARD_TEMP_TRIP_C_X10          900
#define SB_START_MAX_CURRENT_MA_X10      1300U
#define SB_START_MAX_TEMP_C_X10          700
#define SB_BUS_MIN_MV                    13000U
#define SB_BUS_MAX_MV                    17000U
#define SB_DAC_MIN_MV                    1400U
#define SB_DAC_MAX_MV                    2100U
#define SB_SWEEP_START_MV                1800U
#define SB_SWEEP_STOP_MV                 2050U
#define SB_SWEEP_SOFT_CURRENT_MA_X10     1200U
#define SB_SWEEP_SOFT_TEMP_C_X10         450
#define SB_PSU_LIMIT_MA                  140U

#elif SMARTBIAS_ACTIVE_PROFILE == SMARTBIAS_PROFILE_MATRIX_15V_STRESS_150MA_100C

#define SB_FIRMWARE_VERSION              "FW_CAN_INA_15V_STRESS_018"
#define SB_GATE_MODE_TEXT                "GATE_MODE_CAN_SELECTED_15V_STRESS"
#define SB_SUPPLY_TEXT                   "15V"
#define SB_CALIBRATION_ONLY              0
#define SB_TARGET_CURRENT_MA             100.0f
#define SB_TARGET_CURRENT_MA_X10         1000U
#define SB_COMPILED_BASE_DAC_MV          2062U
#define SB_TEMP_FF_MV_PER_C              4.0f
#define SB_TEMP_FF_MV_PER_C_X100         400U
#define SB_TEMP_LIMITER_SET_C             40.0f
#define SB_TEMP_LIMITER_MODE_TEXT         "TEMP_LIMITER_40C"
#define SB_STRESS_BUILD                    1
#define SB_PID_KP_MV_PER_MA              1.20f
#define SB_PID_KI_MV_PER_MA_S            0.06f
#define SB_PID_KD_MV_PER_MA_S            0.10f
#define SB_PID_INTEGRAL_LIMIT            5000.0f
#define SB_CURRENT_LIMIT_MA              150.0f
#define SB_HARD_CURRENT_TRIP_MA_X10      1500U
#define SB_TEMP_LIMIT_C                  100.0f
#define SB_HARD_TEMP_TRIP_C_X10          1000
#define SB_START_MAX_CURRENT_MA_X10      1500U
#define SB_START_MAX_TEMP_C_X10          700
#define SB_BUS_MIN_MV                    13000U
#define SB_BUS_MAX_MV                    17000U
#define SB_DAC_MIN_MV                    1400U
#define SB_DAC_MAX_MV                    2100U
#define SB_SWEEP_START_MV                1800U
#define SB_SWEEP_STOP_MV                 2050U
#define SB_SWEEP_SOFT_CURRENT_MA_X10     1200U
#define SB_SWEEP_SOFT_TEMP_C_X10         450
#define SB_PSU_LIMIT_MA                  140U

#else
#error "Unsupported SMARTBIAS_ACTIVE_PROFILE"
#endif

#endif /* SMARTBIAS_FIRMWARE_PROFILE_H */
