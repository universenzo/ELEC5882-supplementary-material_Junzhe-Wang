#ifndef INA226_PORTABLE_H
#define INA226_PORTABLE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INA226_1A_I2C_ADDRESS_7BIT 0x41U
#define INA226_1A_SHUNT_MILLIOHM   80U
#define INA226_1A_CURRENT_LSB_UA   32U
#define INA226_1A_CALIBRATION      2000U
#define INA226_1A_CONFIG_16AVG     0x4527U
#define INA226_MANUFACTURER_ID_TI  0x5449U
#define INA226_DIE_ID_EXPECTED     0x2260U

typedef bool (*ina226_read_register_fn)(void *context,
                                        uint8_t register_address,
                                        uint16_t *value);
typedef bool (*ina226_write_register_fn)(void *context,
                                         uint8_t register_address,
                                         uint16_t value);

typedef struct {
    ina226_read_register_fn read_register;
    ina226_write_register_fn write_register;
    void *io_context;
    bool identity_valid;
    bool configured;
} ina226_device_t;

typedef struct {
    int32_t shunt_uV;
    uint16_t bus_mV;
    int16_t current_mA_x10;
    uint16_t power_mW;
} ina226_sample_t;

void ina226_device_init(ina226_device_t *device,
                        ina226_read_register_fn read_register,
                        ina226_write_register_fn write_register,
                        void *io_context);
bool ina226_probe_and_configure(ina226_device_t *device);
bool ina226_read_sample(ina226_device_t *device, ina226_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif
