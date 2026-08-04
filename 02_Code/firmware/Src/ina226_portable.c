#include "ina226_portable.h"

#include <stddef.h>

enum {
    INA226_REG_CONFIG = 0x00,
    INA226_REG_SHUNT_VOLTAGE = 0x01,
    INA226_REG_BUS_VOLTAGE = 0x02,
    INA226_REG_POWER = 0x03,
    INA226_REG_CURRENT = 0x04,
    INA226_REG_CALIBRATION = 0x05,
    INA226_REG_MANUFACTURER_ID = 0xFE,
    INA226_REG_DIE_ID = 0xFF
};

static int32_t rounded_divide_signed(int32_t numerator, int32_t denominator)
{
    if (numerator >= 0) {
        return (numerator + (denominator / 2)) / denominator;
    }
    return (numerator - (denominator / 2)) / denominator;
}

void ina226_device_init(ina226_device_t *device,
                        ina226_read_register_fn read_register,
                        ina226_write_register_fn write_register,
                        void *io_context)
{
    if (device == NULL) {
        return;
    }

    device->read_register = read_register;
    device->write_register = write_register;
    device->io_context = io_context;
    device->identity_valid = false;
    device->configured = false;
}

bool ina226_probe_and_configure(ina226_device_t *device)
{
    uint16_t manufacturer_id = 0U;
    uint16_t die_id = 0U;
    uint16_t calibration_readback = 0U;

    if ((device == NULL) || (device->read_register == NULL) ||
        (device->write_register == NULL)) {
        return false;
    }

    if (!device->read_register(device->io_context,
                               INA226_REG_MANUFACTURER_ID,
                               &manufacturer_id) ||
        !device->read_register(device->io_context,
                               INA226_REG_DIE_ID,
                               &die_id)) {
        return false;
    }

    device->identity_valid =
        (manufacturer_id == INA226_MANUFACTURER_ID_TI) &&
        (die_id == INA226_DIE_ID_EXPECTED);
    if (!device->identity_valid) {
        return false;
    }

    if (!device->write_register(device->io_context,
                                INA226_REG_CONFIG,
                                INA226_1A_CONFIG_16AVG) ||
        !device->write_register(device->io_context,
                                INA226_REG_CALIBRATION,
                                INA226_1A_CALIBRATION) ||
        !device->read_register(device->io_context,
                               INA226_REG_CALIBRATION,
                               &calibration_readback)) {
        return false;
    }

    device->configured = (calibration_readback == INA226_1A_CALIBRATION);
    return device->configured;
}

bool ina226_read_sample(ina226_device_t *device, ina226_sample_t *sample)
{
    uint16_t shunt_raw_u16 = 0U;
    uint16_t bus_raw = 0U;
    uint16_t current_raw_u16 = 0U;
    uint16_t power_raw = 0U;
    int32_t shunt_raw;
    int32_t current_raw;

    if ((device == NULL) || (sample == NULL) || !device->configured) {
        return false;
    }

    if (!device->read_register(device->io_context,
                               INA226_REG_SHUNT_VOLTAGE,
                               &shunt_raw_u16) ||
        !device->read_register(device->io_context,
                               INA226_REG_BUS_VOLTAGE,
                               &bus_raw) ||
        !device->read_register(device->io_context,
                               INA226_REG_CURRENT,
                               &current_raw_u16) ||
        !device->read_register(device->io_context,
                               INA226_REG_POWER,
                               &power_raw)) {
        return false;
    }

    shunt_raw = (int32_t)(int16_t)shunt_raw_u16;
    current_raw = (int32_t)(int16_t)current_raw_u16;

    /* INA226 LSBs: shunt 2.5 uV, bus 1.25 mV. */
    sample->shunt_uV = rounded_divide_signed(shunt_raw * 5, 2);
    sample->bus_mV = (uint16_t)(((uint32_t)bus_raw * 5U + 2U) / 4U);

    /* M5Stack 1 A module: current 32 uA/bit, power 0.8 mW/bit. */
    sample->current_mA_x10 =
        (int16_t)rounded_divide_signed(current_raw * 32, 100);
    sample->power_mW = (uint16_t)(((uint32_t)power_raw * 8U + 5U) / 10U);
    return true;
}
