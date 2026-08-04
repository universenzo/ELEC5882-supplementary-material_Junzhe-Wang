/*
 * STM32G431 port for Classic CAN telemetry/configuration and INA226 I2C.
 *
 * CAN uses CubeMX-generated FDCAN1 on PA11/PA12. INA226 transactions use the
 * CubeMX-generated I2C1 HAL handle on PA15/PB9. Keeping all transactions on
 * hi2c1 avoids reinitialising or directly modifying the same peripheral after
 * MX_I2C1_Init().
 */

#include "stm32_can_iv_port.h"

#include "ina226_portable.h"

#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c1;

#define RX_QUEUE_LENGTH       8U
#define I2C_TIMEOUT_MS        20U
#define INA226_ADDRESS_HAL    ((uint16_t)(INA226_1A_I2C_ADDRESS_7BIT << 1))

static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;
static can_iv_frame_t rx_queue[RX_QUEUE_LENGTH];
static volatile uint32_t can_rx_count;
static volatile uint32_t can_tx_count;
static volatile uint32_t can_rx_drop_count;
static volatile uint32_t can_tx_drop_count;

static uint8_t next_queue_index(uint8_t index)
{
    return (uint8_t)((index + 1U) % RX_QUEUE_LENGTH);
}

bool stm32_can_iv_port_start(void)
{
    FDCAN_FilterTypeDef filter = {0};

    rx_head = 0U;
    rx_tail = 0U;
    can_rx_count = 0U;
    can_tx_count = 0U;
    can_rx_drop_count = 0U;
    can_tx_drop_count = 0U;

    /* Accept 0x100..0x103: command, sweep A/B, and PID configuration. */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = CAN_IV_ID_COMMAND;
    filter.FilterID2 = 0x7FCU;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
        return false;
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) {
        return false;
    }
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        return false;
    }
    return HAL_FDCAN_ActivateNotification(&hfdcan1,
                                           FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                           0U) == HAL_OK;
}

bool stm32_can_iv_port_transmit(void *context, const can_iv_frame_t *frame)
{
    FDCAN_TxHeaderTypeDef header = {0};
    (void)context;

    if ((frame == NULL) || (frame->dlc != CAN_IV_CLASSIC_DLC) ||
        (frame->id > 0x7FFU)) {
        ++can_tx_drop_count;
        return false;
    }

    header.Identifier = frame->id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,
                                       &header,
                                       (uint8_t *)frame->data) != HAL_OK) {
        ++can_tx_drop_count;
        return false;
    }
    ++can_tx_count;
    return true;
}

bool stm32_can_iv_port_receive(void *context, can_iv_frame_t *frame)
{
    uint8_t tail;
    (void)context;

    if ((frame == NULL) || (rx_tail == rx_head)) {
        return false;
    }
    tail = rx_tail;
    *frame = rx_queue[tail];
    rx_tail = next_queue_index(tail);
    return true;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t rx_fifo0_interrupts)
{
    if ((hfdcan != &hfdcan1) ||
        ((rx_fifo0_interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U) {
        FDCAN_RxHeaderTypeDef header = {0};
        uint8_t data[CAN_IV_CLASSIC_DLC] = {0};
        uint8_t next_head;

        if (HAL_FDCAN_GetRxMessage(&hfdcan1,
                                   FDCAN_RX_FIFO0,
                                   &header,
                                   data) != HAL_OK) {
            ++can_rx_drop_count;
            return;
        }
        if ((header.IdType != FDCAN_STANDARD_ID) ||
            (header.RxFrameType != FDCAN_DATA_FRAME) ||
            (header.DataLength != FDCAN_DLC_BYTES_8)) {
            ++can_rx_drop_count;
            continue;
        }

        next_head = next_queue_index(rx_head);
        if (next_head == rx_tail) {
            ++can_rx_drop_count;
            continue;
        }
        rx_queue[rx_head].id = (uint16_t)header.Identifier;
        rx_queue[rx_head].dlc = CAN_IV_CLASSIC_DLC;
        (void)memcpy(rx_queue[rx_head].data, data, sizeof(data));
        rx_head = next_head;
        ++can_rx_count;
    }
}

uint32_t stm32_can_iv_port_rx_count(void)
{
    return can_rx_count;
}

uint32_t stm32_can_iv_port_tx_count(void)
{
    return can_tx_count;
}

uint32_t stm32_can_iv_port_rx_drop_count(void)
{
    return can_rx_drop_count;
}

uint32_t stm32_can_iv_port_tx_drop_count(void)
{
    return can_tx_drop_count;
}

bool stm32_ina226_i2c_init(void)
{
    /* MX_I2C1_Init() owns clocks, GPIO and timing. Only validate readiness. */
    return (hi2c1.Instance == I2C1) &&
           (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY);
}

bool stm32_ina226_i2c_lines_high(void)
{
    return ((GPIOA->IDR & GPIO_PIN_15) != 0U) &&
           ((GPIOB->IDR & GPIO_PIN_9) != 0U);
}

bool stm32_ina226_read_register(void *context,
                                uint8_t register_address,
                                uint16_t *value)
{
    uint8_t bytes[2];
    (void)context;

    if (value == NULL) {
        return false;
    }

    if (HAL_I2C_Mem_Read(&hi2c1,
                         INA226_ADDRESS_HAL,
                         register_address,
                         I2C_MEMADD_SIZE_8BIT,
                         bytes,
                         sizeof(bytes),
                         I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    *value = (uint16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
    return true;
}

bool stm32_ina226_write_register(void *context,
                                 uint8_t register_address,
                                 uint16_t value)
{
    uint8_t bytes[2];
    (void)context;

    bytes[0] = (uint8_t)(value >> 8);
    bytes[1] = (uint8_t)(value & 0xFFU);

    return HAL_I2C_Mem_Write(&hi2c1,
                             INA226_ADDRESS_HAL,
                             register_address,
                             I2C_MEMADD_SIZE_8BIT,
                             bytes,
                             sizeof(bytes),
                             I2C_TIMEOUT_MS) == HAL_OK;
}
