#include "hwt101.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

/*
 * HWT101 IIC 协议要点：
 * 1. 默认 7bit 从机地址是 0x50。
 * 2. 读取时先写寄存器地址，不发送 STOP，再用 repeated-start 读数据。
 * 3. 每个角度寄存器为 16 位，低字节在前，高字节在后。
 * 4. 角度从 0x3D 开始连续读取 X/Y/Z，换算公式为 raw / 32768 * 180 度。
 */
#define HWT101_I2C_ADDR       (0x50U)
#define HWT101_REG_ANGLE_X    (0x3DU)
#define HWT101_ANGLE_BYTES    (6U)
#define HWT101_I2C_TIMEOUT    (200000U)
#define ANGLE_SCALE_CENTI_DEG (18000L)
#define ANGLE_RAW_FULL_SCALE  (32768L)

typedef enum {
    I2C_STATUS_IDLE = 0,
    I2C_STATUS_TX_STARTED,
    I2C_STATUS_TX_COMPLETE,
    I2C_STATUS_RX_STARTED,
    I2C_STATUS_RX_COMPLETE,
    I2C_STATUS_ERROR,
} I2C_Status;

static volatile I2C_Status gI2cStatus = I2C_STATUS_IDLE;
static volatile uint8_t gI2cRxBuffer[HWT101_ANGLE_BYTES];
static volatile uint32_t gI2cRxLen;
static volatile uint32_t gI2cRxCount;

static bool HWT101_readRegisters(uint8_t regAddr, uint8_t *buffer,
    uint32_t len);
static bool I2C_waitForStatus(I2C_Status expectedStatus);
static int16_t makeInt16LE(uint8_t lowByte, uint8_t highByte);
static int32_t rawAngleToCentiDegree(int16_t raw);

void HWT101_init(void)
{
    gI2cStatus = I2C_STATUS_IDLE;
    gI2cRxLen = 0U;
    gI2cRxCount = 0U;

    NVIC_ClearPendingIRQ(I2C_INST_INT_IRQN);
    NVIC_EnableIRQ(I2C_INST_INT_IRQN);
}

bool HWT101_readAngles(HWT101_Angle *angle)
{
    uint8_t buffer[HWT101_ANGLE_BYTES];

    if (angle == NULL) {
        return false;
    }

    if (!HWT101_readRegisters(HWT101_REG_ANGLE_X, buffer,
            HWT101_ANGLE_BYTES)) {
        return false;
    }

    angle->rawRoll  = makeInt16LE(buffer[0], buffer[1]);
    angle->rawPitch = makeInt16LE(buffer[2], buffer[3]);
    angle->rawYaw   = makeInt16LE(buffer[4], buffer[5]);

    angle->rollCentiDeg  = rawAngleToCentiDegree(angle->rawRoll);
    angle->pitchCentiDeg = rawAngleToCentiDegree(angle->rawPitch);
    angle->yawCentiDeg   = rawAngleToCentiDegree(angle->rawYaw);

    return true;
}

static bool HWT101_readRegisters(uint8_t regAddr, uint8_t *buffer,
    uint32_t len)
{
    uint8_t txRegAddr = regAddr;

    if ((buffer == NULL) || (len == 0U) || (len > HWT101_ANGLE_BYTES)) {
        return false;
    }

    while (!(DL_I2C_getControllerStatus(I2C_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
    }

    gI2cRxLen = len;
    gI2cRxCount = 0U;
    gI2cStatus = I2C_STATUS_TX_STARTED;

    /* 先写待读取寄存器地址，不发 STOP；随后 repeated-start 进入读阶段。 */
    (void) DL_I2C_fillControllerTXFIFO(I2C_INST, &txRegAddr, 1U);
    DL_I2C_disableInterrupt(I2C_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
    DL_I2C_startControllerTransferAdvanced(I2C_INST, HWT101_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U, DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE, DL_I2C_CONTROLLER_ACK_DISABLE);

    if (!I2C_waitForStatus(I2C_STATUS_TX_COMPLETE)) {
        return false;
    }

    gI2cStatus = I2C_STATUS_RX_STARTED;
    DL_I2C_startControllerTransferAdvanced(I2C_INST, HWT101_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, (uint16_t) len,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_ENABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);

    if (!I2C_waitForStatus(I2C_STATUS_RX_COMPLETE)) {
        return false;
    }

    while (DL_I2C_getControllerStatus(I2C_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
    }

    for (uint32_t i = 0U; i < len; i++) {
        buffer[i] = gI2cRxBuffer[i];
    }

    return true;
}

static bool I2C_waitForStatus(I2C_Status expectedStatus)
{
    uint32_t timeout = HWT101_I2C_TIMEOUT;

    while ((gI2cStatus != expectedStatus) && (gI2cStatus != I2C_STATUS_ERROR) &&
           (timeout > 0U)) {
        timeout--;
    }

    return (gI2cStatus == expectedStatus);
}

static int16_t makeInt16LE(uint8_t lowByte, uint8_t highByte)
{
    return (int16_t) ((uint16_t) lowByte | ((uint16_t) highByte << 8));
}

static int32_t rawAngleToCentiDegree(int16_t raw)
{
    return ((int32_t) raw * ANGLE_SCALE_CENTI_DEG) / ANGLE_RAW_FULL_SCALE;
}

void I2C_INST_IRQHandler(void)
{
    switch (DL_I2C_getPendingInterrupt(I2C_INST)) {
        case DL_I2C_IIDX_CONTROLLER_TX_DONE:
            DL_I2C_disableInterrupt(I2C_INST,
                DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
            gI2cStatus = I2C_STATUS_TX_COMPLETE;
            break;

        case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
            while (!DL_I2C_isControllerRXFIFOEmpty(I2C_INST)) {
                if (gI2cRxCount < gI2cRxLen) {
                    gI2cRxBuffer[gI2cRxCount++] =
                        DL_I2C_receiveControllerData(I2C_INST);
                } else {
                    (void) DL_I2C_receiveControllerData(I2C_INST);
                }
            }
            break;

        case DL_I2C_IIDX_CONTROLLER_RX_DONE:
            while (!DL_I2C_isControllerRXFIFOEmpty(I2C_INST)) {
                if (gI2cRxCount < gI2cRxLen) {
                    gI2cRxBuffer[gI2cRxCount++] =
                        DL_I2C_receiveControllerData(I2C_INST);
                } else {
                    (void) DL_I2C_receiveControllerData(I2C_INST);
                }
            }
            gI2cStatus = (gI2cRxCount == gI2cRxLen) ? I2C_STATUS_RX_COMPLETE :
                                                       I2C_STATUS_ERROR;
            break;

        case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
        case DL_I2C_IIDX_CONTROLLER_NACK:
            gI2cStatus = I2C_STATUS_ERROR;
            break;

        default:
            break;
    }
}
