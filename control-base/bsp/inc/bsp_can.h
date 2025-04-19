#ifndef CAN_H
#define CAN_H

#include "FreeRTOS.h"

#ifdef STM32H723xx
#define FDCAN_IN_USE
#include "fdcan.h"
#else
#define CAN_IN_USE
#include "can.h"
#endif
#include "queue.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CAN_MAX_DEVICE (8)

typedef struct _
{
    uint8_t can_bus;
#ifdef FDCAN_IN_USE
    FDCAN_TxHeaderTypeDef *tx_header;
#else
    CAN_TxHeaderTypeDef *tx_header;
#endif
    uint16_t rx_id;
    uint8_t tx_buffer[8];
    uint8_t rx_buffer[8];
    void (*can_module_callback)(struct _ *);
    void *binding_motor_stats;      /* void pointer to the motor stats
                                    * this is used to bind the motor stats to the can instance
                                    * so that the callback function can decode the can message
                                    */
} CAN_Instance_t;

CAN_Instance_t *CAN_Device_Register(uint8_t can_bus, uint16_t tx_id, uint16_t rx_id, void (*can_module_callback)(CAN_Instance_t *can_instance));
CAN_Instance_t *CAN_Device_Register_Tx_Only(uint8_t can_bus, uint16_t tx_id);
void CAN_Service_Init(void);
HAL_StatusTypeDef CAN_Transmit(CAN_Instance_t *can_instance);
uint32_t CAN_Get_Tx_ID(CAN_Instance_t *can_instance);
#endif