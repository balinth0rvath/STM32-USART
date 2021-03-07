/*
 * receiver.c
 *
 *  Created on: Mar 3, 2021
 *      Author: balint
 */

#include "transmitter.h"
#include "FreeRTOS.h"
#include "task.h"
#include "nrf24.h"
#include "semphr.h"
#include "stdio.h"

static uint8_t payload[10] = {};
static uint8_t payload_length = 10;

extern SemaphoreHandle_t sem_nRF24;
static char message[64] = "";
extern UART_HandleTypeDef huart2;

void receiver_task()
{
  vTaskDelay(100);
  sem_nRF24 = xSemaphoreCreateBinary();

  while(sem_nRF24 == NULL)
  {
    vTaskDelay(100);
  }

  sprintf(message,"Receiver: initializing... \n\r");
  HAL_UART_Transmit(&huart2, (uint8_t*) message, 64, 100);

  nRF24_SetDevice2();

  nRF24_CE_L();

  show_registers();

  nRF24_Init();

  //nRF24_DisableAA(0xFF);

  nRF24_SetRFChannel(42);
  nRF24_SetDataRate(nRF24_DR_1Mbps);
  nRF24_SetCRCScheme(nRF24_CRC_2byte);
  nRF24_SetAddrWidth(3);
  static const uint8_t addr[] = {0x01, 0x10, 0xE3};
  nRF24_SetAddr(nRF24_PIPE1, addr);
  nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_ON, 10);
  nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 10);
  nRF24_SetTXPower(nRF24_TXPWR_0dBm);
  nRF24_SetOperationalMode(nRF24_MODE_RX);
  nRF24_ClearIRQFlags();
  nRF24_SetPowerMode(nRF24_PWR_UP);
  nRF24_CE_H();

  sprintf(message,"Receiver: initialized \n\r");
  HAL_UART_Transmit(&huart2, (uint8_t*) message, 64, 100);

  xSemaphoreGive(sem_nRF24);

  vTaskDelay(100);

  nRF24_RXResult pipe;

  while(1)
  {
    xSemaphoreTake(sem_nRF24, portMAX_DELAY);

    nRF24_SetDevice2();

    if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY)
    {
      pipe = nRF24_ReadPayload(payload, &payload_length);
      nRF24_ClearIRQFlags();
      sprintf(message,"Receiver: packet arrived... \n\r");
      HAL_UART_Transmit(&huart2, (uint8_t*) message, 64, 100);
    }

    xSemaphoreGive(sem_nRF24);

    vTaskDelay(5000);
  }
}
