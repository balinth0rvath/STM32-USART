/*
 * transmitter.c
 *
 *  Created on: Mar 3, 2021
 *      Author: balint
 */

#include "transmitter.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "nrf24.h"
#include "stdio.h"

SemaphoreHandle_t sem_nRF24 = NULL;
static char message[64] = "";
extern UART_HandleTypeDef huart2;

typedef enum {
  nRF24_TX_ERROR = 0,
  nRF24_TX_SUCCESS,
  nRF24_TX_TIMEOUT,
  nRF24_TX_MAXRT
} nRF24_TX_Result;

static nRF24_TX_Result nRF24_TransmitPacket(uint8_t* payload, uint8_t payload_length);

void transmitter_task()
{
  vTaskDelay(200);

  if (sem_nRF24 == NULL)
    while (1);

  xSemaphoreTake(sem_nRF24, portMAX_DELAY);

  sprintf(message,"Transmitter: initializing... \n\r");
  HAL_UART_Transmit(&huart2, (uint8_t*) message, 64, 100);

  nRF24_SetDevice1();

  nRF24_CE_L();
  nRF24_Init();

  //nRF24_DisableAA(0xFF);

  nRF24_SetRFChannel(42);
  nRF24_SetDataRate(nRF24_DR_1Mbps);
  nRF24_SetCRCScheme(nRF24_CRC_2byte);
  nRF24_SetAddrWidth(3);
  const uint8_t addr[] = {0x01, 0x10, 0xE3};
  nRF24_SetAddr(nRF24_PIPETX, addr);
  nRF24_SetAddr(nRF24_PIPE0, addr);
  nRF24_SetTXPower(nRF24_TXPWR_0dBm);
  nRF24_SetAutoRetr(nRF24_ARD_2500us, 10);
  nRF24_EnableAA(nRF24_PIPE0);
  nRF24_SetOperationalMode(nRF24_MODE_TX);
  nRF24_ClearIRQFlags();
  nRF24_SetPowerMode(nRF24_PWR_UP);

  sprintf(message,"Transmitter: initialized \n\r");
  HAL_UART_Transmit(&huart2, (uint8_t*) message, 64, 100);

  xSemaphoreGive(sem_nRF24);

  vTaskDelay(100);

  uint8_t payload[10] = {1,2,3,4,5,6,7,8,9,10};
  uint8_t payload_length = 10;
  uint8_t otx;
  uint8_t otx_plot;
  uint8_t otx_arc;
  nRF24_TX_Result ret;

  while(1)
  {
    xSemaphoreTake(sem_nRF24, portMAX_DELAY);

    nRF24_SetDevice1();

    //sprintf(message,"Transmitter: sending packet... \n\r");
    //HAL_UART_Transmit(&huart2, (uint8_t*) message, 64, 100);

    ret = nRF24_TransmitPacket(payload, payload_length);
    otx = nRF24_GetRetransmitCounters();

    otx_plot = (otx & nRF24_MASK_PLOS_CNT) >> 4;
    otx_arc = otx & nRF24_MASK_ARC_CNT;

    //sprintf(message,"Transmitter: packet loss: %d auto retransmit count: %d \n\r", otx_plot, otx_arc);
    //HAL_UART_Transmit(&huart2, (uint8_t*) message, 64, 100);
    memset(message, 0, 64);
    switch(ret)
    {
    case nRF24_TX_ERROR:
      sprintf(message,"Transmitter: transmit error ERROR \n\r");
      HAL_UART_Transmit(&huart2, (uint8_t*) message, 64, 100);
      break;
    case nRF24_TX_SUCCESS:
      sprintf(message,"Transmitter: transmit SUCCESS \n\r");
      HAL_UART_Transmit(&huart2, (uint8_t*) message, 64, 100);
      break;
    case nRF24_TX_TIMEOUT:
      sprintf(message,"Transmitter: transmit error TIMEOUT \n\r");
      HAL_UART_Transmit(&huart2, (uint8_t*) message, 64, 100);
      break;
    case nRF24_TX_MAXRT:
      //sprintf(message,"Transmitter: transmit error MAXRT \n\r");
      //HAL_UART_Transmit(&huart2, (uint8_t*) message, 64, 100);
      break;
    default:
      break;
    }

    xSemaphoreGive(sem_nRF24);

    vTaskDelay(500);
  }
}

nRF24_TX_Result nRF24_TransmitPacket(uint8_t* payload, uint8_t payload_length)
{
  volatile uint32_t wait = 0xfffff;
  uint8_t status;
  nRF24_CE_L();
  nRF24_WritePayload(payload, payload_length);
  nRF24_CE_H();

  do {
    status = nRF24_GetStatus();
    if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
      break;
    }
    vTaskDelay(1);
  } while (wait--);

  nRF24_CE_L();

  if (!wait)
  {
    return nRF24_TX_TIMEOUT;
  }

  nRF24_ClearIRQFlags();

  if (status & nRF24_FLAG_MAX_RT)
  {
    return nRF24_TX_MAXRT;
  }
  if (status & nRF24_FLAG_TX_DS)
  {
    return nRF24_TX_SUCCESS;
  }
  nRF24_FlushTX();
  return nRF24_TX_ERROR;
}
