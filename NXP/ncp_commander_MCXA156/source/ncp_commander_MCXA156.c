/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    ncp_commander_MCXA156.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_clock.h"
#include "fsl_common.h"
#include "fsl_lpuart.h"
#include "sl_ring_buffer.h"

void LPUART2_Init(void);

static sl_ring_buffer uart0_rx_buf = { 0 };
static sl_ring_buffer uart2_rx_buf = { 0 };

/*
 * @brief   Application entry point.
 */
int main(void)
{
  /* Init board hardware. */
  BOARD_InitBootPins();
  BOARD_InitBootClocks();
  BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
  /* Init FSL debug console. */
  BOARD_InitDebugConsole();
#endif

  LPUART2_Init();
  LPUART_EnableInterrupts(LPUART0, kLPUART_RxDataRegFullInterruptEnable);
  EnableIRQ(LPUART0_IRQn);

  while (1) {
    uint8_t byte;
    while (sl_ringbuffer_read(&uart0_rx_buf, &byte)) {
      LPUART_WriteByte(LPUART2, byte);
      while (!(LPUART_GetStatusFlags(LPUART2) & kLPUART_TransmissionCompleteFlag)) ;
    }

    while (sl_ringbuffer_read(&uart2_rx_buf, &byte)) {
      LPUART_WriteByte(LPUART0, byte);
      while (!(LPUART_GetStatusFlags(LPUART0) & kLPUART_TransmissionCompleteFlag)) ;
    }
  }
  return 0;
}

void LPUART2_Init(void)
{
  lpuart_config_t config;

  LPUART_GetDefaultConfig(&config);
  config.baudRate_Bps = 115200;
  config.enableTx = true;
  config.enableRx = true;
  config.txFifoWatermark = 0;
  config.rxFifoWatermark = 0;

  LPUART_Init(LPUART2, &config, BOARD_DEBUG_UART_CLK_FREQ);

  LPUART_EnableInterrupts(LPUART2, kLPUART_RxDataRegFullInterruptEnable);
  EnableIRQ(LPUART2_IRQn);
}

void LPUART2_IRQHandler(void)
{
  if (LPUART_GetStatusFlags(LPUART2) & kLPUART_RxDataRegFullFlag) {
    uint8_t received = LPUART_ReadByte(LPUART2);
    sl_ringbuffer_write(&uart2_rx_buf, received);
  }
}

void LPUART0_IRQHandler(void)
{
  if (LPUART_GetStatusFlags(LPUART0) & kLPUART_RxDataRegFullFlag) {
    uint8_t received = LPUART_ReadByte(LPUART0);
    sl_ringbuffer_write(&uart0_rx_buf, received);
  }
}
