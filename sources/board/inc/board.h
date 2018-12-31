/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <rtthread.h>

#include "stm32l1xx.h"
#include "stm32l1xx_hal.h"

#include "utilities.h"
#include "gpio.h"

/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS                                     1
#endif

#ifndef FAIL
#define FAIL                                        0
#endif

#define USED_UART_1         1
//#define USED_UART_2         1
//#define USED_UART_3         1

#define DEBUG_UART_TX             UART1_TX
#define DEBUG_UART_RX             UART1_RX

/*!
 * Board MCU pins definitions
 */
#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1

#define USB_DM                                      PA_11
#define USB_DP                                      PA_12

#define JTAG_TMS                                    PA_13
#define JTAG_TCK                                    PA_14
#define JTAG_TDI                                    PA_15
#define JTAG_TDO                                    PB_3
#define JTAG_NRST                                   PB_4

#define BOOT_1                                      PB_2

//UART_1
#define UART1_TX                                    PA_9
#define UART1_RX                                    PA_10

//UART_2
#define UART2_TX                                    PA_2
#define UART2_RX                                    PA_3

//UART_3
#define UART3_TX                                    PB_10
#define UART3_RX                                    PB_11

#define LED_0                                       PC_6    // Active low
#define BUTTON_0                                    PD_2    // low is down

#define SIM7600_RST_PIN                             PB_9
//#define SIM7600_RI_PIN                              NULL
//#define SIM7600_DTR_PIN                             NULL

/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led0;

/*!
 * PushButton GPIO pin object
 */
extern Gpio_t PushButton0;

/*!
 * MCU objects
 */
//extern Uart_t Uart1;
//extern Uart_t Uart2;
//extern Uart_t Uart3;

/*!
 * Possible power sources
 */
enum BoardPowerSources
{
    USB_POWER = 0,
    BATTERY_POWER,
};

/*!
 * \brief Disable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardDisableIrq( void );

/*!
 * \brief Enable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardEnableIrq( void );

/*!
 * \brief Initializes the target board peripherals.
 */
void BoardInitMcu( void );

/*!
 * \brief Initializes the boards peripherals.
 */
void BoardInitPeriph( void );

/*!
 * \brief De-initializes the target board peripherals to decrease power
 *        consumption.
 */
void BoardDeInitMcu( void );

/*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * \retval seed Generated pseudo random seed
 */
uint32_t BoardGetRandomSeed( void );

/*!
 * \brief Gets the board 64 bits unique ID
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId( uint8_t *id );

/*!
 * \brief Get the board power source
 *
 * \retval value  power source [0: USB_POWER, 1: BATTERY_POWER]
 */
uint8_t GetBoardPowerSource( void );

/*!
 * \brief Generates Lower 32 bits of DEVEUI using 96 bits unique device ID
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetDevEUI( uint8_t *id );

void sim7600ce_hw_init(void);

#endif // __BOARD_H__
