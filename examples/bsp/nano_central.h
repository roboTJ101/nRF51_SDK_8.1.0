/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/*
 * Reb Bear Labs BLE Nano chip, configured for central role. For the button demo,
 * this means that the p0_7 digital output has an LED on it
 */
#ifndef REDBEAR_NANO_C_H__
#define REDBEAR_NANO_C_H__

#define LEDS_NUMBER    3

#define LED_OB	   19 // onboard LED
#define LED_PWM    15 // LED soldered to I/O P0_15
#define LED_ADD    7  // LED soldered to I/O P0_7

#define LED_START  LED_ADD
#define BSP_LED_0  LED_ADD
#define BSP_LED_1  LED_PWM
#define BSP_LED_2  LED_OB
#define LED_STOP   LED_OB

#define BUTTONS_LIST {}
#define LEDS_LIST { BSP_LED_0, BSP_LED_1, BSP_LED_2 }

#define BSP_LED_0_MASK    (1<<BSP_LED_0)
#define BSP_LED_1_MASK    (1<<BSP_LED_1)
#define BSP_LED_2_MASK    (1<<BSP_LED_2)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK)
#define LEDS_INV_MASK  LEDS_MASK

// there are no buttons on this board
#define BUTTONS_NUMBER 0
#define BUTTONS_MASK   0x00000000

// UART pins connected to J-Link
#define RX_PIN_NUMBER  11
#define TX_PIN_NUMBER  9
#define CTS_PIN_NUMBER 10
#define RTS_PIN_NUMBER 8
#define HWFC           true

// SPI pins
#define SPIS_MISO_PIN  11    // SPI MISO signal. 
#define SPIS_CSN_PIN   10    // SPI CSN signal. 
#define SPIS_MOSI_PIN  9    // SPI MOSI signal. 
#define SPIS_SCK_PIN   8    // SPI SCK signal. 

#define SPIM0_SCK_PIN       8     /**< SPI clock GPIO pin number. */
#define SPIM0_MOSI_PIN      9     /**< SPI Master Out Slave In GPIO pin number. */
#define SPIM0_MISO_PIN      11     /**< SPI Master In Slave Out GPIO pin number. */
#define SPIM0_SS_PIN        10     /**< SPI Slave Select GPIO pin number. */

#define SPIM1_SCK_PIN       8     /**< SPI clock GPIO pin number. */
#define SPIM1_MOSI_PIN      9     /**< SPI Master Out Slave In GPIO pin number. */
#define SPIM1_MISO_PIN      11     /**< SPI Master In Slave Out GPIO pin number. */
#define SPIM1_SS_PIN        10     /**< SPI Slave Select GPIO pin number. */

#endif /* REDBEAR_NANO_C_H__ */
