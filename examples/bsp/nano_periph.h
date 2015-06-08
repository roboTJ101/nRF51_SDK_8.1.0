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
 * Reb Bear Labs BLE Nano chip, configured for peripheral role. For the button demo,
 * this means that the p0_7 digital input has a momentary button on it
 */

#ifndef REDBEAR_NANO_P_H__
#define REDBEAR_NANO_P_H__

#define LEDS_NUMBER    1

#define LED_START  19
#define BSP_LED_0  19
#define LED_STOP   19

#define LEDS_LIST { BSP_LED_0 }
#define BSP_LED_0_MASK    (1<<BSP_LED_0)

// bsp.c assumes BSP_LED_1_MASK always exists
#define BSP_LED_1_MASK    (1<<BSP_LED_0)

#define LEDS_MASK      (BSP_LED_0_MASK)
#define LEDS_INV_MASK  LEDS_MASK

#define BUTTON_0   7 // Button on I/O p0_7
#define BUTTON_PULL NRF_GPIO_PIN_PULLUP

#define BSP_BUTTON_0 BUTTON_0
#define BUTTON_START BUTTON_0
#define BUTTON_STOP  BUTTON_0

#define BUTTONS_LIST { BUTTON_0 }

#define BUTTONS_NUMBER 1
#define BSP_BUTTON_0_MASK (1<<BUTTON_0)
#define BUTTONS_MASK   (BSP_BUTTON_0_MASK)

// UART pins connected to J-Link
#define RX_PIN_NUMBER  11
#define TX_PIN_NUMBER  9
#define CTS_PIN_NUMBER 10
#define RTS_PIN_NUMBER 8
#define HWFC           true

#endif /* REDBEAR_NANO_P_H__ */
