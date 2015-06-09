/*
 * Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */

 /**@file
 *
 * @defgroup XXXX
 * @{
 * @ingroup  YYYY
 *
 * @brief    ZZZZZ.
 */

#ifndef CLIENT_HANDLING_H__
#define CLIENT_HANDLING_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "device_manager.h"
#include "app_pwm.h"

#define MAX_CLIENTS  DEVICE_MANAGER_MAX_CONNECTIONS  /**< Max number of clients. */

//APP_PWM_INSTANCE(PWM1,1); // Create the PWM1 instance using TIMER1
const nrf_drv_timer_t m_pwm_PWM1_timer;
uint32_t m_pwm_PWM1_cb[APP_PWM_CB_SIZE];
const app_pwm_t PWM1;

/**@brief Funtion for initializing the module.
 */
void client_handling_init(void);

/**@brief Funtion for returning the current number of clients.
 *
 * @return  The current number of clients.
 */
uint8_t client_handling_count(void);

/**@brief Function for setting pwm flag after set up is complete
 *
 * @param[in] pwm_id ID of the PWM instance
 *
 */
void pwm_ready_callback(uint32_t pwm_id);

/**@brief Funtion for creating a new client.
 *
 * @param[in] p_handle    Device Manager Handle. For link related events, this parameter
 *                        identifies the peer.
 *
 * @param[in] conn_handle Identifies link for which client is created.
 * @return NRF_SUCCESS on success, any other on failure.
 */
uint32_t client_handling_create(const dm_handle_t * p_handle, uint16_t conn_handle);

/**@brief Funtion for freeing up a client by setting its state to idle.
 *
 * @param[in] p_handle  Device Manager Handle. For link related events, this parameter
 *                      identifies the peer.
 *
 * @return NRF_SUCCESS on success, any other on failure.
 */
uint32_t client_handling_destroy(const dm_handle_t * p_handle);

/**@brief Funtion for handling client events.
 *
 * @param[in] p_ble_evt  Event to be handled.
 */
void client_handling_ble_evt_handler(ble_evt_t * p_ble_evt);


/**@brief Funtion for handling device manager events.
 *
 * @param[in] p_handle       Identifies device with which the event is associated.
 * @param[in] p_event        Event to be handled.
 * @param[in] event_result   Event result indicating whether a procedure was successful or not.
 */
ret_code_t client_handling_dm_event_handler(const dm_handle_t    * p_handle,
                                              const dm_event_t     * p_event,
                                              const ret_code_t     event_result);

#endif // CLIENT_HANDLING_H__

/** @} */
