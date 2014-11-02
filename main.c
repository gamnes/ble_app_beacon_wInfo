/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the license.txt file.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ble_advdata.h"
#include "boards.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "nrf_delay.h"


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_util.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "ble_debug_assert_handler.h"
#include "nrf_soc.h"
#include "ble_radio_notification.h"

#include "app_gpiote.h"
#include "app_button.h"
#include "app_timer.h"


#define DEVICE_NAME                             "GG_TEMP"
#define APP_ADV_INTERVAL                     0x81A                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to ~1.2 s). */
#define APP_ADV_TIMEOUT_IN_SECONDS           0                                       /**< The advertising timeout in units of seconds. */

#define DEAD_BEEF                            0xDEADBEEF                                /**< Value used as error code on stack dump, can be used to identify stack location on stack */

static volatile bool m_do_update = false;

/*****************************************************************************
* Error Handling Functions
*****************************************************************************/


/**@brief Error handler function, which is called when an error has occurred. 
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name) {
    nrf_gpio_pin_set(LED_0);
    nrf_gpio_pin_set(LED_1);
    ble_debug_assert_handler(error_code, line_num, p_file_name);
    // On assert, the system can only recover with a reset.
    //NVIC_SystemReset();
}

/**@brief Assert macro callback function.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/*****************************************************************************
* Static Event Handling Functions
*****************************************************************************/

/**@brief Application's BLE Stack event handler.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt) {
    uint32_t        err_code      = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id) {
        default:
            break;
    }

    APP_ERROR_CHECK(err_code);
}

/**@brief Dispatches a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt) {
    on_ble_evt(p_ble_evt);
}

/**@brief GAP initialization.
 *
 * @details This function shall be used to setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void) {
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
}

static uint32_t do_temperature_measurement(void) {
    uint32_t retval = 0xFE000000;

    return retval;
}

/* This function measures the battery voltage using the bandgap as a reference.
 * 3.6 V will return 100 %, so depending on battery voltage, it might need scaling. */
static uint32_t do_battery_measurement(void)
{
    uint8_t retval = 0xAA;

    return retval;
}

/**@brief Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_service_data_t service_data[2];
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    uint32_t temperature;
    temperature = do_temperature_measurement();
    service_data[0].service_uuid = BLE_UUID_HEALTH_THERMOMETER_SERVICE;
    service_data[0].data.p_data = (uint8_t *) &temperature;
    service_data[0].data.size = sizeof(temperature);

    uint8_t battery = do_battery_measurement();
    service_data[1].service_uuid = BLE_UUID_BATTERY_SERVICE;
    service_data[1].data.p_data = &battery;
    service_data[1].data.size = sizeof(battery);

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.p_service_data_array = service_data;
    advdata.service_data_count = 2;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
}

void radio_notification_callback(bool is_radio_active)
{
    m_do_update = is_radio_active;
}

/**@brief Initialize Radio Notification event handler.
*/
static void radio_notification_init(void) {
    uint32_t err_code;

    err_code = ble_radio_notification_init(NRF_APP_PRIORITY_HIGH,
            NRF_RADIO_NOTIFICATION_DISTANCE_1740US,
            radio_notification_callback);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt) {
    
}


/**@brief BLE stack initialization.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
    uint32_t err_code;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = 0;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Start advertising.
*/
static void advertising_start(void)
{
    uint32_t err_code;
    ble_gap_adv_params_t                  adv_params;

    // Initialise advertising parameters (used when starting advertising)
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    adv_params.p_peer_addr = NULL;                           // Undirected advertisement
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_set(LED_0);
}



/*****************************************************************************
 * Main Function
 *****************************************************************************/

/**@brief Application main function.
*/
int main(void)
{
    uint32_t err_code;

    nrf_gpio_range_cfg_output(LED_0, LED_1);
    nrf_gpio_pin_set(LED_0);

    ble_stack_init();
    radio_notification_init();

    // Initialize Bluetooth Stack parameters
    gap_params_init();
    advertising_init();

    // Actually start advertising
    advertising_start();

    // Enter main loop
    for (;;)
    {
        if (m_do_update)
        {
            advertising_init();
            m_do_update = false;
        }

        // Switch to a low power state until an event is available for the application
        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
}

/**
 * @}
 */







