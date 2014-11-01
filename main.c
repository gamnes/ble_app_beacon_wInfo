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

#include "app_gpiote.h"
#include "app_button.h"
#include "app_timer.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT  0                                 /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define ADVERTISING_LED_PIN_NO           LED_0                             /**< Is on when device is advertising. */

#define APP_CFG_NON_CONN_ADV_TIMEOUT     0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define NON_CONNECTABLE_ADV_INTERVAL     MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH           0x17                              /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH              0x15                              /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                  0x02                              /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI                0xC3                              /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER           0x004C                            /**< Company identifier for Apple Inc. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                  0x01, 0x02                        /**< Major value used to identify Beacons. */ 
#define APP_MINOR_VALUE                  0x03, 0x04                        /**< Minor value used to identify Beacons. */ 
#define APP_BEACON_UUID                  0x01, 0x12, 0x23, 0x34, \
                                         0x45, 0x56, 0x67, 0x78, \
                                         0x89, 0x9a, 0xab, 0xbc, \
                                         0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */

#define DEAD_BEEF                        0xDEADBEEF                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define HR_INC_BUTTON_PIN_NO                 BUTTON_0                                   /**< Button used to increment heart rate. */
#define HR_DEC_BUTTON_PIN_NO                 BUTTON_1                                   /**< Button used to decrement heart rate. */

#define TIMER_PRESCALERS  6U       /**< Prescaler setting for timer. */

#define GG_USE_HR_IN_ADV                     1

#define APP_TIMER_PRESCALER                  0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS                 5                                          /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE              5                                          /**< Size of timer operation queues. */

#define BUTTON_DETECTION_DELAY               APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)   /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define MIN_HEART_RATE                       60                                         /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                       300                                        /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_CHANGE                    2                                          /**< Value by which the heart rate is incremented/decremented during button press. */

#define APP_GPIOTE_MAX_USERS                 1                                          /**< Maximum number of users of the GPIOTE handler. */

#define BUTTON_DETECTION_DELAY               APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)   /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define MAJ_VAL_OFFSET_IN_BEACON_INFO    18                                /**< Position of the MSB of the Major Value in m_beacon_info array. */

#define LFCLK_FREQUENCY           (32768UL)                               /**< LFCLK frequency in Hertz, constant. */
#define RTC_FREQUENCY             (8UL)                                   /**< Required RTC working clock RTC_FREQUENCY Hertz. Changable. */
#define COMPARE_COUNTERTIME       (3UL)                                   /**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. */
#define COUNTER_PRESCALER         ((LFCLK_FREQUENCY / RTC_FREQUENCY) - 1)   /* f = LFCLK/(prescaler + 1) */

#define GPIO_TOGGLE_TICK_EVENT    (LED_0)                                 /**< Pin number to toggle when there is a tick event in RTC. */
#define GPIO_TOGGLE_COMPARE_EVENT (LED_1)                                 /**< Pin number to toggle when there is compare event in RTC. */

static volatile uint16_t startAdvertising = 0;
static volatile uint16_t stopAdvertising = 1;

static volatile uint16_t                     m_cur_heart_rate;                          /**< Current heart rate value. */
static volatile uint16_t major_value = (0x12345678 & 0xFFFF0000) >> 16;
static volatile uint16_t minor_value = (0x12345678 & 0x0000FFFF);

static volatile uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;


#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define UICR_ADDRESS                     0x10001080                        /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                     /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this 
                         // implementation. 
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the 
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value. 
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons. 
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons. 
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in 
                         // this implementation. 
};


/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover on reset.
    //NVIC_SystemReset();
}


/**@brief Callback function for asserts in the SoftDevice.
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
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the GPIOTE module.
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}


/**@brief Function for the Timer initialization.
 *
* @details Initializes the timer module. This creates and starts application timers.
*/
static void timers_init(void)
{
    //uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t        err_code;
    ble_advdata_t   advdata;
    uint8_t         flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;
    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

    //THIS IS NOT USED
#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
    // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
    // UICR instead of using the default values. The major and minor values obtained from the UICR
    // are encoded into advertising data in big endian order (MSB First).
    // To set the UICR used by this example to a desired value, write to the address 0x10001080
    // using the nrfjprog tool. The command to be used is as follows.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
    // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
    // the following command should be used.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
    uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
    uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

    uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;

    m_beacon_info[index++] = MSB(major_value);
    m_beacon_info[index++] = LSB(major_value);

    m_beacon_info[index++] = MSB(minor_value);
    m_beacon_info[index++] = LSB(minor_value);
#endif
#if defined(GG_USE_HR_IN_ADV)

#endif

    manuf_specific_data.data.p_data        = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size          = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_NO_NAME;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.p_manuf_specific_data   = &manuf_specific_data;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}



/**@brief Function for handling button events.
 *
 * @param[in]   pin_no   The pin number of the button pressed.
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    if (button_action == APP_BUTTON_PUSH)
    {
        switch (pin_no)
        {
            case HR_INC_BUTTON_PIN_NO:
                // Increase Heart Rate measurement
                m_cur_heart_rate += HEART_RATE_CHANGE;
                if (m_cur_heart_rate > MAX_HEART_RATE)
                {
                    m_cur_heart_rate = MIN_HEART_RATE; // Loop back
                }
                nrf_gpio_pin_toggle(LED_1);
                break;
                
            case HR_DEC_BUTTON_PIN_NO:
                // Decrease Heart Rate measurement
                m_cur_heart_rate -= HEART_RATE_CHANGE;
                if (m_cur_heart_rate < MIN_HEART_RATE)
                {
                    m_cur_heart_rate = MAX_HEART_RATE; // Loop back
                }
                
                uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;
                
                m_beacon_info[index++] = MSB(major_value);
                m_beacon_info[index++] = LSB(major_value);

                m_beacon_info[index++] = MSB(minor_value);
                m_beacon_info[index++] = LSB(minor_value);
                
                sd_ble_gap_adv_stop();
                
                //https://devzone.nordicsemi.com/question/15077/stop-advertising/
                
                //nrf_delay_ms(4);
                advertising_init();
                advertising_start();
                break;
                
            default:
                APP_ERROR_HANDLER(pin_no);
                break;
        }
    }    
}

/**@brief Function for initializing the button module.
 */
static void buttons_init(void)
{
    // Configure HR_INC_BUTTON_PIN_NO and HR_DEC_BUTTON_PIN_NO as wake up buttons and also configure
    // for 'pull up' because the eval board does not have external pull up resistors connected to
    // the buttons.
    static app_button_cfg_t buttons[] =
    {
        {HR_INC_BUTTON_PIN_NO, false, BUTTON_PULL, button_event_handler},
        {HR_DEC_BUTTON_PIN_NO, false, BUTTON_PULL, button_event_handler}  // Note: This pin is also BONDMNGR_DELETE_BUTTON_PIN_NO
    };
    
    APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY, false);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    nrf_gpio_cfg_output(ADVERTISING_LED_PIN_NO);
    nrf_gpio_cfg_output(LED_1);
}


/**
 * @brief Function for configuring: pin 0 for input, pin 8 for output, 
 * and configures GPIOTE to give an interrupt on pin change.
 */
/*
static void gpio_init(void)
{
    nrf_gpio_cfg_input(BUTTON_0, BUTTON_PULL);
    nrf_gpio_cfg_output(LED_0);

    nrf_gpio_pin_write(LED_0, BUTTON_0);

    // Enable interrupt:
    NVIC_EnableIRQ(GPIOTE_IRQn);
    NRF_GPIOTE->CONFIG[0] =  (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos)
                           | (0 << GPIOTE_CONFIG_PSEL_Pos)  
                           | (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
    NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Set << GPIOTE_INTENSET_IN0_Pos;
}
*/


/** @brief Function for handling the GPIOTE interrupt which is triggered on pin 0 change.
 */
 /*
void GPIOTE_IRQHandler(void)
{
    // Event causing the interrupt must be cleared.
    if ((NRF_GPIOTE->EVENTS_IN[0] == 1) && 
        (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN0_Msk))
    {
        NRF_GPIOTE->EVENTS_IN[0] = 0;
    }
    nrf_gpio_pin_toggle(8);
}
*/

/** @brief Function for handling timer 2 peripheral interrupts.
 */
void TIMER2_IRQHandler(void)
{
    // Clear interrupt  
    if ((NRF_TIMER2->EVENTS_COMPARE[1] != 0) && 
       ((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE1_Msk) != 0))
    {
        NRF_TIMER2->EVENTS_COMPARE[1] = 0;
    }
    // Process buttons
    if (nrf_gpio_pin_read(BUTTON_1) == 0) {
        nrf_gpio_pin_toggle(LED_0);
    }
}


/** @brief Function for initializing the Timer 2 peripheral.
 */
static void timer2_init(void)
{
    // Start 16 MHz crystal oscillator .
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
    {
        //Do nothing.
    }

    NRF_TIMER2->MODE      = TIMER_MODE_MODE_Timer;
    NRF_TIMER2->PRESCALER = 6;

    // Clears the timer, sets it to 0.
    NRF_TIMER2->TASKS_CLEAR = 1;

    // Load the initial values to TIMER2 CC registers.
    NRF_TIMER2->CC[1] = 256U;

    // Interrupt setup.
    NRF_TIMER2->INTENSET = (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);
}


/** @brief Function starting the internal LFCLK XTAL oscillator.
 */
static void lfclk_config(void)
{
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        //Do nothing.
    }
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
}


/** @brief Function for configuring the RTC with TICK to 100Hz and COMPARE0 to 10 sec.
 */
static void rtc_config(void)
{
    NVIC_EnableIRQ(RTC1_IRQn);                                  // Enable Interrupt for the RTC in the core.
    NRF_RTC1->PRESCALER = COUNTER_PRESCALER;                    // Set prescaler to a TICK of RTC_FREQUENCY.
    NRF_RTC1->CC[0]     = COMPARE_COUNTERTIME * RTC_FREQUENCY;  // Compare0 after approx COMPARE_COUNTERTIME seconds.

    // Enable TICK event and TICK interrupt:
    //NRF_RTC0->EVTENSET = RTC_EVTENSET_TICK_Msk;
    //NRF_RTC0->INTENSET = RTC_INTENSET_TICK_Msk;

    // Enable COMPARE0 event and COMPARE0 interrupt:
    NRF_RTC1->EVTENSET = RTC_EVTENSET_COMPARE0_Msk;
    NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Msk;
}

/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
void RTC1_IRQHandler()
{
    if ((NRF_RTC1->EVENTS_TICK != 0) &&
        ((NRF_RTC1->INTENSET & RTC_INTENSET_TICK_Msk) != 0))
    {
        NRF_RTC1->EVENTS_TICK = 0;
        nrf_gpio_pin_toggle(GPIO_TOGGLE_TICK_EVENT);
    }
    
    if ((NRF_RTC1->EVENTS_COMPARE[0] != 0) &&
        ((NRF_RTC1->INTENSET & RTC_INTENSET_COMPARE0_Msk) != 0))
    {
        NRF_RTC1->EVENTS_COMPARE[0] = 0;
        nrf_gpio_pin_toggle(GPIO_TOGGLE_COMPARE_EVENT);
        NRF_RTC1->TASKS_CLEAR = 1;
        
        if (nrf_gpio_pin_read(BUTTON_1) == 0) {
            nrf_gpio_pin_toggle(GPIO_TOGGLE_TICK_EVENT);
            startAdvertising = 1;
            stopAdvertising = 0;
        }
        if (nrf_gpio_pin_read(BUTTON_0) == 0) {
            nrf_gpio_pin_toggle(GPIO_TOGGLE_TICK_EVENT);
            startAdvertising = 0;
            stopAdvertising = 1;
        }
    }
}


/** @brief Function for Configuring PIN8 and PIN9 as outputs.
 */
static void gpio_config(void)
{
    nrf_gpio_cfg_output(GPIO_TOGGLE_TICK_EVENT);
    nrf_gpio_cfg_output(GPIO_TOGGLE_COMPARE_EVENT);

    nrf_gpio_pin_write(GPIO_TOGGLE_TICK_EVENT, 0);
    nrf_gpio_pin_write(GPIO_TOGGLE_COMPARE_EVENT, 0);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for doing power management.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    
    // Initialize. for use with buttons
    //timers_init();
    //gpiote_init();
    //buttons_init();
    
    // Use for checking when timer instead
    // ####
    leds_init();
    nrf_gpio_cfg_input(BUTTON_0, BUTTON_PULL);
    nrf_gpio_cfg_input(BUTTON_1, BUTTON_PULL);
    //timer2_init();
    
    //gpio_config();
    lfclk_config();
    rtc_config();
    
    //ble_stack_init();
    //advertising_init();

    // Start execution.
    //advertising_start();
    
    // Start handling button presses
    //err_code = app_button_enable();
    //APP_ERROR_CHECK(err_code);
    
    // Enable interrupt on Timer 2.
    //NVIC_EnableIRQ(TIMER2_IRQn);
    //__enable_irq();

    // Start the timer.
    //NRF_TIMER2->TASKS_START = 1;
    NRF_RTC1->TASKS_START = 1;
    // ####

    // Enter main loop.
    for (;;)
    {
        //power_manage();
        if (startAdvertising) {
            // Stop RTC clock interrupts
            //NRF_CLOCK->TASKS_LFCLKSTOP = 1;
            //NRF_RTC0->TASKS_STOP = 1;
            //NVIC_DisableIRQ(RTC1_IRQn);
            
            // Enable advertising
            ble_stack_init();
            advertising_init();
            advertising_start();
            nrf_delay_ms(1000);
            sd_ble_gap_adv_stop();
            
            // Try to start RTC clock interrupts again
            //NVIC_EnableIRQ(RTC1_IRQn);
            startAdvertising = 0;
        }
        if (stopAdvertising) {
            
        }
    }
}

/**
 * @}
 */
