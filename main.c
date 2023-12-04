/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
//for adc
#include "nrf_adc.h"
#include "nrf_adc.c"
#include "nrf_delay.h"
//for interrupt
#include "nrf_timer.h"
#include "nrf_drv_timer.h"
//for I2C
#include "nrf_drv_twi.h"
#include "nrf_drv_twi.c"


//LED for nrf51dk
#define LED1 21
#define LED2 22
#define LED3 23
#define LED4 24

//LED for greenchip
#define LED_Power_PIN_NO          8        		                               /**< LED to indicate advertising state. */
#define LED_Connect_PIN_NO        9		                                       /**< LED to indicate connected state. */
#define LED_Comunitction_PIN_NO   11		                                   /**< LED to indicate connected state. */


#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "Alamda"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

/**LED*/
static void leds_init(void)
{
	nrf_gpio_cfg_output(LED1);
	nrf_gpio_cfg_output(LED2);
	nrf_gpio_cfg_output(LED3);
	nrf_gpio_cfg_output(LED4);
	nrf_gpio_cfg_output(LED_Power_PIN_NO);
	nrf_gpio_cfg_output(LED_Comunitction_PIN_NO);
	nrf_gpio_cfg_output(LED_Connect_PIN_NO);
	
}




/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        while (app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while (app_uart_put('\r') != NRF_SUCCESS);
    while (app_uart_put('\n') != NRF_SUCCESS);
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);

}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' i.e '\r\n' (hex 0x0D) or if the string has reached a length of
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                err_code = ble_nus_string_send(&m_nus, data_array, index);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_advdata_t          scanrsp;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**@breif Function for ADC
* ADC initialization
*/

volatile int32_t adc_value;

void ADC_IRQHandler(void)
{
	nrf_adc_conversion_event_clean();
	adc_value = nrf_adc_result_get();
	nrf_adc_start();
}

#ifndef NRF_APP_PRORITY_HIGH
#define NRF_APP_PRIORTY_HIGH 1
#endif

void adc_config(void)
{
	const nrf_adc_config_t nrf_adc_config = NRF_ADC_CONFIG_DEFAULT;
	//initialize and configure ADC
	nrf_adc_configure((nrf_adc_config_t *)&nrf_adc_config);
	//nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_2);
	nrf_adc_int_enable(ADC_INTENSET_END_Enabled << ADC_INTENCLR_END_Pos);
	NVIC_SetPriority(ADC_IRQn, NRF_APP_PRIORTY_HIGH);
	NVIC_EnableIRQ(ADC_IRQn);
}

/**@breif Function for I2C
* I2C initialization
*/
//nrf51
#define I2C_SCL_PIN        7
#define I2C_SDA_PIN        30
//green chip
//#define I2C_SCL_PIN        24
//#define I2C_SDA_PIN        23

#define MPU_6050_I2C_ADDR    0X68     //MPU6050 I2C-ADRESS

/* MPU-6050 Register Map */
#define WHO_AM_I           0x75
#define ACCEL_XOUT_H       0x3B
#define TEMP_OUT_H         0x41    
#define GYRO_XOUT_H        0x43         
#define PWR_MGMT_1         0x6B

//TWI instance
static const nrf_drv_twi_t m_twi_mpu_6050 = NRF_DRV_TWI_INSTANCE(0);

//I2C
void twi_init(void)
{
	ret_code_t err_code;
	const nrf_drv_twi_config_t twi_mpu_6050_config = {
	  .scl = I2C_SCL_PIN,
		.sda = I2C_SDA_PIN,
		.frequency = NRF_TWI_FREQ_400K,
		.interrupt_priority = APP_IRQ_PRIORITY_HIGH
	};
  err_code = nrf_drv_twi_init(&m_twi_mpu_6050,&twi_mpu_6050_config,NULL,NULL);
	APP_ERROR_CHECK(err_code);
	
	nrf_drv_twi_enable(&m_twi_mpu_6050);
}
/**
* @breif Function for writing to registers on the MPU-6050.
*/
ret_code_t mpu_6050_I2C_register_write(uint8_t reg_addr, uint8_t*p_tx_data,uint8_t bytes)
{
	ret_code_t ret_code;
	
	uint8_t tx_data[bytes+1];
	
	tx_data[0] = reg_addr;
	
	for(uint8_t i = 0; i<bytes; i++)
	{
		tx_data[i+1] = p_tx_data[i];
	}
	
	ret_code = nrf_drv_twi_tx(&m_twi_mpu_6050,MPU_6050_I2C_ADDR, tx_data, sizeof(tx_data),false);
	
	return ret_code;
	
}

/**
* @breif Functin for reading from registers on the MPU-6050.
*/

ret_code_t mpu_6050_I2C_register_read(uint8_t reg_addr, uint8_t*p_rx_data,uint32_t bytes)
{
	ret_code_t ret_code;
	
	ret_code = nrf_drv_twi_tx(&m_twi_mpu_6050,MPU_6050_I2C_ADDR,&reg_addr,1,false);
	
	if(ret_code != NRF_SUCCESS)
	{
		return ret_code;
	}
	
	ret_code = nrf_drv_twi_rx(&m_twi_mpu_6050,MPU_6050_I2C_ADDR,p_rx_data,bytes);
	
	return ret_code;
	
}

/**@breif mpu6050 
*/
int16_t x_val;
int16_t y_val;
int16_t z_val;
int16_t x_gyro;
int16_t y_gyro;
int16_t z_gyro;
double temp_c;
//calibrated value
int16_t x_cal;
int16_t y_cal;
int16_t z_cal;

void mpu_6050_init(void)
{
	ret_code_t err_code;
	
	uint8_t tx_data = 0;
	
	//write zero to the PWR_MGMT_1 register to wake up the MPU-6050
	err_code = mpu_6050_I2C_register_write(PWR_MGMT_1,&tx_data,1);
	
	APP_ERROR_CHECK(err_code);
}

void mpu_6050_get_device_id(uint8_t*p_dev_id)
{
	ret_code_t err_code;
	
	uint8_t rx_data;
	
	//Read the I2C address of the mpu-6050 from the who-am-i register
	err_code = mpu_6050_I2C_register_read(WHO_AM_I,&rx_data,1);
	
	APP_ERROR_CHECK(err_code);
	
	*p_dev_id = rx_data;
	
}

void mpu_6050_read_acc(int16_t*p_x_val,int16_t*p_y_val,int16_t*p_z_val)
{
	ret_code_t err_code;
	
	//Raw accelerometer measurements buffer
	uint8_t acc_data[6];
	
	//Read the six accelerometer data registers starting from ACCEL_XOUT_H
	err_code = mpu_6050_I2C_register_read(ACCEL_XOUT_H,acc_data,sizeof(acc_data));
	
	APP_ERROR_CHECK(err_code);
	
	/*Combine the two 8-bit data registers to a 16-bit value for each axis by left shifting ACCEL_Xout_h
	eight times and or it with accel_xout_l.*/
	*p_x_val = (acc_data[0]<<8)|acc_data[1];
	*p_y_val = (acc_data[2]<<8)|acc_data[3];
	*p_z_val = (acc_data[4]<<8)|acc_data[5];

}
void mpu_6050_read_gyro(int16_t * p_x_gyro, int16_t * p_y_gyro, int16_t * p_z_gyro)
{
    ret_code_t err_code;
    
    uint8_t gyro_data[6];
    
    //Read the 6 gyroscope data registers starting from GYRO_XOUT_H
    err_code = mpu_6050_I2C_register_read(GYRO_XOUT_H, gyro_data, sizeof(gyro_data));
    
    APP_ERROR_CHECK(err_code);
    
    /*  Combine the two 8-bit data registers to a 16-bit value
        for each axis by left shifting GYRO_xOUT_H eight times 
        and OR it with GYRO_xOUT_L. */
    *p_x_gyro = (gyro_data[0]<<8)|gyro_data[1];
    *p_y_gyro = (gyro_data[2]<<8)|gyro_data[3];
    *p_z_gyro = (gyro_data[4]<<8)|gyro_data[5];
}
	
void mpu_6050_read_temp(double*p_temp)
{
	ret_code_t err_code;
	
	//signed 16-bit integer to store the meaurements
	int16_t temp;
	
	//Raw temperature measurements buffer
	uint8_t temp_data[2];
	
	//Read the two temperature data registers starting from TEMP_OUT_H
	err_code = mpu_6050_I2C_register_read(GYRO_XOUT_H, temp_data,sizeof(temp_data));
	
	APP_ERROR_CHECK(err_code);
	
	/*Combine the two 8-bit data registers to a 16-bit value
	  for each axis by left shiffing TEMP_OUT_H eight tiems 
	  and OR it with TEMP_OUT_L.*/
	temp = (temp_data[0]<<8)|temp_data[1];
	
	/*Convert raw measurement to degress C using formula from TEMP_OUT_H
	  register description 
	  in MPU-6050 Register Map and Descriptions document*/
		
		*p_temp = (temp/340.0)+36.53;
	
}

/**@breif calibration for mpu6050 
*
*/

int buffersize=1000;
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;
/*
 * Values we prefer to read from sensors in each axes when placing the device horizontally, package letters facing
 * up, without any movement.
 *
 * The goal values for all sensors in each axes are 0, except for z axis of accelerometer. The goal value for z
 * axis of accelerometer is 16384, meaning +1 g, indicating the object is under gravity.
 */
const int16_t acc_x_goal = 0;
const int16_t acc_y_goal = 0;
const int16_t acc_z_goal = 16384;
const int16_t gyro_x_goal = 0;
const int16_t gyro_y_goal = 0;
const int16_t gyro_z_goal = 0;

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
int ax_cali,ay_cali,az_cali;

void calibration()
{
	long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
	
	while(i<(buffersize+101))
	{
		//read raw accel/gyro measurements from device
		mpu_6050_read_acc(&x_val,&y_val,&z_val);
		mpu_6050_read_gyro(&x_gyro,&y_gyro,&z_gyro);
		
		if(i>100 && i<=(buffersize+100))
			{ //First 100 measures are discarded
				buff_ax=buff_ax+x_val;
				buff_ay=buff_ay+y_val;
				buff_az=buff_az+z_val;
				buff_gx=buff_gx+x_gyro;
				buff_gy=buff_gy+y_gyro;
				buff_gz=buff_gz+z_gyro;			
	    }
		if(i==(buffersize+100))
    {
			mean_ax=buff_ax/buffersize;
			mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
			printf("MEAN accel x : %d, y:%d,Z:%d \r\n",mean_ax,mean_ay,mean_az);
			
			ax_offset=-mean_ax/8;
	    ay_offset=-mean_ay/8;
	    az_offset=(16384-mean_az)/8;
	
	    gx_offset=-mean_gx/4;
	    gy_offset=-mean_gy/4;
	    gz_offset=-mean_gz/4;
		  printf("offset accel x : %d, y:%d,Z:%d \r\n",ax_offset,ay_offset,az_offset);
		}
		i++;

	}
}


void calibration_x()
{
	ax_offset=-mean_ax/8;
	ay_offset=-mean_ay/8;
	az_offset=(16384-mean_az)/8;
	
	gx_offset=-mean_gx/4;
	gy_offset=-mean_gy/4;
	gz_offset=-mean_gz/4;
	
	printf("offset accel x : %d, y:%d,Z:%d \r\n",ax_offset,ay_offset,az_offset);
	
	while(1)
  {
		int ready=0;
		
		
		//meansensors();
		
		
		 if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) 
		printf("mean accel x : %d, y:%d,Z:%d \r\n",ax_offset,ay_offset,az_offset);
		break;
		 
	}
	
}
int detection;

void posture()
{
	if(ax_cali<=0 && ay_cali <=-1 && az_cali==0)
   {
		 detection = 0; //person lying on left side
	 }
	 if(ax_cali ==0 && ay_cali == 3 && az_cali<=0)
   {
		 detection = 1; //person lying on right side
	 }
	 if(ax_cali ==0 && ay_cali == 0 && az_cali>=2)
	 {
		 detection = 2; //upright posture lying posture
	 }
	 if(ax_cali == -3 && ay_cali ==0 && az_cali<=0)
	 {
		 detection = 3; // upright posture
	 }
	 

}

int adc_result;
void adc_detection()
{
	if(adc_value>= 0 && adc_value<40)
	{  adc_result = 10;	}
	if(adc_value>=40 && adc_value<80)
	{ adc_result = 30; }
	if(adc_value>=80 && adc_value<100)
	{ adc_result = 40;  }
	if(adc_value>=100 && adc_value<120)
	{ adc_result = 50; }
	if(adc_value>=120 && adc_value<160)
	{ adc_result = 60; }
	if(adc_value>=160 && adc_value<200)
	{adc_result = 70;}
	if(adc_value>=200 && adc_value<240)
	{adc_result = 85;}

}


void alamda_sensor()
{
	if(detection == 0 && detection == 1)
	{
		adc_result = 0.8*adc_result;
	}
	if(detection == 2)
	{
		adc_result = 0.6*adc_result;
	}
}

/**@breif timer to interrupt
*/
void start_timer2(void)
{
	//NRF_TIMER2->MODE = TIMER_MODE_MODE_Counter;         //counter_mode
	NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;            //timer mode
	NRF_TIMER2->TASKS_CLEAR = 1;                          //clear time
	NRF_TIMER2->PRESCALER =9;                            // Set presclaser. Higher number gives slower timer.     
	NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;    //Set counter to 16 bit resolutin
	NRF_TIMER2->CC[0] = 0;                              //Set vlaue for Timer2 comapre register 0
	NRF_TIMER2->CC[1] = 31250;                        	    //Set value for timer2 compare register 1
	NRF_TIMER2->CC[2] = 31250;
	NRF_TIMER2->CC[3] = 62500;
	//Enable interrupt on Timer2, both for CC[0] and CC[1] compare match events
	NRF_TIMER2->INTENSET = ( TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos
	                       | TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos
	                       | TIMER_INTENSET_COMPARE2_Enabled << TIMER_INTENSET_COMPARE2_Pos
	                       | TIMER_INTENSET_COMPARE3_Enabled << TIMER_INTENSET_COMPARE3_Pos
	                       );
	NVIC_EnableIRQ(TIMER2_IRQn);
	NRF_TIMER2->TASKS_START = 1;                         //Start Timer2
}
void TIMER2_IRQHandler(void)
{
	if((NRF_TIMER2->EVENTS_COMPARE[0] !=0 )&&((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE0_Msk) !=0))
	{ //1易帚
		NRF_TIMER2->EVENTS_COMPARE[0] = 0;
		nrf_gpio_pin_clear(LED3);
		//nrf_gpio_pin_clear(LED_Connect_PIN_NO);
		nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_3);	
		adc_detection();
		
		
	}
	if((NRF_TIMER2->EVENTS_COMPARE[1] !=0 ) &&((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE1_Msk) !=0))
	{ //3易帚
		NRF_TIMER2->EVENTS_COMPARE[1] = 0;
		//nrf_gpio_pin_set(LED_Connect_PIN_NO);
		nrf_gpio_pin_set(LED3);
		 
			
		
	}
	if((NRF_TIMER2->EVENTS_COMPARE[2] != 0) &&((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE2_Msk) !=0))
	{ //2易帚
		
		NRF_TIMER2->EVENTS_COMPARE[2] = 0;
		nrf_gpio_pin_clear(LED4);
	
	}
		
	if((NRF_TIMER2->EVENTS_COMPARE[3] != 0) &&((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE3_Msk) !=0))
	{ //4易帚
		NRF_TIMER2->EVENTS_COMPARE[3] = 0;
		nrf_gpio_pin_set(LED4);
	  mpu_6050_read_acc(&x_val,&y_val,&z_val);
		ax_cali = (x_val-ax_offset)/4096;
	  ay_cali = (y_val-ay_offset)/4096;
		az_cali = (z_val-az_offset)/4096;
		//printf("Cali acc x : %d, y:%d,Z:%d \r\n",ax_cali,ay_cali,az_cali);
		posture();
		
	}
	
}

/**@brief Application main function.
 */
int main(void)
{
	
    uint32_t err_code;
    bool erase_bonds;

    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    uart_init();
	  leds_init();

    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();

    printf("\r\nUART Start!\r\n");
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
	
	  //Initialize I2C
		twi_init();
		
		//set up ADC
		adc_config();		
		nrf_adc_start();
		
		//Initialize MPU-6050
		mpu_6050_init();
		uint8_t device_id;
		printf("MPU6050 test start \r\n");
		
		//Read Device ID
		mpu_6050_get_device_id(&device_id);
		printf("mpu6050 ID : %x \r\n",device_id);
		
		//calibrate mpu-6050
		calibration();
		
		
		
		//set up timer
		start_timer2();

    // Enter main loop.
    for (;;)
    {
			  //send ble
			  uint8_t str0[1];
			  //uint8_t str1[1];
			  //sprintf((char*)str0,"%d,%d,%d,%d,%d\n",(int)adc_result,(int)ax_cali,(int)ay_cali,(int)az_cali,(int)detection);
			  //sprintf((char*)str0,"%d,%d\n",(int)adc_value,(int)detection);
		 	  alamda_sensor();
			  sprintf((char*)str0,"%d",(int)adc_result);
			  ble_nus_string_send(&m_nus,str0,strlen((char*)str0));
			  nrf_delay_ms(1000); 
 			
			
			  NRF_TIMER2->TASKS_COUNT = 1;
        power_manage();
    }
}


/**
 * @}
 */
