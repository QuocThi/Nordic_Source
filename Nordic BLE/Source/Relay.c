/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/**
 * @brief BLE Heart Rate and Running speed Relay application main file.
 *
 * @detail This application demonstrates a simple "Relay".
 * Meaning we pass on the values that we receive. By combining a collector part on
 * one end and a sensor part on the other, we show that the s130 can function
 * simultaneously as a central and a peripheral device.
 *
 * In the figure below, the sensor ble_app_hrs connects and interacts with the relay
 * in the same manner it would connect to a heart rate collector. In this case, the Relay
 * application acts as a central.
 *
 * On the other side, a collector (such as Master Control panel or ble_app_hrs_c) connects
 * and interacts with the relay the same manner it would connect to a heart rate sensor peripheral.
 *
 * Led layout:
 * LED 1: Central side is scanning       LED 2: Central side is connected to a peripheral
 * LED 3: Peripheral side is advertising LED 4: Peripheral side is connected to a central
 *
 * @note While testing, be careful that the Sensor and Collector are actually connecting to the Relay,
 *       and not directly to each other!
 *
 *    Peripheral                  Relay                    Central
 *    +--------+        +-----------|----------+        +-----------+
 *    | Heart  |        | Heart     |   Heart  |        |           |
 *    | Rate   | -----> | Rate     -|-> Rate   | -----> | Collector |
 *    | Sensor |        | Collector |   Sensor |        |           |
 *    +--------+        +-----------|   and    |        +-----------+
 *                      | Running   |   Running|
 *    +--------+        | Speed    -|-> Speed  |
 *    | Running|------> | Collector |   Sensor |
 *    | Speed  |        +-----------|----------+
 *    | Sensor |
 *    +--------+
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "peer_manager.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_hrs.h"
#include "ble_hrs_c.h"
#include "ble_conn_state.h"
#include "nrf_fstorage.h"
#include "fds.h"
#include "ble_lbs.h"
#include "ble_lbs_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"
#include "SEGGER_RTT.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


//#define PERIPHERAL_ADVERTISING_LED      LED_1
//#define PERIPHERAL_CONNECTED_LED        LED_2
//#define CENTRAL_SCANNING_LED            LED_3
//#define CENTRAL_CONNECTED_LED           LED_4

#define PERIPHERAL_CONNECTED_LED        LED_3
#define PERIPHERAL_ADVERTISING_LED      LED_4
#define STATE_FROM_PHONE_CHANGE         LED_1
#define CENTRAL_CONNECTED_LED           LED_2
#define LEDBUTTON_LED                 NRF_GPIO_PIN_MAP(1,04)
#define CENTRAL_SCANNING_LED          NRF_GPIO_PIN_MAP(1,06)

#define DEVICE_NAME                     "DoAn"                                     /**< Name of device used for advertising. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                       /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                         /**< The advertising interval (in units of 0.625 ms). This value corresponds to 187.5 ms. */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size in octets. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size in octets. */

#define SCAN_INTERVAL                   0x00A0                                      /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                     0x0050                                      /**< Determines scan window in units of 0.625 millisecond. */

#define SCAN_DURATION                   0x0000                                      /**< Duration of the scanning in units of 10 milliseconds. If set to 0x0000, scanning will continue until it is explicitly disabled. */


#define MIN_CONNECTION_INTERVAL         (uint16_t) MSEC_TO_UNITS(7.5, UNIT_1_25_MS) /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL         (uint16_t) MSEC_TO_UNITS(30, UNIT_1_25_MS)  /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY                   0                                           /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT             (uint16_t) MSEC_TO_UNITS(4000, UNIT_10_MS)  /**< Determines supervision time-out in units of 10 milliseconds. */
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                 /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

/**@brief   Priority of the application BLE event handler.
 * @note    You shouldn't need to modify this value.
 */
#define APP_BLE_OBSERVER_PRIO           1


static ble_lbs_t m_lbs;                                             /**< Heart rate service instance. */
static ble_lbs_c_t m_ble_lbs_c;                                         /**< Heart rate service client instance. */

NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_TOTAL_LINK_COUNT);              /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                      /**< Database discovery module instances. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint16_t m_conn_handle_ble_c  = BLE_CONN_HANDLE_INVALID;     /**< Connection handle for the HRS central application */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */
uint8_t current_Role = BLE_GAP_ROLE_PERIPH;
bool first_time_init_central = false;
uint16_t phone_relay_conn = 5;
uint16_t role_turn = 0;
//typedef struct{
//              ble_gap_addr_t save_peer_addr;
//              ble_gap_scan_params_t   save_m_scan_params;
//              ble_gap_conn_params_t   save_m_connection_param;
//} save_phone_relay_conn_t;
//save_phone_relay_conn_t *save_phone_relay_conn;

//bool first_time_init_peripheral = false;
/**@brief names which the central applications will scan for, and which will be advertised by the peripherals.
 *  if these are set to empty strings, the UUIDs defined below will be used
 */
static char const m_target_periph_name[] = "Peripheral";

/**@brief UUIDs which the central applications will scan for if the name above is set to an empty string,
 * and which will be advertised by the peripherals.
 */
static ble_uuid_t m_adv_uuids[] =
{
    {LBS_UUID_SERVICE,        BLE_UUID_TYPE_VENDOR_BEGIN}
};
/*****************************************************************************************************************************/
/*****************************************************************************************************************************/
/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};
/*****************************************************************************************************************************/
/*****************************************************************************************************************************/

/**@brief Parameters used when scanning. */
static ble_gap_scan_params_t const m_scan_params =
{
    .extended      = 1,
    .active        = 1,
    .interval      = SCAN_INTERVAL,
    .window        = SCAN_WINDOW,
    .timeout       = SCAN_DURATION,
    .scan_phys     = BLE_GAP_PHY_1MBPS,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
};

static uint8_t m_scan_buffer_data[BLE_GAP_SCAN_BUFFER_EXTENDED_MIN]; /**< buffer where advertising reports will be stored by the SoftDevice. */

/**@brief Pointer to the buffer where advertising reports will be stored by the SoftDevice. */
static ble_data_t m_scan_buffer =
{
    m_scan_buffer_data,
    BLE_GAP_SCAN_BUFFER_EXTENDED_MIN
};

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    MIN_CONNECTION_INTERVAL,
    MAX_CONNECTION_INTERVAL,
    SLAVE_LATENCY,
    SUPERVISION_TIMEOUT
};


/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initiating scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    (void) sd_ble_gap_scan_stop();

    err_code = sd_ble_gap_scan_start(&m_scan_params, &m_scan_buffer);
    // It is okay to ignore this error since we are stopping the scan anyway.
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for initiating advertising and scanning.
 */

static void advertising_start(void)
{
    NRF_LOG_INFO("RUN!");
    ret_code_t           err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("========== Advertising -PERIPHERAL_ADVERTISING_LED : ON ============")
    //user_led_on(PERIPHERAL_ADVERTISING_LED);
}

static void adv_scan_start(void)
{
    ret_code_t err_code;
    NRF_LOG_INFO("****************** Central SCANNING *******************");
    //check if there are no flash operations in progress
    if (!nrf_fstorage_is_busy(NULL))
    {
        // Start scanning for peripherals and initiate connection to devices which
        // advertise Heart Rate or Running speed and cadence UUIDs.
        //test_function();
        scan_start();

        // Turn on the LED to signal scanning.
        NRF_LOG_INFO("===================== Central Scanning - CENTRAL_SCANNING_LED : ON ============")
        //user_led_on(CENTRAL_SCANNING_LED);

        // Start advertising.
       // NRF_LOG_INFO("$$$$$$$$$$$ Disconnect so RE-CONNECT 4 $$$$$$$$$");
        //advertising_start();
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            //test_function();
            adv_scan_start();

        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // This can happen when the local DB has changed.
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}

/*===========================================================================================================================*/
/*===========================================================================================================================*/
/**@brief Handles events coming from the LED Button central module.
 */
static void lbs_c_evt_handler(ble_lbs_c_t * p_lbs_c, ble_lbs_c_evt_t * p_lbs_c_evt)
{
    switch (p_lbs_c_evt->evt_type)
    {
        case BLE_LBS_C_EVT_DISCOVERY_COMPLETE:
        {
        if (m_conn_handle_ble_c == BLE_CONN_HANDLE_INVALID)
           {
            ret_code_t err_code;
                m_conn_handle_ble_c = p_lbs_c_evt->conn_handle;
            err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c,
                                                p_lbs_c_evt->conn_handle,
                                                &p_lbs_c_evt->params.peer_db);
            //NRF_LOG_INFO("LED Button service discovered on conn_handle 0x%x.", p_lbs_c_evt->conn_handle);

            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
            //Initiate bonding.
            err_code = pm_conn_secure(m_conn_handle_ble_c, false);
                            if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            // LED Button service discovered. Enable notification of Button.
            err_code = ble_lbs_c_button_notif_enable(p_lbs_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("========= STATE FROM PHONE NOW: %d ============",new_state_from_phone);
            ble_lbs_led_status_send(&m_ble_lbs_c, new_state_from_phone);
            }
        } break; // BLE_LBS_C_EVT_DISCOVERY_COMPLETE

        case BLE_LBS_C_EVT_BUTTON_NOTIFICATION:
        {
            //NRF_LOG_INFO("Button state changed on peer to 0x%x.", p_lbs_c_evt->params.button.button_state);
            if (p_lbs_c_evt->params.button.button_state)
            {
                NRF_LOG_INFO("############### BUTTON on peer ON - LEDBUTTON_LED : ON ######################");
                //user_led_off(LEDBUTTON_LED);
            }
            else
            {
                  NRF_LOG_INFO("############### BUTTON on peer OFF - LEDBUTTON_LED : OFF ######################");
                //user_led_off(LEDBUTTON_LED);
            }
        } break; // BLE_LBS_C_EVT_BUTTON_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}



/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_adv_report  Advertising report from the SoftDevice.
 */
static void on_adv_report(ble_gap_evt_adv_report_t const * p_adv_report)
{
    ret_code_t err_code;

    if (strlen(m_target_periph_name) != 0)
    {
        if (ble_advdata_name_find(p_adv_report->data.p_data,
                                  p_adv_report->data.len,
                                  m_target_periph_name))
        {
            // Initiate connection.
            NRF_LOG_INFO("####################################### INIT CONNECTION WITH PERIPHERAL, Peer_addr: %d ",p_adv_report->peer_addr.addr_id_peer);
    
//            save_phone_relay_conn->save_m_connection_param = m_connection_param;
//            save_phone_relay_conn->save_m_scan_params = m_scan_params;
//            save_phone_relay_conn->save_peer_addr = p_adv_report->peer_addr;
            err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                          &m_scan_params,
                                          &m_connection_param,
                                          APP_BLE_CONN_CFG_TAG);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_INFO("Connection Request Failed, reason %d", err_code);
            }
        }
        else
        {
            err_code = sd_ble_gap_scan_start(NULL, &m_scan_buffer);
            APP_ERROR_CHECK(err_code);
        }
    }
    else  
    {
        ble_uuid_t target_uuid_lbs = {.uuid = LBS_UUID_SERVICE, .type = BLE_UUID_TYPE_VENDOR_BEGIN};

        // We do not want to connect to two peripherals offering the same service, so when
        // a UUID is matched, we check whether we are not already connected to a peer which
        // offers the same service.
        if (   (ble_advdata_uuid_find(p_adv_report->data.p_data, p_adv_report->data.len, &target_uuid_lbs)
                && (m_conn_handle_ble_c == BLE_CONN_HANDLE_INVALID)))
        {
            // Initiate connection.

            NRF_LOG_INFO("================ INIT CONNECTION WITH PERIPHERAL 1 ================");
            err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                          &m_scan_params,
                                          &m_connection_param,
                                          APP_BLE_CONN_CFG_TAG);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_WARNING("Connection Request Failed, reason %d", err_code);
            }
        }
        else
        {
            err_code = sd_ble_gap_scan_start(NULL, &m_scan_buffer);
            APP_ERROR_CHECK(err_code);
        }
    }
}


/**@brief Function for assigning new connection handle to available instance of QWR module.
 *
 * @param[in] conn_handle New connection handle.
 */
static void multi_qwr_conn_handle_assign(uint16_t conn_handle)
{
    for (uint32_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
    {
        if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            ret_code_t err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], conn_handle);
            APP_ERROR_CHECK(err_code);
            break;
        }
    }
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_lbs_on_db_disc_evt(&m_ble_lbs_c, p_evt);
}


/**
 * @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief LED Button client initialization.
 */
static void lbs_c_init(void)
{
    NRF_LOG_INFO("===== RUN lbs_c_init Central Init =====");
    ret_code_t       err_code;
    ble_lbs_c_init_t lbs_c_init_obj;

    lbs_c_init_obj.evt_handler = lbs_c_evt_handler;
    err_code = ble_lbs_c_init(&m_ble_lbs_c, &lbs_c_init_obj);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("====== END lbs_c_init Central Init =======");
}


/**@brief   Function for handling BLE events from central applications.
 *
 * @details This function parses scanning reports and initiates a connection to peripherals when a
 *          target UUID is found. It updates the status of LEDs used to report central applications
 *          activity.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_central_evt(ble_evt_t const * p_ble_evt)
{
    NRF_LOG_INFO("========== TEST CENTRAL HANDLER RUN? =============");
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
//    if(!first_time_init_peripheral)
//    {
//    if(role == BLE_GAP_ROLE_PERIPH)
//        {
//          NRF_LOG_INFO("=========== Role change to BLE_GAP_ROLE_CENTRAL ============");
//          //ble_lbs_on_db_disc_evt(p_ble_evt, &m_ble_lbs_c);
//          //gatt_init();
//          db_discovery_init();
//          lbs_c_init();
//          adv_scan_start();
//        }
//    }
//    if((role_turn > 0) && ((role_turn % 2) ==0 ))
//    {
//        NRF_LOG_INFO("===================================== SEND PHONE STATE CHANGE TIME 2+");
//        ble_lbs_led_status_send(&m_ble_lbs_c, new_state_from_phone);
//    }
    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
        // discovery, update LEDs status and resume scanning if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
//            relay_peripheral_conn = p_gap_evt->conn_handle;
//            relay_peripheral_connection = relay_peripheral_conn;
//             NRF_LOG_INFO("========== Connected RELAY - PERIPHERAL. with CONN_HANDLE = %d =============", relay_peripheral_conn);
            err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c, p_gap_evt->conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_db_disc, p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);

            // Update LEDs status, and check if we should be looking for more
            // peripherals to connect to.
            user_led_on(CENTRAL_CONNECTED_LED);
            user_led_off(CENTRAL_SCANNING_LED);
//            NRF_LOG_INFO("Central connected");
//            // If no Heart Rate sensor or RSC sensor is currently connected, try to find them on this peripheral.
//            if (   (m_conn_handle_ble_c  == BLE_CONN_HANDLE_INVALID))
//            {
//                NRF_LOG_INFO("Gap Connected!", p_gap_evt->conn_handle);
//
//                err_code = ble_db_discovery_start(&m_db_discovery[0], p_gap_evt->conn_handle);
//                if (err_code == NRF_ERROR_BUSY)
//                {
//                    err_code = ble_db_discovery_start(&m_db_discovery[1], p_gap_evt->conn_handle);
//                    APP_ERROR_CHECK(err_code);
//                }
//                else
//                {
//                    APP_ERROR_CHECK(err_code);
//                }
//            }
//
//            // Assing connection handle to the QWR module.
//            multi_qwr_conn_handle_assign(p_gap_evt->conn_handle);
//
//            // Update LEDs status, and check if we should be looking for more peripherals to connect to.
//            NRF_LOG_INFO("Turn on CENTRAL_CONNECTED_LED");
//            user_led_on(CENTRAL_CONNECTED_LED);
//            if (ble_conn_state_central_conn_count() == NRF_SDH_BLE_CENTRAL_LINK_COUNT)
//            {
//                NRF_LOG_INFO("============= Central CONNECTED -  CENTRAL_SCANNING_LED : OFF ======================");
//                //user_led_off(CENTRAL_SCANNING_LED);
//            }
//            else
//            {
//                // Resume scanning.
//                NRF_LOG_INFO(" ================= Central Resume Scanning -  CENTRAL_SCANNING_LED : ON ===============");
//                //user_led_on(CENTRAL_SCANNING_LED);
//                scan_start();
//            }
        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer which disconnected,
        // update the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED: break;
//        {
//            if (p_gap_evt->conn_handle == m_conn_handle_ble_c)
//            {
//                NRF_LOG_INFO(" GAP Disconnect!!",
//                             p_gap_evt->params.disconnected.reason);
//
//                m_conn_handle_ble_c = BLE_CONN_HANDLE_INVALID;
//            }
//
//            if ( (m_conn_handle_ble_c  == BLE_CONN_HANDLE_INVALID))
//            {
//                // Start scanning
//                scan_start();
//
//                // Update LEDs status.
//                NRF_LOG_INFO(" ================= Central Resume Scanning -  CENTRAL_SCANNING_LED : ON ===============");
//                //user_led_on(CENTRAL_SCANNING_LED);
//            }
//
//            if (ble_conn_state_central_conn_count() == 0)
//            {
//                user_led_off(CENTRAL_CONNECTED_LED);
//            }
//        } break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_ADV_REPORT:
        {
            NRF_LOG_INFO("============================================ RELAY-PERIPHERAL CONN IN REPORT");
            ble_gap_evt_adv_report_t const * p_adv_report;
            p_adv_report = &p_gap_evt->params.adv_report;
            on_adv_report(&p_gap_evt->params.adv_report);
        } break; // BLE_GAP_ADV_REPORT

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief   Function for handling BLE events from peripheral applications.
 * @details Updates the status LEDs used to report the activity of the peripheral applications.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_peripheral_evt(ble_evt_t const * p_ble_evt)
{
    ret_code_t err_code;
    if(!first_time_init_central)
    {
      if(role == BLE_GAP_ROLE_CENTRAL)
        {
          first_time_init_central = true;
          NRF_LOG_INFO("=========== Role change to BLE_GAP_ROLE_CENTRAL ============");
          //ble_lbs_on_db_disc_evt(p_ble_evt, &m_ble_lbs_c);
          gatt_init();
          db_discovery_init();
          lbs_c_init();
          //ble_lbs_c_on_ble_evt(p_ble_evt,&m_ble_lbs_c);
          //on_ble_central_evt(p_ble_evt);
          adv_scan_start();
        }
    }
    else
    {
      if((role_turn > 0) && ((role_turn % 2) ==0 ))
      {
        NRF_LOG_INFO("===================================== SEND PHONE STATE CHANGE TIME 2+");
        ble_lbs_led_status_send(&m_ble_lbs_c, new_state_from_phone);
      }
    }
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("************** Peripheral connected- PERIPHERAL_ADVERTISING_LED : OFF, PERIPHERAL_CONNECTED_LED : ON *******************");
            user_led_on(PERIPHERAL_CONNECTED_LED);
            //user_led_on(PERIPHERAL_CONNECTED_LED);
            // Assing connection handle to the QWR module.
            phone_relay_conn = p_ble_evt->evt.gap_evt.conn_handle;
            NRF_LOG_INFO("=========== RELAY - PHONE CONN_HANDLE: %d ==============", phone_relay_conn);
            multi_qwr_conn_handle_assign(p_ble_evt->evt.gap_evt.conn_handle);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("******************** Peripheral disconnected - PERIPHERAL_CONNECTED_LED : OFF *******************");
            user_led_off(PERIPHERAL_CONNECTED_LED);
            NRF_LOG_INFO("$$$$$$$$$$$ Disconnect so RE-CONNECT 1 $$$$$$$$$");
            advertising_start();
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling advertising events.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
        {
            NRF_LOG_INFO("================ Fast advertising - PERIPHERAL_ADVERTISING_LED : ON ==========================");
            //user_led_on(PERIPHERAL_ADVERTISING_LED);
        } break;

        case BLE_ADV_EVT_IDLE:
        {
             NRF_LOG_INFO("$$$$$$$$$$$ Disconnect so RE-CONNECT 2 $$$$$$$$$");
             advertising_start();
            //APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for checking if a bluetooth stack event is an advertising timeout.
 *
 * @param[in] p_ble_evt Bluetooth stack event.
 */
static bool ble_evt_is_advertising_timeout(ble_evt_t const * p_ble_evt)
{
    return (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_SET_TERMINATED);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    //uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
 //   uint16_t role        = ble_conn_state_role(conn_handle);
       NRF_LOG_INFO("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ CHECK ROLE TO SWAP: %d ",role);
      if(role == BLE_GAP_ROLE_PERIPH)
      {
        NRF_LOG_INFO("===== RUN PERIPHERAL ROLE =====");
//        if(last_role == BLE_GAP_ROLE_CENTRAL)
//        {
//            scan_start();
//        }
        ble_lbs_on_ble_evt(p_ble_evt,&m_lbs);
        on_ble_peripheral_evt(p_ble_evt);
      }
      else if(role == BLE_GAP_ROLE_CENTRAL)
    {
      NRF_LOG_INFO("=========== RUN CENTRAL ROLE ============");
//      if(last_role == BLE_GAP_ROLE_PERIPH)
//      {
//          scan_start();
//      }
      on_ble_central_evt(p_ble_evt);
      lbs_c_evt_handler(p_ble_evt,&m_ble_lbs_c);
    }
    // Based on the role this device plays in the connection, dispatch to the right handler.
//    if (role == BLE_GAP_ROLE_PERIPH || ble_evt_is_advertising_timeout(p_ble_evt))
//    {
//        NRF_LOG_INFO("--------- RUN PERIPHERAL MODE FIRST -----------");
//        //ble_lbs_on_db_disc_evt(p_ble_evt, &m_lbs);
//        on_ble_peripheral_evt(p_ble_evt);
//    }
//    else if ((role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT) )
//    {
//        NRF_LOG_INFO("------------ RUN CENTRAL MODE FIRST -----------");
//        //ble_lbs_on_db_disc_evt(p_ble_evt, &m_ble_lbs_c);
//        on_ble_central_evt(p_ble_evt);
//    }
}
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);


//         const uint32_t LOCAL_ERR_CODE = (err_code);         
//        if (LOCAL_ERR_CODE != NRF_SUCCESS)                  
//        {                                                   
//           test_function(5000);              
//        } 
//        else
//        {
//          test_function(3000);
//        }
  
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to
 *                            wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONNECTION_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONNECTION_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = SUPERVISION_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle_ble_c, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID; // Start upon connection.
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;  // Ignore events.
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void led_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state)
{
    NRF_LOG_INFO("Show new ROLE info: %d",role);
    if (led_state)
    {
        NRF_LOG_INFO("=========================== PHONE change led ON - STATE_FROM_PHONE_CHANGE : ON =============================");
        user_led_on(STATE_FROM_PHONE_CHANGE);
    }
    else
    {
        NRF_LOG_INFO("=========================== PHONE change led OFF - STATE_FROM_PHONE_CHANGE : OFF =============================");
        user_led_off(STATE_FROM_PHONE_CHANGE);
    }
    role_turn++;
    last_role = BLE_GAP_ROLE_PERIPH;
    role = BLE_GAP_ROLE_CENTRAL;
    NRF_LOG_INFO("================================================================================================================= CENTRAL ROLE! ");
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    NRF_LOG_INFO("00000000000000000000000000 RUN Peripheral Init Service 0000000000000000000000000000000");
    ret_code_t         err_code;
    ble_lbs_init_t     init     = {0};
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
     memset(&init, 0, sizeof(init));
    // Initialize LBS.
    init.led_write_handler = led_write_handler;
    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("00000000000000000000000000 END Peripheral Init Service 0000000000000000000000000000000");
}

/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */

void user_led_on(uint32_t pin_number)
{
    nrf_gpio_cfg_output(pin_number);
    nrf_gpio_pin_set(pin_number);
}

void user_led_off(uint32_t pin_number)
{
    nrf_gpio_cfg_output(pin_number);
    nrf_gpio_pin_clear(pin_number);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{

    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

    ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE,BLE_UUID_TYPE_VENDOR_BEGIN}};
    
    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));
    //NRF_LOG_INFO("\n====================RUN ADVERTISING INITFUNCTION=======================");
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;
   // NRF_LOG_INFO("advdata NAME_TYPE: %d",advdata.name_type);
    //NRF_LOG_INFO("advdata UUID: %X",srdata.uuids_complete.p_uuids->uuid);
    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);
    // NRF_LOG_INFO("\n======================END ADVERTISING INITFUNCTION=======================");
}

/**@brief Function for initializing logging.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop). If there is no pending log operation,
          then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}

/****************************************************************************************************************************
****************************************************************************************************************************/
/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
 /*
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON_PIN:  
            NRF_LOG_INFO("Button Characteractise was pressed!");
            err_code = ble_lbs_led_status_send(&m_ble_lbs_c, button_action);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            if (err_code == NRF_SUCCESS)
            {
                NRF_LOG_INFO("LBS write LED state %d", button_action);
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}
*/

/****************************************************************************************************************************
****************************************************************************************************************************/

/**@brief Function for initializing the button handler module.
 */
 /*
static void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON_PIN, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}*/


/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    // Initialize.
    log_init();
    timer_init();
    buttons_leds_init(&erase_bonds);
    leds_init();
    //buttons_init();

    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    conn_params_init();
    //peer_manager_init();
    services_init();
    //lbs_c_init();
    user_led_off(PERIPHERAL_CONNECTED_LED);
    user_led_off(STATE_FROM_PHONE_CHANGE);
    advertising_init();
    // Start execution.
     //adv_scan_start();
     //NRF_LOG_INFO("$$$$$$$$$$$ Disconnect so RE-CONNECT 3 $$$$$$$$$");
     advertising_start();
//   if (erase_bonds == true)
//    {
//        // Scanning and advertising is done upon PM_EVT_PEERS_DELETE_SUCCEEDED event.
//        NRF_LOG_INFO("Delete bonds!");
//        delete_bonds();
//    }
//    else
//    {
//        NRF_LOG_INFO("Scaning...!");
//        adv_scan_start();
//    }

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}
