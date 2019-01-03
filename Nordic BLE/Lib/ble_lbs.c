/**
 * Copyright (c) 2013 - 2018, Nordic Semiconductor ASA
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
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_LBS)
#include "ble_lbs.h"
#include "ble_srv_common.h"
#include "nrf_log.h"

uint16_t role = BLE_GAP_ROLE_PERIPH;
uint16_t last_role = BLE_GAP_ROLE_INVALID;
uint16_t new_state_from_phone = NO_STATE;
/**@brief Function for handling the Write event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(ble_lbs_t * p_lbs, ble_evt_t const * p_ble_evt)
{
     NRF_LOG_INFO("This on_write run to handle WRITE event!");
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write; // From GATTS event struct
      //ret_code_t            err_code;
     // err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
     //                                    BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      //VERIFY_SUCCESS(err_code);
    NRF_LOG_INFO("In on_write function()! VALUE_HANDLE: %d -- PEVT_WRITE_LEN: %d -- PLBS_LEDWRITE_HANDLE: %d",p_lbs->led_char_handles.value_handle,p_evt_write->len,p_lbs->led_write_handler);

    if (   (p_evt_write->handle == p_lbs->led_char_handles.value_handle)
        && (p_evt_write->len == 1)
        && (p_lbs->led_write_handler != NULL))
    {
        //NRF_LOG_INFO("Role Chane: %d",role);
        if(p_evt_write->data[0] == 0)
        {
          new_state_from_phone = LED_OFF;
        }
        else if(p_evt_write->data[0] == 1)
        {
          new_state_from_phone = LED_ON;
        }
        p_lbs->led_write_handler(p_ble_evt->evt.gap_evt.conn_handle, p_lbs, p_evt_write->data[0]);
    }
}


void ble_lbs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    NRF_LOG_INFO("\n=======================RUN ble_lbs_on_ble_evt in ble_lbs_c=========================")
    NRF_LOG_INFO("This function run when catch a WRITE event!");
    ble_lbs_t * p_lbs = (ble_lbs_t *)p_context;
   ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
   NRF_LOG_INFO("Length of data before config: %d",p_evt_write->len);
  NRF_LOG_INFO("Header Event ID: %d",p_ble_evt->header.evt_id);
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_write(p_lbs, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
    NRF_LOG_INFO("\n=======================END ble_lbs_on_ble_evt in ble_lbs_c=========================")
}


/**@brief Function for adding the LED Characteristic.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_lbs_init LED Button Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t led_char_add(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init)
{
    NRF_LOG_INFO("\n=========RUN LED CHAR ADD===========");
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.char_props.write = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    ble_uuid.type = p_lbs->uuid_type;
    ble_uuid.uuid = LBS_UUID_LED_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
    attr_char_value.p_value   = NULL;

    return sd_ble_gatts_characteristic_add(p_lbs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_lbs->led_char_handles);
}


/**@brief Function for adding the Button Characteristic.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_lbs_init LED Button Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t button_char_add(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init)
{
    NRF_LOG_INFO("\n=========RUN BUTTON CHAR ADD ===========");
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_lbs->uuid_type;
    ble_uuid.uuid = LBS_UUID_BUTTON_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
    attr_char_value.p_value   = NULL;

    return sd_ble_gatts_characteristic_add(p_lbs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_lbs->button_char_handles);
    
}


uint32_t ble_lbs_init(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
    // Initialize service structure.
    p_lbs->led_write_handler = p_lbs_init->led_write_handler;

    // Add service.
    ble_uuid128_t base_uuid = {LBS_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_lbs->uuid_type);
    if(err_code == NRF_ERROR_NO_MEM)
    {
      NRF_LOG_INFO("NRF_ERROR_NO_MEM!\r\n");
    }
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_lbs->uuid_type;
    ble_uuid.uuid = LBS_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_lbs->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics.
    err_code = button_char_add(p_lbs, p_lbs_init);
    VERIFY_SUCCESS(err_code);

    err_code = led_char_add(p_lbs, p_lbs_init);
    VERIFY_SUCCESS(err_code);
    NRF_LOG_INFO("000000000000000000000000000 OK! Init lbs from PERIPHRAL Complete! 000000000000000000000000000000");
    return NRF_SUCCESS;
}


uint32_t ble_lbs_on_button_change(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t button_state)
{
    NRF_LOG_INFO("\n========Send Button State Change from Periph to Central=========")
    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(button_state);
    NRF_LOG_INFO("\nSize of button state: %d",len);
    NRF_LOG_INFO("\nButton change state from Periphral!");
    memset(&params, 0, sizeof(params));
    params.type   = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_lbs->button_char_handles.value_handle;
    NRF_LOG_INFO("\nValue handle: %d",params.handle);
    params.p_data = &button_state;
    params.p_len  = &len;
     NRF_LOG_INFO("\n======== END Send Button State Change from Periph to Central=========")
    return sd_ble_gatts_hvx(conn_handle, &params);
}
#endif // NRF_MODULE_ENABLED(BLE_LBS)
