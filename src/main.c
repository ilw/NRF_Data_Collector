/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_power.h"
#include "app_error.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
#include "ringbuf.h"


#define ENDLINE_STRING "\r\n"

// USB DEFINES START

static bool m_usb_connected = false;

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

static char m_cdc_data_array[BLE_NUS_MAX_DATA_LEN];

/** @brief CDC_ACM class instance */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

// USB DEFINES END







#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

//TODO Change this?
#define UART_TX_BUF_SIZE        8192                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        2048                                    /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define ECHOBACK_BLE_UART_DATA  0                                       /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */

//#define CONNECTED_MESSAGE "Connected to device with Hearable EEG & PPG Service."

//TODO Change this?
BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/**@brief NUS UUID. */
//TODO Change this?
//static ble_uuid_t const m_match_uuid =
//{
//    .uuid = BLE_UUID_EEG_NUS_SERVICE,
//    .type = NUS_SERVICE_UUID_TYPE
//};

static char const m_target_periph_name[] = "Hearable";
#define PREFIX_LENGTH 4
#define EEG_PREFIX "EEG "
#define PPG_PREFIX "PPG "
#define ACC_PREFIX "ACC "

#define RINGBUF_SIZE 16384 //Power of 2!
#define USB_PACKET_SIZE 2048

static uint8_t usbBuffer[3][USB_PACKET_SIZE];

struct ringbuf eegRing,ppgRing,accRing;
static uint8_t ringBuffer[RINGBUF_SIZE];
static uint8_t ringBuffer2[RINGBUF_SIZE];
static uint8_t ringBuffer3[RINGBUF_SIZE];


static uint8_t BLE_connected=0;



/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for starting scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    NRF_LOG_INFO("STARTING SCAN");
    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);

    APP_ERROR_CHECK(ret);
}


/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;
    NRF_LOG_DEBUG("Scan event: %d",p_scan_evt->scan_evt_id);
    switch(p_scan_evt->scan_evt_id)
    {
         case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
         {
              err_code = p_scan_evt->params.connecting_err.err_code;
              APP_ERROR_CHECK(err_code);
         } break;

         case NRF_BLE_SCAN_EVT_CONNECTED:
         {
              ble_gap_evt_connected_t const * p_connected =
                               p_scan_evt->params.connected.p_connected;
             // Scan is automatically stopped by the connection.
             NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                      p_connected->peer_addr.addr[0],
                      p_connected->peer_addr.addr[1],
                      p_connected->peer_addr.addr[2],
                      p_connected->peer_addr.addr[3],
                      p_connected->peer_addr.addr[4],
                      p_connected->peer_addr.addr[5]
                      );
         } break;

         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
         {
             NRF_LOG_INFO("Scan timed out.");
             scan_start();
         } break;

         default:
             break;
    }
}


/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

// Setting filters for scanning.
	err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
	APP_ERROR_CHECK(err_code);

	err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
	APP_ERROR_CHECK(err_code);
//    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_match_uuid);
//    APP_ERROR_CHECK(err_code);

//    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
//    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
//TODO Change this?
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}



/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;
    int i;
    uint16_t count=0;
    switch (p_ble_nus_evt->evt_type)
    {
    	case BLE_NUS_C_EVT_DISCOVERY_AVAILABLE:
    		NRF_LOG_INFO("Discovery available.");
			if (p_ble_nus_c->handles.nus_eeg_rx_handle
					&& p_ble_nus_c->handles.nus_eeg_tx_handle
					&& p_ble_nus_c->handles.nus_eeg_tx_cccd_handle
					&& p_ble_nus_c->handles.nus_ppg_rx_handle
					&& p_ble_nus_c->handles.nus_ppg_tx_handle
					&& p_ble_nus_c->handles.nus_ppg_tx_cccd_handle
					&& p_ble_nus_c->handles.nus_acc_rx_handle
					&& p_ble_nus_c->handles.nus_acc_tx_handle
					&& p_ble_nus_c->handles.nus_acc_tx_cccd_handle
			)
			{
				if (!BLE_connected)
				{
					err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c,&(p_ble_nus_c->handles.nus_ppg_tx_cccd_handle));
					APP_ERROR_CHECK(err_code);
					err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c,&(p_ble_nus_c->handles.nus_eeg_tx_cccd_handle));
					APP_ERROR_CHECK(err_code);
					err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c,&(p_ble_nus_c->handles.nus_acc_tx_cccd_handle));
					APP_ERROR_CHECK(err_code);
					printf("Connected to device with Hearable EEG & PPG & ACCEL Service.");
					BLE_connected=1;
				}
			}
			else
			{
//				err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
//				APP_ERROR_CHECK(err_code);
				NRF_LOG_DEBUG("Didn't discover all the necessary characteristic handles");
			}
			break;
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
        	NRF_LOG_INFO("Discovery complete event.");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, p_ble_nus_evt->srv_uuid, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);
			break;

        case BLE_NUS_C_EVT_NUS_EEG_TX_EVT:
        	count =0;
        	for (i=0;i<p_ble_nus_evt->data_len;i++)
        		count+= ringbuf_put( &eegRing, *(p_ble_nus_evt->p_data +i)  );
         	if (count != p_ble_nus_evt->data_len)
        	{
        		NRF_LOG_ERROR("EEG data lost");
        		bsp_indication_set(BSP_INDICATE_RCV_ERROR);
        	}
        	break;

        case BLE_NUS_C_EVT_NUS_PPG_TX_EVT:
        	count =0;
			for (i=0;i<p_ble_nus_evt->data_len;i++)
				count+= ringbuf_put(&ppgRing, *(p_ble_nus_evt->p_data +i));
			if (count != p_ble_nus_evt->data_len)
			{
				NRF_LOG_ERROR("PPG data lost");
				bsp_indication_set(BSP_INDICATE_RCV_ERROR);
			}
        	break;

        case BLE_NUS_C_EVT_NUS_ACC_TX_EVT:
			count =0;
			for (i=0;i<p_ble_nus_evt->data_len;i++)
				count+= ringbuf_put(&accRing, *(p_ble_nus_evt->p_data +i));
			if (count != p_ble_nus_evt->data_len)
			{
				NRF_LOG_ERROR("ACCEL data lost");
				bsp_indication_set(BSP_INDICATE_RCV_ERROR);
			}
			break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            BLE_connected=0;
            scan_start();
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */


/**
 * @brief Function for handling shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    NRF_LOG_DEBUG("BLE_EVT: 0x%x, ",p_ble_evt->header.evt_id);
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle,BLE_UUID_EEG_NUS_SERVICE, NULL);
            APP_ERROR_CHECK(err_code);
            err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle,BLE_UUID_PPG_NUS_SERVICE, NULL);
            APP_ERROR_CHECK(err_code);
            err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle,BLE_UUID_ACC_NUS_SERVICE, NULL);
            APP_ERROR_CHECK(err_code);


            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected");
            ble_gap_phys_t const desired_phys =
			{
					.rx_phys = BLE_GAP_PHY_2MBPS,
					.tx_phys = BLE_GAP_PHY_2MBPS,
			};
            err_code = sd_ble_gap_phy_update(p_gap_evt->conn_handle,&desired_phys);
            APP_ERROR_CHECK(err_code);
            // start discovery of services. The NUS Client waits for a discovery result
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:

            NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
        	NRF_LOG_INFO("Updating connection parameters, min: %d, max:%d, latency: %d, timeout: %d", \
        			p_gap_evt->params.conn_param_update_request.conn_params.min_conn_interval, \
					p_gap_evt->params.conn_param_update_request.conn_params.max_conn_interval, \
					p_gap_evt->params.conn_param_update_request.conn_params.slave_latency, \
					p_gap_evt->params.conn_param_update_request.conn_params.conn_sup_timeout);
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_INFO("PHY update request.");
            ble_gap_phys_t const phys =
            {
                    .rx_phys = BLE_GAP_PHY_2MBPS,
                    .tx_phys = BLE_GAP_PHY_2MBPS,
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
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
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
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler = ble_nus_c_evt_handler;

    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}


//USB Code start

/** @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            /*Set up the first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_cdc_data_array,
                                                   1);
            UNUSED_VARIABLE(ret);
            NRF_LOG_INFO("CDC ACM port opened");
            break;
        }

        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            NRF_LOG_INFO("CDC ACM port closed");
            if (m_usb_connected)
            {
            }
            break;

        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            break;

        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            static uint8_t index = 0;
            index++;

            do
            {
                if ((m_cdc_data_array[index - 1] == '\n') ||
                    (m_cdc_data_array[index - 1] == '\r') ||
                    (index >= (m_ble_nus_max_data_len)))
                {
                    if (index > 1)
                    {
                        NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                        NRF_LOG_HEXDUMP_DEBUG(m_cdc_data_array, index);

                        do
                        {
                            uint16_t length = (uint16_t)index;
                            if (length + sizeof(ENDLINE_STRING) < BLE_NUS_MAX_DATA_LEN)
                            {
                                memcpy(m_cdc_data_array + length, ENDLINE_STRING, sizeof(ENDLINE_STRING));
                                length += sizeof(ENDLINE_STRING);
                            }

//                            ret = ble_nus_data_send(&m_nus,
//                                                    (uint8_t *) m_cdc_data_array,
//                                                    &length,
//                                                    m_conn_handle);

                            if (ret == NRF_ERROR_NOT_FOUND)
                            {
                                NRF_LOG_INFO("BLE NUS unavailable, data received: %s", m_cdc_data_array);
                                break;
                            }

                            if (ret == NRF_ERROR_RESOURCES)
                            {
                                NRF_LOG_ERROR("BLE NUS Too many notifications queued.");
                                break;
                            }

                            if ((ret != NRF_ERROR_INVALID_STATE) && (ret != NRF_ERROR_BUSY))
                            {
                                APP_ERROR_CHECK(ret);
                            }
                        }
                        while (ret == NRF_ERROR_BUSY);
                    }

                    index = 0;
                }

                /*Get amount of data transferred*/
                size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
                NRF_LOG_DEBUG("RX: size: %lu char: %c", size, m_cdc_data_array[index - 1]);

                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                            &m_cdc_data_array[index],
                                            1);
                if (ret == NRF_SUCCESS)
                {
                    index++;
                }
            }
            while (ret == NRF_SUCCESS);

            break;
        }
        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            break;

        case APP_USBD_EVT_DRV_RESUME:
            break;

        case APP_USBD_EVT_STARTED:
            break;

        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            break;

        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;

        case APP_USBD_EVT_POWER_REMOVED:
        {
            NRF_LOG_INFO("USB power removed");
            m_usb_connected = false;
            app_usbd_stop();
        }
            break;

        case APP_USBD_EVT_POWER_READY:
        {
            NRF_LOG_INFO("USB ready");
            m_usb_connected = true;
            app_usbd_start();
        }
            break;

        default:
            break;
    }
}

// USB CODE END


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module. */
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


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
//        nrf_pwr_mgmt_run();
    }
}


int main(void)
{
    // Initialize.
	int i;
	ret_code_t ret;
	static const app_usbd_config_t usbd_config = {.ev_state_proc = usbd_user_ev_handler    };
    log_init();
    timer_init();
    buttons_leds_init();
	app_usbd_serial_num_generate();
    db_discovery_init();
    power_management_init();

	ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    ble_stack_init();
    gatt_init();
    nus_c_init();
    scan_init();
    
	ret = app_usbd_power_events_enable();
    APP_ERROR_CHECK(ret);



    /////////////////////
    //Ring buffer init

    ringbuf_init (&eegRing, ringBuffer, RINGBUF_SIZE);
    ringbuf_init (&ppgRing, ringBuffer2, RINGBUF_SIZE);
    ringbuf_init (&accRing, ringBuffer3, RINGBUF_SIZE);


    //TODO Fix this initialisation so that it automatically changes with changes in prefix and prefix length
    	usbBuffer[0][0]='E';
    	usbBuffer[0][1]='E';
    	usbBuffer[0][2]='G';
    	usbBuffer[0][3]='_';

    	usbBuffer[1][0]='P';
    	usbBuffer[1][1]='P';
    	usbBuffer[1][2]='G';
    	usbBuffer[1][3]='_';

    	usbBuffer[2][0]='A';
		usbBuffer[2][1]='C';
		usbBuffer[2][2]='C';
		usbBuffer[2][3]='_';
    ///////////////////////////////////



    // Start execution.
    NRF_LOG_INFO("BLE UART central example started.\r\n");
    NRF_LOG_INFO("BLE UART central example started.");
    scan_start();

    // Enter main loop.
    for (;;)
    {
		while (app_usbd_event_queue_process());

		while (ringbuf_elements(&eegRing) >= (USB_PACKET_SIZE-PREFIX_LENGTH))
		{
			for (i=0;i<(USB_PACKET_SIZE-PREFIX_LENGTH);i++) 	usbBuffer[0][i+PREFIX_LENGTH] = ringbuf_get(&eegRing);
			ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, &usbBuffer[0],USB_PACKET_SIZE);
			if(ret != NRF_SUCCESS) NRF_LOG_INFO("CDC ACM unavailable");
		}

		while (ringbuf_elements(&ppgRing) >= (USB_PACKET_SIZE -PREFIX_LENGTH))
		{
			for (i=0;i<(USB_PACKET_SIZE-PREFIX_LENGTH);i++) 	usbBuffer[1][i+PREFIX_LENGTH] = ringbuf_get(&ppgRing);
			ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, &usbBuffer[1],USB_PACKET_SIZE);
			if(ret != NRF_SUCCESS) NRF_LOG_INFO("CDC ACM unavailable");
		}

		while (ringbuf_elements(&accRing) >= (USB_PACKET_SIZE -PREFIX_LENGTH))
		{
			for (i=0;i<(USB_PACKET_SIZE-PREFIX_LENGTH);i++) 	usbBuffer[2][i+PREFIX_LENGTH] = ringbuf_get(&accRing);
			ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, &usbBuffer[2],USB_PACKET_SIZE);
			if(ret != NRF_SUCCESS) NRF_LOG_INFO("CDC ACM unavailable");
		}

        idle_state_handle();
    }
}
