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
// Board/nrf6310/ble/ble_app_hrs_rtx/main.c
/**
 *
 * @brief Heart Rate Service Sample Application with RTX main file.
 *
 * This file contains the source code for a sample application using RTX and the
 * Heart Rate service (and also Battery and Device Information services).
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "sensorsim.h"
#include "softdevice_handler.h"
#include "ble_nus_c.h"
#include "app_timer.h"
#include "app_util.h"
#include "peer_manager.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "fds.h"
#include "fstorage.h"
#include "ble_conn_state.h"
#include "nrf_drv_clock.h"
#define NRF_LOG_MODULE_NAME "Bachelor"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "m_bus_receiver.h"
#include "ble_nus.h"


#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                      /**< Include the Service Changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT      7                               /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT   1                               /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#define TOTAL_LINK_COUNT          CENTRAL_LINK_COUNT + PERIPHERAL_LINK_COUNT /**< Total number of links used by the application. */
#define ELEMENTS_IN_MY_DATA_STRUCT		  7


#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE    GATT_MTU_SIZE_DEFAULT           /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define UART_TX_BUF_SIZE        256                             /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        512                             /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN      /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_TIMER_PRESCALER     0                               /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE 2                               /**< Size of timer operation queues. */

#define SCAN_INTERVAL           0x00A0                          /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0050                          /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_ACTIVE             1                               /**< If 1, performe active scanning (scan requests). */
#define SCAN_SELECTIVE          0                               /**< If 1, ignore unknown devices (non whitelisted). */
#define SCAN_TIMEOUT            0x0000                          /**< Timout when scanning. 0x0000 disables timeout. */


#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS) /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(75, UNIT_1_25_MS) /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY           0                               /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS) /**< Determines supervision time-out in units of 10 millisecond. */

#define UUID16_SIZE             2                               /**< Size of 16 bit UUID */
#define UUID32_SIZE             4                               /**< Size of 32 bit UUID */
#define UUID128_SIZE            16                              /**< Size of 128 bit UUID */


/* For slave config*/
#define DEVICE_NAME              		       "Nordic_UART"    /**< Name of device. Will be included in the advertising data. */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */
#define APP_ADV_INTERVAL                64                      /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                     /**< The advertising timeout (in units of seconds). */



static ble_nus_c_t              m_ble_nus_c[TOTAL_LINK_COUNT];                    /**< Instance of NUS service. Must be passed to all NUS_C API calls. */
static ble_db_discovery_t       m_ble_db_discovery[TOTAL_LINK_COUNT];             /**< Instance of database discovery module. Must be passed to all db_discovert API calls */
static uint8_t           		m_ble_nus_c_count;  
static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */
static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */


static SemaphoreHandle_t m_ble_event_ready;  /**< Semaphore raised if there is a new event to be processed in the BLE thread. */
static SemaphoreHandle_t m_uart_event_ready; //Semaphore raised if there is a new event to be processed in the uart thread
static SemaphoreHandle_t uart_data;          //Mutex for the uart module. 

static TaskHandle_t m_ble_stack_thread;      /**< Definition of BLE stack thread. */
static TaskHandle_t m_uart_stack_thread;     //Task for the uart thread.
static TaskHandle_t m_logger_thread;         /**< Definition of Logger thread. */

static uint8_t curr_connection = 0;
/**
 * @brief Connection parameters requested for connection.
 */
static const ble_gap_conn_params_t m_connection_param =
  {
    (uint16_t)MIN_CONNECTION_INTERVAL,  // Minimum connection
    (uint16_t)MAX_CONNECTION_INTERVAL,  // Maximum connection
    (uint16_t)SLAVE_LATENCY,            // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT       // Supervision time-out
  };

/**
 * @brief Parameters used when scanning.
 */
static const ble_gap_scan_params_t m_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION == 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION == 3)
        .use_whitelist = 0,
    #endif
};


typedef struct 
{
	uint8_t type;												/**< Type of slave device */
	uint8_t address;										/**< Address given by central */
	uint8_t actions;										/**< etc */		
	uint8_t actions2;										/**< etc */
	uint8_t temp;												/**< Integer part of extern temp sensor on NRF52 */
	uint8_t temp_dec;										/**< Fractional part of extern temp semsor on NRF52 */
	uint8_t priority;										/**< The priority of the slave device */
}my_data;

static my_data slave_data[CENTRAL_LINK_COUNT];


/**
 * @brief NUS uuid
 */
static const ble_uuid_t m_nus_uuid =
  {
    .uuid = BLE_UUID_NUS_SERVICE,
    .type = NUS_SERVICE_UUID_TYPE
  };

/**@brief Function for asserts in the SoftDevice.
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


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t ret;

    ret = sd_ble_gap_scan_start(&m_scan_params);

		APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
		NRF_LOG_INFO("Uart_c Scan started\r\n");
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
	NRF_LOG_INFO("call to ble_nus_on_db_disc_evt for instance %d and link 0x%02x!\r\n\n",
                    p_evt->conn_handle,
                    p_evt->conn_handle);
					
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c[p_evt->conn_handle], p_evt);
	curr_connection = p_evt->conn_handle;
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' i.e '\r\n' (hex 0x0D) or if the string has reached a length of
 *          @ref NUS_MAX_DATA_LENGTH.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    //static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    //static uint8_t index = 0;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
					
//            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
//            index++;

//            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
//            {
//                while (ble_nus_c_string_send(&m_ble_nus_c, data_array, index) != NRF_SUCCESS)
//                {
//                    // repeat until sent.
//                }
//                index = 0;
//            }
            break;
        /**@snippet [Handling data from UART] */
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


/**@brief Function to send data over Nordic Uart serial
 * @param[in]  What slave data to send
 * @param[out] bool true or false, sending successful not not
 */
bool send_data(uint8_t slave_nr)
{
		
	uint8_t data[ELEMENTS_IN_MY_DATA_STRUCT];
	uint32_t err_code;
	
	data[0] = slave_data[slave_nr].type;
	data[1] = slave_data[slave_nr].address;
	data[2] = slave_data[slave_nr].actions;
	data[3] = slave_data[slave_nr].actions2;
	data[4] = slave_data[slave_nr].temp;
	data[5] = slave_data[slave_nr].temp_dec;
	data[6] = slave_data[slave_nr].priority;
	
	
 	err_code = ble_nus_c_string_send(&m_ble_nus_c[slave_nr],data,ELEMENTS_IN_MY_DATA_STRUCT);
	APP_ERROR_CHECK(err_code);
	
	if(err_code == NRF_SUCCESS )
	{
			NRF_LOG_INFO("	sending complete \n\r");
			return true;
	}
	else
		{
			NRF_LOG_INFO("	sending not complete \n\r");
			
			return false;
		}	
		
		
	/* err_code = ble_nus_string_send(&m_nus,data,ELEMENTS_IN_MY_DATA_STRUCT);	
	APP_ERROR_CHECK(err_code);
	if(err_code == NRF_SUCCESS )
	{
			NRF_LOG_INFO("	sending to mobile complete \n\r");
			return true;
	}
	else
		{
			NRF_LOG_INFO("	sending to mobile not complete \n\r");
			
			return false;
		} */	
}

/**@brief Function to print data to slaves
 * 
 *  
 */
void print_slave_data(void)
{
	for(int i=0; i<2;i++) //CENTRAL_LINK_COUNT
	{
		NRF_LOG_INFO("	Type sent:			0x%02x\n\r",slave_data[i].type);
		NRF_LOG_INFO("	Address sent:			0x%02x\n\r",slave_data[i].address);
		NRF_LOG_INFO("	Etc sent:			0x%02x\n\r",slave_data[i].actions);
		NRF_LOG_INFO("	Etc sent:			0x%02x\n\r",slave_data[i].actions2);
		NRF_LOG_INFO("	Temp sent:			0x%02x\n\r",slave_data[i].temp);
		NRF_LOG_INFO("	Temp_dec sent:			0x%02x\n\r",slave_data[i].temp_dec);
		NRF_LOG_INFO("	Priority sent:			0x%02x\n\n\r",slave_data[i].priority);
	}
 
 }
 
 
/**@brief Callback handling NUS Client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS Client Handle. This identifies the NUS client
 * @param[in]   p_ble_nus_evt Pointer to the NUS Client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, const ble_nus_c_evt_t * p_ble_nus_evt)
{
    uint32_t err_code;

	
    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_rx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("	The device has the Nordic UART Service\r\n");
            break;

        case BLE_NUS_C_EVT_NUS_RX_EVT:
            	err_code = bsp_indication_set(BSP_INDICATE_RCV_OK);
				APP_ERROR_CHECK(err_code);
					if(0x00 != p_ble_nus_evt->p_data[0])
					{
						slave_data[curr_connection].type = p_ble_nus_evt->p_data[0]; 
						slave_data[curr_connection].address = curr_connection;
						slave_data[curr_connection].actions = p_ble_nus_evt->p_data[2];
						slave_data[curr_connection].actions2 = p_ble_nus_evt->p_data[3]; 
						slave_data[curr_connection].temp = p_ble_nus_evt->p_data[4]; 		
						slave_data[curr_connection].temp_dec = p_ble_nus_evt->p_data[5]; 
						slave_data[curr_connection].priority = p_ble_nus_evt->p_data[6];
					}
						send_data(curr_connection);
					
						print_slave_data();
								
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected\r\n");
            scan_start();
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */

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

/**@brief Reads an advertising report and checks if a uuid is present in the service list.
 *
 * @details The function is able to search for 16-bit, 32-bit and 128-bit service uuids.
 *          To see the format of a advertisement packet, see
 *          https://www.bluetooth.org/Technical/AssignedNumbers/generic_access_profile.htm
 *
 * @param[in]   p_target_uuid The uuid to search fir
 * @param[in]   p_adv_report  Pointer to the advertisement report.
 *
 * @retval      true if the UUID is present in the advertisement report. Otherwise false
 */
static bool is_uuid_present(const ble_uuid_t *p_target_uuid,
                            const ble_gap_evt_adv_report_t *p_adv_report)
{
    uint32_t err_code;
    uint32_t index = 0;
    uint8_t *p_data = (uint8_t *)p_adv_report->data;
    ble_uuid_t extracted_uuid;

    while (index < p_adv_report->dlen)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if ( (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE)
           || (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE)
           )
        {
            for (uint32_t u_index = 0; u_index < (field_length / UUID16_SIZE); u_index++)
            {
                err_code = sd_ble_uuid_decode(  UUID16_SIZE,
                                                &p_data[u_index * UUID16_SIZE + index + 2],
                                                &extracted_uuid);
                if (err_code == NRF_SUCCESS)
                {
                    if ((extracted_uuid.uuid == p_target_uuid->uuid)
                        && (extracted_uuid.type == p_target_uuid->type))
                    {
                        return true;
                    }
                }
            }
        }

        else if ( (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_MORE_AVAILABLE)
                || (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_COMPLETE)
                )
        {
            for (uint32_t u_index = 0; u_index < (field_length / UUID32_SIZE); u_index++)
            {
                err_code = sd_ble_uuid_decode(UUID16_SIZE,
                &p_data[u_index * UUID32_SIZE + index + 2],
                &extracted_uuid);
                if (err_code == NRF_SUCCESS)
                {
                    if ((extracted_uuid.uuid == p_target_uuid->uuid)
                        && (extracted_uuid.type == p_target_uuid->type))
                    {
                        return true;
                    }
                }
            }
        }

        else if ( (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE)
                || (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE)
                )
        {
            err_code = sd_ble_uuid_decode(UUID128_SIZE,
                                          &p_data[index + 2],
                                          &extracted_uuid);
            if (err_code == NRF_SUCCESS)
            {
                if ((extracted_uuid.uuid == p_target_uuid->uuid)
                    && (extracted_uuid.type == p_target_uuid->type))
                {
                    return true;
                }
            }
        }
        index += field_length + 1;
    }
    return false;
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t              err_code;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            const ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;

            if (is_uuid_present(&m_nus_uuid, p_adv_report))
            {

                err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                              &m_scan_params,
                                              &m_connection_param);

                if (err_code == NRF_SUCCESS)
                {
                    // scan is automatically stopped by the connect
                    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
                    APP_ERROR_CHECK(err_code);
                    NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x\r\n",
                             p_adv_report->peer_addr.addr[0],
                             p_adv_report->peer_addr.addr[1],
                             p_adv_report->peer_addr.addr[2],
                             p_adv_report->peer_addr.addr[3],
                             p_adv_report->peer_addr.addr[4],
                             p_adv_report->peer_addr.addr[5]
                             );
                }
            }
        }break; // BLE_GAP_EVT_ADV_REPORT

        case BLE_GAP_EVT_CONNECTED:
		
			NRF_LOG_INFO("Connection 0x%x established, starting DB discovery.\r\n",
                         p_gap_evt->conn_handle);
		
            //NRF_LOG_DEBUG("Connected to target\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            // start discovery of services. The NUS Client waits for a discovery result
            err_code = ble_db_discovery_start(&m_ble_db_discovery[p_gap_evt->conn_handle],
                                              p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);
			
			if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }
			
			if (ble_conn_state_n_centrals() == CENTRAL_LINK_COUNT)
            {
                err_code = bsp_indication_set(BSP_INDICATE_IDLE);
				APP_ERROR_CHECK(err_code);
            }
            else
            {
                // Resume scanning.
                err_code = bsp_indication_set(BSP_INDICATE_SCANNING);
				APP_ERROR_CHECK(err_code);
                scan_start();
            }
			
			
			
            break; // BLE_GAP_EVT_CONNECTED
			
            
		case BLE_GAP_EVT_DISCONNECTED:
        {
            uint32_t central_link_cnt; // Number of central links.

            NRF_LOG_INFO("LBS central link 0x%x disconnected (reason: 0x%x)\r\n",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);

            // Start scanning
            scan_start();

            // Update LEDs status.
            err_code = bsp_indication_set(BSP_INDICATE_SCANNING);
			APP_ERROR_CHECK(err_code);
			
            central_link_cnt = ble_conn_state_n_centrals();
            if (central_link_cnt == 0)
            {
                bsp_board_led_off(1);
            }
        } 	
			break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                //NRF_LOG_DEBUG("Scan timed out.\r\n");
                scan_start();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.\r\n");
            }
            break; // BLE_GAP_EVT_TIMEOUT

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            //NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            //NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
				APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *          been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	uint16_t conn_handle;
	conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
	
	
    on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);	
	
	 // Make sure that an invalid connection handle are not passed since
    // our array of modules is bound to TOTAL_LINK_COUNT.
    if (conn_handle < TOTAL_LINK_COUNT)
    {
        ble_db_discovery_on_ble_evt(&m_ble_db_discovery[conn_handle], p_ble_evt);
        ble_nus_c_on_ble_evt(&m_ble_nus_c[conn_handle], p_ble_evt);
    }
	
	ble_nus_on_ble_evt(&m_nus, p_ble_evt);
	
}

/**
 * @brief Event handler for new BLE events
 *
 * This function is called from the SoftDevice handler.
 * It is called from interrupt level.
 *
 * @return The returned value is checked in the softdevice_handler module,
 *         using the APP_ERROR_CHECK macro.
 */
static uint32_t ble_new_event_handler(void)
{
    BaseType_t yield_req = pdFALSE;

    // The returned value may be safely ignored, if error is returned it only means that
    // the semaphore is already given (raised).
    UNUSED_VARIABLE(xSemaphoreGiveFromISR(m_ble_event_ready, &yield_req));
    portYIELD_FROM_ISR(yield_req);
    return NRF_SUCCESS;
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, ble_new_event_handler);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
		NRF_LOG_FLUSH();
    APP_ERROR_CHECK(err_code);
		
		
	 // Use the max config: 8 central, 0 periph, 10 VS UUID
    //ble_enable_params.common_enable_params.vs_uuid_count = 10;
	
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{		NRF_LOG_INFO("bps_event_handler \r\n\n\n");
    //uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
			
            /* err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            } */
            break;
		case BSP_EVENT_KEY_1:
		
			
			
			break;
        default:
            break;
    }
}

/**@brief Function for initializing the UART.
 */

static void uart_init(void)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud9600
      };

    APP_UART_FIFO_INIT(&comm_params,
                        UART_RX_BUF_SIZE,
                        UART_TX_BUF_SIZE,
                        uart_event_handle,
                        APP_IRQ_PRIORITY_LOWEST,
                        err_code);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the NUS Client.
 */
static void nus_c_init(void)
{
    uint32_t         err_code;
    ble_nus_c_init_t nus_c_init_t;

    nus_c_init_t.evt_handler = ble_nus_c_evt_handler;
	
	
	for(m_ble_nus_c_count = 0; m_ble_nus_c_count < TOTAL_LINK_COUNT; m_ble_nus_c_count++)
	{
		err_code = ble_nus_c_init(&m_ble_nus_c[m_ble_nus_c_count], &nus_c_init_t);
		APP_ERROR_CHECK(err_code);
	}
	m_ble_nus_c_count = 0;
	
	
}


/**@brief Function for initializing buttons and leds.
 */
static void buttons_leds_init(void)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the Database Discovery Module.
 */
static void db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init(db_disc_handler);
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
				
		case BLE_ADV_EVT_DIRECTED:
            NRF_LOG_INFO("BLE_ADV_EVT_DIRECTED\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            APP_ERROR_CHECK(err_code);
            break; 
				
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
				
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
				
				case BLE_ADV_EVT_FAST_WHITELIST:
            NRF_LOG_INFO("BLE_ADV_EVT_FAST_WHITELIST\r\n");
					NRF_LOG_INFO("BLE_ADV_EVT_FAST_WHITELIST\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
						break; 
			
				case BLE_ADV_EVT_SLOW_WHITELIST:
            NRF_LOG_INFO("BLE_ADV_EVT_SLOW_WHITELIST\r\n");
					NRF_LOG_INFO("BLE_ADV_EVT_SLOW_WHITELIST\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            break; 
				
				
        default:
            break;
    }
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
						
																	
						NRF_LOG_INFO("	Byte0		0x%02x\n\r",p_data[0]);
						NRF_LOG_INFO("	Byte1		0x%02x\n\r",p_data[1]);
						NRF_LOG_INFO("	Byte2				0x%02x\n\r",p_data[2]);
						NRF_LOG_INFO("	Byte3				0x%02x\n\r",p_data[3]);
						NRF_LOG_INFO("	Byte4			0x%02x\n\r",p_data[4]);
						NRF_LOG_INFO("	Byte5		0x%02x\n\r",p_data[5]);
						NRF_LOG_INFO("	Byte6			0x%02x\n\n\r",p_data[6]);			
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
		options.ble_adv_whitelist_enabled =true;
		options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;
		

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
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
    cp_init.disconnect_on_fail             = true;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Thread for handling the Application's BLE Stack events.
 *
 * @details This thread is responsible for handling BLE Stack events sent from on_ble_evt().
 *
 * @param[in]   arg    Pointer used for passing some arbitrary information (context) from the
 *                     osThreadCreate() call to the thread.
 */
static void ble_stack_thread(void * arg)
{
  uint32_t err_code;

	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	
	UNUSED_VARIABLE(arg);
		
	uart_init();
	err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);
			
	buttons_leds_init();
	db_discovery_init();
	ble_stack_init();
	nus_c_init();
	gap_params_init();
	services_init();
	advertising_init();
	conn_params_init();
	
	
    // Start scanning for peripherals and initiate connection
    // with devices that advertise NUS UUID.
	scan_start();
	
	 err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
	 APP_ERROR_CHECK(err_code);
	

//    //for(;;)
		while(1)
    {
        /* Wait for event from SoftDevice */
        while (pdFALSE == xSemaphoreTake(m_ble_event_ready, portMAX_DELAY))
        {
            // Just wait again in the case when INCLUDE_vTaskSuspend is not enabled
        }

        // This function gets events from the SoftDevice and processes them by calling the function
        // registered by softdevice_ble_evt_handler_set during stack initialization.
        // In this code ble_evt_dispatch would be called for every event found.
        intern_softdevice_events_execute();
    }
}

static void uart_stack_thread(void * arg)
{
	UNUSED_PARAMETER(arg);

	//for(;;)
	while(1)
	{
		//Do something cool 
		 vTaskSuspend(NULL);
	}
}

#if NRF_LOG_ENABLED
/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    //for(;;)
		while(1)
    {
				//NRF_LOG_INFO("HeapSize is: %i\r\n",xPortGetFreeHeapSize());
			
        NRF_LOG_FLUSH();
        vTaskSuspend(NULL); // Suspend myself
    }
}
#endif //NRF_LOG_ENABLED

/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
     vTaskResume(m_logger_thread);
	 
}


/**@brief Function for application main entry.
 */
int main(void)
{
	
    
	ret_code_t err_code;
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
		
    // Do not start any interrupt that uses system functions before system initialisation.
    // The best solution is to start the OS before any other initalisation.

    // Init a semaphore for the BLE thread.
    m_ble_event_ready = xSemaphoreCreateBinary();
    if (NULL == m_ble_event_ready)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
		
		//Init a semaphore for the Uart thread
		m_uart_event_ready = xSemaphoreCreateBinary();
		if (NULL == m_uart_event_ready)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
		
		//Init a mutex for the uart data module/thread. 
		uart_data = xSemaphoreCreateMutex();
		if (NULL == uart_data)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }


    // Start execution.
    if (pdPASS != xTaskCreate(ble_stack_thread, "BLE", 256, NULL, 3, &m_ble_stack_thread))  //This task must always have the highest order
    {	
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
		
		if(pdPASS != xTaskCreate(uart_stack_thread, "UART", 256, NULL, 2, &m_uart_stack_thread))   //Init of the uart thread task
		{
			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
		}

#if NRF_LOG_ENABLED
    // Start execution.
    if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 256, NULL, 1, &m_logger_thread))     //Init of the task to spit out uart. 
    {
			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif //NRF_LOG_ENABLED

    /* Activate deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
		
		
    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    //for(;;)
		while(true)
    {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);  //Should not go here
    }
}


