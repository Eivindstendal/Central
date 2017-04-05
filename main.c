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
#include "ble_nus.h"
#include "ble_nus_c.h"
#include "app_timer.h"
#include "app_util.h"
#include "peer_manager.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "FreeRTOS.h"
#include "event_groups.h"
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
#include "math.h"

#define  PREFERRED_CONSUME_LIMIT 	 3000													/**<kwt   Will turn off devices in low priority*/
#define  NORMAL_MAX_CONSUME_LIMIT  5000													/**<will turn off devices in low and medium priority*/
#define  MAX_CONSUME_LIMIT 	 			10000													/**<will turn off devices in low, medium and highest_priority*/
#define  BOILER_KW                 2000
#define  SLAVE_OFF_INTERVAL      	900000                             /**< Turn slave off after 15min (ms). */

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                      /**< Include the Service Changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */
#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT      7                               /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT   1                               /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#define TOTAL_LINK_COUNT          CENTRAL_LINK_COUNT + PERIPHERAL_LINK_COUNT /**< Total number of links used by the application. */
#define ELEMENTS_IN_MY_DATA_STRUCT		  7
#define SLAVE_TYPE						'A'																										/**< The type slave, capital letter means it has a temp sensor*/ 

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
#define CLOCK_UPDATE_INT       1000
#define ACK_WAIT_INTERVAL      3000

#define M_BUS_RECEIVER_INTERVAL      		2000//10000                             
#define OSTIMER_WAIT_FOR_QUEUE          	 10                              /**< Number of ticks to wait for the timer queue to be ready */


/* For slave config*/
#define DEVICE_NAME              		       "El_hub_central"  											  /**< Name of device. Will be included in the advertising data. */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */
#define APP_ADV_INTERVAL                64                      /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                     /**< The advertising timeout (in units of seconds). */
#define ACK_WAIT_INTERVAL      3000	



APP_TIMER_DEF(m_clock_timer);
APP_TIMER_DEF(ack_timer);
static ble_nus_c_t              m_ble_nus_c[TOTAL_LINK_COUNT];                    /**< Instance of NUS service. Must be passed to all NUS_C API calls. */
static ble_db_discovery_t       m_ble_db_discovery[TOTAL_LINK_COUNT];             /**< Instance of database discovery module. Must be passed to all db_discovert API calls */
static uint8_t           		m_ble_nus_c_count;  
static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */
static ble_nus_t                m_nus;                                      /**< Structure to identify the Nordic UART Service. */
//bool waiting_ack[CENTRAL_LINK_COUNT];  // Må endres til en semaphore

EventGroupHandle_t waiting_ack;

static SemaphoreHandle_t m_ble_event_ready;  /**< Semaphore raised if there is a new event to be processed in the BLE thread. */
static SemaphoreHandle_t uart_event_rx_ready; //Semaphore raised if there is a new event to be processed in the uart thread
static SemaphoreHandle_t uart_mutex_tx;          //Mutex for the uart module. 


static QueueHandle_t uart_event_queue;
static QueueHandle_t m_bus_adr_queue;
static QueueHandle_t clock_hour;

static TaskHandle_t m_ble_stack_thread;      /**< Definition of BLE stack thread. */
static TaskHandle_t m_uart_stack_thread;     //Task for the uart thread.
static TaskHandle_t m_logger_thread;         /**< Definition of Logger thread. */
static TaskHandle_t m_controller_task;        //Task that controlls everything.

static TimerHandle_t m_bus_receiver_timer;   //Definition of the m_bus_receiver timer.
static TimerHandle_t slave_off_timer;

static uint8_t curr_connection = 0; // er det greit å bruke global variabel?


static const char m_target_periph_name[] = "7dk29kshnsdc";          /**< If you want to connect to a peripheral using a given advertising name, type its name here. */


/**@brief Variable length data encapsulation in terms of length and pointer to data */
typedef struct
{
    uint8_t  * p_data;      /**< Pointer to data. */
    uint16_t   data_len;    /**< Length of data. */
} data_t;


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
	uint8_t ack;										/**< etc */		
	uint8_t state;										/**< etc */
	int8_t wanted_temp;												/**< Integer part of extern temp sensor on NRF52 */
	int8_t current_temp;										/**< Fractional part of extern temp semsor on NRF52 */
	uint8_t priority;										/**< The priority of the slave device */
}my_data;
static my_data slave_data[CENTRAL_LINK_COUNT];


///**
// * @brief NUS uuid
// */
//static const ble_uuid_t m_nus_uuid =
//  {
//    .uuid = BLE_UUID_NUS_SERVICE,
//    .type = NUS_SERVICE_UUID_TYPE
//  };

  
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
		NRF_LOG_INFO("Uart c Scan started\r\n");
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
   

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
							UNUSED_VARIABLE(xSemaphoreGiveFromISR(uart_event_rx_ready, NULL));
            break;
        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
						NRF_LOG_INFO("Com error %0x,%i?\r\n\r\n\r\n", p_event->data.error_communication,p_event->data.error_communication);
            APP_ERROR_HANDLER(p_event->data.error_communication);  //Her må det lages noe annet. 
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;
				case APP_UART_TX_EMPTY:
						//xSemaphoreGiveFromISR(uart_mutex_tx, NULL); Denne må ikke brukes. 
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
	EventBits_t bits = xEventGroupGetBits(waiting_ack);
	
	data[0] = SLAVE_TYPE;
	data[1] = slave_data[slave_nr].address;
	data[2] = 0;
	data[3] = slave_data[slave_nr].state;
	data[4] = slave_data[slave_nr].wanted_temp;
	data[5] = slave_data[slave_nr].current_temp;
	data[6] = slave_data[slave_nr].priority;
		
	if(1 == (bits & (1 << slave_nr)))// checking if we are waiting on ack    // if(true == waiting_ack[slave_nr])							
	{		
		NRF_LOG_INFO("	Sending not complete, waiting on ack\r\n");
		return false;
	}
			err_code = ble_nus_c_string_send(&m_ble_nus_c[slave_nr],data,ELEMENTS_IN_MY_DATA_STRUCT);
			APP_ERROR_CHECK(err_code);
			NRF_LOG_INFO("	send_string err_code %d \n\r",err_code);
	
	
	if(err_code == NRF_SUCCESS )
	{
			NRF_LOG_INFO("	sending complete \n\r");
					//waiting_ack[slave_nr]= true;
					xEventGroupSetBits(waiting_ack, 1 << slave_nr );		
					err_code = app_timer_start(ack_timer, 
															 APP_TIMER_TICKS(ACK_WAIT_INTERVAL, 
															 APP_TIMER_PRESCALER),
																			NULL);
			return true;
	}
	else
		{
			NRF_LOG_INFO("	sending not complete \n\r");
			
			
			return false;
		}	
}


/**@brief Function to send ack over Nordic Uart serial
 * @param[in]  What slave data to send
 * @param[out] bool true or false, sending successful not not
 */
bool send_ack(uint8_t slave_nr)
{
	uint8_t data[ELEMENTS_IN_MY_DATA_STRUCT];
	uint32_t err_code;
	
	data[0] = SLAVE_TYPE;
	data[1] = slave_data[slave_nr].address;
	data[2] = 1;
	data[3] = slave_data[slave_nr].state;
	data[4] = slave_data[slave_nr].wanted_temp;
	data[5] = slave_data[slave_nr].current_temp;
	data[6] = slave_data[slave_nr].priority;

			err_code = ble_nus_c_string_send(&m_ble_nus_c[slave_nr],data,ELEMENTS_IN_MY_DATA_STRUCT);
			APP_ERROR_CHECK(err_code);
	
	if(err_code == NRF_SUCCESS )
	{
			NRF_LOG_INFO("	ack sent \n\r");		
			return true;
	}
	else
		{
			NRF_LOG_INFO("	ack failed \n\r");			
			return false;
		}	
}

 
/**@brief Function to print data to slaves
 * 
 *  
 */
void print_slave_data(void)
{
	for(int i=0; i<=3;i++)
	{
		NRF_LOG_INFO("	Type:				%c\n\r",slave_data[i].type);
		NRF_LOG_INFO("	Address:			%d\n\r",slave_data[i].address);
		NRF_LOG_INFO("	ack:				%d\n\r",slave_data[i].ack);
		NRF_LOG_INFO("	state:				%d\n\r",slave_data[i].state);
		NRF_LOG_INFO("	Wanted_temp:			%d\n\r",slave_data[i].wanted_temp);
		NRF_LOG_INFO("	Current_temp:			%d\n\r",slave_data[i].current_temp);
		NRF_LOG_INFO("	Priority:			%d\n\n\r",slave_data[i].priority);
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
					if('B' == p_ble_nus_evt->p_data[0]||
																							'b' == p_ble_nus_evt->p_data[0]||
																							'C' == p_ble_nus_evt->p_data[0]||
																							'c' == p_ble_nus_evt->p_data[0])
					{
						
							NRF_LOG_INFO("	Recieved data from slave \r\n");
							slave_data[curr_connection].type = p_ble_nus_evt->p_data[0]; 
							slave_data[curr_connection].address = curr_connection;
							slave_data[curr_connection].ack = p_ble_nus_evt->p_data[2];
							slave_data[curr_connection].state = p_ble_nus_evt->p_data[3]; 
							//slave_data[curr_connection].wanted_temp = p_ble_nus_evt->p_data[4]; 		
							slave_data[curr_connection].current_temp = p_ble_nus_evt->p_data[5]; 
							slave_data[curr_connection].priority = p_ble_nus_evt->p_data[6];
						
						
						if(0 == p_ble_nus_evt->p_data[2])
						{
							send_ack(curr_connection);
														
						}
						else if(1== p_ble_nus_evt->p_data[2])
						{
							
							NRF_LOG_INFO("ack recieved \r\n");
							app_timer_stop(ack_timer);
							//waiting_ack[curr_connection] = false;
							xEventGroupClearBits(waiting_ack, 1 << curr_connection );												
						}
					}					
						print_slave_data();
								
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected\r\n");
						NRF_LOG_INFO("Denne?\r\n");
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


/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  Type of data to be looked for in advertisement data.
 * @param[in]  Advertisement report length and pointer to report.
 * @param[out] If data type requested is found in the data report, type data length and
 *             pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index + 2];
            p_typedata->data_len = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}


///**@brief Reads an advertising report and checks if a uuid is present in the service list.
// *
// * @details The function is able to search for 16-bit, 32-bit and 128-bit service uuids.
// *          To see the format of a advertisement packet, see
// *          https://www.bluetooth.org/Technical/AssignedNumbers/generic_access_profile.htm
// *
// * @param[in]   p_target_uuid The uuid to search fir
// * @param[in]   p_adv_report  Pointer to the advertisement report.
// *
// * @retval      true if the UUID is present in the advertisement report. Otherwise false
// */
//static bool is_uuid_present(const ble_uuid_t *p_target_uuid,
//                            const ble_gap_evt_adv_report_t *p_adv_report)
//{
//    uint32_t err_code;
//    uint32_t index = 0;
//    uint8_t *p_data = (uint8_t *)p_adv_report->data;
//    ble_uuid_t extracted_uuid;

//    while (index < p_adv_report->dlen)
//    {
//        uint8_t field_length = p_data[index];
//        uint8_t field_type   = p_data[index + 1];

//        if ( (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE)
//           || (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE)
//           )
//        {
//            for (uint32_t u_index = 0; u_index < (field_length / UUID16_SIZE); u_index++)
//            {
//                err_code = sd_ble_uuid_decode(  UUID16_SIZE,
//                                                &p_data[u_index * UUID16_SIZE + index + 2],
//                                                &extracted_uuid);
//                if (err_code == NRF_SUCCESS)
//                {
//                    if ((extracted_uuid.uuid == p_target_uuid->uuid)
//                        && (extracted_uuid.type == p_target_uuid->type))
//                    {
//                        return true;
//                    }
//                }
//            }
//        }

//        else if ( (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_MORE_AVAILABLE)
//                || (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_COMPLETE)
//                )
//        {
//            for (uint32_t u_index = 0; u_index < (field_length / UUID32_SIZE); u_index++)
//            {
//                err_code = sd_ble_uuid_decode(UUID16_SIZE,
//                &p_data[u_index * UUID32_SIZE + index + 2],
//                &extracted_uuid);
//                if (err_code == NRF_SUCCESS)
//                {
//                    if ((extracted_uuid.uuid == p_target_uuid->uuid)
//                        && (extracted_uuid.type == p_target_uuid->type))
//                    {
//                        return true;
//                    }
//                }
//            }
//        }

//        else if ( (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE)
//                || (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE)
//                )
//        {
//            err_code = sd_ble_uuid_decode(UUID128_SIZE,
//                                          &p_data[index + 2],
//                                          &extracted_uuid);
//            if (err_code == NRF_SUCCESS)
//            {
//                if ((extracted_uuid.uuid == p_target_uuid->uuid)
//                    && (extracted_uuid.type == p_target_uuid->type))
//                {
//                    return true;
//                }
//            }
//        }
//        index += field_length + 1;
//    }
//    return false;
//}


/**@brief Function for searching a given name in the advertisement packets.
 *
 * @details Use this function to parse received advertising data and to find a given
 * name in them either as 'complete_local_name' or as 'short_local_name'.
 *
 * @param[in]   p_adv_report   advertising data to parse.
 * @param[in]   name_to_find   name to search.
 * @return   true if the given name was found, false otherwise.
 */
static bool find_adv_name(const ble_gap_evt_adv_report_t *p_adv_report, const char * name_to_find)
{
    uint32_t err_code;
    data_t   adv_data;
    data_t   dev_name;

    // Initialize advertisement report for parsing
    adv_data.p_data     = (uint8_t *)p_adv_report->data;
    adv_data.data_len   = p_adv_report->dlen;


    //search for advertising names
    err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
                                &adv_data,
                                &dev_name);
    if (err_code == NRF_SUCCESS)
    {
        if (memcmp(name_to_find, dev_name.p_data, dev_name.data_len )== 0)
        {
            return true;
        }
    }
    else
    {
        // Look for the short local name if it was not found as complete
        err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
                                    &adv_data,
                                    &dev_name);
        if (err_code != NRF_SUCCESS)
        {
            return false;
        }
        if (memcmp(m_target_periph_name, dev_name.p_data, dev_name.data_len )== 0)
        {
            return true;
        }
    }
    return false;
}

	


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event
 */
static void on_ble_central_evt(ble_evt_t * p_ble_evt)
{
    uint32_t              err_code;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            bool do_connect = false;
						const ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;
						

					
						if (find_adv_name(&p_gap_evt->params.adv_report, m_target_periph_name))
            {
                 
							NRF_LOG_INFO("Name match.\r\n");
							do_connect = true;
						}
									
						
						if(do_connect)
						{
							NRF_LOG_INFO("Connecting :D \r\n");
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

/**@brief Function for handling BLE Stack events involving peripheral applications. Manages the
 * LEDs used to report the status of the peripheral applications.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_peripheral_evt(ble_evt_t * p_ble_evt)
{
    ret_code_t err_code;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Phone connected\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            break; //BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Phone disconnected\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            break;//BLE_GAP_EVT_DISCONNECTED

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;//BLE_EVT_USER_MEM_REQUEST

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
    uint16_t role;
		
		ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
	
		conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		role        = ble_conn_state_role(conn_handle);
		
		if (role == BLE_GAP_ROLE_PERIPH)
    {
			// Manages peripheral LEDs.
			on_ble_peripheral_evt(p_ble_evt);
			
			ble_conn_params_on_ble_evt(p_ble_evt);
			ble_advertising_on_ble_evt(p_ble_evt);	
			 // Dispatch to peripheral applications.
			ble_nus_on_ble_evt(&m_nus, p_ble_evt);
			
		}else if ((role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT))
		{
				if (p_ble_evt->header.evt_id != BLE_GAP_EVT_DISCONNECTED)
        {
            on_ble_central_evt(p_ble_evt);
        }
				
				bsp_btn_ble_on_ble_evt(p_ble_evt);
				
		
				// Make sure that an invalid connection handle are not passed since
				// our array of modules is bound to TOTAL_LINK_COUNT.
				if (conn_handle < TOTAL_LINK_COUNT)
				{
						ble_db_discovery_on_ble_evt(&m_ble_db_discovery[conn_handle], p_ble_evt);
						ble_nus_c_on_ble_evt(&m_ble_nus_c[conn_handle], p_ble_evt);
				}
				
				//ble_nus_on_ble_evt(&m_nus, p_ble_evt);
				
				if (p_ble_evt->header.evt_id == BLE_GAP_EVT_DISCONNECTED)
        {
            on_ble_central_evt(p_ble_evt);
        }
			
		}
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
{		
	NRF_LOG_INFO("bps_event_handler \r\n\n\n");
    uint32_t err_code;
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
		
					NRF_LOG_INFO("Start adverticing! \r\n");
					err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
					APP_ERROR_CHECK(err_code);
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
        .use_parity   = true,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud2400
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


/**@brief Function for finding lowest priority slave that is still in ON state.
 *
 * @details 
 *
 * @param[out]   arg   If there is no slave in an active state 0xFF is returned, otherwise slave_lowest_priority is returned.
 */
uint8_t find_lowest_priority(void)
{
	uint8_t slave_lowest_priority =0xFF;
	uint8_t lowest_diff =100;
	uint8_t temp_diff;
	
	
	for(int i; i<= CENTRAL_LINK_COUNT; i++)
	{
		if(1 <= slave_data[i].state)
		{
			temp_diff = slave_data[i].wanted_temp - slave_data[i].current_temp;
			if(temp_diff < lowest_diff)
			{
				lowest_diff = temp_diff;
				slave_lowest_priority = i;
			}
		}
	}
	if(0xFF == slave_lowest_priority)
		NRF_LOG_INFO("Can not find any slaves in active modus \r\n");
	
	
	return slave_lowest_priority;
}


/**@brief Function for finding highest priority slave that is in OFF state.
 *
 * @details 
 *
 * @param[out]   arg   If there is no slave in an off state 0xFF is returned, otherwise slave_lowest_priority is returned.
 */
uint8_t find_highest_priority(void)
{
	uint8_t slave_highest_priority =0xFF;
	uint8_t highest_priority;
	
	for(int i; i<= CENTRAL_LINK_COUNT; i++)
	{
		if(100 > slave_data[i].state && slave_data[i].priority < highest_priority)
		{
				slave_highest_priority = i;
				highest_priority = slave_data[i].priority;
			
		}
	}
	
	if(0xFF == slave_highest_priority)
		NRF_LOG_INFO("Can not find any slaves in active modus \r\n");
			
	
	return slave_highest_priority;
}


/**@brief 
 *
 * @details 
 *
 * @param[in] xTimer Handler to the timer that called this function.
 *                   You may get identifier given to the function xTimerCreate using pvTimerGetTimerID.
 */
static void slave_off_timer_receiver_timeout(TimerHandle_t xTimer)
{
	UNUSED_PARAMETER(xTimer);
	static struct aMessage *my_rx_message;
	uint8_t slave_nr = find_highest_priority();
	uint32_t consume_diff;
	
  if(xQueueReceive(uart_event_queue, &(my_rx_message), ( TickType_t ) 10 ))
	{
		if(my_rx_message->Power  < PREFERRED_CONSUME_LIMIT)
		{	
			
			if(slave_nr != 0xFF)
			{
				switch (slave_data[slave_nr].type)
				{
					case 'b':
					break;
							
					case 'B':
						slave_data[slave_nr].state = 100;				
					break;
					//Oven with temp sensor
							
					case 'c':					
					break;
					//Oven without temp sensor
							
					case 'C': 
						consume_diff = PREFERRED_CONSUME_LIMIT - my_rx_message->Power;
						if(consume_diff>(BOILER_KW))	
						{
							slave_data[slave_nr].state = 100;
						}	
						else 
						{
							slave_data[slave_nr].state = floor(((10*consume_diff)/BOILER_KW)*10); 
						}		
					//Boiler with temp sensor	
					break;
							
					case 'D':					
					break;
							
					default:
					break;
				
				
				}
				send_data(slave_nr);
			}
		}else
		{
			if(pdPASS != xTimerStart(slave_off_timer, OSTIMER_WAIT_FOR_QUEUE))
			{
				APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
			}
		}
	}
}


static void controller_task (void * arg)
{
	UNUSED_PARAMETER(arg);
	static struct aMessage *my_rx_message;
	static uint32_t consume_diff;
	static uint8_t hour;
	
	static uint8_t boiler_kw = 2; // 2 KW
	static uint8_t slave_nr;
	static uint8_t lowest_priority_to_turn_off;
	
		
	while(1)
	{
		if( uart_event_queue != 0 )
		{
				if(xQueueReceive(clock_hour, &(hour), ( TickType_t ) 10 ))
				{}
				

				if(xQueueReceive(uart_event_queue, &(my_rx_message), ( TickType_t ) 10 ))
				{
					//NRF_LOG_INFO("Number: %i, Stat %0x, Voltage: %i\r\n", my_rx_message->Message_number, my_rx_message->STAT, my_rx_message->Voltage);
					//NRF_LOG_INFO("Current: %i, Power: %i, Power tot: %i\r\n", my_rx_message->Current, my_rx_message->Power, my_rx_message->Total_power);
					lowest_priority_to_turn_off =19;
		
					if((my_rx_message->Power > PREFERRED_CONSUME_LIMIT )||( 16 <= hour && 19 > hour )||( 6 <= hour && 8 > hour))
					{
						slave_nr = find_lowest_priority();
							
						if(my_rx_message->Power > NORMAL_MAX_CONSUME_LIMIT)
						{
							lowest_priority_to_turn_off = 9;
								
						}else if(my_rx_message->Power > MAX_CONSUME_LIMIT)
						{
							lowest_priority_to_turn_off = 1;
						}
							
										
						if( 0xFF != slave_nr && lowest_priority_to_turn_off < slave_data[slave_nr].priority)
						{
							switch (slave_data[slave_nr].type)
							{
								case 'b':
								break;
									
								case 'B':
									
									slave_data[slave_nr].state = 0;
																	
								break;
								//Oven with temp sensor
									
								case 'c':					
								break;
								//Oven without temp sensor
									
								case 'C': 
									consume_diff = my_rx_message->Power - PREFERRED_CONSUME_LIMIT;
									if(consume_diff > boiler_kw)	
									{
										slave_data[slave_nr].state = 0;
									}	
									else 
									{
										slave_data[slave_nr].state = floor(((10*consume_diff)/BOILER_KW)*10);  
									}
								//Boiler with temp sensor	
								break;
									
								case 'D':	
								// Temp sensor
								break;
									
								default:
								break;
									
							}
								send_data(slave_nr);
								if(pdPASS != xTimerStart(slave_off_timer, OSTIMER_WAIT_FOR_QUEUE))
								{
									APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
								}
								
						}
							
					}
				}
		}
	}
}


/**@brief Function for initializing buttons and leds.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;
		uint32_t err_code;
    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);
	
    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
		
		*p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/** @brief Function for initializing the Database Discovery Module.
 */
static void db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief  Clock to controll time
 */
static void update_clock(void)
{	

	static uint8_t hour = 10;
	static uint8_t minutes = 36;
	static uint8_t seconds =0;
	
	seconds++;
	if(60<=seconds)
	{
		//send_data(curr_connection);
		seconds =0;
		minutes++;
		print_slave_data();
		if(60<=minutes)
		{
			minutes = 0;
			hour++;
			
			if(24<= hour)
			{
				hour = 0;
			}
		}	
		NRF_LOG_INFO("The time is: %02d:%02d \r\n",hour,minutes);

	}	
		xQueueSend( clock_hour, ( void * ) &hour, ( TickType_t ) 100 );
}


/**@brief  Timeout handler for the repeated timer
 */
static void timer_handler(void * p_context)
{	
    update_clock();
}


/**@brief  Timeout handler for the ack timer
 */
static void ack_timer_handler(void * p_context)
{	
	EventBits_t bits = xEventGroupGetBits(waiting_ack);
	
	for(int i =0; i<=PERIPHERAL_LINK_COUNT;i++ )
	{
		if(1 == (bits & (1 << i)))					//if(true == waiting_ack[i])
			send_data(i);			
	}
}


/**@brief Create timers.
 */
static void create_timers()
{   
    uint32_t err_code;
	
    err_code = app_timer_create(&m_clock_timer, APP_TIMER_MODE_REPEATED, timer_handler);
		APP_ERROR_CHECK(err_code);
		err_code = app_timer_create(&ack_timer, APP_TIMER_MODE_SINGLE_SHOT,ack_timer_handler);
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
		//uint32_t err_code;			
		uint8_t temp;
		uint8_t slave_nr;
	/* 	uint8_t tall;
		uint8_t tall2; */
		
						/* NRF_LOG_INFO("	Byte0		%c\n\r",p_data[0]);
						NRF_LOG_INFO("	Byte1		%c\n\r",p_data[1]);
						NRF_LOG_INFO("	Byte2				%c\n\r",p_data[2]);
						NRF_LOG_INFO("	Byte3				%c\n\r",p_data[3]);
						NRF_LOG_INFO("	Byte4			%c\n\r",p_data[4]);
						NRF_LOG_INFO("	Byte5		%c\n\r",p_data[5]);
						NRF_LOG_INFO("	Byte6			%c\n\n\r",p_data[6]);		
						NRF_LOG_INFO("	Byte7		%c\n\r",p_data[7]);
						NRF_LOG_INFO("	Byte8		%c\n\r",p_data[8]);
						NRF_LOG_INFO("	Byte9				%c\n\r",p_data[9]);
						NRF_LOG_INFO("	Byte10				%c\n\r",p_data[10]);
						NRF_LOG_INFO("	Byte11		%c\n\r",p_data[11]);
						NRF_LOG_INFO("	Byte12  	%c\n\r",p_data[12]);
						NRF_LOG_INFO("	Byte13			%c\n\n\r",p_data[13]);
						NRF_LOG_INFO("	Byte14			%c\n\r",p_data[14]);
						NRF_LOG_INFO("	Byte15		%c\n\r",p_data[15]);
						NRF_LOG_INFO("	Byte16			%c\n\n\r",p_data[16]);		
						NRF_LOG_INFO("	Byte17		%c\n\r",p_data[17]);
						NRF_LOG_INFO("	Byte18		%c\n\r",p_data[18]);
						NRF_LOG_INFO("	Byte19				%c\n\r",p_data[19]); */
						
	if('s'==p_data[0]&&'l'==p_data[1]&& 'a'== p_data[2]&& 'v'==p_data[3]&& 'e'== p_data[4])
	{
		slave_nr = (p_data[5]-'0')*10 + (p_data[6]-'0');
		temp = (p_data[7]-'0')*10 + (p_data[8]-'0');
		NRF_LOG_INFO("slave_nr: %d temp: %d \r\n",slave_nr,temp);
		if('-' == p_data[9])
		{
			slave_data[slave_nr].wanted_temp = -temp;
		}else
			slave_data[slave_nr].wanted_temp = temp;
				
		send_data(slave_nr);
		
	}
	else
	{
		
	}
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
		

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL); // NULL chaged from &scanrsp
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

/**@brief Function for initiating advertising and scanning.
 */
static void adv_scan_start(void)
{
    ret_code_t err_code;
    uint32_t count;

    //check if there are no flash operations in progress
    err_code = fs_queued_op_count_get(&count);
    APP_ERROR_CHECK(err_code);

    if (count == 0)
    {
        // Start scanning for peripherals and initiate connection to devices which
        // advertise Heart Rate or Running speed and cadence UUIDs.
        scan_start();

        // Turn on the LED to signal scanning.
        bsp_board_led_on(BSP_INDICATE_SCANNING);
				
        // Start advertising.
        err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
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
            NRF_LOG_INFO("Connected to a previously bonded device.\r\n");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Link secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
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
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
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
            adv_scan_start();
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
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
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }NRF_LOG_FLUSH();
}

/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));


    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
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
	bool erase_bonds;
	
	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	buttons_leds_init(&erase_bonds);

	
	
		
	UNUSED_VARIABLE(arg);
		
		
		
		
	//Timer_clock	
	create_timers();
	err_code = app_timer_start(m_clock_timer, 
											 APP_TIMER_TICKS(CLOCK_UPDATE_INT, 
											 APP_TIMER_PRESCALER),
															NULL);	
	uart_init();
	err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);
			
	
	db_discovery_init();
	ble_stack_init();
	nus_c_init();
	gap_params_init();
	services_init();
	advertising_init();
	conn_params_init();
	peer_manager_init(erase_bonds);
	
	if (erase_bonds)
    {
        NRF_LOG_INFO("Bonds erased!\r\n");
    }
		
	// Start scanning for peripherals and initiate connection
		// with devices that advertise NUS UUID.
		adv_scan_start();
		
 


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

/**@brief 
 *
 * @details 
 *
 * @param[in] xTimer Handler to the timer that called this function.
 *                   You may get identifier given to the function xTimerCreate using pvTimerGetTimerID.
 */
static void m_bus_timer_receiver_timeout(TimerHandle_t xTimer)
{
  UNUSED_PARAMETER(xTimer);
	static uint8_t adr_of_mbus=0;
	//NRF_LOG_INFO("Timout %i\r\n", adr_of_mbus);

	if(m_bus_adr_queue!=0)
	{
		if(xQueueReceiveFromISR(m_bus_adr_queue, &(adr_of_mbus), NULL ))
		{
						
			if(xSemaphoreTake(uart_mutex_tx, (( TickType_t ) 10 ) == pdTRUE))
			{

				m_bus_send_request(adr_of_mbus, C_FIELD_FCB_NOT_SET_REQUEST);
							//SEGGER_RTT_WriteString(0,"Sendt, 	Her?");
				if ( xSemaphoreGive( uart_mutex_tx ) != pdTRUE )
				{
						 // We would not expect this call to fail because we must have
						 // obtained the semaphore to get here.
				}
			}
		}
		else
		{
						
			if(xSemaphoreTake(uart_mutex_tx, (( TickType_t ) 10 ) == pdTRUE))
			{
				m_bus_send_request(adr_of_mbus, C_FIELD_FCB_NOT_SET_REQUEST);
							//SEGGER_RTT_WriteString(0,"Sendt, 	Her?");
				if ( xSemaphoreGive( uart_mutex_tx ) != pdTRUE )
				{
						 // We would not expect this call to fail because we must have
						 // obtained the semaphore to get here.
				}
			}
		}
	}

	else
	{ 
		if(xSemaphoreTake(uart_mutex_tx, (( TickType_t ) 10 ) == pdTRUE))
		{
			m_bus_send_request((uint8_t) adr_of_mbus, C_FIELD_FCB_NOT_SET_REQUEST);
			
			if ( xSemaphoreGive( uart_mutex_tx ) != pdTRUE )
			{
           // We would not expect this call to fail because we must have
           // obtained the semaphore to get here.
			}
		}
	}
}



static void uart_stack_thread(void * arg)
{
	UNUSED_PARAMETER(arg);
	
	static struct aMessage *myMessage;
	static uint32_t message_counter = 0;
	static uint8_t data_array[62];
	static uint8_t counter = 0;
	static uart_event_states m_uart_event_states = RESET_ACC;
	static last_uart_event_states last_m_uart_event_state = LAST_RESET_ACC;
	static uint8_t adr_of_m_bus = 0;
	while(1)
	{
		
		switch(m_uart_event_states)
		{
			case INIT_M_BUS_STATE:
				
				if(xSemaphoreTake(uart_mutex_tx, 500) == pdTRUE)
				{
						m_bus_receiver_init(adr_of_m_bus); //Initialice the m-bus receiver
						NRF_LOG_INFO("INIT %i\r\n", adr_of_m_bus);
						if ( xSemaphoreGive( uart_mutex_tx ) != pdTRUE )
						{
                // We would not expect this call to fail because we must have
                // obtained the semaphore to get here.
						}
						last_m_uart_event_state = LAST_INIT_M_BUS_STATE;
						m_uart_event_states = WAITING_RESPONSE_STATE;
				}
				
				else
				{
					m_uart_event_states = INIT_M_BUS_STATE;
				}
				
				break;
				
				case RESET_ACC: //Resetting the device. Start up state. 
				if(xSemaphoreTake(uart_mutex_tx, 500 ) == pdTRUE)
				{
						//NRF_LOG_INFO("RESET %i\r\n", adr_of_m_bus);
						m_bus_receiver_reset_application(adr_of_m_bus); //Resetting the m-bus receiver
						if ( xSemaphoreGive( uart_mutex_tx ) != pdTRUE )
						{
                // We would not expect this call to fail because we must have
                // obtained the semaphore to get here.
						}
						last_m_uart_event_state = LAST_RESET_ACC;
						m_uart_event_states = WAITING_RESPONSE_STATE;
				}
				
				else
				{
					m_uart_event_states = RESET_ACC;
				}
				break;
	
			case RESET_PARTIAL_STATE:
				
				if(xSemaphoreTake(uart_mutex_tx, 500 ) == pdTRUE)
				{
						m_bus_receiver_reset_partial_power(adr_of_m_bus); //Initialice the m-bus receiver
						
						if ( xSemaphoreGive( uart_mutex_tx ) != pdTRUE )
						{
                // We would not expect this call to fail because we must have
                // obtained the semaphore to get here.
						}
						last_m_uart_event_state = LAST_RESET_PARTIAL_STATE;
						m_uart_event_states = WAITING_RESPONSE_STATE;
						
				}
				
				else
				{
					m_uart_event_states = RESET_PARTIAL_STATE;
				}
				break;
			
			case WAITING_RESPONSE_STATE:

						switch(last_m_uart_event_state)
						{
							case LAST_INIT_M_BUS_STATE:
								
								if(xSemaphoreTake(uart_event_rx_ready, 500) ==pdTRUE )
								{
									if(response_from_m_bus(RESPONSE_INIT))
									{
										if (pdPASS == xTimerStart(m_bus_receiver_timer, OSTIMER_WAIT_FOR_QUEUE))
										{
											if( m_bus_adr_queue != 0 )
											{
												xQueueSend( m_bus_adr_queue, ( void * ) &adr_of_m_bus, ( TickType_t ) 100 );
											}
											else
											{
												m_uart_event_states = INIT_M_BUS_STATE;
												break;
											}
										}
										else
										{
											APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
										}
										
										m_uart_event_states = READING_M_BUS_RESPONSE;
									}
									else
									{
										m_uart_event_states = INIT_M_BUS_STATE;
									}
								}
								
								else
								{
									m_uart_event_states = INIT_M_BUS_STATE;
								}
								break;

								
							case LAST_RESET_ACC:
								
								if(xSemaphoreTake(uart_event_rx_ready, 500) ==pdTRUE )
								{
									if(response_from_m_bus(RESPONSE_RESET_ACC))
									{
										m_uart_event_states = INIT_M_BUS_STATE;
										
									}
									else
									{
										adr_of_m_bus++;
										m_uart_event_states = RESET_ACC;
									}
								}
								else
								{
									adr_of_m_bus++;
									m_uart_event_states = RESET_ACC;
								}
								break;
								
								
								
							case LAST_RESET_PARTIAL_STATE:
								if(xSemaphoreTake(uart_event_rx_ready, 500) ==pdTRUE )
								{
									
									if(response_from_m_bus(RESPONSE_RESET_PARTIAL))
									{
										m_uart_event_states = READING_M_BUS_RESPONSE;
									}
									else
									{
										m_uart_event_states = RESET_PARTIAL_STATE;
									}
								}
								else
								{
									m_uart_event_states = RESET_PARTIAL_STATE;
								}
								break;
							default:
								break;
						}
				//vTaskDelay(10);
				break;
			
			case READING_M_BUS_RESPONSE: //Inne her må det skrives om litt. Må ha en sjekk før en teller opp til 61 tror jeg. Kanskje best å vente på 0x16?
				if(xSemaphoreTake(uart_event_rx_ready,portMAX_DELAY) ==pdTRUE )
				{
					UNUSED_VARIABLE(app_uart_get(&data_array[counter]));
					//if(data_array[0] == RESPONSE_START_FIELD)
					if(counter == 61)
					{
						if(data_array[0]!=RESPONSE_START_FIELD && data_array[1]!=RESPONSE_L_READ && data_array[2]!= RESPONSE_L_READ && data_array[61] != RESPONSE_STOP_FIELD)
						{
							while(app_uart_flush() !=NRF_SUCCESS);
							counter = 0; 
						}
						else
						{
							myMessage->Message_number = message_counter++;
							myMessage->STAT = data_array[16];
							
							myMessage->Total_power = bcdtobyte(data_array[22]) + (100*bcdtobyte(data_array[23])) + (10000*bcdtobyte(data_array[24])) + (1000000*bcdtobyte(data_array[25]));  //Denne 
							myMessage->Partial_power = bcdtobyte(data_array[29]) + (100*bcdtobyte(data_array[30])) + (10000*bcdtobyte(data_array[31])) + (1000000*bcdtobyte(data_array[32])); //Og denne er koded i bcb. 
							myMessage->Voltage = (data_array[38]) + (data_array[39]<<8);
							myMessage->Current = data_array[45] + (data_array[46]<<8);
							myMessage->Power = (data_array[51] + (data_array[52]<<8))*10;
							myMessage->Reactive_power = data_array[58] + (data_array[59]<<8);
							
							if( uart_event_queue != 0 )
							{
								myMessage = &xMessage;
								xQueueSend( uart_event_queue, ( void * ) &myMessage, ( TickType_t ) 0 );
							}
							counter = 0;
						}
						vTaskDelay(1);
						break;
					}
					counter++;
				}
				vTaskDelay(1);
				break;
						
			default:
				vTaskDelay(1);
				break;
			}
		
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
		uart_event_rx_ready = xSemaphoreCreateBinary();
		if (NULL == uart_event_rx_ready)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
		
		//Init a mutex for the uart data module/thread. 
		uart_mutex_tx = xSemaphoreCreateMutex();
		if (NULL == uart_mutex_tx)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
		
		waiting_ack = xEventGroupCreate();
    /* Was the event group created successfully? */
    if( waiting_ack == NULL )
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
		
		m_bus_adr_queue = xQueueCreate (1, sizeof (uint8_t));
		if (NULL == m_bus_adr_queue)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
		clock_hour = xQueueCreate (1, sizeof (uint8_t));
		if (NULL == clock_hour)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
		
    m_bus_receiver_timer = xTimerCreate("M_BUS", M_BUS_RECEIVER_INTERVAL, pdTRUE, NULL, m_bus_timer_receiver_timeout);
		if(NULL == m_bus_receiver_timer)
		{
			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
		}
		
		slave_off_timer = xTimerCreate("Slave_off_state", SLAVE_OFF_INTERVAL, pdTRUE, NULL, slave_off_timer_receiver_timeout);
		if(NULL == slave_off_timer)
		{
			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
		}
		
		uart_event_queue = xQueueCreate (1, sizeof(struct aMessage * ));
		if (NULL == uart_event_queue)
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
		if(pdPASS != xTaskCreate(controller_task, "controllerntask", 256, NULL, 2, &m_controller_task))   //Init of the uart thread task
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


