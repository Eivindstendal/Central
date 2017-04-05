
#ifndef M_BUS_RECEIVER
#define M_BUS_RECEIVER

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*
Initialisation definitions
*/
#define START_INIT 0x10  										//Start field value 
#define SEND_OR_REPLY_INIT 0x40							//Send or reply, reset- field value
#define STOP_INIT 0x16 											//Stop field value.

#define TOTAL_LENGTH_INIT_FRAME 0x05				//Total length of the init telegram
#define RESPONSE_INIT 0XE5									//Response from m_bus after initialisation

/*
Changing primary address definitions
*/
#define START_CHANGING_ADDR 0x68						//Start field value
#define FIELD_LENGHT_CHANGING_ADDR 0x06			//Field Length value
#define C_FIELD_CHANGING_ADDR 0x68					//C field value
#define CI_FIELD_CHANGING_ADDR 0x51         //CI field value
#define DIF_FIELD_CHANGING_ADDR 0x01				//DIF field value
#define VIF_FIELD_CHANGING_ADDR 0x7A				//VIF field value
#define STOP_FIELD_CHANGING_ADDR 0x16				//Stop field value

#define TOTAL_LENGTH_CHANGING_ADDR 0x0C			//Total length of the changing primary address telegram
#define RESPONSE_CHANGING_ADDR 0xE5					//Response from m_bus after changing the primary address.

/*
Reset ACC(application reset)
*/
#define START_RESET_ACC 0x68								//Start field value
#define FIELD_LENGTH_RESET_ACC 0x03					//Field length value
#define C_FIELD_RESET_ACC 0x53							//C field value
#define CI_FIELD_RESET_ACC 0x50							//CI field value
#define STOP_FIELD_RESET_ACC 0x16						//Stop field value

#define TOTAL_LENGTH_RESET_ACC 0x09					//Total length of the application reset telegrame
#define RESPONSE_RESET_ACC 0xE5							//Response from m_bus after application reset

/*
Reset total partial power
*/
#define START_RESET_PARTIAL 0x68						//Start field value
#define FIELD_LENGTH_RESET_PARTIAL 0x04			//Field length value
#define C_FIELD_RESET_PARTIAL 0x53					//C field value
#define CI_FIELD_RESET_PARTIAL 0x50					//CI field value
#define RESET_COUNTER_PARTIAL 0x01					//Reset counter field value
#define STOP_FIELD_RESET_PARTIAL 0x16				//Stop field value

#define TOTAL_LENGTH_RESET_PARTIAL 0x0A			//Total length of the partial power reset telegram
#define RESPONSE_RESET_PARTIAL 0xE5					//Response from m_bus after partial power reset.

/*
REQ_UD2: Definitions for requesting data from m_bus receiver
*/
#define START_REQUEST 0x10									//Start field value
#define C_FIELD_FCB_NOT_SET_REQUEST 0x5B		//C field value with FCB bit set to 0
#define C_FIELD_FCB_SET_REQUEST 0x7B				//C field value with FCB bit set to 1
#define STOP_REQUESET 0x16									//Stop field value

#define TOTAL_LENGHT_REQUEST 0x05						//Totalt length of the REQ_UD2 telegram.
//Response from REQ_UD2 is a 62 bytes telegram from the m_bus receiver. 

/*
Definition for response telegram from m_bus after REQ_UD2 reguest.
*/
#define RESPONSE_START_FIELD 0x68        		//1 and 4 byte in the telegram
#define RESPONSE_L_READ 0x38								//2 and 3 byte in the telegram
#define RESPONSE_STOP_FIELD 0x16						//62 byte in the telegram



/*
A definition of the adresse field. 
*/
#define A_FIELD 0xAA												//Address field. 

/*
States used in the uart stack thread
*/
typedef enum
{
	INIT_M_BUS_STATE,
	RESET_ACC,
	RESET_PARTIAL_STATE,
	WAITING_RESPONSE_STATE,
	READING_M_BUS_RESPONSE
} uart_event_states;

typedef enum
{
	LAST_INIT_M_BUS_STATE,
	LAST_RESET_ACC,
	LAST_RESET_PARTIAL_STATE,
} last_uart_event_states;


/*
Structure to save the values from the m_bus receiver. 
*/
struct aMessage
{
	uint32_t Message_number;
	uint8_t STAT;
	uint32_t Total_power;
	uint32_t Partial_power;
	uint16_t Voltage;
	uint16_t Current;
	uint16_t Power; //Denne
	uint16_t Reactive_power;
} xMessage;



/**@brief Function for initializing the m-bus receiver
 *
 * @details IMPORTENT! Must be protected with a mutex
 *          
 *        
 *
 * @param[in]   adr_off_m_bus      The adresse that we want to set the m-bus receiver to
 *
 */
void m_bus_receiver_init(uint8_t adr_off_m_bus);


/**@brief Function for changing the m-bus receivers primary address
 *
 * @details IMPORTENT! Must be protected with a mutex
 *          
 *        
 *
 * @param[in]   primary_adr_off_m_bus      The primary adress off the m-bus receiver
 * @param[in]   new_address_off_m_bus      The new adress that we want to set the m-bus receiver to
 *
 */
void m_bus_receiver_changing_primary_address(uint8_t primary_adr_off_m_bus, uint8_t new_address_off_m_bus);


/**@brief Function for resetting the m-bus receiver
 *
 * @details IMPORTENT! Must be protected with a mutex
 *          
 *        
 *
 * @param[in]   primary_adr_off_m_bus      The primary adress off the m-bus receiver
 *
 */
void m_bus_receiver_reset_application(uint8_t primary_adr_off_m_bus);


/**@brief Function for resetting the m-bus receiver partial power
 *
 * @details IMPORTENT! Must be protected with a mutex
 *          
 *        
 *
 * @param[in]   primary_adr_off_m_bus      The primary adress off the m-bus receiver
 *
 */
void m_bus_receiver_reset_partial_power(uint8_t primary_adr_off_m_bus);


/**@brief Function for requesting telegram from the uart module
 *
 * @details M_bus_receiver requires a REQ_UD2 query in order to send response. The response is a RSP_UD telegram 62 bytes.
 *          IMPORTENT! Must be protected with a mutex
 *        
 *
 * @param[in] adr_off_m_bus    The adress of the m_bus_receiver that we want response from.
 * @param[in] c_field          The c-field, with or without fcb set.     
 *
 */
 void m_bus_send_request(uint8_t primary_adr_off_m_bus, uint8_t c_field);
 
 
 /**@brief Function for checking response from the m_bus_receiver init
 *
 * @details Response from the m_bus_receiver should be 0xE5 after the initialisation, or all other response then RSP_UD.  
 *          (Don`t need to be protected with a mutex)
 *
 *
 * @param[in] exp_response    The expected response from the m_bus receiver
 *
 * @retval True if the response from m_bresponse_from_m_busus_receiver is 0xE5 (#define RESPONSE 0xE5)
 */
bool response_from_m_bus (uint8_t exp_response);

 /**@brief Function for checking response from the m_bus_receiver init
 *
 * @details Response from the m_bus_receiver should be 0xE5 after the initialisation, or all other response then RSP_UD.  
 *          (Don`t need to be protected with a mutex)
 *
 *
 * @param[in] exp_response    The expected response from the m_bus receiver
 *
 * @retval True if the response from m_bus_receiver is 0xE5 (#define RESPONSE 0xE5)
 */
bool telegram_structure_check (uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4);

 /**@brief Function for decoding bcd.
 *
 * @details   
 *          (Don`t need to be protected with a mutex)
 *
 *
 * @param[in] 
 *
 * @retval 
 */
uint8_t bcdtobyte(uint8_t bcd);
#endif //M_BUS_RECEIVER
