
#include <stdint.h>
#include <string.h>
#include "m_bus_receiver.h"
#include "app_uart.h"
#include "nrf_log.h"




void m_bus_receiver_init(uint8_t adr_off_m_bus)
{
	
	while(app_uart_flush() !=NRF_SUCCESS);

	
	uint8_t checksum = SEND_OR_REPLY_INIT+adr_off_m_bus;
	uint8_t data_array[TOTAL_LENGTH_INIT_FRAME]={START_INIT, SEND_OR_REPLY_INIT, adr_off_m_bus, checksum , STOP_INIT};
	
	
	for(uint8_t i=0; i<TOTAL_LENGTH_INIT_FRAME; i++)
	{
		while (app_uart_put( data_array[i]) != NRF_SUCCESS);
	}
}


void m_bus_receiver_changing_primary_address(uint8_t primary_adr_off_m_bus, uint8_t new_address_off_m_bus)
{
	
	while(app_uart_flush() !=NRF_SUCCESS);

	
	uint8_t checksum = C_FIELD_CHANGING_ADDR + CI_FIELD_CHANGING_ADDR + primary_adr_off_m_bus;
	
	uint8_t data_array[TOTAL_LENGTH_CHANGING_ADDR]={START_CHANGING_ADDR, FIELD_LENGHT_CHANGING_ADDR, FIELD_LENGHT_CHANGING_ADDR, START_CHANGING_ADDR, C_FIELD_CHANGING_ADDR,
	primary_adr_off_m_bus, CI_FIELD_CHANGING_ADDR, DIF_FIELD_CHANGING_ADDR, VIF_FIELD_CHANGING_ADDR, new_address_off_m_bus, checksum, STOP_FIELD_CHANGING_ADDR};
	
	
	for(uint8_t i=0; i<TOTAL_LENGTH_CHANGING_ADDR; i++)
	{
		while (app_uart_put( data_array[i]) != NRF_SUCCESS);
	}
}

void m_bus_receiver_reset_application(uint8_t primary_adr_off_m_bus)
{
	
	while(app_uart_flush() !=NRF_SUCCESS);

	
	uint8_t checksum = C_FIELD_RESET_ACC + CI_FIELD_RESET_ACC + primary_adr_off_m_bus;
	
	uint8_t data_array[TOTAL_LENGTH_RESET_ACC]={START_RESET_ACC, FIELD_LENGTH_RESET_ACC, FIELD_LENGTH_RESET_ACC, START_RESET_ACC, C_FIELD_RESET_ACC, primary_adr_off_m_bus,
	CI_FIELD_RESET_ACC, checksum, STOP_FIELD_RESET_ACC};
	
	
	for(uint8_t i=0; i<TOTAL_LENGTH_RESET_ACC; i++)
	{
		while (app_uart_put(data_array[i]) != NRF_SUCCESS);
	}
}


void m_bus_receiver_reset_partial_power(uint8_t primary_adr_off_m_bus)
{
	
	while(app_uart_flush() !=NRF_SUCCESS);

	
	uint8_t checksum = C_FIELD_RESET_PARTIAL + CI_FIELD_RESET_PARTIAL + primary_adr_off_m_bus;
	
	uint8_t data_array[TOTAL_LENGTH_RESET_PARTIAL]={START_RESET_PARTIAL, FIELD_LENGTH_RESET_PARTIAL, FIELD_LENGTH_RESET_PARTIAL, START_RESET_PARTIAL, C_FIELD_RESET_PARTIAL, 
		primary_adr_off_m_bus, CI_FIELD_RESET_PARTIAL, checksum, STOP_FIELD_RESET_PARTIAL};
	
	
	for(uint8_t i=0; i<TOTAL_LENGTH_RESET_PARTIAL; i++)
	{
		while (app_uart_put( data_array[i]) != NRF_SUCCESS);
	}
}


void m_bus_send_request (uint8_t primary_adr_off_m_bus, uint8_t c_field)
{
	
	while(app_uart_flush() !=NRF_SUCCESS);
	//c_field_fcb_set_or_not = C_FIELD_REQ_UD2;
	
	uint8_t checksum = c_field + primary_adr_off_m_bus;
	uint8_t data_array[TOTAL_LENGHT_REQUEST]={START_REQUEST, c_field, primary_adr_off_m_bus, checksum , STOP_REQUESET};
	
	
	for(uint8_t i=0; i<TOTAL_LENGHT_REQUEST; i++)
	{
		while (app_uart_put(data_array[i]) != NRF_SUCCESS);
	}
}

bool response_from_m_bus (uint8_t exp_response)
{
	static uint8_t response_array[1];
	UNUSED_VARIABLE(app_uart_get(&response_array[0]));
	
	while(app_uart_flush() !=NRF_SUCCESS);
	
	if(exp_response == response_array[0])
		return true;
	else
		return false;
}

uint8_t bcdtobyte(uint8_t bcd)
{
	uint8_t msd = (uint8_t) bcd;
	uint8_t lsd = (uint8_t) bcd;
	
	msd /= 16;
	lsd &= 0x0F;
	//Sjekk på om ting stemmer overens burde komme her.
	
	return msd*10 + lsd;
}
