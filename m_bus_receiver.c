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

	uint8_t checksum = C_FIELD_RESET_PARTIAL + primary_adr_off_m_bus + CI_FIELD_RESET_PARTIAL ;
	
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

uint32_t bcdtobyte_EX(uint8_t bcd[4])
{
	uint32_t temp;
	
	for(uint8_t i=0; i<4; i++)
	{
		uint8_t msd = (uint8_t) bcd[i];
		uint8_t lsd = (uint8_t) bcd[i];
		
		msd /= 16;
		lsd &= 0x0F;
		temp = msd *10 + lsd;
	}
	
	return temp;
}

bool telegram_structure_check (uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3)
{
	if(byte0==RESPONSE_START_FIELD && byte1==RESPONSE_L_READ && byte2==RESPONSE_L_READ && byte3==RESPONSE_START_FIELD)
		return true;
	else
		return false;
}

bool correct_checksum(uint8_t c_field, uint8_t adr, uint8_t ci_field, uint8_t checksum)
{
	uint8_t temp_check = c_field + adr + ci_field;
	NRF_LOG_INFO("c_field %0x, adr %0x, ci_field %0x, cheksum %0x, and temp check %0x\r\n",c_field,adr,ci_field,checksum,temp_check);
	NRF_LOG_INFO("c_field (%i), adr (%i), ci_field (%i), cheksum (%i), and temp check (%i)\r\n\r\n\r\n",c_field,adr,ci_field,checksum,temp_check);
	if(temp_check == checksum)
		return true;
	else
		return false;
}

uint8_t telegram_structure_response(uint8_t counter)
{
	static struct aMessage *my_response_struct;
	
	uint8_t response;
	static uint8_t response_array[62];
	
	UNUSED_VARIABLE(app_uart_get(&response_array[counter]));
	
	switch (counter)
	{
		case 3:
			if(!telegram_structure_check(response_array[0], response_array[1], response_array[2], response_array[3]))
				response = WRONG_TELEGRAM;
			else
				response = NOT_FINISH;
			break;
			
		case 61:
			NRF_LOG_INFO("case 61\r\n");
			if(!correct_checksum(response_array[4], response_array[5], response_array[6], response_array[60]))
				response = WRONG_TELEGRAM;
			else if(response_array[61]==RESPONSE_STOP_FIELD)
			{
				my_response_struct->Message_number = my_response_struct->Message_number+1;
				my_response_struct->adr = response_array[4];
				my_response_struct->STAT = response_array[16];
				my_response_struct->Total_power = bcdtobyte(response_array[22]) + (100*bcdtobyte(response_array[23])) + (10000*bcdtobyte(response_array[24])) + (1000000*bcdtobyte(response_array[25]));
				my_response_struct->Partial_power = bcdtobyte(response_array[29]) + (100*bcdtobyte(response_array[30])) + (10000*bcdtobyte(response_array[31])) + (1000000*bcdtobyte(response_array[32]));
				my_response_struct->Voltage = (response_array[38]) + (response_array[39]<<8);
				my_response_struct->Current = response_array[45] + (response_array[46]<<8);
				my_response_struct->Power = response_array[51] + (response_array[52]<<8);
				my_response_struct->Reactive_power = response_array[58] + (response_array[59]<<8);
				response = FINISH;
			}
			else
				response = WRONG_TELEGRAM;
			break;
		
		default:
			response = NOT_FINISH;
			break;
	}
	
	return response;
}




