/*
 * Header.c
 *
 * Created: 31-08-2020 PM 12:26:30
 *  Author: Dell
 */ 


#include "test_header.h"
#include <string.h>
#include <stdlib.h>
#include <avr/interrupt.h>
//#include <util/delay.h>

extern char rec_bufferglob[];
extern char buffer[];
extern uint16_t numberhold;
extern uint8_t OneTimeRunFunFlag;
extern uint8_t flag ;
extern uint16_t time_in_seconds;

/*************************************************************************************************************************************

Function Name	->		GpioPinInit

Return Type		->		Void

Parameter In	->		Void

Remark			->		This function will Initialize the GPIO pins

*************************************************************************************************************************************/


void GpioPinInit(void)
{
	Tx2Pin.INorOUT = OUTPUT;
	Tx2Pin.PinNumber = 1;
	Tx2Pin.PORT = &PORTH;
	
	Rx2Pin.INorOUT = INPUT;
	Rx2Pin.PinNumber = 0;
	Rx2Pin.PORT = &PORTH;
	
	Motor_Dir.INorOUT = OUTPUT;
	Motor_Dir.PinNumber = 0;
	Motor_Dir.PORT = &PORTG;
	
	Motor_Steps.INorOUT = OUTPUT;
	Motor_Steps.PinNumber = 0;
	Motor_Steps.PORT = &PORTC;
	
	Reagent_A_pump.INorOUT = OUTPUT;
	Reagent_A_pump.PinNumber = 6;
	Reagent_A_pump.PORT = &PORTB;
	
	Reagent_B_pump.INorOUT = OUTPUT;
	Reagent_B_pump.PinNumber = 5;
	Reagent_B_pump.PORT = &PORTB;
	
	Reagent_C_pump.INorOUT = OUTPUT;
	Reagent_C_pump.PinNumber = 4;
	Reagent_C_pump.PORT = &PORTB;
	
	Reagent_D_pump.INorOUT = OUTPUT;
	Reagent_D_pump.PinNumber = 5;
	Reagent_D_pump.PORT = &PORTH;
	
	Reagent_E_pump.INorOUT = OUTPUT;
	Reagent_E_pump.PinNumber = 4;
	Reagent_E_pump.PORT = &PORTH;
	
	Drain_pump.INorOUT = OUTPUT;
	Drain_pump.PinNumber = 3;
	Drain_pump.PORT = &PORTH;
	
	Pinch_nozzle.INorOUT = OUTPUT;
	Pinch_nozzle.PinNumber = 7;
	Pinch_nozzle.PORT = &PORTD;
	
	Blower.INorOUT = OUTPUT;
	Blower.PinNumber = 3;
	Blower.PORT = &PORTL;
	
	GPIO_Init(&Rx2Pin);
	GPIO_Init(&Tx2Pin);
	GPIO_Init(&Motor_Dir);
	GPIO_Init(&Motor_Steps);
	GPIO_Init(&Reagent_A_pump);
	GPIO_Init(&Reagent_B_pump);
	GPIO_Init(&Reagent_C_pump);
	GPIO_Init(&Reagent_D_pump);
	GPIO_Init(&Reagent_E_pump);
	GPIO_Init(&Drain_pump);
	GPIO_Init(&Pinch_nozzle);
	GPIO_Init(&Blower);
	
}

/*************************************************************************************************************************************

Function Name	->		EEPROM_write

Return Type		->		Void

Parameter 1		->		unsigned int, Address of EEPROM

Parameter 2		->		unsigned char, data that want to write on address of EEPROM

Remark			->		This function will write the 1 byte data at given address of EEPROM

*************************************************************************************************************************************/

void EEPROM_write(unsigned int uiAddress, unsigned char ucData)
{
	cli();
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE))
	;
	/* Set up address and Data Registers */
	EEAR = uiAddress;
	EEDR = ucData;
	/* Write logical one to EEMPE */
	EECR |= (1<<EEMPE);
	/* Start eeprom write by setting EEPE */
	EECR |= (1<<EEPE);
	sei();
}


/*************************************************************************************************************************************

Function Name	->		EEPROM_read

Return Type		->		unsigned char, it will return Data stored at given address

Parameter 1		->		unsigned int, Address of EEPROM

Remark			->		This function will read the 1 byte data from given address 

*************************************************************************************************************************************/


unsigned char EEPROM_read(unsigned int uiAddress)
{
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE))
	;
	/* Set up address register */
	EEAR = uiAddress;
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	/* Return data from Data Register */
	return EEDR;
}


/*************************************************************************************************************************************

Function Name	->		EEPROM_Write2Bytes

Return Type		->		Void

Parameter 1		->		unsigned int, Address of EEPROM at which 2 byte data want to write

Parameter 2		->		unsigned int, 2 byte data to be write on given address

Remark			->		This function will write 2 byte data at given address. lower byte will be store at given address and
						upper byte will store at (address + 1) location 

*************************************************************************************************************************************/


void EEPROM_Write2Bytes(unsigned int uiAddress, uint16_t ucData )
{
	cli();
	// splitting 2 bytes data in single bytes
	uint8_t byteL = (uint8_t)(ucData & 0xFF);
	uint8_t byteH = (uint8_t) ( (ucData >> 8) & 0xFF ); // upper 8 bits masking
	
	EEPROM_write(uiAddress, byteL);		// lower 8 bit at lower address
	EEPROM_write(uiAddress + 1 , byteH);	// upper 8 bit at upper address
	sei();
}


/*************************************************************************************************************************************

Function Name	->		EEPROM_Read2Bytes

Return Type		->		uint16_t, 2 byte data from given address of EEPROM

Parameter 1		->		unsigned int, Address of EEPROM at which 2 byte data want to read

Remark			->		This function will read 2 byte data at given address. lower byte will be read given address and
						upper byte will be read from (address + 1) location 

*************************************************************************************************************************************/


uint16_t EEPROM_Read2Bytes( unsigned int uiAddress )
{
	uint16_t data=0;
	uint8_t byteL = EEPROM_read(uiAddress);
	uint8_t byteH = EEPROM_read(uiAddress + 1);
	
	data = byteL;
	data |=   (uint16_t)(byteH << 8);
	
	return data;
}


/*************************************************************************************************************************************

Function Name	->		ADC_Init

Return Type		->		Void

Parameter In	->		Void

Remark			->		This function will initialize ADC

*************************************************************************************************************************************/

void ADC_Init(void)
{
	// AVCC with external capacitor at AREF pin, ADC Left Adjust Result OFF, AD0 SELECTED
	ADMUX |= (1<<6);
	
	// ADC ENABLE, ADC PRESCALER BY /128
	ADCSRA |= (1<<7) | (7<<0);
	
	
}


/*************************************************************************************************************************************

Function Name	->		readADC_channel

Return Type		->		uint16_t, 10 bit ADC value

Parameter 1		->		uint8_t, ADC channel number e.g. for ADC0 write 0, ADC13 write 13 etc. 

Remark			->		This function will return 10 bit ADC value from given ADC channel 

*************************************************************************************************************************************/

uint16_t readADC_channel(uint8_t channel_NO)
{	
	uint16_t temp=0;
	
	
	// to select channel b/w ADC0 TO ADC7
	if (channel_NO < 8)
		{
			ADCSRB &= ~( 1<<3);
			DIDR0 |= (1<< (channel_NO));
		}
		
	// to select channel b/w ADC8 TO ADC15
	else
		{
			ADCSRB |= ( 1<<3);		// mux5=1
			DIDR2 |= ( 1<< (channel_NO % 8));
		}
		
		
	ADMUX = (ADMUX & 0xE0) | ( channel_NO & 0x07);
	
	// ADC Start Conversion
	ADCSRA |= (1<<6);
	
	// Waiting for conversion completion
	while( ((ADCSRA>>4) & 0x01)==0 );
	
	temp = (uint8_t)ADCL;
	temp |=  (uint16_t)( ADCH << 8 );
	
	// ADIF CLEAR
	//ADCSRA |= (1 << ADIF);
	
	return temp;
	
}


/*************************************************************************************************************************************

Function Name	->		GPIO_Init

Return Type		->		Void

Parameter 1		->		GPIO_Config * (structure address), address of structure variable containing Pin info

Remark			->		This function will set given pin as input, output

*************************************************************************************************************************************/


void GPIO_Init(GPIO_Config *pNAME)
{
	if ( pNAME->INorOUT == 0)	// input
		*(pNAME->PORT-1) &= ~(1<<pNAME->PinNumber);
	else
		*(pNAME->PORT-1) |= (1<<pNAME->PinNumber);
}


/*************************************************************************************************************************************

Function Name	->		GPIO_WriteToPin

Return Type		->		Void

Parameter 1		->		GPIO_Config * (structure address), address of structure variable containing Pin info

Parameter 2		->		uint8_t, High or Low

Remark			->		This function will write 1 or 0 on gievn pin

*************************************************************************************************************************************/


void GPIO_WriteToPin( GPIO_Config *pNAME, uint8_t HorL)
{
	if (HorL== HIGH)
		{
			*(pNAME->PORT) |= (1<< pNAME->PinNumber);
		}
		
	else
		{
			*(pNAME->PORT) &= ~( 1<< pNAME->PinNumber );
		}
}


/*************************************************************************************************************************************

Function Name	->		GPIO_ReadFromPin

Return Type		->		uint8_t, 1 or 0

Parameter 1		->		GPIO_Config * (structure address), address of structure variable containing Pin info

Remark			->		This function will read given pin as 0 or 1

*************************************************************************************************************************************/


uint8_t GPIO_ReadFromPin(GPIO_Config *pNAME)
{
	uint8_t temp=0;
	temp = ( (*(pNAME->PORT-2)) >> pNAME->PinNumber ) & 0x01;
	return temp;
}


void GPIO_Port_Clear(volatile uint8_t *port)
{
	*(port) = 0x00;
}

///////////////// USART0 TX //////////////////////////


/*************************************************************************************************************************************

Function Name	->		USART0_Init

Return Type		->		Void

Parameter 1		->		unsigned int, baud rate

Remark			->		This function will initialize USART0 with given baud rate

*************************************************************************************************************************************/

void USART0_Init( unsigned int ubrr)
{
	/* Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;

	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0) |(1<< RXCIE0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C =  (3<<UCSZ00);
}


/*************************************************************************************************************************************

Function Name	->		USART0_Transmit

Return Type		->		Void

Parameter 1		->		unsigned char, 1 byte Data to be sent

Remark			->		This function will transmit 1 byte data

*************************************************************************************************************************************/

void USART0_Transmit( char data)
{
	
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = data;

}


/*************************************************************************************************************************************

Function Name	->		USART0_transmitstring

Return Type		->		Void

Parameter 1		->		char*, Address of string

Remark			->		This function will send character 1 by 1

*************************************************************************************************************************************/

void USART0_transmitstring( char *ch)		
{
// 	GPIO_WriteToPin(&TX_RX_En_De, HIGH);
// 	_delay_ms(200);
	while(*ch != '\0')
	{
		USART0_Transmit(*ch);
		ch++;
	}
// 	GPIO_WriteToPin(&TX_RX_En_De, LOW);
// 	_delay_ms(200);
}


///////////////// USART2 TX //////////////////////////


/*************************************************************************************************************************************

Function Name	->		USART2_Init

Return Type		->		Void

Parameter 1		->		unsigned int, baud rate

Remark			->		This function will initialize USART2 with given baud rate

*************************************************************************************************************************************/

void USART2_Init( unsigned int ubrr)
{
	/* Set baud rate */
	UBRR2H = (unsigned char)(ubrr>>8);
	UBRR2L = (unsigned char)ubrr;

	/* Enable receiver and transmitter */
	UCSR2B = (1<<RXEN2)|(1<<TXEN2) |(1<< RXCIE2);
	/* Set frame format: 8data, 1stop bit */
	UCSR2C =  (3<<UCSZ20);
}


/*************************************************************************************************************************************

Function Name	->		USART2_Transmit

Return Type		->		Void

Parameter 1		->		unsigned char, 1 byte Data to be sent

Remark			->		This function will transmit 1 byte data

*************************************************************************************************************************************/

void USART2_Transmit( char data)
{
// 	_delay_ms(10);
// 	GPIO_WriteToPin(&TX_RX_En_De, HIGH);
// 	_delay_ms(10);
	/* Wait for empty transmit buffer */
	while ( !( UCSR2A & (1<<UDRE2)) );
	/* Put data into buffer, sends the data */
	UDR2 = data;
// 	_delay_ms(10);
// 	GPIO_WriteToPin(&TX_RX_En_De, LOW);
// 	_delay_ms(10);
}


/*************************************************************************************************************************************

Function Name	->		USART2_Receive

Return Type		->		unsigned char, Received byte data

Parameter 1		->		Void

Remark			->		This function will return 1 byte data received at USART2

*************************************************************************************************************************************/

unsigned char USART2_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSR2A & (1<<RXC2)) );
	/* Get and return received data from buffer */
	return UDR2;
}


/*************************************************************************************************************************************

Function Name	->		USART0_Receive

Return Type		->		unsigned char, Received byte data

Parameter 1		->		Void

Remark			->		This function will return 1 byte data received at USART0

*************************************************************************************************************************************/

unsigned char USART0_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) );
	/* Get and return received data from buffer */
	return UDR0;
}


/*************************************************************************************************************************************

Function Name	->		USART2_transmitstring

Return Type		->		Void

Parameter 1		->		const char*, Address of string

Remark			->		This function will send character 1 by 1

*************************************************************************************************************************************/

void USART2_transmitstring(const char *ch)
{
// 	 	_delay_ms(10);
// 	 	GPIO_WriteToPin(&TX_RX_En_De, HIGH);
// 	 	_delay_ms(10);
	while(*ch != '\0')
		{
			USART2_Transmit(*ch);
			ch++;
		}
		
// 		_delay_ms(10);
// 		GPIO_WriteToPin(&TX_RX_En_De, LOW);
// 		_delay_ms(10);
}




void Delay(uint8_t time)
{
	uint8_t i=0;
	
	for (i=0; i<time; i++)
	{
		
		TCNT1 =0xC2F7;	// This value will generate 1 Sec delay for prescaler 1024 and 16 mhz crystel 
		TCCR1B = (5<<0);	// prescaler 1024
		while ((TIFR1 & 0x01)==0);	// waiting for timer overflow 
		TCCR1B=0;			// timer off
		TCNT1 = 0;			// timer buffer clear
		TIFR1 |= (1<<0);	// timer overflow bit clear by writing it 1
	}
}


void Timer1_init(void)
{
	TCCR1A =0;		// compare output mode is disable for channel A,B and C
	TCCR1C =0;		// Force Output Compare for Channel A, B and Channel C are disable
	TCNT1 =0xC2F7;	// This value will generate 1 Sec delay for prescaler 1024 and 16 MHz crystal
	TCCR1B = (5<<0);	// prescaler 1024
	TIMSK1 |= (1<<0);	// Interrupt enable
	
}




void GPIO_TogglePin( GPIO_Config *pNAME )
{
	*(pNAME->PORT) ^= (1<< pNAME->PinNumber);
}


// This function will return a number based on string sent by nextion display
uint8_t MatchCommand(char *command)
{	
	
	
	//numberhold = 0;
	unsigned char *token = NULL;
	token = strtok(command, "|");
	
	//memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));
	
	/* These if statement used to find out which icon pressed */
	if (!strcmp(command, "program 1"))
		return PROGRAM_1;
	if (!strcmp(command, "program 2"))
		return PROGRAM_2;
	if (!strcmp(command, "program 3"))
		return PROGRAM_3;
	if (!strcmp(command, "program 4"))
		return PROGRAM_4;
	if (!strcmp(command, "p1edit"))
		return P1EDIT;
	if (!strcmp(command, "start"))
		return START;
	if (!strcmp(command, "back"))
		return BACK;
	if (!strcmp(command, "stblotym"))
		return REG_START_BLO_TIME;
	if (!strcmp(command, "endblotym"))
		return REG_END_BLO_TIME;
	if (!strcmp(command, "rega"))
		return REGA;
	if (!strcmp(command, "regb"))
		return REGB;
	if (!strcmp(command, "regc"))
		return REGC;
	if (!strcmp(command, "regd"))
		return REGD;
	if (!strcmp(command, "rege"))
		return REGE;
	if (!strcmp(command, "service"))
		return SERVICE;
	
	
	/* Here we are getting data to be stored from display */
	if (!strcmp(token, "REGABT"))
		{
			token = strtok(NULL, "|");
			numberhold = *(token);
			numberhold |= (*(token+1))<<8;
			
			memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
			return REGABT;
		}
		
	if (!strcmp(token, "REGENBT"))
	{
		token = strtok(NULL, "|");
		numberhold = *(token);
		numberhold |= (*(token+1))<<8;
		
		memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
		return REGENBT;
	}
	
	if (!strcmp(token, "REGAWTYM"))
	{
		token = strtok(NULL, "|");
		numberhold = *(token);
		numberhold |= (*(token+1))<<8;
		
		memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
		return REGAWTYM;
	}
	
	
	
	if (!strcmp(token, "REGAQTY"))
	{
		token = strtok(NULL, "|");
		numberhold = *(token);
		numberhold |= (*(token+1))<<8;
		memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
		return REGAQTY;
	} 
	
	if (!strcmp(token, "SPINTYM"))
	{
		token = strtok(NULL, "|");
		numberhold = *(token);
		numberhold |= (*(token+1))<<8;
		memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
		return SPINTIME;
	}
	else
		return 0;
}

/*
uint16_t GetValueFromDisplay()
{
	uint16_t temp=0;
	unsigned char *token = NULL;
	token = strtok(rec_bufferglob, "|");
	if (!strcmp(token, "REGABT"))
	{
		token = strtok(NULL, "|");
		temp = *(token);
		temp |= (*(token+1))<<8;
		return temp;
	}
	
	else
		return 0;
}
*/

void Send_FF_to_Display()
{
	uint8_t i=0;
	for(i=0; i<3; i++)
		USART0_Transmit(0xFF);

	memset(buffer, '\0', PACKET_SIZE * sizeof(buffer[0]));	// rec_bufferglob clear
	
}

void EEPROM_DisplayDataInit(void)
{
	uint8_t i=0;
	uint8_t Address = 0x00;
	for(i=0; i<49; i++)				//Here 49 is number of data to be stored
	{
		if (EEPROM_Read2Bytes(Address)==0xFFFF)
			EEPROM_Write2Bytes(Address, DataInitValue);
		Address += 2;
	}
	
}

void ReagentSelected(uint8_t qty_Add, uint8_t wait_Add)
{
	while(1)
	{
		_delay_ms(DELAY_IN_LOOP);
		USART2_transmitstring("qty ");
		USART2_Transmit(OneTimeRunFunFlag);
		if (OneTimeRunFunFlag==0)
		{
			USART0_transmitstring("n0.val=");
			itoa(EEPROM_Read2Bytes(qty_Add), buffer, 10);
			USART0_transmitstring(buffer);
			Send_FF_to_Display();
			OneTimeRunFunFlag++;
		}
		
		switch(MatchCommand(rec_bufferglob))
		{
			case REGAQTY:
			{
				EEPROM_Write2Bytes(qty_Add, numberhold);
				OneTimeRunFunFlag=0;
				
				while(1)
				{
					_delay_ms(DELAY_IN_LOOP);
					USART2_transmitstring("wait ");
					USART2_Transmit(OneTimeRunFunFlag);
					if (OneTimeRunFunFlag==0)
					{
						USART0_transmitstring("n0.val=");
						itoa(EEPROM_Read2Bytes(wait_Add), buffer, 10);
						USART0_transmitstring(buffer);
						Send_FF_to_Display();
						OneTimeRunFunFlag++;
					}
					
					switch(MatchCommand(rec_bufferglob))
					{
						case REGAWTYM:
						{
							EEPROM_Write2Bytes(wait_Add, numberhold);
							strcpy(rec_bufferglob, "back");
							flag = 1;
							break;
						}
					}
					
					if (MatchCommand(rec_bufferglob)==BACK)
					{
						OneTimeRunFunFlag=0;
						if (flag!=1)
						{
							memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
						}
						break;
					}
					
				}
				
				
				
				break;
			}
		}
		
		if (MatchCommand(rec_bufferglob)==BACK)
		{
			OneTimeRunFunFlag=0;
			memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
			flag = 0;
			break;
		}
		
	}
}

void StartBlowerTimeSelected(uint8_t startBlower_Add)
{
	while(1)
	{
		_delay_ms(DELAY_IN_LOOP);
		if (OneTimeRunFunFlag==0)
		{
			USART0_transmitstring("n0.val=");
			itoa(EEPROM_Read2Bytes(startBlower_Add), buffer, 10);
			USART0_transmitstring(buffer);
			Send_FF_to_Display();
			OneTimeRunFunFlag++;
		}
		
		switch(MatchCommand(rec_bufferglob))
		{
			case REGABT:
			{
				EEPROM_Write2Bytes(startBlower_Add, numberhold);
				strcpy(rec_bufferglob, "back");
				break;
			}
		}
		
		if (MatchCommand(rec_bufferglob)==BACK)
		{
			OneTimeRunFunFlag=0;
			memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
			break;
		}
		
	}
}

void EndBlowerTimeSelected(uint8_t EndBlower_Add)
{
	while(1)
	{
		_delay_ms(DELAY_IN_LOOP);
		if (OneTimeRunFunFlag==0)
		{
			USART0_transmitstring("n0.val=");
			itoa(EEPROM_Read2Bytes(EndBlower_Add), buffer, 10);
			USART0_transmitstring(buffer);
			Send_FF_to_Display();
			OneTimeRunFunFlag++;
		}
		
		switch(MatchCommand(rec_bufferglob))
		{
			case REGENBT:
			{
				EEPROM_Write2Bytes(EndBlower_Add, numberhold);
				strcpy(rec_bufferglob, "back");
				break;
			}
		}
		
		if (MatchCommand(rec_bufferglob)==BACK)
		{
			OneTimeRunFunFlag=0;
			memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
			break;
		}
		
	}
}


void SpinTimeSelected(uint8_t spin_time_Add)
{
	while(1)
	{
		_delay_ms(DELAY_IN_LOOP);
		if (OneTimeRunFunFlag==0)
		{
			USART0_transmitstring("n0.val=");
			itoa(EEPROM_Read2Bytes(spin_time_Add), buffer, 10);
			USART0_transmitstring(buffer);
			Send_FF_to_Display();
			OneTimeRunFunFlag++;
		}
		
		switch(MatchCommand(rec_bufferglob))
		{
			case SPINTIME:
			{
				EEPROM_Write2Bytes(spin_time_Add, numberhold);
				strcpy(rec_bufferglob, "back");
				break;
			}
		}
		
		if (MatchCommand(rec_bufferglob)==BACK)
		{
			OneTimeRunFunFlag=0;
			memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
			break;
		}
		
	}
}


void my_delay_us(int us)
{
	while (0 < us--)
	{
		_delay_us(1);
	}
}


// This function will increase speed of motor gradually for 2.5 seconds
void Increase_gradually_motor(void)
{
	uint32_t i = 0;
	
	// 2 revolutions with 30us delay
	for (i=0; i<STEPS_PER_REVOLUTIONS_32th*2; i++)
	{
		GPIO_WriteToPin(&Motor_Steps, HIGH);
		_delay_us(30);
		GPIO_WriteToPin(&Motor_Steps, LOW);
		_delay_us(30);
	}
	
	// 2 revolutions with 15us delay
	for (i=0; i<STEPS_PER_REVOLUTIONS_32th*2; i++)
	{
		GPIO_WriteToPin(&Motor_Steps, HIGH);
		_delay_us(15);
		GPIO_WriteToPin(&Motor_Steps, LOW);
		_delay_us(15);
	}
	
	// 10 revolutions with 10us delay
	for (i=0; i<STEPS_PER_REVOLUTIONS_32th*10; i++)
	{
		GPIO_WriteToPin(&Motor_Steps, HIGH);
		_delay_us(10);
		GPIO_WriteToPin(&Motor_Steps, LOW);
		_delay_us(10);
	}
	
}

// This function will decrease speed of motor gradually for 2.5 seconds
void Decrease_gradually_motor(void)
{
	uint32_t i = 0;
	
	// 10 revolutions with 10us delay
	for (i=0; i<STEPS_PER_REVOLUTIONS_32th*10; i++)
	{
		GPIO_WriteToPin(&Motor_Steps, HIGH);
		_delay_us(10);
		GPIO_WriteToPin(&Motor_Steps, LOW);
		_delay_us(10);
	}
	
	// 2 revolutions with 15us delay
	for (i=0; i<STEPS_PER_REVOLUTIONS_32th*2; i++)
	{
		GPIO_WriteToPin(&Motor_Steps, HIGH);
		_delay_us(15);
		GPIO_WriteToPin(&Motor_Steps, LOW);
		_delay_us(15);
	}
	
	// 2 revolutions with 30us delay
	for (i=0; i<STEPS_PER_REVOLUTIONS_32th*2; i++)
	{
		GPIO_WriteToPin(&Motor_Steps, HIGH);
		_delay_us(30);
		GPIO_WriteToPin(&Motor_Steps, LOW);
		_delay_us(30);
	}
	
}

void Spin_motor(uint8_t time_in_sec)
{
	
	if(time_in_sec > 0)
	{	
		uint32_t i = 0;
		Increase_gradually_motor();
		// in for loop 39 is number of revolutions in 5 sec at 10us delay
		for (i=0; i<STEPS_PER_REVOLUTIONS_32th*39u*(time_in_sec/5 - 1); i++)
		{
			GPIO_WriteToPin(&Motor_Steps, HIGH);
			_delay_us(10);
			GPIO_WriteToPin(&Motor_Steps, LOW);
			_delay_us(10);
		}
		Decrease_gradually_motor();
	}
}

void Send_Text_On_Screen(const char	*text)
{
	_delay_ms(DELAY_IN_LOOP);

	strcat(buffer, "text.txt=");
	strcat(buffer, "\"");
	strcat(buffer, text);
	strcat(buffer, "\"");
	USART0_transmitstring(buffer);
	USART2_transmitstring(buffer);
	Send_FF_to_Display();
	memset(buffer, '\0', PACKET_SIZE * sizeof(buffer[0]));	// rec_bufferglob clear
}

void Blower_ON(uint16_t Blower_time_sec)
{
	
	Timer1_init();
	while( (Blower_time_sec - time_in_seconds) != 0 )
		GPIO_WriteToPin(&Blower, HIGH);
	GPIO_WriteToPin(&Blower, LOW);
	TIMSK1 &= ~(1<<0);	// Interrupt disable
	time_in_seconds=0;
}

void Dispense_Reagent(uint8_t Quantity, GPIO_Config *pPump_Name)
{
	
}