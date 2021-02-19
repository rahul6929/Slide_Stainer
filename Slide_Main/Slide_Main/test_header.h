/*
 * test_header.h
 *
 * Created: 05-09-2020 PM 10:31:28
 *  Author: Dell
 */ 


#ifndef TEST_HEADER_H_
#define TEST_HEADER_H_

#define F_CPU 16000000UL	
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 16000000UL									// for 16 Mhz crystel
#define F_CPU 16000000UL
#define BAUD 9600										// set baud rate 9600
#define MYUBRR F_CPU/16/BAUD-1							// calculating baud rate
#define F_CPU 16000000UL

#define PACKET_SIZE									20					// 



////////////////////// General Macros //////////////////////
#define INPUT										0
#define OUTPUT										1
#define HIGH										1
#define LOW											0
#define SET											1
#define RESET										0
#define LED_ON										1
#define LED_OFF										0
#define ENABLE										1
#define	DISABLE										0


// These macros representing the string sending by Nextion display when Touch/Release event happans 
#define PROGRAM_1									1		
#define PROGRAM_2									2
#define PROGRAM_3									3
#define PROGRAM_4									4
#define P1EDIT										5
#define START										6
#define REG_START_BLO_TIME							7						
#define REGAQTY										8
#define REGABT										9
#define REGENBT										10
#define REG_END_BLO_TIME							11
#define REGA										12
#define REGAWTYM									13
#define REGB										14
#define REGC										15
#define REGD										16
#define REGE										17
#define SERVICE										18
#define SPINTIME									19
#define BACK										100
#define DELAY_IN_LOOP								100

#define TIME										60				// Seconds
#define STEPS_PER_REVOLUTIONS_32th					6400U			// 1/32th micro stepping

/* EEprom Addresses of parameter*/

#define REG_A_QTY									0x15
#define REG_A_SPIN_TIME								0x25

#define DataInitValue								0


/* Data Addresses for program 1 */
#define P1_REG_A_START_BLOWER_TIME_ADD				0x00
#define P1_REG_A_END_BLOWER_TIME_ADD				0x02
#define P1_REG_A_QTY_ADD							0x04
#define P1_REG_A_WAIT_TIME_ADD						0x06
#define P1_REG_B_QTY_ADD							0x08
#define P1_REG_B_WAIT_TIME_ADD						0x0A
#define P1_REG_C_QTY_ADD							0x0C
#define P1_REG_C_WAIT_TIME_ADD						0x0E
#define P1_REG_D_QTY_ADD							0x10
#define P1_REG_D_WAIT_TIME_ADD						0x12
#define P1_REG_E_QTY_ADD							0x14
#define P1_REG_E_WAIT_TIME_ADD						0x16


/* Data Addresses for program 2 */
#define P2_REG_A_START_BLOWER_TIME_ADD				0x18
#define P2_REG_A_END_BLOWER_TIME_ADD				0x1A
#define P2_REG_A_QTY_ADD							0x1C
#define P2_REG_A_WAIT_TIME_ADD						0x1E
#define P2_REG_B_QTY_ADD							0x20
#define P2_REG_B_WAIT_TIME_ADD						0x22
#define P2_REG_C_QTY_ADD							0x24
#define P2_REG_C_WAIT_TIME_ADD						0x26
#define P2_REG_D_QTY_ADD							0x28
#define P2_REG_D_WAIT_TIME_ADD						0x2A
#define P2_REG_E_QTY_ADD							0x2C
#define P2_REG_E_WAIT_TIME_ADD						0x2E

/* Data Addresses for program 3 */
#define P3_REG_A_START_BLOWER_TIME_ADD				0x30
#define P3_REG_A_END_BLOWER_TIME_ADD				0x32
#define P3_REG_A_QTY_ADD							0x34
#define P3_REG_A_WAIT_TIME_ADD						0x36
#define P3_REG_B_QTY_ADD							0x38
#define P3_REG_B_WAIT_TIME_ADD						0x3A
#define P3_REG_C_QTY_ADD							0x3C
#define P3_REG_C_WAIT_TIME_ADD						0x3E
#define P3_REG_D_QTY_ADD							0x40
#define P3_REG_D_WAIT_TIME_ADD						0x42
#define P3_REG_E_QTY_ADD							0x44
#define P3_REG_E_WAIT_TIME_ADD						0x46

/* Data Addresses for program 4 */
#define P4_REG_A_START_BLOWER_TIME_ADD				0x48
#define P4_REG_A_END_BLOWER_TIME_ADD				0x4A
#define P4_REG_A_QTY_ADD							0x4C
#define P4_REG_A_WAIT_TIME_ADD						0x4E
#define P4_REG_B_QTY_ADD							0x50
#define P4_REG_B_WAIT_TIME_ADD						0x52
#define P4_REG_C_QTY_ADD							0x54
#define P4_REG_C_WAIT_TIME_ADD						0x56
#define P4_REG_D_QTY_ADD							0x58
#define P4_REG_D_WAIT_TIME_ADD						0x5A
#define P4_REG_E_QTY_ADD							0x5C
#define P4_REG_E_WAIT_TIME_ADD						0x5E

#define SPIN_TIME_ADD								0x60


typedef struct
{
	volatile uint8_t PinNumber;
	volatile uint8_t *PORT;
	volatile uint8_t INorOUT;
	
}GPIO_Config;



// _____________________________________ Functions Declarations __________________________________

void ADC_Init(void);

void Timer1_init(void);
void Delay(uint8_t);

uint16_t readADC_channel(uint8_t channel_NO);

void GPIO_Init(GPIO_Config *pNAME);

void GpioPinInit(void);

void GPIO_WriteToPin( GPIO_Config *pNAME, uint8_t HorL);

uint8_t GPIO_ReadFromPin(GPIO_Config *pNAME);

void GPIO_Port_Clear(volatile uint8_t *port);


void USART0_Init(unsigned int ubrr);
void USART0_Transmit(char data);

void USART0_transmitstring(char *ch);


void USART2_Init(unsigned int ubrr);
void USART2_Transmit( char data);

void USART2_transmitstring(const char *ch);

void Send_FF_to_Display(void);

uint16_t GetValueFromDisplay();

void AI_x_LED_status(uint8_t ADC_channel, uint16_t countdiffer );

unsigned char USART2_Receive( void );

unsigned char USART0_Receive( void );

void EEPROM_write(unsigned int uiAddress, unsigned char ucData);

unsigned char EEPROM_read(unsigned int uiAddress);

void EEPROM_Write2Bytes(unsigned int uiAddress, uint16_t ucData );

uint16_t EEPROM_Read2Bytes( unsigned int uiAddress );

void GPIO_TogglePin( GPIO_Config *pNAME );

uint8_t MatchCommand(char *);

void EEPROM_DisplayDataInit(void);

void ReagentSelected(uint8_t qty_Add, uint8_t wait_Add);

void StartBlowerTimeSelected(uint8_t startBlower_Add);

void EndBlowerTimeSelected(uint8_t EndBlower_Add);

void SpinTimeSelected(uint8_t spin_time_Add);

void my_delay_us(int us);

// ___________________________________ variable declarations _______________________________________

GPIO_Config Tx2Pin;
GPIO_Config Rx2Pin;

GPIO_Config Motor_Dir;
GPIO_Config Motor_Steps;

GPIO_Config Reagent_A_pump;
GPIO_Config Reagent_B_pump;
GPIO_Config Reagent_C_pump;
GPIO_Config Reagent_D_pump;
GPIO_Config Reagent_E_pump;
GPIO_Config Drain_pump;
GPIO_Config Pinch_nozzle;


#endif /* TEST_HEADER_H_ */