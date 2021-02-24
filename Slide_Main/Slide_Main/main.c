/*
 * Slide_Main.c
 *
 * Created: 22-01-2021 11.33.04 AM
 * Author : PC-X4
 */ 


#define F_CPU	16000000UL				// Crystel frequency 16MHz
#include "test_header.h"
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <avr/interrupt.h>

unsigned char tempbuffer[PACKET_SIZE];
char buffer[PACKET_SIZE];
char rec_bufferglob[PACKET_SIZE];									// global buffer to store rec data		15 to 20
volatile uint8_t myindex =0;										// index for rec_bufferglob
volatile uint8_t rec_stop=0;										// flag to indicate data has received
volatile uint8_t rec_start=0;										// flag to indicate data start coming
char rec_buff[PACKET_SIZE];											// temporary buffer to store receive data			15 to 20
//volatile uint8_t txtime=TIME;										// store seconds, after this interval data will send automatically
//volatile uint8_t txflag=0;											// flag to indicate txtime complete
volatile numberhold=3;
uint8_t OneTimeRunFunFlag=0;
uint8_t flag = 0;	
volatile uint16_t time_in_seconds=0;					
								
int main(void)
{
	//uint8_t OneTimeRunFunFlag=0;
	uint32_t counting = 0;
	
	
	float stepdelay;
	USART2_Init(MYUBRR);
	USART0_Init(MYUBRR);
	GpioPinInit();
	_delay_ms(1000);
	//Timer1_init();
	USART2_transmitstring("sdjcsdb");
	USART0_transmitstring("page Main");
	Send_FF_to_Display();
	if (EEPROM_Read2Bytes(P4_REG_E_WAIT_TIME_ADD)==0xFFFF)
		EEPROM_DisplayDataInit();
	GPIO_WriteToPin(&Motor_Dir, HIGH);
	//TIMSK1 &= ~(1<<0);	// Interrupt disable

	sei();		// To enable Global Interrupt, cli(); for disable
	/*
	while(1)
	{
		GPIO_WriteToPin(&Motor_Dir, HIGH);
		for(counting =0; counting < ((uint32_t)STEPS_PER_REVOLUTIONS_32th*10); counting++)
		{
		GPIO_WriteToPin(&Motor_Steps, HIGH);
		_delay_us(13);
		GPIO_WriteToPin(&Motor_Steps, LOW);
		_delay_us(13);
		}
		_delay_ms(1500);
		GPIO_WriteToPin(&Motor_Dir, LOW);
		for(counting =0; counting < ((uint32_t)STEPS_PER_REVOLUTIONS_32th*10); counting++)
		{
			GPIO_WriteToPin(&Motor_Steps, HIGH);
			_delay_us(13);
			GPIO_WriteToPin(&Motor_Steps, LOW);
			_delay_us(13);
		}
		
	} */
    while (1) 
    {	
		_delay_ms(DELAY_IN_LOOP);
		//USART2_transmitstring("\nat Home ");
		//USART0_transmitstring("Home ");
		switch(MatchCommand(rec_bufferglob))
		{
			// this case will work when we touch Program 1 on screen
			case PROGRAM_1:
					{	
						
						while(1)
						{
							_delay_ms(DELAY_IN_LOOP);
							switch(MatchCommand(rec_bufferglob))
							{	
								case START:
									{
										_delay_ms(100);
										USART0_transmitstring("page STATUS");
										Send_FF_to_Display();
										Send_Text_On_Screen("Program 1 Initiated");
										_delay_ms(1500);
										
										Send_Text_On_Screen("Drying Cycle In Progress...");
										Blower_ON(EEPROM_Read2Bytes(P1_REG_A_START_BLOWER_TIME_ADD));
										
										Send_Text_On_Screen("Cycle 1 In Progress...");
										Dispense_Reagent(EEPROM_Read2Bytes(P1_REG_A_QTY_ADD), &Reagent_A_pump);
										Reagent_Wait_Time(EEPROM_Read2Bytes(P1_REG_A_WAIT_TIME_ADD));
										//Spin_motor(EEPROM_Read2Bytes(SPIN_TIME_ADD));
										
										Send_Text_On_Screen("Cycle 2 In Progress...");
										Dispense_Reagent(EEPROM_Read2Bytes(P1_REG_B_QTY_ADD), &Reagent_B_pump);
										
										Send_Text_On_Screen("Cycle 3 In Progress...");
										Dispense_Reagent(EEPROM_Read2Bytes(P1_REG_C_QTY_ADD), &Reagent_C_pump);
										
										Send_Text_On_Screen("Cycle 4 In Progress...");
										Dispense_Reagent(EEPROM_Read2Bytes(P1_REG_D_QTY_ADD), &Reagent_D_pump);
										 										
 										Send_Text_On_Screen("Cycle 5 In Progress...");
 										Dispense_Reagent(EEPROM_Read2Bytes(P1_REG_E_QTY_ADD), &Reagent_E_pump); 
																				
										Send_Text_On_Screen("Draining Cycle In Progress...");
										_delay_ms(1000);
										Send_Text_On_Screen("End of Program 1, Please \nremove the slide tray.");
										Blower_ON(EEPROM_Read2Bytes(P1_REG_A_END_BLOWER_TIME_ADD));
										_delay_ms(2000);
										USART0_transmitstring("page Main");
										Send_FF_to_Display();
										strcpy(rec_bufferglob, "back");
										//memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
										break;
									}
								case P1EDIT:
									{	
										while(1)
										{
											_delay_ms(DELAY_IN_LOOP);
											switch(MatchCommand(rec_bufferglob))
											{
												case REG_START_BLO_TIME:
												{
													StartBlowerTimeSelected(P1_REG_A_START_BLOWER_TIME_ADD);
													break;
												}
													
												case REG_END_BLO_TIME:
												{
													EndBlowerTimeSelected(P1_REG_A_END_BLOWER_TIME_ADD);
													break;
												}
												
												case REGA:
												{
													ReagentSelected(P1_REG_A_QTY_ADD, P1_REG_A_WAIT_TIME_ADD);
													break;
												}
												
												case REGB:
												{
													ReagentSelected(P1_REG_B_QTY_ADD, P1_REG_B_WAIT_TIME_ADD);
													break;
												}
												
												case REGC:
												{
													ReagentSelected(P1_REG_C_QTY_ADD, P1_REG_C_WAIT_TIME_ADD);
													break;
												}
												
												case REGD:
												{
													ReagentSelected(P1_REG_D_QTY_ADD, P1_REG_D_WAIT_TIME_ADD);
													break;
												}
												case REGE:
												{
													ReagentSelected(P1_REG_E_QTY_ADD, P1_REG_E_WAIT_TIME_ADD);
													break;
												}
													
											}
											
											if (MatchCommand(rec_bufferglob)==BACK)
											{
												memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
												break;
											}
											
										}
																	
										break;
									}
									
							}
							
							if (MatchCommand(rec_bufferglob)==BACK)
								{
									memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
									break;
								}
						}
						
						break;
					}
				
			case PROGRAM_2:
					{
						while(1)
						{
							_delay_ms(DELAY_IN_LOOP);
							switch(MatchCommand(rec_bufferglob))
							{
								case START:
								{
									USART2_transmitstring("Start");
									break;
								}
								case P1EDIT:
								{
									while(1)
									{
										_delay_ms(DELAY_IN_LOOP);
										switch(MatchCommand(rec_bufferglob))
										{
											case REG_START_BLO_TIME:
											{
												StartBlowerTimeSelected(P2_REG_A_START_BLOWER_TIME_ADD);
												break;
											}
											
											case REG_END_BLO_TIME:
											{
												EndBlowerTimeSelected(P2_REG_A_END_BLOWER_TIME_ADD);
												break;
											}
											
											case REGA:
											{
												ReagentSelected(P2_REG_A_QTY_ADD, P2_REG_A_WAIT_TIME_ADD);
												break;
											}
											
											case REGB:
											{
												ReagentSelected(P2_REG_B_QTY_ADD, P2_REG_B_WAIT_TIME_ADD);
												break;
											}
											
											case REGC:
											{
												ReagentSelected(P2_REG_C_QTY_ADD, P2_REG_C_WAIT_TIME_ADD);
												break;
											}
											
											case REGD:
											{
												ReagentSelected(P2_REG_D_QTY_ADD, P2_REG_D_WAIT_TIME_ADD);
												break;
											}
											case REGE:
											{
												ReagentSelected(P2_REG_E_QTY_ADD, P2_REG_E_WAIT_TIME_ADD);
												break;
											}
											
										}
										
										if (MatchCommand(rec_bufferglob)==BACK)
										{
											memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
											break;
										}
										
									}
									
									break;
								}
								
							}
							
							if (MatchCommand(rec_bufferglob)==BACK)
							{
								memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
								break;
							}
						}
						break;
					}
					
			case PROGRAM_3:
					{
						while(1)
						{
							_delay_ms(DELAY_IN_LOOP);
							switch(MatchCommand(rec_bufferglob))
							{
								case START:
								{
									USART2_transmitstring("Start");
									break;
								}
								case P1EDIT:
								{
									while(1)
									{
										_delay_ms(DELAY_IN_LOOP);
										switch(MatchCommand(rec_bufferglob))
										{
											case REG_START_BLO_TIME:
											{
												StartBlowerTimeSelected(P3_REG_A_START_BLOWER_TIME_ADD);
												break;
											}
											
											case REG_END_BLO_TIME:
											{
												EndBlowerTimeSelected(P3_REG_A_END_BLOWER_TIME_ADD);
												break;
											}
											
											case REGA:
											{
												ReagentSelected(P3_REG_A_QTY_ADD, P3_REG_A_WAIT_TIME_ADD);
												break;
											}
											
											case REGB:
											{
												ReagentSelected(P3_REG_B_QTY_ADD, P3_REG_B_WAIT_TIME_ADD);
												break;
											}
											
											case REGC:
											{
												ReagentSelected(P3_REG_C_QTY_ADD, P3_REG_C_WAIT_TIME_ADD);
												break;
											}
											
											case REGD:
											{
												ReagentSelected(P3_REG_D_QTY_ADD, P3_REG_D_WAIT_TIME_ADD);
												break;
											}
											case REGE:
											{
												ReagentSelected(P3_REG_E_QTY_ADD, P3_REG_E_WAIT_TIME_ADD);
												break;
											}
											
										}
										
										if (MatchCommand(rec_bufferglob)==BACK)
										{
											memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
											break;
										}
										
									}
									
									break;
								}
								
							}
							
							if (MatchCommand(rec_bufferglob)==BACK)
							{
								memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
								break;
							}
						}
						break;
					}
					
			case PROGRAM_4:
					{
						while(1)
						{
							_delay_ms(DELAY_IN_LOOP);
							switch(MatchCommand(rec_bufferglob))
							{
								case START:
								{
									USART2_transmitstring("Start");
									break;
								}
								case P1EDIT:
								{
									while(1)
									{
										_delay_ms(DELAY_IN_LOOP);
										switch(MatchCommand(rec_bufferglob))
										{
											case REG_START_BLO_TIME:
											{
												StartBlowerTimeSelected(P4_REG_A_START_BLOWER_TIME_ADD);
												break;
											}
											
											case REG_END_BLO_TIME:
											{
												EndBlowerTimeSelected(P4_REG_A_END_BLOWER_TIME_ADD);
												break;
											}
											
											case REGA:
											{
												ReagentSelected(P4_REG_A_QTY_ADD, P4_REG_A_WAIT_TIME_ADD);
												break;
											}
											
											case REGB:
											{
												ReagentSelected(P4_REG_B_QTY_ADD, P4_REG_B_WAIT_TIME_ADD);
												break;
											}
											
											case REGC:
											{
												ReagentSelected(P4_REG_C_QTY_ADD, P4_REG_C_WAIT_TIME_ADD);
												break;
											}
											
											case REGD:
											{
												ReagentSelected(P4_REG_D_QTY_ADD, P4_REG_D_WAIT_TIME_ADD);
												break;
											}
											case REGE:
											{
												ReagentSelected(P4_REG_E_QTY_ADD, P4_REG_E_WAIT_TIME_ADD);
												break;
											}
											
										}
										
										if (MatchCommand(rec_bufferglob)==BACK)
										{
											memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
											break;
										}
										
									}
									
									break;
								}
								
							}
							
							if (MatchCommand(rec_bufferglob)==BACK)
							{
								memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
								break;
							}
						}
						break;
					  }
					  
			case SERVICE:
				{
					while(1)
					{
						_delay_ms(DELAY_IN_LOOP);
						SpinTimeSelected(SPIN_TIME_ADD);
						break;
					}
					break;
				}
				
			case TESTBENCH:
				{
					while(1)
					{
						if (!strcmp(rec_bufferglob, "motorA_ON"))
							GPIO_WriteToPin(&Reagent_A_pump, HIGH);
						if (!strcmp(rec_bufferglob, "motorA_OFF"))
							GPIO_WriteToPin(&Reagent_A_pump, LOW);
						
						if (!strcmp(rec_bufferglob, "motorB_ON"))
							GPIO_WriteToPin(&Reagent_B_pump, HIGH);
						if (!strcmp(rec_bufferglob, "motorB_OFF"))
							GPIO_WriteToPin(&Reagent_B_pump, LOW);
						
						if (!strcmp(rec_bufferglob, "motorC_ON"))
							GPIO_WriteToPin(&Reagent_C_pump, HIGH);
						if (!strcmp(rec_bufferglob, "motorC_OFF"))
							GPIO_WriteToPin(&Reagent_C_pump, LOW);
						
						if (!strcmp(rec_bufferglob, "motorD_ON"))
							GPIO_WriteToPin(&Reagent_D_pump, HIGH);
						if (!strcmp(rec_bufferglob, "motorD_OFF"))
							GPIO_WriteToPin(&Reagent_D_pump, LOW);
						
						if (!strcmp(rec_bufferglob, "motorE_ON"))
							GPIO_WriteToPin(&Reagent_E_pump, HIGH);
						if (!strcmp(rec_bufferglob, "motorE_OFF"))
							GPIO_WriteToPin(&Reagent_E_pump, LOW);
					
						if (!strcmp(rec_bufferglob, "motorDRAIN_ON"))
							GPIO_WriteToPin(&Drain_pump, HIGH);
						if (!strcmp(rec_bufferglob, "motorDRAIN_OFF"))
							GPIO_WriteToPin(&Drain_pump, LOW);
					
						if (!strcmp(rec_bufferglob, "FAN_ON"))
							GPIO_WriteToPin(&Fan, HIGH);
						if (!strcmp(rec_bufferglob, "FAN_OFF"))
							GPIO_WriteToPin(&Fan, LOW);
					
						if (!strcmp(rec_bufferglob, "BLOWER_ON"))
							GPIO_WriteToPin(&Blower, HIGH);
						if (!strcmp(rec_bufferglob, "BLOWER_OFF"))
							GPIO_WriteToPin(&Blower, LOW);
					
					
						if (MatchCommand(rec_bufferglob)==BACK)
						{
							memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
							break;
						}
						//memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
					}
					break;
				}
					
		}
		
    }
	
	return 0;
}



ISR(USART2_RX_vect)
{
	unsigned char rec = USART2_Receive();
	USART2_Transmit(rec);
	//USART0_Transmit(rec);
	if ( (rec == '<') && (rec_start == 0)) // new packet receiving
	{
		myindex = 0;
		rec_start = 1;
		memset(rec_buff, '\0', PACKET_SIZE * sizeof(rec_buff[0]));		// rec_buff clear
		memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));	// rec_bufferglob clear
	}
	
	else if ( ( rec != '>') && (rec_start == 1)  )				// collecting data in buffer
	{
		rec_buff[myindex] = rec;
		myindex++;
		//rec = '\0';
		
	}
	
	if ( (rec == '>' ) && (rec_start == 1) )					// ending character received
	{
		strcpy(rec_bufferglob, rec_buff);						// copy data into rec_bufferglob
		rec_stop = 1;											// rec_stop flag 1
		rec_start = 0;											// clear flag
		memset(rec_buff, '\0', PACKET_SIZE * sizeof(rec_buff[0]));		// clearing rec_buff so that it can receive new data
		
	}
}



ISR(USART0_RX_vect)
{
	unsigned char rec = USART0_Receive();
	USART2_Transmit(rec);
	//USART0_Transmit(rec);
	if ( (rec == '<') && (rec_start == 0)) // new packet
	{
		myindex = 0;
		rec_start = 1;
		memset(rec_buff, '\0', PACKET_SIZE * sizeof(rec_buff[0]));
		memset(rec_bufferglob, '\0', PACKET_SIZE * sizeof(rec_bufferglob[0]));
	}
	
	else if ( ( rec != '>') && (rec_start == 1)  )	// collecting data in buffer
	{
		rec_buff[myindex] = rec;
		myindex++;
		//rec = '\0';
		
	}
	
	if ( (rec == '>' ) && (rec_start == 1) )
	{
		strcpy(rec_bufferglob, rec_buff);
		//USART2_transmitstring(rec_bufferglob);
		rec_stop = 1;
		rec_start = 0;
		memset(rec_buff, '\0', PACKET_SIZE * sizeof(rec_buff[0]));
		
	}
	if(myindex > (PACKET_SIZE -1 ))
	{
		myindex = 0;
		rec_start = 0;
	}
}




///////////////////////////////////---- ISR for TIMER1_OVF_vect -----////////////////////////////////////////
/*
ISR(TIMER1_OVF_vect)
{
	time_in_seconds++;
	USART2_Transmit(time_in_seconds);
	txtime--;
	if (txtime == 0)
	{
		txflag=1;		// flag ON to indicate time has over
		TCNT1 =0xC2F7;	// This value will generate 1 Sec delay for prescaler 1024 and 16 mhz crystel
		TCCR1B = (5<<0);	// prescalar 1024
	}
	//USART2_transmitstring("int ");
	TCNT1 =0xC2F7;	// This value will generate 1 Sec delay for prescaler 1024 and 16 mhz crystel
	TCCR1B = (5<<0);	// prescaler 1024
	
}
*/
ISR(TIMER1_OVF_vect)
{
	time_in_seconds++;
	itoa(time_in_seconds, buffer, 10);
	USART2_transmitstring("\ntime in seconds = ");
	USART2_transmitstring(buffer);
	memset(buffer, '\0', PACKET_SIZE * sizeof(buffer[0]));	// rec_bufferglob clear
	TCNT1 =0xC2F7;	// This value will generate 1 Sec delay for prescaler 1024 and 16 mhz crystel
	TCCR1B = (5<<0);	// prescaler 1024
	
}

