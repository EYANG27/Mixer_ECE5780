#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"

#define GREEN (1 << 9)
#define ORANGE (1 << 8)
#define BLUE (1 << 7)
#define RED (1 << 6)

/* Variables -----------------------------------------------*/
static char valve_ID;
static char action_ID;


/* Functions -----------------------------------------------*/
void Init_USART3(void);
void Transmit_Char(char c);
void Transmit_String(char* str);
void USART3_4_IRQHandler(void);
void Process_TDR(char valve_ID, char action_ID);
void Control_Valves(void);