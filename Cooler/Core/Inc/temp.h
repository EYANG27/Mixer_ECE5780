#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"

extern float temp;
extern uint8_t Presence;
extern uint8_t Temp_byte1;
extern uint8_t Temp_byte2;

void Init_TIM6(void);

void delay(uint32_t us);

uint8_t TempStart(void);

void TempWrite(uint8_t data);

uint8_t TempRead(void);

float ConvertTemp(uint8_t LSB, uint8_t MSB);

void Init_TempSense(GPIO_TypeDef * GPIO, uint16_t TX, uint16_t RX);

float TempSense(void);