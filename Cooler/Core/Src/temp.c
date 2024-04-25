/* -------------------------------------------------------------------------------------------------------------
 *  Temperature Sensor Functions
 * -------------------------------------------------------------------------------------------------------------
 * 
 * Uses PA8,9
 * 
 * 
 */ 

#include "temp.h"

#define DS18B20_PORT GPIOA
#define TX_PIN GPIO_PIN_8
#define RX_PIN GPIO_PIN_9

float temp;
float prevTemp;
uint8_t Presence;
uint8_t Temp_byte1;
uint8_t Temp_byte2;

void Init_TIM6(void) {
	// Microsecond timer setup
	__HAL_RCC_TIM6_CLK_ENABLE();

	TIM6->PSC = 7;
	TIM6->ARR = 0xFFFF;
	
	TIM6->CR1 |= TIM_CR1_CEN;
}

void delay (uint32_t us)
{
    uint16_t start = TIM6->CNT;
    while ((TIM6->CNT - start)<us);
}

void Init_TempSense(GPIO_TypeDef * GPIO, uint16_t TX, uint16_t RX) {
	GPIO_InitTypeDef TXInit = {
		TX,
		GPIO_MODE_OUTPUT_OD,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_NOPULL
	};
	HAL_GPIO_Init(GPIO, &TXInit);
	
	GPIO_InitTypeDef RXInit = {
		RX,
		GPIO_MODE_INPUT,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_NOPULL
	};
	HAL_GPIO_Init(GPIO, &RXInit);
}

uint8_t TempStart (void)
{
	uint8_t Response = 0;
	HAL_GPIO_WritePin(GPIOA, TX_PIN, GPIO_PIN_RESET);  // pull the pin low
	delay(480);   // delay according to datasheet
	
	HAL_GPIO_WritePin(GPIOA, TX_PIN, GPIO_PIN_SET);  // release line
	delay(80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin(GPIOA, RX_PIN))) Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = 0;

	delay(400); // 480 us delay totally.

	return Response;
}

void TempWrite(uint8_t data)
{

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			HAL_GPIO_WritePin (DS18B20_PORT, TX_PIN, GPIO_PIN_RESET);  // pull the pin LOW
			delay (1);  // wait for 1 us

			HAL_GPIO_WritePin (DS18B20_PORT, TX_PIN, GPIO_PIN_SET);  // release the pin
			delay (60);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			HAL_GPIO_WritePin (DS18B20_PORT, TX_PIN, GPIO_PIN_RESET);  // pull the pin LOW
			delay (60);  // wait for 60 us

			HAL_GPIO_WritePin (DS18B20_PORT, TX_PIN, GPIO_PIN_SET);  // release the pin
		}
		delay(4); // recovery delay
	}
}

uint8_t TempRead(void)
{
	uint8_t value=0;

	for (int i=0;i<8;i++)
	{

		HAL_GPIO_WritePin (GPIOA, TX_PIN, GPIO_PIN_RESET);  // pull the data pin LOW
		delay (1);  // wait for 2 us
		
		HAL_GPIO_WritePin (GPIOA, TX_PIN, GPIO_PIN_SET);  // release line
		delay(15); // recovery delay
		
		if (HAL_GPIO_ReadPin(GPIOA, RX_PIN))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay (60);  // wait for 60 us
	}
	return value;
}

float ConvertTemp(uint8_t LSB, uint8_t MSB) {
	float result = 0;
	
	if (LSB & (1 << 0)) {
		result = result + 0.0625;
	}
	if (LSB & (1 << 1)) {
		result = result + 0.125;
	}
	if (LSB & (1 << 2)) {
		result = result + 0.25;
	}
	if (LSB & (1 << 3)) {
		result = result + 0.5;
	}
	if (LSB & (1 << 4)) {
		result = result + 1;
	}
	if (LSB & (1 << 5)) {
		result = result + 2;
	}
	if (LSB & (1 << 6)) {
		result = result + 4;
	}
	if (LSB & (1 << 7)) {
		result = result + 8;
	}
	
	if (MSB & (1 << 0)) {
		result = result + 16;
	}
	if (MSB & (1 << 1)) {
		result = result + 32;
	}
	if (MSB & (1 << 2)) {
		result = result + 64;
	}
	if (MSB & (1 << 3)) {
		result = -result;
	}
	
	return result;
	
}

float TempSense(void) {
	Presence = TempStart(); // Reset pulse
	HAL_Delay(1);
	TempWrite (0xCC);  // skip ROM
	TempWrite (0x44);  // Issue temp convert request
	HAL_Delay (800);	 // Wait for temp conversion to finish

	Presence = TempStart(); // Reset pulse
	delay(250);
	TempWrite (0xCC);  // skip ROM
	delay(250);
	TempWrite (0xBE);  // Read Scratch-pad
	delay(250);

	Temp_byte1 = TempRead(); // temp LSB
	delay(250);
	Temp_byte2 = TempRead(); // temp MSB
	delay(250);
	
	TempStart(); // Issue reset to stop reading data bytes
	
	temp = ConvertTemp(Temp_byte1, Temp_byte2);
	float diff = prevTemp - temp;
	if (diff<0) diff=-diff;
	if (temp < 0) {
		temp = prevTemp;
	}
	return temp;
}