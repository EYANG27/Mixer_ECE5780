/* -------------------------------------------------------------------------------------------------------------
 *  Temperature Sensor Functions
 * -------------------------------------------------------------------------------------------------------------
 * 
 * Uses the "one-wire" communication protocol that uses three connections: power, ground, data.
 * 
 * GPIO pins PA8,9 are connected to power and data lines.
 * To initiate communication, a master device pulls the data line low then specifies a slave address and finally sends a message.
 * In this case, the message is a temperature reading which requires a fixed amount of time to occur so this code manually implements
 * time delays to allow temperature readings.
 *
 * @Authors: Edison Yang, Freddie Rice, Shem Snow
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

/*
* Timer 6 is used to ensure the temperature readings have adaquate time to report to the master device.
*/
void Init_TIM6(void) {
	// Microsecond timer setup
	__HAL_RCC_TIM6_CLK_ENABLE();

	TIM6->PSC = 7;
	TIM6->ARR = 0xFFFF;
	
	TIM6->CR1 |= TIM_CR1_CEN;
}

/*
* Each step on the one wire protocol requires a different amount of time so we implemented a delay function that 
* can be told how much time to wait. 
*/
void delay (uint32_t us)
{
    uint16_t start = TIM6->CNT;
    while ((TIM6->CNT - start)<us);
}

/*
* Configures the temperature sensor to communicate using the one wire protocol.
*/
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
	
	/*
	Presence = TempStart(); // Reset pulse
	HAL_Delay(1);
	TempWrite (0xCC);  // skip ROM
	delay(250);
	TempWrite (0x4E);  // Issue write scratchpad
	delay(250);
	TempWrite(0x00); // MSB of temp
	delay(250);
	TempWrite(0x00); // LSB of temp
	delay(250);
	TempWrite(0x1F); // write to control register for 9 bit resolution
	*/
}

/*
* Since the one wire protocol requires a lengthy startup process, we've separated the concerns of performing the actual temperature sensing and the
* repetitive steps of the protocol.
*/
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

/*
* Saving temperature is a separate concern than reading temperature. This method addresses the concerns of saving the temperature being read
* into a place it can be referenced later.
*/
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

/*
* Reads a temperature value from the STM's memory. This is called by the motor.c file to determine the duty cycle for the pump.
*/
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

/*
* Converts the digital values obtained by the temperature sensor into degrees celsius.
*/
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


/*
* The actual temperature sensing code. The data line is pulled low and the receiver is specified.
* Then a temperature sensing command is initiates, that temperature is converted into celsius, and stored into the stm's memory.
*/
float TempSense(void) {
	Presence = TempStart(); // Reset pulse
	HAL_Delay(1);
	TempWrite (0xCC);  // skip ROM (means this message is for all connected device)
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
	} else {
		prevTemp = temp;
	}
	return temp;
}