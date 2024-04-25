 /* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
	*
	*
	*
	* This main file, along with the four supporting .c files holds the firmware for a cooling system.
	* It uses USART to open and close valves intended to supply or drain fluid in the system.
	* It measures temperature and compares it to a threshold value. The error between read and actual temperature is used in a 
	* Proportional Integral (PI) control system to drive a pump that is intended to pass hot water through a heat exchanger.
	* This program is capable of running two types of temperature sensors.
	* 	1: An analog thermocouple whose voltage changes at the terminals as the physical device is subject to temperature changes.
	*		2: A digital temperature sensor that uses a "one-wire" protocol (which is pretty much I2C but with only a data line and no clock).
	* 
	* Pin Table:
	*		PA4 - Motor Enable
	*		PA5,6 - Motor Direction
	* 	PC6,7,8,9 - LEDs
	*		PC4,5 - USART3 Tx and Rx for the valves
	* 	PA8,9 - Temperature sensor TX and RX
	*		PB11,12,13 - Valve control
	*
	* @Authors: Edison Yang, Freddie Rice, Shem Snow
  */

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"
#include "motor.h"
#include "temp.h"
#include "Valves.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

// Valve control.
void Init_LEDs(void);
void Init_Valve_Pins(void);

// Temperature control.
void Run_Temp_Timer(void);
#define DS18B20_PORT GPIOA
#define TX_PIN GPIO_PIN_8
#define RX_PIN GPIO_PIN_9


// ADC code we didn't use.
void Init_ADC(void);
void Calibrate_and_start_ADC(void);
void Sense_Temperature(void);


/**
  * @brief  The application entry point. It initializes all the peripherals and pins necessary for the system.
	* Then it starts a timer that will interrupt the main process at every timer expiration to sense the temperature and adjust the pump accordingly.
	* With the timer running, the main process of controlling the vales is entered forever. It just uses USART to constantly check for 
	* user inputs that will open or close the valves.
  * @retval int
  */
int main(void) {

  // Reset the system.
  HAL_Init();
  SystemClock_Config();
	
	// Feed the clock into the peripherals we'll use.
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Configure the pins and each peripheral
	Init_LEDs();
	Init_USART3();
	Init_Valve_Pins();
	Init_Pump();
	Init_TempSense(DS18B20_PORT, TX_PIN, RX_PIN);
	Init_TIM6();
	
	// Alternatively, an ADC may be used to sense the temperature (instead of a digital temperature sensor).
	Init_ADC();
	Calibrate_and_start_ADC();
	
	Run_Temp_Timer();
	
	// The two methods Control_Valves and Sense_Temperature are time sharing (running concurrently) because of the timer.
	Control_Valves();
}

/*
* Interrupt handler for timer 2. This method determines which type of temperature sensor is used in the system.
*/
void TIM2_IRQHandler(void) {
	
	// Run the temperature sensor.
	PI_update();
	//Sense_Temperature();
	
	// Clear the pending flag for the interrrupt status register
	TIM2->SR ^= 1;
}

// _________________________________________________________ Peripheral and Pin Initializations __________________________________________________________________________________

/*
* Configures the LEDs to be controllable in the GPIOC->ODR. LEDs are initialized to off.
*/
void Init_LEDs(void) {
	
	// Initialize Port C: LEDs and pins
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Set the MODER to output mode (01)
	GPIOC->MODER &= ~(1<< (2*6 +1) );
	GPIOC->MODER &= ~(1<< (2*7 +1) );
	GPIOC->MODER &= ~(1<< (2*8 +1) );
	GPIOC->MODER &= ~(1<< (2*9 +1) );
	
	GPIOC->MODER |= 1 << 2*6;
	GPIOC->MODER |= 1 << 2*7;
	GPIOC->MODER |= 1 << 2*8;
	GPIOC->MODER |= 1 << 2*9;
	
	// Set output type register to push-pull (0)
	GPIOC->OTYPER &= ~(1<<6);
	GPIOC->OTYPER &= ~(1<<7);
	GPIOC->OTYPER &= ~(1<<8);
	GPIOC->OTYPER &= ~(1<<9);
	
	// Set output speed register to low speed (x0)
	GPIOC->OSPEEDR &= ~(1 << 2*6);
	GPIOC->OSPEEDR &= ~(1 << 2*7);
	GPIOC->OSPEEDR &= ~(1 << 2*8);
	GPIOC->OSPEEDR &= ~(1 << 2*9);
	
	// Set the pins to no pull-up, pull-down (00)
	GPIOC->PUPDR &= ~(3<<2*6);
	GPIOC->PUPDR &= ~(3<<2*7);
	GPIOC->PUPDR &= ~(3<<2*8);
	GPIOC->PUPDR &= ~(3<<2*9);
	
	// Initialize each light to be off
	GPIOC->ODR &= ~RED;
	GPIOC->ODR &= ~ORANGE;
	GPIOC->ODR &= ~BLUE;
	GPIOC->ODR &= ~GREEN;
}


/*
* The valves we used don't just require an on/off signal to operate. They need 5 volts flowing one way to open and 5 v flowing the other way to close.
* Because of this, we opted for using an H-bridge that requires two inputs whose values are opposite to each other to control the direction of current.
* 
* Therefore this method does the same thing as Init_LEDs except it uses GPIOB pins 11-13 and sets them to be the exact opposite signal as the LED
* pins PC6,8, and 9 which we used for valve control in our project.
*/
void Init_Valve_Pins(void) {
	
	// Configure GPIOB 11, 12, and 13 to control the valves.

	// Configure the pins to general purpose output mode (01)
	GPIOB->MODER &= ~(2 << 2*11);
	GPIOB->MODER &= ~(2 << 2*12);
	GPIOB->MODER &= ~(2 << 2*13);
	GPIOB->MODER |= (1 << 2*11);
	GPIOB->MODER |= (1 << 2*12);
	GPIOB->MODER |= 1 << 2*13;
	
	// Push Pull (0)
	GPIOB->OTYPER &= ~(1 << 11);
	GPIOB->OTYPER &= ~(1 << 12);
	GPIOB->OTYPER &= ~(1 << 13);
	
	// Low speed (x0)
	GPIOB->OSPEEDR &= ~(1 << 2*11);
	GPIOB->OSPEEDR &= ~(1 << 2*12);
	GPIOB->OSPEEDR &= ~(1 << 2*13);
	
	// Pull down (10)
	GPIOB->PUPDR |= (2 << 2*11);
	GPIOB->PUPDR |= (2 << 2*12);
	GPIOB->PUPDR |= (2 << 2*13);
	GPIOB->PUPDR &= ~(1 << 2*11);
	GPIOB->PUPDR &= ~(1 << 2*12);
	GPIOB->PUPDR &= ~(1 << 2*13);
	
	// Initialize on (so it's the opposite of the LED pins).
	GPIOB->ODR |= (7 << 11);
}
// _________________________________________________________ Temperature Control __________________________________________________________________________________

/*
* Enables the timer 2 peripheral and sets its priority in the NVIC. Timer 2 is configured by the PSC and ARR to expire every 5 seconds.
*
* Special instruction: the priority of timer 2 should be lower than the systick otherwise the program will deadlock because systick is 
* used in the temperature sensing code to ensure adaquate time for reading temperatures.
*/
void Run_Temp_Timer(void) {
	// Use a timer to run the temperature sensor.
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable timer 2
	TIM2->PSC = 7999;
	TIM2->ARR = 5000;
	// Configure the timer to generate an interrrupt on the update event. DIER enables direct memory access for a given timer.
	TIM2->DIER |= 1;
	// Configure timer 2 to start.
	TIM2->CR1 |= TIM_CR1_CEN;
	// Enable the interrupt in the NVIC for the timer
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 3);
	
	// Increase priority of systick, it is important to the functionality of the temp sense code, so it must execute
	NVIC_SetPriority(SysTick_IRQn, 1);
}



// _________________________________________________________ System __________________________________________________________________________________
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
}
#endif


// _________ ADC code we could, but decided not to, use because the thermocouple's voltage doesn't change with temperature _________________________________________

/*
* Enables and configures the ADC to channel 10 so that it reads analog values on PC0 and saves them into the data register ADC1->DR
*/
void Init_ADC(void) {
	
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	
	// ADC1 on channel 10 automatically uses PC0.
	GPIOC->MODER |= 3; // Analog mode (11)
	GPIOC->PUPDR &= ~3; // No pull-up, pull down (00)
	ADC1->CHSELR |= (1 << 10 ); // Configure the pin for ADC conversion on channel 10
	
	// 8-bit resolution (10)
	ADC1->CFGR1 |= (2 << 3);
	ADC1->CFGR1 &= ~(1 << 3);
	
	// Continuous conversion mode
	ADC1->CFGR1 |= (1 << 13);
	
	// Hardware triggers disabled (software-triggered only)
	ADC1->CFGR1 &= ~(3 << 10);
	
	
	// Alternatively, we can use ADC1 on channel 11 and use PC1.
	
	
	// 8-bit resolution
	//ADC1->CFGR1 |= (0x2 << ADC_CFGR1_RES_Pos);
	
	// Continuous conversion mode
	//ADC1->CFGR1 |= (ADC_CFGR1_CONT_Msk);
	
	// Hardware triggers disabled (software trigger only)
	//ADC1->CFGR1 &= ~(ADC_CFGR1_EXTEN_Msk);
	
	// Set channel 11 for PC1
	//ADC1->CHSELR |= (ADC_CHSELR_CHSEL11);
}

void Calibrate_and_start_ADC(void) {
	
	// ___________________Calibrate (reference appendix A.7.1)___________________
	// Calibration is initialted when ADEN = 1. So initialize it to zero/disable it.
	if( (ADC1->CR & ADC_CR_ADEN) !=0)
		ADC1->CR |= 1 << 1;
	// Wait for the action to complete.
	while ( (ADC1->CR & ADC_CR_ADEN) != 0) {
	}
	// Clear the DMA bit so 
	ADC1->CFGR1 &= ~(1);
	// Trigger the calibration in the control register
	ADC1->CR |= (1 << 31);
	// Wait for the action to complete.
	while ( (ADC1->CR & ADC_CR_ADCAL) != 0) {
	}
	
	
	// ___________________ Enable Sequence code (reference appendix A.7.1)___________________
	if( (ADC1->ISR & ADC_ISR_ADRDY) != 0)
		ADC1->ISR |= ADC_ISR_ADRDY;
	ADC1->CR |= ADC_CR_ADEN;
	// Wait for the action to complete
	while ( (ADC1->ISR & ADC_ISR_ADRDY) == 0 ) {
	}
	
	// _____________________ Start _____________________
	ADC1->CR |= (1 << 2);
}



/*
* This method sets thresholds for the value of temperature. The values are random numbers the sensor reads and not in Farenheight or Celsius.
* Depending on the observed analog value on PC0, the LED corresponding to that temperature level will brighten. 
* The duty cycle for the pump is also set.
*/
void Sense_Temperature(void) {
	
	// Store the analog value into a variable
	int16_t HOTTEST = 55;
	int16_t ROOM_TEMP = 67;
	int16_t FOUNTAIN_WATER = 80 ;
	int16_t ICE_WATER = 90;
	
	//ConfigPB6();
	int16_t temperature = ADC1->DR;
		
		GPIOC->ODR &= ~(RED | BLUE | GREEN | ORANGE);
		if(temperature < HOTTEST) {
			GPIOC->ODR &= ~(RED | BLUE | GREEN);
			GPIOC->ODR |= ORANGE; // HOTTEST (in fire)
			pwm_setDutyCycle(0);
		}
		else if(temperature < ROOM_TEMP) {
			GPIOC->ODR &= ~(BLUE | GREEN | ORANGE);
			GPIOC->ODR |= RED; // Room Temperature.
			pwm_setDutyCycle(25);
		}
		else if(temperature < FOUNTAIN_WATER) {
			GPIOC->ODR &= ~(RED | BLUE | ORANGE);
			GPIOC->ODR |= GREEN; // Cold (water from the drinking fountain)
			pwm_setDutyCycle(50);
		}
		else if (temperature < ICE_WATER) {
			GPIOC->ODR &= ~(RED | GREEN | ORANGE);
			GPIOC->ODR |= BLUE; // COLDEST (ice water)
			pwm_setDutyCycle(100);
		}
		else { // Indicate an error.
			GPIOC->ODR &= ~(GREEN | RED);
			GPIOC->ODR |= BLUE;
			GPIOC->ODR |= ORANGE;
		}
}
