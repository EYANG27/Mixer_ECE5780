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
	* TODO: write documentation
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

//#include "DallasTemperature_STM32.h"
//#include "OneWire_STM32.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void Init_USART3(void);
void Init_LEDs(void);
void Transmit_Char(char c);
void Transmit_String(char* str);
void USART3_4_IRQHandler(void);
void Process_TDR(char valve_ID, char action_ID);
void Control_Valves(void);

void Init_ADC(void);
void Calibrate_and_start_ADC(void);
void Init_Valve_Pins(void);


/* Global variables -----------------------------------------------*/
int GREEN = (1 << 9);
int ORANGE = (1 << 8);
int BLUE = (1 << 7);
int RED = (1 << 6);

#define DS18B20_PORT GPIOA
#define TX_PIN GPIO_PIN_8
#define RX_PIN GPIO_PIN_9

// Valve control
volatile char received_byte;
volatile uint8_t message_received_flag;
static char valve_ID;
static char action_ID;

/**
  * @brief  The application entry point.
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
	Init_ADC();
	Init_Valve_Pins();
	Init_Pump();
	Init_TempSense(DS18B20_PORT, TX_PIN, RX_PIN);
	Init_TIM6();
	
	// Set initial conditions
	message_received_flag = 0;
	received_byte = '&'; // Initialized it to some junk that would never be processed.
	
	// Start the ADC.
	Calibrate_and_start_ADC();
	
	// Use a timer to run the temperature sensor.
//	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable timer 2
//	TIM2->PSC = 7999;
//	TIM2->ARR = 5000;
	// Configure the timer to generate an interrrupt on the UEV event. DIER enables direct memory access for a given timer.
//	TIM2->DIER |= 1;
	// Configure timer 2 to start.
//	TIM2->CR1 |= 1;
	// Enable the interrupt in the NVIC for the timer
//	NVIC_EnableIRQ(TIM2_IRQn);
//	NVIC_SetPriority(TIM2_IRQn, 3);
	
	// The two methods Control_Valves and Sense_Temperature are time sharing (running concurrently) because of the timer.
//	Control_Valves();
while(1)
	PI_update();
}

/*
* Interrupt handler for timer 2
*/
void TIM2_IRQHandler(void) {
	
	// Run the temperature sensor.
	PI_update();
	
	// Clear the pending flag for the interrrupt status register
	TIM2->SR ^= 1;
}

// _________________________________________________________ Peripheral and Pin Initializations __________________________________________________________________________________
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

void Init_USART3(void) {
	
	// Feed the clock into the USART3 peripheral
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	// Set the discovery board pins (connected to USART3) to alternate function mode (10)
	GPIOC->MODER |= 2 << (2*4); // pin 4
	GPIOC->MODER &= ~(1<< (2*4) );
	
	GPIOC->MODER |= 2 << (2*5); // pin 5
	GPIOC->MODER &= ~(1<< (2*5) );
	
	// Multiplex to alternate function mode 1 ([3:0] = 0001)
	GPIOC->AFR[0] &= ~(14 << 4*4); // pin 4
	GPIOC->AFR[0] |= (1 << 4*4);
	
	GPIOC->AFR[0] &= ~(14 << 4*5); // pin 5
	GPIOC->AFR[0] |= (1 << 4*5);
	
  // Set the Baud rate for communcation to be 115,200 bits/second. The system clock is 8 MHz.
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;
	
	// Enable USART in the control register.
	USART3->CR1 |= 12; // ..1100, bit 2 enables the receiver and bit 3 enables the transmitter.
	USART3->CR1 |= 1; // bit zero is the general enable bit.
	
	
	// Enable the "receive register not empty interrrupt".
	USART3->CR1 |= 1<<5;
		
	// Enable the interrupt in the NVIC and set its priority.
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 2);
}

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
	
	// Initialize off
	GPIOB->ODR &= ~(7 << 11);
}

// _________________________________________________________ Interrupt Handlers __________________________________________________________________________________

/*
* Does nothing if the RECEIVE data register is empty, otherwise this method saves the data in the Receive data register (RDR)
*  to a global variable and sets a flag indicating that action.
*/
void USART3_4_IRQHandler(void) {
	if( (USART3->ISR & (1<<5) ) ) {
		received_byte = USART3->RDR | 0xFF;
		message_received_flag = 1;
	}
}
// _________________________________________________________ USART Handling __________________________________________________________________________________
void Control_Valves(void) {
	while(1) {
		
		// Display a prompt to the user to get two characters.
		Transmit_String("\nSpecify the valve to open/close: ");
		
		// Receive the first character
		while( !(USART3->ISR & (1<<5) )) {
		}
		received_byte = USART3->RDR & 0xFF;
		Transmit_String("\nValve "); Transmit_Char(received_byte); Transmit_String(" is selected.");
		
		// If it's an accepted valve ID, save it and receive the second character.
		if ( received_byte == '1' || received_byte == '2' || received_byte == '3') {
			valve_ID = received_byte;
			
			// Recieve the second character
			while( !(USART3->ISR & (1<<5) )) {
			}
			received_byte = USART3->RDR & 0xFF;
			action_ID = received_byte;
			
			// If it's not a valid command, display an error message and abort the operation.
			char* valve_op = "failed.";
			if(received_byte == 'o')
				valve_op = "\nOpening valve ";
			else if(received_byte == 'c')
				valve_op = "\nClosing valve ";
			else{
				Transmit_String("\n Invalid operation.");
				continue;
			}
			
			// Otherwise display what should be going on with the valve (Ex: "Closing valve 2").
			Transmit_String(valve_op); Transmit_Char(received_byte);
			
			// Then perform the operation.
			Process_TDR(valve_ID, action_ID);
			continue;
		}
		
		// Otherwise the input was invalid. Broadcast an error message and return to the beginning state.
		Transmit_String("\nThat's is not a valid command, Try again."); // Error message for invalid character
	}
}
/*
* Does nothing while the transmit data register is empty.
* When it's not empty, it transmits the character.
*/
void Transmit_Char(char c) {
	
	while( !(USART3->ISR & (1<<7) ) ) {
		//HAL_Delay(100);
	}
	USART3->TDR = c;
}

/*
* Loops over each character in the "str" array and transmits the character there. 
*/
void Transmit_String(char* str) {
	 for(int i = 0; str[i] != '\0'; i++) {
		 //HAL_Delay(50);
		 Transmit_Char(str[i]);
	 }
}

/*
* Drives the GPIO pins for controlling the valves.
* 
* The valve_ID is a letter that indicates which valve to operate on (1, 2, or 3).
* The second element is 'o' or 'c' to specify open or close.
* If the input is not in the set of predefined commands then an error message is displayed.
*/
void Process_TDR(char valve_ID, char action_ID) {
	
	int LED;
	int odr_mask;
	
	// Specify which valve (and LED) to operate on.
	switch(valve_ID) {
		case '1':
			LED = RED;
			odr_mask |= (1 << 11);
			break;
		case '2':
			LED = GREEN;
			odr_mask |= (1 << 12);
			break;
		case '3':
			LED = BLUE;
			odr_mask |= (1 << 13);
			break;
		default:
			Transmit_String("\nSomething went wrong. ");
			return;
	}
	
	// Determine the specified action and perform it.
	switch(action_ID) {
		case 'o':
			GPIOB->ODR |= (odr_mask);
			GPIOC->ODR |= (LED);
			 
			break;
		case 'c':
			GPIOC->ODR &= ~(LED);
			GPIOB->ODR &= ~(odr_mask);
			break;
		default:
			Transmit_String("Wrong number. ");
	}
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
