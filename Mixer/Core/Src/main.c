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
	*
	*
	*
	*
	*
	*
	*
	* @Authors: Eddison Yang, Freddie Rice, Shem Snow
  */

#include "main.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

// USART methods
void Init_USART3(void);
void Init_LEDs(void);
void Transmit_Char(char c);
void Transmit_String(char* str);
void USART3_4_IRQHandler(void);
void Process_TDR(char valve_ID, char action_ID);

/* Global variables -----------------------------------------------*/
int GREEN = (1 << 9);
int ORANGE = (1 << 8);
int BLUE = (1 << 7);
int RED = (1 << 6);

volatile char received_byte;
volatile uint8_t message_received_flag;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

  // Reset, , enable, and configure the clock and all peripherals.
  HAL_Init();
  Init_LEDs();
	Init_USART3();
  SystemClock_Config();
	
	// Set initial conditions
	message_received_flag = 0;
	received_byte = '&'; // Initialized it to some junk that would never be processed.
	
	
	// Instantiate a null-terminated array to hold incoming messages.
	char valve_ID;
	char action_ID;

	// Run
  while (1) {
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
			
			Process_TDR(valve_ID, action_ID);
			continue;
		}
		
		// Otherwise the input was invalid. Broadcast an error message and return to the beginning state.
		Transmit_String("\nThat's is not a valid command, Try again."); // Error message for invalid character
    
  }
}

// _________________________________________________________ Peripheral Initializations __________________________________________________________________________________
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
* Decodes the input message as an action.
* The first element is a letter that indicates which LED to change.
* The second element is the number and indicates the action to take on it.
* If the input is not in the set of predefined commands then an error message is displayed.
*/
void Process_TDR(char valve_ID, char action_ID) {
	
	int LED;
	int valve_register_bit;
	switch(valve_ID) {
		case '1':
			LED = RED;
		valve_register_bit = 1; // TODO:
			break;
		case '2':
			LED = GREEN;
			valve_register_bit = 1; // TODO:
			break;
		case '3':
			LED = BLUE;
			valve_register_bit = 1; // TODO:
			break;
		default:
			Transmit_String("\nSomething went wrong. ");
			return;
	}

	switch(action_ID) {
		case 'o':
			GPIOC->ODR &= ~(LED);
			// TODO: drive the pin on the STM corresponding to the valve. 
			break;
		case 'c':
			GPIOC->ODR |= LED;
			// TODO: drive the pin on the STM corresponding to the valve.
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
