#include "Valves.h"
//#include <stdio.h>
//#include <stdlib.h>
//#include "stm32f0xx.h"

volatile char received_byte = 0;
volatile uint8_t message_received_flag = '&';

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
			GPIOB->ODR &= ~(odr_mask);
			GPIOC->ODR |= (LED);
			 
			break;
		case 'c':
			GPIOB->ODR |= (odr_mask);
			GPIOC->ODR &= ~(LED);
			break;
		default:
			Transmit_String("Wrong number. ");
	}
}