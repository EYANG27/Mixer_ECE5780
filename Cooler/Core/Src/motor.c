/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 * This lab makes so we can control a motor using a push button.
 * Pressing the button progresses the system into the following states in order:
 * 
 * - Speed changes from 0 to 80 RPM
 * - Speed changes from 80 to 50 RPM
 * - Speed changes from 50 to 80 RPM
 * - Speed changes from 80 to 0 RPM
 */
#include "motor.h"
#include "temp.h"

volatile int16_t desired_temp = 20;		// Desired fluid temperature
volatile int16_t error_integral = 0;  // Integrated error signal
volatile uint8_t duty_cycle = 0;    	// Output PWM duty cycle
volatile int16_t error = 0;         	// Speed error signal
volatile uint8_t Kp = 25;            	// Proportional gain
volatile uint8_t Ki = 0;            	// Integral gain

// Sets up the PWM and direction signals to drive the H-Bridge
void Init_Pump(void) {
    
    // Set up pin PA4 for H-bridge PWM output (TIMER 14 CH1)
    GPIOA->MODER |= (1 << 9);
    GPIOA->MODER &= ~(1 << 8);

    // Set PA4 to AF4,
    GPIOA->AFR[0] &= 0xFFF0FFFF; // clear PA4 bits,
    GPIOA->AFR[0] |= (1 << 18);

    // Set up a PA5, PA6 as GPIO output pins for motor direction control
    GPIOA->MODER &= 0xFFFFC3FF; // clear PA5, PA6 bits,
    GPIOA->MODER |= (1 << 10) | (1 << 12);
    
    //Initialize one direction pin to high, the other low
    GPIOA->ODR |= (1 << 5);
    GPIOA->ODR &= ~(1 << 6);

    // Set up PWM timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->CR1 = 0;                         // Clear control registers
    TIM14->CCMR1 = 0;                       // (prevents having to manually clear bits)
    TIM14->CCER = 0;

    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
    TIM14->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
    TIM14->CCER |= TIM_CCER_CC1E;           // Enable capture-compare channel 1
    TIM14->PSC = 1;                         // Run timer on 24Mhz
    TIM14->ARR = 1200;                      // PWM at 20kHz
    TIM14->CCR1 = 0;                        // Start PWM at 0% duty cycle
    
    TIM14->CR1 |= TIM_CR1_CEN;              // Enable timer
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(uint8_t duty) {
    if(duty <= 100) {
        TIM14->CCR1 = ((uint32_t)duty*TIM14->ARR)/100;  // Use linear transform to produce CCR1 value
        // (CCR1 == "pulse" parameter in PWM struct used by peripheral library)
    }
}

/* Run PI control loop
*
* Make sure to use the indicated variable names. This allows STMStudio to monitor
* the condition of the system!
*
* target_rpm -> target motor speed in RPM
* motor_speed -> raw motor speed in encoder counts
* error -> error signal (difference between measured speed and target)
* error_integral -> integrated error signal
* Kp -> Proportional Gain
* Ki -> Integral Gain
* output -> raw output signal from PI controller
* duty_cycle -> used to report the duty cycle of the system 
* adc_value -> raw ADC counts to report current
*
*/
void PI_update(void) {
	
	float temperature = TempSense(); // Read temperature sensor
	temp = temperature; // debug viewing
	
	// Calculate error signal and write it to the "error" variable
  error = temperature - desired_temp;
    /* Hint: Remember that your calculated motor speed may not be directly in RPM!
     *       You will need to convert the target or encoder speeds to the same units.
     *       I recommend converting to whatever units result in larger values, gives
     *       more resolution.
     */
    
    
  // Calculate integral portion of PI controller, write to "error_integral" variable
	error_integral = error_integral + error;
	
  // Clamp the value of the integral to a limited positive range (0-3200).
  if(error_integral < 0)
		error_integral = 0;
	else if(error_integral > 3200)
		error_integral = 3200;
    
  // Calculate proportional portion, add integral and write to "output" variable
  int16_t output = (Kp*error) + Ki * error_integral;
  
	 
	// Clamp the output value between 0 and 100
	if(output < 0)
		output = 0;
	else if(output > 100)
		output = 100;
    
  pwm_setDutyCycle(output);
  duty_cycle = output;            // For debug viewing

}
