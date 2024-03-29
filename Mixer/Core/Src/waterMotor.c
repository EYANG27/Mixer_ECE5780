#include "main.h"
#include "tim.h"

/* Define PWM duty cycle values */
#define PWM_MAX_DUTY_CYCLE 1000  // Adjust this value as needed for your pump
#define PWM_MIN_DUTY_CYCLE 500
#define PWM_STEP_SIZE 100

/* PWM duty cycle variable */
uint16_t pwmDutyCycle = PWM_MIN_DUTY_CYCLE;

int main(void)
{
  /* Initialize peripherals */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init(); // Initialize the timer configured for PWM

  /* Start PWM generation */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  while (1)
  {
    /* Increase duty cycle */
    pwmDutyCycle += PWM_STEP_SIZE;
    if (pwmDutyCycle > PWM_MAX_DUTY_CYCLE)
      pwmDutyCycle = PWM_MIN_DUTY_CYCLE; // wrap around to minimum duty cycle
    
    /* Update PWM duty cycle */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwmDutyCycle);
    
    /* Delay between duty cycle changes */
    HAL_Delay(1000); // Adjust delay as needed
  }
}
