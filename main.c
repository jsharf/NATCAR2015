/*
 * main.c
 */
#include <stdbool.h>
#include <stdint.h>

#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "inc/hw_adc.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_uart.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"

#include "fast_utils.h"

//#define PERIOD (4096)
//
//volatile uint32_t deadband;
//
//int main(void)
//{
//	deadband = PERIOD;
////	SysCtlClockSet(
////	SYSCTL_SYSDIV_2_5 |
////	SYSCTL_USE_PLL |
////	SYSCTL_XTAL_16MHZ |
////	SYSCTL_OSC_MAIN);
//
//	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
//
//	PWM
//
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//
//	    GPIOPinConfigure(GPIO_PA0_U0RX);
//	    GPIOPinConfigure(GPIO_PA1_U0TX);
//	    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
//
//	    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600,
//	        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
//
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
//
//	// Enable PE2 as an output
//	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, 1 << 2);
//
//	// Enable PF0 and PF4 as inputs
//	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, (1 << 4) | (1 << 0));
//	GPIOPadConfigSet(GPIO_PORTF_BASE, (1 << 4) | (1 << 0), GPIO_STRENGTH_8MA,
//			GPIO_PIN_TYPE_STD_WPU);
//
//	while (1)
//	{
//
//		if (PERIOD - deadband)
//		{
//			GPIOPinWrite(GPIO_PORTE_BASE, 1 << 2, 1 << 2);
//			SysCtlDelay(PERIOD - deadband);
//		}
//
//		if (deadband)
//		{
//			GPIOPinWrite(GPIO_PORTE_BASE, 1 << 2, 0);
//			SysCtlDelay(deadband);
//		}
//
//		uint8_t portfstate = GPIOPinRead(GPIO_PORTF_BASE, (1 << 4) | (1 << 0));
//		if ((~portfstate) & (1 << 4))
//		{
//			if (deadband < PERIOD)
//				deadband++;
//			else
//				deadband = 0;
//		}
//		if (portfstate & (1 << 0))
//		{
//			if (deadband)
//				deadband--;
//			else
//				deadband = PERIOD;
//		}
//	}
//
//	return 0;
//}

#define PWM_FREQUENCY 1000

int main(void)
{
	volatile uint32_t ui32Load;
	volatile uint32_t ui32PWMClock;
	volatile uint32_t ui32Adjust;
	ui32Adjust = 0;

	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	SysCtlPWMClockSet(SYSCTL_PWMDIV_32);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinConfigure(GPIO_PD0_M1PWM0);

	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

	ui32PWMClock = SysCtlClockGet() / 32;
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Adjust);
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);

	while(1)
	{

		if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00)
		{

			if (ui32Adjust > 1)
			{
				ui32Adjust--;
			}
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Adjust);
		}

		if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)==0x00)
		{
			if (ui32Adjust < ui32Load)
			{
				ui32Adjust++;
			}
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Adjust);
		}

		SysCtlDelay(100000);
	}

}

