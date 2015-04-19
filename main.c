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

#include "Motor/motor.h"
#include "Camera/camera.h"

#include "debug_serial.h"

float speed;

const char darkness_charset[] = " .,:;!?#############";

const char b64str[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789/+";

inline uint32_t map_val(uint32_t val, uint32_t inmin, uint32_t inmax, uint32_t outmin, uint32_t outmax)
{
	uint32_t retval = ((val - inmin)*(outmax - outmin));
	uint32_t divisor = (inmax - inmin);
	if(divisor != 0)
		return retval / divisor;
	return 0;
}

long abs(long a)
{
	if(a < 0)
		return -a;
	return a;
}

camera_sample_t derivative_buffer[128];

int main(void)
{
	speed = 0.0f;

	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

	GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

	Serial_init(UART_DEBUG_MODULE, 115200);

	Serial_puts(UART_DEBUG_MODULE, "Hello there!\r\n", 100);

	Serial_puts(UART_DEBUG_MODULE, "Initializing motor control...\r\n", 100);
	motor_init();

	Serial_puts(UART_DEBUG_MODULE, "Initializing camera...\r\n", 100);
	camera_init();

	Serial_puts(UART_DEBUG_MODULE, "Starting main control loop...\r\n", 100);

	while (1)
	{
		uint8_t buttonsMask = GPIOPinRead(GPIO_PORTF_BASE,
				GPIO_PIN_4 | GPIO_PIN_0) ^ (GPIO_PIN_4 | GPIO_PIN_0);

		if ((buttonsMask & (GPIO_PIN_4 | GPIO_PIN_0))
				== (GPIO_PIN_4 | GPIO_PIN_0))
		{
			motor_brake();
			speed = 0.0f;
		}
		else if (buttonsMask & GPIO_PIN_4)
		{
			if (speed < 1.0f)
				speed += 0.001f;

			servo_setPosf(speed);
			motor_setSpeedf(speed);
		}
		else if (buttonsMask & GPIO_PIN_0)
		{
			if (speed > 0.0f)
				speed -= 0.001f;

			servo_setPosf(speed);
			motor_setSpeedf(speed);
		}

		int i;
		camera_sample_t minval, maxval;
		minval = 1 << 13;
		maxval = 0;
		for (i = 0; i < 128; i++)
		{
			if (camera_buffer[i] > maxval)
				maxval = camera_buffer[i];
			if (camera_buffer[i] < minval)
				minval = camera_buffer[i];
		}

		for (i = 1; i < 128; i++)
		{
			derivative_buffer[i - 1] = abs(
					((int16_t) camera_buffer[i])
							- ((int16_t) camera_buffer[i - 1]));
		}


#ifdef DEBUG_B64_OUT
		for (i = 0; i < 128; i++)
		{
			Serial_putc(UART_DEBUG_MODULE, b64str[(camera_buffer[i] >> 6) & 0x3F]);
			Serial_putc(UART_DEBUG_MODULE, b64str[camera_buffer[i] & 0x3F]);
		}
		Serial_puts(UART_DEBUG_MODULE, "\r\n", 2);
#endif

#ifdef DEBUG_LINE_TO_CONSOLE
		for (i = 1; i < 128; i++)
		{
			derivative_buffer[i - 1] = abs(((int16_t)camera_buffer[i]) - ((int16_t)camera_buffer[i - 1]));
			Serial_putc(UART_DEBUG_MODULE, darkness_charset[derivative_buffer[i - 1] >> 9]);
		}
		Serial_puts(UART_DEBUG_MODULE, "\r\n", 2);
#endif

	}

}

