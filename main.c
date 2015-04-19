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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "Motor/motor.h"
#include "Camera/camera.h"

#include "PID/PIDControl.h"

#include "ShellInterface/shell.h"

#include "debug_serial.h"

float speed;

float threshold_ratio;

const char darkness_charset[] = " .,:;!?#############";

const char b64str[] =
		"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789/+";

inline uint32_t map_val(uint32_t val, uint32_t inmin, uint32_t inmax,
		uint32_t outmin, uint32_t outmax)
{
	uint32_t retval = ((val - inmin) * (outmax - outmin));
	uint32_t divisor = (inmax - inmin);
	if (divisor != 0)
		return retval / divisor;
	return 0;
}

//long abs(long a)
//{
//	if (a < 0)
//		return -a;
//	return a;
//}

static PIDController_t servopid;

bool killswitch;



int set_main(char* argv[], int argc)
{
	char printbuf[64];
	if(argc != 3)
	{
		Serial_puts(UART_DEBUG_MODULE, "USAGE:\r\n\tset <VAR> <VAL>\r\n", 100);
		return -1;
	}

	bool succ = false, succui = false;
	float val = fast_sntof(argv[2], fast_strlen(argv[2]), 10, &succ);
	uint32_t valui = fast_sntoul(argv[2], fast_strlen(argv[2]), 10, &succui);

	if (succ)
	{
		if (fast_strcmp(argv[1], "P") == 0 || fast_strcmp(argv[1], "p") == 0)
		{
			servopid.coefficients.p = val;
			fast_snprintf(printbuf, 64, "P=%d.%05d\r\n",
					(long) servopid.coefficients.p,
					(long) (100000
							* (servopid.coefficients.p
									- (long) servopid.coefficients.p)));
		}
		if (fast_strcmp(argv[1], "I") == 0 || fast_strcmp(argv[1], "i") == 0)
		{
			servopid.coefficients.i = val;
			fast_snprintf(printbuf, 64, "I=%d.%05d\r\n",
					(long) servopid.coefficients.i,
					(long) (100000
							* (servopid.coefficients.i
									- (long) servopid.coefficients.i)));
		}
		if (fast_strcmp(argv[1], "D") == 0 || fast_strcmp(argv[1], "d") == 0)
		{
			servopid.coefficients.d = val;
			fast_snprintf(printbuf, 64, "D=%d.%05d\r\n",
					(long) servopid.coefficients.d,
					(long) (100000
							* (servopid.coefficients.d
									- (long) servopid.coefficients.d)));
		}
		if (fast_strcmp(argv[1], "thresh") == 0)
		{
			threshold_ratio = val;
			fast_snprintf(printbuf, 64, "D=%d.%05d\r\n",
					(long) threshold_ratio,
					(long) (100000
							* (threshold_ratio
									- (long) threshold_ratio)));
		}
	}

	if(succui)
	{
		if (fast_strcmp(argv[1], "smaxb") == 0)
		{
			servo_max_highband = valui;
			fast_snprintf(printbuf, 64, "servo_max_highband=%d\r\n", valui);
		}
		if (fast_strcmp(argv[1], "sminb") == 0)
		{
			servo_min_highband = valui;
			fast_snprintf(printbuf, 64, "servo_min_highband=%d\r\n", valui);
		}
	}

	if(!(succ || succui))
	{
		Serial_puts(UART_DEBUG_MODULE, "Invalid arguments!\r\n", 100);
		return -1;
	}

	Serial_puts(UART_DEBUG_MODULE, printbuf, 64);
	return 0;
}

int get_main(char* argv[], int argc)
{
	char printbuf[64] = "No arg specified!\r\n";
	if(argc != 2)
	{
		Serial_puts(UART_DEBUG_MODULE, "USAGE:\r\n\tget <VAR>\r\n", 100);
		return -1;
	}

	if(argc == 2)
	{
		if (fast_strcmp(argv[1], "P") == 0 || fast_strcmp(argv[1], "p") == 0)
		{
			fast_snprintf(printbuf, 64, "P=%d.%05d\r\n", (long)servopid.coefficients.p,(long)(100000*(servopid.coefficients.p - (long)servopid.coefficients.p)));
		}
		if (fast_strcmp(argv[1], "I") == 0 || fast_strcmp(argv[1], "i") == 0)
		{
			fast_snprintf(printbuf, 64, "I=%d.%05d\r\n", (long)servopid.coefficients.i,(long)(100000*(servopid.coefficients.i - (long)servopid.coefficients.i)));
		}
		if (fast_strcmp(argv[1], "D") == 0 || fast_strcmp(argv[1], "d") == 0)
		{
			fast_snprintf(printbuf, 64, "D=%d.%05d\r\n", (long)servopid.coefficients.d,(long)(100000*(servopid.coefficients.d - (long)servopid.coefficients.d)));
		}
		if (fast_strcmp(argv[1], "thresh") == 0)
		{
			fast_snprintf(printbuf, 64, "threshold_ratio=%d.%05d\r\n", (long)threshold_ratio,(long)(100000*(threshold_ratio - (long)threshold_ratio)));
		}
		if (fast_strcmp(argv[1], "smaxb") == 0)
		{
			fast_snprintf(printbuf, 64, "servo_max_highband=%d\r\n",
					servo_max_highband);
		}
		if (fast_strcmp(argv[1], "sminb") == 0)
		{
			fast_snprintf(printbuf, 64, "servo_min_highband=%d\r\n",
					servo_min_highband);
		}
		Serial_puts(UART_DEBUG_MODULE, printbuf, 64);
	}

	return 0;
}

int kill_main(char* argv[], int argc)
{
	killswitch = true;
	motor_setSpeedf(0.0f);
	motor_brake();

	if(argc == 2 && fast_strcmp(argv[1], "unkill") == 0)
	{
		killswitch = false;
		motor_releaseBrake();
		Serial_puts(UART_DEBUG_MODULE, "Released brake.\r\n", 100);
	}
	else
	{
		Serial_puts(UART_DEBUG_MODULE, "KILLSWITCH ENGAGED!\r\n", 100);
	}

	return 0;
}

int drive_main(char* argv[], int argc)
{
	char printbuf[64];
	if(argc != 2)
	{
		return -1;
	}

	bool succ = false;
	float val = fast_sntof(argv[1], fast_strlen(argv[1]), 10, &succ);

	if(succ)
	{
		motor_setSpeedf(val);
		fast_snprintf(printbuf, 64, "Speed=%d.%05d\r\n", (long)val,(long)(100000*(val - (long)val)));
	}
	else
	{
		Serial_puts(UART_DEBUG_MODULE, "Invalid argument!\r\n", 100);
		return -1;
	}

	Serial_puts(UART_DEBUG_MODULE, printbuf, 100);

	return 0;
}

camera_sample_t derivative_buffer[128];

#define THRESHOLD_RATIO (0.65f)

int main(void)
{
	threshold_ratio = THRESHOLD_RATIO;
	killswitch = false;
	speed = 0.0f;

	SysCtlClockSet(
			SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
					| SYSCTL_XTAL_16MHZ);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

	GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0,
			GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

	Serial_init(UART_DEBUG_MODULE, 9600);

	Serial_puts(UART_DEBUG_MODULE, "Hello there!\r\n", 100);

	Serial_puts(UART_DEBUG_MODULE, "Initializing motor control...\r\n", 100);
	motor_init();

	Serial_puts(UART_DEBUG_MODULE, "Initializing camera...\r\n", 100);
	camera_init();

	Serial_puts(UART_DEBUG_MODULE, "Starting main control loop...\r\n", 100);


	PID_PIDController(&servopid, 0.1f, 0.0f, 0.01f, 2.0f);

	uint32_t loopdelay = SysCtlClockGet() / 100 / 3;

	shell_init();
	shell_registerProgram("get", get_main);
	shell_registerProgram("set", set_main);
	shell_registerProgram("drive", drive_main);
	shell_registerProgram("k", kill_main);

	while (1)
	{
		int i;
		static camera_sample_t minderiv, maxderiv;

		minderiv = 1 << 13;
		maxderiv = 0;

		// Compute the derivative of the linecam data
		for (i = 1; i < CAMERA_SAMPLES; i++)
		{
			derivative_buffer[i - 1] = abs(
					((int16_t) camera_buffer[i])
							- ((int16_t) camera_buffer[i - 1]));
		}

		// Get the min and max derivative values
		for (i = 0; i < (CAMERA_SAMPLES - 1); i++)
		{
			if (derivative_buffer[i] > maxderiv)
				maxderiv = derivative_buffer[i];
			if (derivative_buffer[i] < minderiv)
				minderiv = derivative_buffer[i];
		}

		// Compute the average x value of the threshold values
		int numpositions = 0;
		float avgpos = 0;

		for (i = 0; i < (CAMERA_SAMPLES - 1); i++)
		{
			float absval = derivative_buffer[i] - minderiv;
			float range = maxderiv - minderiv;

			if(absval > (range*threshold_ratio))
			{
				avgpos += i;
				numpositions++;
			}
		}

		if(numpositions != 0)
			avgpos /= numpositions;
		else
			avgpos = CAMERA_SAMPLES / 2;

		// Scale and apply with PID to servo
		avgpos /= CAMERA_SAMPLES;

		float servopos = PID_calculate(&servopid, avgpos, 0.5f, 0.01f);
		servo_setPosf(0.5f - servopos);

		shell_poll();

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


		SysCtlDelay(loopdelay);
	}

}

