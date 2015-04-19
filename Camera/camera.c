/*
 * camera.c
 *
 *  Created on: Apr 17, 2015
 *      Author: Kevin
 */

#include "camera.h"

#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/udma.h"
#include "driverlib/interrupt.h"

#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_adc.h"
#include "inc/hw_udma.h"
#include "inc/hw_ints.h"

#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"

uint8_t uDMAControlTable[1024] __attribute__ ((aligned(1024)));

#define CAMERA_SAMPLES (128)

#define CAMERA_CLOCK_DIV (32)

#define CAMERA_SAMPLE_PERIOD (64)

uint32_t camera_PWMClockFreq;

uint32_t intcounter;

volatile uint32_t testval;

camera_sample_t camera_DoubleBuffer[2][CAMERA_SAMPLES];
camera_sample_t *camera_buffer;
uint32_t camera_DBSelected;

void ADCSeq3Handler()
{
	*(uint32_t *) (ADC0_BASE + ADC_O_ISC) = ADC_INT_DMA_SS3;

	intcounter++;

	uint32_t mode = uDMAChannelModeGet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT);

	if (!camera_DBSelected && mode == UDMA_MODE_STOP)
	{
		camera_DBSelected = 1;
		uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT,
				UDMA_MODE_PINGPONG, (void*) (ADC0_BASE + ADC_O_SSFIFO3),
				camera_DoubleBuffer[camera_DBSelected], CAMERA_SAMPLES);
		intcounter = 0;
	}

	mode = uDMAChannelModeGet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT);

	if (camera_DBSelected && mode == UDMA_MODE_STOP)
	{
		camera_DBSelected = 0;
		uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
				UDMA_MODE_PINGPONG, (void*) (ADC0_BASE + ADC_O_SSFIFO3),
				camera_DoubleBuffer[camera_DBSelected], CAMERA_SAMPLES);
		intcounter = 0;
	}


}

void camera_init()
{
	testval = 0x5A5A5A5A;
	intcounter = 0;
	int i, j;
	for(i = 0; i < 2; i++)
	{
		for(j = 0; j < CAMERA_SAMPLES; j++)
		{
			camera_DoubleBuffer[i][j] = 0;
		}
	}
	// Calculate the PWM clock frequency
	camera_PWMClockFreq = SysCtlClockGet() / CAMERA_CLOCK_DIV;

	// Enable the PWM peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

	// Enable the GPIO port
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	// Configure PD0 as the PWM output for the drive motor
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
	GPIOPinConfigure(GPIO_PB7_M0PWM1);
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
	GPIOPinConfigure(GPIO_PB4_M0PWM2);

	// Set the camera clock pulse period
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, CAMERA_SAMPLE_PERIOD - 1);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, (CAMERA_SAMPLE_PERIOD / 2) - 1);

	// Set the camera enable pulse period
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, (CAMERA_SAMPLE_PERIOD * CAMERA_SAMPLES) - 1);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ((CAMERA_SAMPLE_PERIOD / 2) * 2) - 1);

	// Enable the PWM output
	PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT | PWM_OUT_2_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);
	PWMGenEnable(PWM0_BASE, PWM_GEN_1);

	PWMSyncTimeBase(PWM0_BASE, PWM_GEN_0_BIT | PWM_GEN_1_BIT);

	// Enable PWM trigger on zero count on Generator 0
	PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_0, PWM_TR_CNT_ZERO); // PWM_TR_CNT_ZERO/PWM_TR_CNT_LOAD

	/********************************************
	 * 			  ADC CONFIGURATION				*
	 ********************************************
	 */

	// Enable ADC0 module
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

	ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_FULL, 1);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

	// Configure and enable the ADC sequence; single sample
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PWM0, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 3);

	ADCSequenceDMAEnable(ADC0_BASE, 3);

	// Start writing into the first buffer
	camera_DBSelected = 0;
	// Expose the other buffer
	camera_buffer = camera_DoubleBuffer[1];

	/********************************************
	 * 			  uDMA CONFIGURATION			*
	 ********************************************
	 */

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);
	uDMAEnable();

	uDMAControlBaseSet(uDMAControlTable);

	uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC3,
		UDMA_ATTR_USEBURST |
		UDMA_ATTR_ALTSELECT |
		UDMA_ATTR_HIGH_PRIORITY |
		UDMA_ATTR_REQMASK);

	uDMAChannelControlSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
	uDMAChannelControlSet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);

	uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, &testval, camera_DoubleBuffer[0], CAMERA_SAMPLES);
	uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, &testval, camera_DoubleBuffer[1], CAMERA_SAMPLES);

	IntEnable(INT_ADC0SS3);
	ADCIntEnableEx(ADC0_BASE, ADC_INT_DMA_SS3);



	uDMAChannelEnable(UDMA_CHANNEL_ADC3);
}


void camera_registerCallback(camera_callback_t callback)
{

}
