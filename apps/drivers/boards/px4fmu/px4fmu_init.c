/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4fmu_init.c
 *
 * PX4FMU-specific early startup code.  This file implements the
 * nsh_archinitialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialisation.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/spi.h>
#include <nuttx/i2c.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/arch.h>

#include "stm32_internal.h"
#include "px4fmu_internal.h"
#include "stm32_uart.h"

#include <arch/board/board.h>
#include <arch/board/drv_led.h>
#include <arch/board/drv_eeprom.h>

#include <drivers/drv_hrt.h>

#include <systemlib/cpuload.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lib_lowprintf(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lib_lowprintf
#  else
#    define message printf
#  endif
#endif

/****************************************************************************
 * Protected Functions
 ****************************************************************************/

extern int adc_devinit(void);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void stm32_boardinitialize(void)
{
	/* configure SPI interfaces */
	stm32_spiinitialize();

	/* configure LEDs */
	up_ledinit();
}

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

static struct spi_dev_s *spi1;
static struct spi_dev_s *spi3;
static struct i2c_dev_s *i2c1;
static struct i2c_dev_s *i2c2;
static struct i2c_dev_s *i2c3;

#include <math.h>

#ifdef __cplusplus
__EXPORT int matherr(struct __exception *e)
{
	return 1;
}
#else
__EXPORT int matherr(struct exception *e)
{
	return 1;
}
#endif

__EXPORT int nsh_archinitialize(void)
{
	int result;

	/* INIT 1 Lowest level NuttX initialization has been done at this point, LEDs and UARTs are configured */

	/* INIT 2 Configuring PX4 low-level peripherals, these will be always needed */

	/* configure the high-resolution time/callout interface */
#ifdef CONFIG_HRT_TIMER
	hrt_init();
#endif

	/* configure CPU load estimation */
#ifdef CONFIG_SCHED_INSTRUMENTATION
	cpuload_initialize_once();
#endif

	/* set up the serial DMA polling */
#ifdef SERIAL_HAVE_DMA
	{
		static struct hrt_call serial_dma_call;
		struct timespec ts;

		/*
		 * Poll at 1ms intervals for received bytes that have not triggered
		 * a DMA event.
		 */
		ts.tv_sec = 0;
		ts.tv_nsec = 1000000;

		hrt_call_every(&serial_dma_call,
			       ts_to_abstime(&ts),
			       ts_to_abstime(&ts),
			       (hrt_callout)stm32_serial_dma_poll,
			       NULL);
	}
#endif

	message("\r\n");

	up_ledoff(LED_BLUE);
	up_ledoff(LED_AMBER);

	up_ledon(LED_BLUE);

	/* Configure user-space led driver */
	px4fmu_led_init();

	/* Configure SPI-based devices */

	spi1 = up_spiinitialize(1);

	if (!spi1) {
		message("[boot] FAILED to initialize SPI port 1\r\n");
		up_ledon(LED_AMBER);
		return -ENODEV;
	}

	// Default SPI1 to 1MHz and de-assert the known chip selects.
	SPI_SETFREQUENCY(spi1, 10000000);
	SPI_SETBITS(spi1, 8);
	SPI_SETMODE(spi1, SPIDEV_MODE3);
	SPI_SELECT(spi1, PX4_SPIDEV_GYRO, false);
	SPI_SELECT(spi1, PX4_SPIDEV_ACCEL, false);
	SPI_SELECT(spi1, PX4_SPIDEV_MPU, false);
	up_udelay(20);

	message("[boot] Successfully initialized SPI port 1\r\n");

	/* initialize I2C2 bus */

	i2c2 = up_i2cinitialize(2);

	if (!i2c2) {
		message("[boot] FAILED to initialize I2C bus 2\n");
		up_ledon(LED_AMBER);
		return -ENODEV;
	}

	/* set I2C2 speed */
	I2C_SETFREQUENCY(i2c2, 400000);


	i2c3 = up_i2cinitialize(3);

	if (!i2c3) {
		message("[boot] FAILED to initialize I2C bus 3\n");
		up_ledon(LED_AMBER);
		return -ENODEV;
	}

	/* set I2C3 speed */
	I2C_SETFREQUENCY(i2c3, 400000);

	/* try to attach, don't fail if device is not responding */
	(void)eeprom_attach(i2c3, FMU_BASEBOARD_EEPROM_ADDRESS,
			    FMU_BASEBOARD_EEPROM_TOTAL_SIZE_BYTES,
			    FMU_BASEBOARD_EEPROM_PAGE_SIZE_BYTES,
			    FMU_BASEBOARD_EEPROM_PAGE_WRITE_TIME_US, "/dev/baseboard_eeprom", 1);

#if defined(CONFIG_STM32_SPI3)
	/* Get the SPI port */

	message("[boot] Initializing SPI port 3\n");
	spi3 = up_spiinitialize(3);

	if (!spi3) {
		message("[boot] FAILED to initialize SPI port 3\n");
		up_ledon(LED_AMBER);
		return -ENODEV;
	}

	message("[boot] Successfully initialized SPI port 3\n");

	/* Now bind the SPI interface to the MMCSD driver */
	result = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR, CONFIG_NSH_MMCSDSLOTNO, spi3);

	if (result != OK) {
		message("[boot] FAILED to bind SPI port 3 to the MMCSD driver\n");
		up_ledon(LED_AMBER);
		return -ENODEV;
	}

	message("[boot] Successfully bound SPI port 3 to the MMCSD driver\n");
#endif /* SPI3 */

	/* initialize I2C1 bus */

	i2c1 = up_i2cinitialize(1);

	if (!i2c1) {
		message("[boot] FAILED to initialize I2C bus 1\n");
		up_ledon(LED_AMBER);
		return -ENODEV;
	}

	/* set I2C1 speed */
	I2C_SETFREQUENCY(i2c1, 400000);

	/* INIT 3: MULTIPORT-DEPENDENT INITIALIZATION */

	/* Get board information if available */

	/* Initialize the user GPIOs */
	px4fmu_gpio_init();

#ifdef CONFIG_ADC
	int adc_state = adc_devinit();

	if (adc_state != OK) {
		/* Try again */
		adc_state = adc_devinit();

		if (adc_state != OK) {
			/* Give up */
			message("[boot] FAILED adc_devinit: %d\n", adc_state);
			return -ENODEV;
		}
	}

#endif

	return OK;
}