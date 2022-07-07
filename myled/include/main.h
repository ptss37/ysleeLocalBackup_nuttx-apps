

/****************************************************************************
 * myled/main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <nuttx/signal.h>
#include <semaphore.h>

#include <arch/board/board.h>
#include <sched.h>
#include "s32k1xx_lpspi.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
#define DEFAULT_PRIORITY 	100
#define DEFAULT_STACK_SIZE 	1024

#define LEDS_NUMBER			4

#define OFF			0b000
#define BLUE		0b001
#define GRREN		0b010
#define SKY			0b011
#define RED			0b100
#define MAGENTA		0b101
#define YELLOW		0b110
#define WHITE		0b111

#define COLOR_TO_RGB(COLOR, BRIGHTNESS)	\
	(((0b100&COLOR)*BRIGHTNESS) << 4)+	\
	(((0b010&COLOR)*BRIGHTNESS) << 2)+	\
	((0b001)*BRIGHTNESS)

#define BRIGHTNESS_DEFAULT	10
#define BRIGHTNESS_DARK		1
#define BRIGHTNESS_MAX		0xFF

#define CONFIG_EXAMPLES_WS2812_DEVNAME "/dev/leddrv0"
#define US_DELAY  1000*20
/****************************************************************************
 * Types
 ****************************************************************************/

struct led_s
{
	uint8_t color;
	bool blink;
	struct timespec startTime;
	struct timespec startToggleTime;
	struct timespec OnTime;
	struct timespec OffTime;
	uint8_t blinkCount;
	struct timespec duration;

};

struct ws2812_s
{
	struct led_s LEDs[LEDS_NUMBER];
	bool mode;
	uint8_t blinkCount;
	struct timespec duration;	
	bool needUpdate;

} gCurrentWS2812, gTimedWS2812, gSteadyWS2812;

typedef enum
{
	STOP, STEADY_NONBLINK, TIMED_NONBLINK, TIMED_BLINK, STEADY_BLINK
} mainState_t;


int gLEDTID;
sem_t gLEDsem;
uint8_t gMainState;
static int fd;
static uint32_t rgb[LEDS_NUMBER];
/****************************************************************************
 * Public Functions
 ****************************************************************************/
static int ledTaskFunc(int argc, char *argv[]);

void setOneLEDSigHdlr(int signo, siginfo_t *siginfo, void *arg);

uint8_t getMainState(void);

void setMainState(int mState);
