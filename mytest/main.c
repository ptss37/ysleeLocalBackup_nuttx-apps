

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
#include <sys/types.h>


#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include "main.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
// #define DEFAULT_PRIORITY 	100
// #define DEFAULT_STACK_SIZE 	1024

// #define LEDS_NUMBER			4

// #define OFF			0b000
// #define RED			0b001
// #define GRREN		0b010
// #define YELLOW		0b011
// #define BLUE		0b100
// #define MAGENTA		0b101
// #define SKY			0b110
// #define WHITE		0b111

// #define COLOR_TO_RGB(COLOR, BRIGHTNESS)	\
// 	(((0b100&COLOR)*BRIGHTNESS) << 4)+	\
// 	(((0b010&COLOR)*BRIGHTNESS) << 2)+	\
// 	((0b001)*BRIGHTNESS)

// #define BRIGHTNESS_DEFAULT	10
// #define BRIGHTNESS_DARK		1
// #define BRIGHTNESS_MAX		0xFF

// #define CONFIG_EXAMPLES_WS2812_DEVNAME "/dev/leddrv0"
// #define US_DELAY  1000*20
#define MY_DELAY 1000*1000*1
/****************************************************************************
 * Types
 ****************************************************************************/

// struct LED_s
// {
// 	uint8_t color;
// 	bool blink;
// 	struct timespec startTime;
// 	struct timespec startToggleTime;
// 	struct timespec OnTime;
// 	struct timespec OffTime;
// 	uint8_t blinkCount;
// 	struct timespec duration;

// };

// struct ws2812_s
// {
// 	struct LED_s gLEDs[LEDS_NUMBER];
// 	bool mode;
// 	uint8_t blinkCount;
// 	struct timespec duration;	
// 	bool needUpdate;

// } gCurrentWS2812, gTimedWS2812, gSteadyWS2812;

// typedef enum
// {
// 	STOP, STEADY_NONBLINK, TIMED_NONBLINK, TIMED_BLINK, STEADY_BLINK
// } mainState_t;


extern int gLEDTID;
extern struct ws2812_s gSteadyWS2812;
// sem_t gLEDSem;
// uint8_t gMainState;
// static int fd;
// static uint32_t rgb[LEDS_NUMBER];
/****************************************************************************
 * Public Functions
 ****************************************************************************/
// static int ledTaskFunc(int argc, char *argv[]);

// void setOneLEDSigHdlr(int signo, siginfo_t *siginfo, void *arg);

// uint8_t getMainState(void);

// void setMainState(int mState);


/****************************************************************************
 * hello_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
	printf("<YS>MyTask : Start\n");
	
	// 초기화
	int lvRetValue;

	//색 설정
	gSteadyWS2812.needUpdate = true;
	for( int i = 0 ; i < LEDS_NUMBER ; ++i)
	{
		gSteadyWS2812.LEDs[i].blink = false;
		gSteadyWS2812.LEDs[i].color = GREEN;
	}


	// 시그널 전송
	printf("<YS> mytest : Task ID : %d\n", gLEDTID);
	lvRetValue = kill(gLEDTID, SIGALRM);
	if (lvRetValue != 0)
	{
		printf("<YS>시그널 전송 실패\n");
	}
	printf("<YS>MyTask : End\n");

	return 0;
}
