

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
#include <arch/board/board.h>
#include <sched.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>
#include <semaphore.h>

#include <ws2812.h>
#include "s32k1xx_lpspi.h"
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

// #define COLOR_TO_RGB(COLOR, BRIGHTNESS) 	(((0b100&COLOR)*BRIGHTNESS) << 4)+(((0b010&COLOR)*BRIGHTNESS) << 2)+((0b001)*BRIGHTNESS)

// #define BRIGHTNESS_DEFAULT	10
// #define BRIGHTNESS_DARK		1
// #define BRIGHTNESS_MAX		0xFF

// #define CONFIG_EXAMPLES_WS2812_DEVNAME "/dev/leddrv0"
// #define US_DELAY  1000*20
/****************************************************************************
 * Types
 ****************************************************************************/

// struct led_s
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
// 	struct led_s LEDs[LEDS_NUMBER];
// 	bool mode;
// 	uint8_t blinkCount;
// 	struct timespec duration;	
// 	bool needUpdate;

// } gCurrentWS2812, gTimedWS2812, gSteadyWS2812;

// typedef enum
// {
// 	STOP, STEADY_NONBLINK, TIMED_NONBLINK, TIMED_BLINK, STEADY_BLINK
// } mainState_t;


// int gLEDTID;
// sem_t gLEDSem;
// uint8_t gMainState;
// static int fd;
// static uint32_t rgb[LEDS_NUMBER];
/****************************************************************************
 * Public Functions
 ****************************************************************************/
static int ledTaskFunc(int argc, char *argv[]);

// void setOneLEDSigHdlr(int signo, siginfo_t *siginfo, void *arg);

// uint8_t getMainState(void);

// void setMainState(int mState);


/****************************************************************************
 * hello_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
	static struct spi_dev_s *myspi;
	struct sigaction act;
	int lvRetValue;

	// 초기화
	// - 시그널 등록
	memset(&act, 0, sizeof(struct sigaction));
	act.sa_sigaction = setOneLEDSigHdlr;
	act.sa_flags = SA_SIGINFO;

	sigemptyset(&act.sa_mask);

	lvRetValue = sigaction(SIGALRM, &act, NULL);
	if(lvRetValue != 0)
	{
		printf("<YS>시그널 등록 실패\n");
	}
	else
	{
		printf("<YS>시그널 등록 성공\n");
	}

	//  - SPI 장치 초기화 및 장치등록
	myspi = s32k1xx_lpspibus_initialize(1);
	printf("<YS>SPI 장치 초기화 완료\n");

	ws2812_leds_register(CONFIG_EXAMPLES_WS2812_DEVNAME, myspi,LEDS_NUMBER);
 	if (fd < 0)
	{
		fprintf(stderr, "ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_WS2812_DEVNAME, errno); 
		return -1;
	}
	else
	{
		printf("<YS>SPI장치 등록 완료\n");	
	}
	
	//파일 Open
	fd = open(CONFIG_EXAMPLES_WS2812_DEVNAME, O_RDWR);
	if (fd < 0)
	{
	fprintf(stderr, "ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_WS2812_DEVNAME, errno); 
	return -1;
	}

	// WS2812 장치 초기화
	for( int i = 0 ; i < LEDS_NUMBER ; ++i)
	{
		gSteadyWS2812.LEDs[i].blink = false;
		gSteadyWS2812.LEDs[i].color = OFF;
	}
	gCurrentWS2812.needUpdate = false; 
	gTimedWS2812 = gSteadyWS2812 = gCurrentWS2812;

	// 세마포어 초기화
	lvRetValue = sem_init(&gLEDSem, 0, 0);
	sem_setprotocol(&gLEDSem, SEM_PRIO_NONE);
	if (lvRetValue < 0)
	{
		printf("<YS> myled : 세마포어 초기화 실패\n");
	}
	else
	{
		printf("<YS> myled : 세마포어 초기화 성공\n");
	}

	// Task 생성
	lvRetValue = task_create("led", DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, ledTaskFunc, NULL);
	if (lvRetValue < 0)
	{
		printf("<YS>Task 생성 실패\n");
	}
	else
	{
		printf("<YS>Task 생성 성공\n");
		gLEDTID = lvRetValue;
		printf("<YS> myled : Task ID : %d\n", gLEDTID);
	}
	
	return 0;
	
}

static int ledTaskFunc(int argc, char *argv[])
{
	int lvRetValue = 0;
	struct timespec currentTime;
	gMainState = OFF;
	int sem = 0;

	//loop start..
	while(1)

	sem_getvalue(&gLEDSem, &sem);

	{
		printf("<YS> myled : gLEDSem : %d\n",sem);
		// sem. 있을때까지 Task 정지
		sem_wait(&gLEDSem);
	
		sem_post(&gLEDSem);
		printf("<YS> myled : gLEDSem : %d\n",sem);
		// 표시할 LED 정보 초기화
		if(gCurrentWS2812.needUpdate == true)
		{
			printf("<YS> myled : gCurrentWS2812 is needed to update\n");
			if(gTimedWS2812.needUpdate == true)
			{
				gCurrentWS2812 = gTimedWS2812;
				gCurrentWS2812.needUpdate = gTimedWS2812.needUpdate = false;
				
				for(int i = 0; i < LEDS_NUMBER ; ++i)
				{
					lvRetValue |= gCurrentWS2812.LEDs[i].blink;
				}
				if(lvRetValue != false) {setMainState(STEADY_BLINK);printf("<YS> myled : Current state is %d\n", getMainState());}
				else {setMainState(TIMED_NONBLINK);printf("<YS> myled : Current state is %d\n", getMainState());}
			}
			else
			{
				gCurrentWS2812 = gSteadyWS2812;
				gCurrentWS2812.needUpdate = gSteadyWS2812.needUpdate = false;

				for(int i = 0; i < LEDS_NUMBER ; ++i)
				{
					lvRetValue |= gCurrentWS2812.LEDs[i].blink;
				}
				if(lvRetValue != false){ setMainState(TIMED_BLINK);printf("<YS> myled : Current state is %d\n", getMainState()); }
				else{ setMainState(STEADY_NONBLINK);printf("<YS> myled : Current state is %d\n", getMainState()); }
			}

			lvRetValue = false;
		}

		//초기 색깔 Set
		for(int i = 0; i < LEDS_NUMBER ; ++i)
		{
			rgb[i] = (uint32_t)COLOR_TO_RGB(gCurrentWS2812.LEDs[i].color,BRIGHTNESS_DARK);
		}
		if(lseek(fd, 0, SEEK_SET))
		{

		}
		if(write(fd, rgb, 4*LEDS_NUMBER))
		{
			
		}
		
		// Timestamp 업데이트
		for( int i = 0 ; i < LEDS_NUMBER ; ++i )
		{
			// 시작시간 체크
			clock_gettime(CLOCK_REALTIME, &gCurrentWS2812.LEDs[i].startTime);

			// 토글시작시간 체크
			clock_gettime(CLOCK_REALTIME, &gCurrentWS2812.LEDs[i].startToggleTime);
		}

		// // 토글검사
		// // Blink = ON 이면 토글해야되는지 검사
		// for( int i = 0 ; i < LEDS_NUMBER ; ++i )
		// {
		// 	// Blink인지 검사
		// 	if(gCurrentWS2812.LEDs[i].blink == true)
		// 	{
		// 		// 현재 토글시간 체크
		// 		clock_gettime(CLOCK_REALTIME, &currentTime);

		// 		// 토글해야되는지 검사
		// 		if(checkNeedToToggle(i, currentTime) == true)
		// 		{
		// 			toggleLED(i);
		// 		}

		// 	}
		// }

		// state 전이 판단
		usleep(1000*1000*0.5);
		printf("<YS> myled : loops\n");
		printf("<YS> myled : Current state is %d\n", getMainState());
		switch (getMainState())
		{
			case STEADY_BLINK:
			break;
			
			case TIMED_BLINK:
			case TIMED_NONBLINK:
				// if(checkNeedStop() == true) { gCurrentWS2812.needUpdate = true; }
				sem_wait(&gLEDSem); // <YS> 테스트용(지워야 됨)
			break;
			
			case STEADY_NONBLINK:
				sem_wait(&gLEDSem);
				setMainState(OFF);
				printf("<YS> myled : Current state is %d\n", getMainState());
			break;
			
			case OFF:
				
			break;
		}
	}
	printf("<YS>Loop close\n");
	close(fd);
	return -1;
}

// 
void setOneLEDSigHdlr(int signo, siginfo_t *siginfo, void *arg)
{	
	printf("<YS> myled.setOneLEDSigHdlr : Here is setOneLEDSigHdlr\n");
	// 초기화 해줘야되는지 판단
	switch (getMainState())
	{
		case STEADY_BLINK:
		printf("<YS> myled.setOneLEDSigHdlr : Here is BLINK case in setOneLEDSigHdlr\n");
			gCurrentWS2812.needUpdate = true;

			break;

		case TIMED_BLINK:
		case TIMED_NONBLINK:
		printf("<YS> myled.setOneLEDSigHdlr : Here is TIMED case in setOneLEDSigHdlr\n");
			break;

		case STEADY_NONBLINK:
		printf("<YS> myled.setOneLEDSigHdlr : Here is STEADY_NONBLINK case in setOneLEDSigHdlr\n");
			gCurrentWS2812.needUpdate = true;
			sem_post(&gLEDSem);
			break;

		case OFF:
		printf("<YS> myled.setOneLEDSigHdlr : Here is OFF case in setOneLEDSigHdlr\n");
			gCurrentWS2812.needUpdate = true;
			
			sem_post(&gLEDSem);
			break;
	}
}

mainState_t getMainState(void)
{
	return gMainState;	
}

void setMainState(mainState_t mState)
{
	gMainState = mState;
}

// int checkNeedToToggle(int ledIndex, struct timespec currentTime)
// {
// 	if( gCurrentWS2812.LEDs[ledIndex].OnTime.tv_sec > (gCurrentWS2812.LEDs[ledIndex].startToggleTime.tv_sec - currentTime.tv_sec) )
// 	{
// 		return true;
// 	}
// 	else
// 	{
// 		return false;
// 	}
// }

// int toggleLED(int ledIndex)
// {
// 	rgb[ledIndex] = (uint32_t)COLOR_TO_RGB(gCurrentWS2812.LEDs[ledIndex].color,BRIGHTNESS_DARK);
// 	if(lseek(fd, 0, SEEK_SET))
// 	{

// 	}
// 	if(write(fd, rgb, 4*LEDS_NUMBER))
// 	{
		
// 	}
// }
// 동시 접근 처리?