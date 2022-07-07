

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
	// 변수 선언
	static struct spi_dev_s *myspi;
	int lvRetValue;

	//뮤텍스 초기화
	pthread_mutex_init(&gLEDLock, NULL);

	// WS2812 초기화
	// - SPI 초기화
	myspi = s32k1xx_lpspibus_initialize(1);
	if(myspi == NULL)
	{
		printf("<YS> myled앱에서 SPI 장치 초기화 실패\n");
		return -1;
	}
	
	// - WS2812 장치 등록
	ws2812_leds_register(CONFIG_EXAMPLES_WS2812_DEVNAME, myspi,LEDS_NUMBER);
 	if (fd < 0)
	{
		fprintf(stderr, "ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_WS2812_DEVNAME, errno); 
		return -1;
	}
	
	// - WS2812 Open
	fd = open(CONFIG_EXAMPLES_WS2812_DEVNAME, O_RDWR);
	if (fd < 0)
	{
		fprintf(stderr, "ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_WS2812_DEVNAME, errno); 
		return -1;
	}

	// - LED색 초기화 : Non-Blink Off 로 set
	for( int i = 0 ; i < LEDS_NUMBER ; ++i)
	{
		gSteadyWS2812.LEDs[i].blink = false;
		gSteadyWS2812.LEDs[i].color = OFF;
	}
	gCurrentWS2812.needUpdate = false; 
	gTimedWS2812 = gSteadyWS2812 = gCurrentWS2812;

	// 세마포어 초기화
	// - Named 세마포어 생성
	gLEDSem = sem_open(LED_SEM_NAME, O_CREAT|O_EXCL, 0, 0);
	if(gLEDSem == SEM_FAILED)
	{
		printf("<YS> myled앱에서 named 세마포어 생성 실패\n");
		return -1;
	}

	// - 세마포어 프로토콜 설정
	lvRetValue = sem_setprotocol(gLEDSem, SEM_PRIO_NONE);
	if (lvRetValue < 0)
	{
		printf("<YS> myled앱에서 세마포어 프로토콜 설정 실패\n");
		return -1;
	}

	// Task 생성
	lvRetValue = task_create("led", DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, ledTaskFunc, NULL);
	if (lvRetValue < 0)
	{
		printf("<YS> myled앱에서 생성 실패\n");
		return -1;
	}

	return 0;
	
}

static int ledTaskFunc(int argc, char *argv[])
{
	int lvRetValue = 0;
	gMainState = OFF;
	
	int sem;

	// 루프 시작 
	while(1)
	{
		// Task 제어
		// 세마포어 있을 때 까지 Task 정지
		lvRetValue = sem_wait(gLEDSem);
		if(lvRetValue != 0)
		{
			printf("<YS> myled앱에서 sem_wait()가 세마포어 없이 빠져나옴\n", lvRetValue);
		}
		
		// 세마포어 있을 경우 계속 실행을 위해 세마포어 증가
		sem_post(gLEDSem);
		if(lvRetValue != 0)
		{
			printf("<YS> 에러! : myled앱에서 sem_post\n", lvRetValue);
		}		


		//
		요청이랑 sem.은 같이 묶어서 뮤텍스로 관리
		요청이 있으면(0b11, 0b10, 0b01) 업데이트
		Timed 요청은 2개짜리 요청 (3레벨, 0b11)
		폴해보고 하고 반영할때까지 뮤텍스 락언락
		변수 이름은 uint8_t requestFlag
		요청하는 쪽은 무조건 세마포어가 1 이하면 1로 증가시킴
		끝내는 쪽은 새로운 일(요청)도 없고
		내가 하던일(Timed도 아니고(sem), Blink도 아닐경우)도 끝나면 무조건 감소시킴
		Timed끝나면 1>이면 1까지 wait, 

		

		// 루프 초기화
		// 초기화 요청 있으면,
		if(gCurrentWS2812.needUpdate == true)
		{
			
			// Timed 초기화 필요시,
			if(gTimedWS2812.needUpdate == true)
			{
				gCurrentWS2812 = gTimedWS2812;
				gCurrentWS2812.needUpdate = gTimedWS2812.needUpdate = false;
				
				lvRetValue = 0;
				for(int i = 0; i < LEDS_NUMBER ; ++i)
				{
					lvRetValue |= gCurrentWS2812.LEDs[i].blink;
				}
				if(lvRetValue != false)
				{
					setMainState(TIMED_BLINK);
				}
				else
				{
					setMainState(TIMED_NONBLINK);
				}
			}
			// : Steady 초기화 필요시
			else
			{
				gCurrentWS2812 = gSteadyWS2812;
				gCurrentWS2812.needUpdate = gSteadyWS2812.needUpdate = false;

				lvRetValue = 0;
				for(int i = 0; i < LEDS_NUMBER ; ++i)
				{
					lvRetValue |= gCurrentWS2812.LEDs[i].blink;
				}

				if(lvRetValue != false)
				{
					setMainState(STEADY_BLINK);
				}
				else
				{
					setMainState(STEADY_NONBLINK);
				}
			}

			// LED 색깔 초기화
			for(int i = 0; i < LEDS_NUMBER ; ++i)
			{
				rgb[i] = (uint32_t)COLOR_TO_RGB(gCurrentWS2812.LEDs[i].color,BRIGHTNESS_DEFAULT);
				gCurrentWS2812.LEDs[i].toggle = 0;
			}
			updateWS2812(rgb);

			// 토글비트 초기화
			for(int i = 0; i < LEDS_NUMBER ; ++i)
			{
				gCurrentWS2812.LEDs[i].toggle = 0;
			}

			// TimeStamp 초기화
			// Timed종료시간 계산 
			if(clock_gettime(CLOCK_REALTIME, &gTimedEndTime) == -1)
			{
				printf("<YS> 에러! : get startTime 실패\n");
				return -1;
			}
			addMsToClock(gCurrentWS2812.durationMs, &gTimedEndTime);
			
			// 최초 토글시간 계산
			for( int i = 0 ; i < LEDS_NUMBER ; ++i )
			{
				if(gCurrentWS2812.LEDs[i].blink == true)
				{
					if(calToggleTime(i) == -1)
					{
						printf("<YS> 에러! : get toggleTime 실패\n");
					}
					// else
					// {
					// 	printf("\n<YS> 최초토글시간 계산\n");
					// }
				}
			}
		}
		else if( getMainState() == OFF )
		{
			sem_wait(gLEDSem);
			continue;
		}
		
		// Blink 제어
		// : 각 LED 마다
		for( int i = 0 ; i < LEDS_NUMBER ; ++i )
		{
			// 해당 LED가 Blink = On 이면
			if(gCurrentWS2812.LEDs[i].blink == true)
			{	
				// 토글해야하는 시간 지났는지 체크
				lvRetValue = checkNeedToggle(i);
				if(lvRetValue == -1)
				{
					printf("<YS> 에러! : 현재시간 get 실패\n");
				}
				// 시간 지났으면 해당 LED RGB,토글 업데이트
				else if (lvRetValue == 1)
				{
					// 해당 LED RGB 업데이트
					rgb[i] = (uint32_t)((COLOR_TO_RGB(gCurrentWS2812.LEDs[i].color,BRIGHTNESS_DEFAULT))*gCurrentWS2812.LEDs[i].toggle);
					// 해당 LED toggle 업데이트
					gCurrentWS2812.LEDs[i].toggle = !gCurrentWS2812.LEDs[i].toggle;	
					// WS2812 set
					updateWS2812(rgb);
					// 다음토글시간 계산
					calToggleTime(i);
				}
			}
		}

		// 디버그용 루프 딜레이
		//usleep(1000*1000*0.05);


		if(pthread_mutex_lock(&gStateLock) != 0)
		{
			printf("<YS> 에러! : gStateLock 실패\n");
		}


		// 요청하는 쪽에서는 무조건 상태 바뀌길 원함
		// → currentWS2812.needUpdate는 항상 1
		// → 세마포어 0이면 1 증가
		// → 바귀길 원하는 WS2812.needUpdate 항상 1

		// 루프 종료 판단
		switch ( getMainState() )
		{
			case STEADY_BLINK:
				//현재
				세마포어 1개
				//판단
				Timed


				if(pthread_mutex_unlock(&gStateLock) != 0)
				{
					printf("<YS> 에러! : gStateLock 실패\n");
				}
				break;
			
			case TIMED_BLINK:
			case TIMED_NONBLINK:

				// 현재시간 get
				if(clock_gettime(CLOCK_REALTIME, &gCurrentTime) == -1)
				{
					printf("<YS> 에러! : get startTime 실패\n");
					return -1;
				}
				// 종료시간 비교
				if(checkTimeout(gCurrentTime, gTimedEndTime) == 1)
				{
					gSteadyWS2812.needUpdate = true;
					gCurrentWS2812.needUpdate = true;
					sem_wait(gLEDSem);
					setMainState(STEADY_NONBLINK);
				}
				if(pthread_mutex_unlock(&gStateLock) != 0)
				{
					printf("<YS> 에러! : gStateLock 실패\n");
				}
			
				break;
			
			case STEADY_NONBLINK:

				sem_wait(gLEDSem);
				setMainState(OFF);

				if(pthread_mutex_unlock(&gStateLock) != 0)
				{
					printf("<YS> 에러! : gStateLock 실패\n");
				}
			
				break;
			
			case OFF:
				break;
		}

	
	}
	
	printf("<YS> 에러 : myled task가 루프 빠져나옴\n");
	close(fd);
	
	return -1;
}

mainState_t getMainState(void)
{
	
	return gMainState;	
}

void setMainState(mainState_t mState)
{	
	gMainState = mState;
	printf("<YS> myled앱에서 현재 state 가 '%d'으로 set 됨.\n", getMainState());
}

void updateWS2812(uint32_t * buf)
{
	if(lseek(fd, 0, SEEK_SET))
	{

	}
	if(write(fd, buf, 4*LEDS_NUMBER))
	{
		
	}
}

int calToggleTime(int ledIndex)
{
	if(clock_gettime(CLOCK_REALTIME, &gCurrentWS2812.LEDs[ledIndex].toggleTime) == -1)
	{
		printf("<YS> 에러! : get toggleTime 실패\n");
		return -1;
	}
	else
	{	
		// printf("<YS> 계산전 toggle time\n");
		// printf("<YS> LED%d toggleTime = %d, ", ledIndex, gCurrentWS2812.LEDs[ledIndex].toggleTime.tv_sec);
		// printf("%d\n", gCurrentWS2812.LEDs[ledIndex].toggleTime.tv_nsec);
		
		addMsToClock(gCurrentWS2812.LEDs[ledIndex].togglePeriodMs, &gCurrentWS2812.LEDs[ledIndex].toggleTime );
		
		// printf("<YS> 계산 후 toggle time\n");
		// printf("<YS> LED%d toggleTime = %d, ", ledIndex, gCurrentWS2812.LEDs[ledIndex].toggleTime.tv_sec);
		// printf("%d\n", gCurrentWS2812.LEDs[ledIndex].toggleTime.tv_nsec);
		return 0;
	}
}

int checkNeedToggle(int ledIndex)
{

	if(clock_gettime(CLOCK_REALTIME, &gCurrentTime) == -1)
	{
		printf("<YS> 에러! : get gCurrentTime 실패\n");
		return -1;
	}
	else if( checkTimeout(gCurrentTime, gCurrentWS2812.LEDs[ledIndex].toggleTime) )
	{
		// printf("<YS> 현재시간 = %d, ", c.tv_sec);
		// printf("%d\n", c.tv_nsec);

		// printf("<YS> ^오^\n");
		return 1;
	}
	else
	{	
		// printf("<YS> 현재시간 = %d, ", c.tv_sec);
		// printf("%d\n", c.tv_nsec);

		// printf("<YS> ㅜ0ㅜ\n");
		return 0;
	}
}

int checkTimeout(struct timespec c, struct timespec w)
{
	if( (c.tv_sec > w.tv_sec) || ( (c.tv_sec == w.tv_sec) && (c.tv_nsec >= w.tv_nsec) ) )
	{
		return 1;
	}
	else
	{	
		return 0;
	}
}

void addMsToClock(int ms, struct timespec* c)
{
	c->tv_sec += ((time_t)ms / 1000) + ((c->tv_nsec + (((time_t)ms % 1000) *1000*1000)) / (MAX_NSEC+1));
	c->tv_nsec = (c->tv_nsec + (((time_t)ms % 1000) *1000*1000)) % (MAX_NSEC+1);
}



void setSteadyLED(void)
{	
	// 락
	sem_t *ledSem = NULL;
	ledSem = sem_open(LED_SEM_NAME, 0);
	int semValue;

	// 초기화 해줘야되는지 판단
	switch (getMainState())
	{
		case STEADY_BLINK:
			gSteadyWS2812.needUpdate = true;
			gCurrentWS2812.needUpdate = true;
		break;

		case TIMED_BLINK:
		case TIMED_NONBLINK:
		break;

		case STEADY_NONBLINK:
			gSteadyWS2812.needUpdate = true;
			gCurrentWS2812.needUpdate = true;
			sem_post(ledSem);
		break;

		case OFF:
			gSteadyWS2812.needUpdate = true;
			gCurrentWS2812.needUpdate = true;
			sem_getvalue(ledSem, &semValue);
			if(semValue == 0)
			{	
				sem_post(ledSem);
			}
		break;
	}
	sem_close(ledSem);
	
}

void setTimedLED(void)
{	
	
	sem_t *ledSem = NULL;
	ledSem = sem_open(LED_SEM_NAME, 0);
	// 초기화 해줘야되는지 판단
	switch (getMainState())
	{
		case STEADY_BLINK:
		case TIMED_BLINK:
		case TIMED_NONBLINK:

			gTimedWS2812.needUpdate = true;
			gCurrentWS2812.needUpdate = true;
			break;

		case STEADY_NONBLINK:

			gTimedWS2812.needUpdate = true;
			gCurrentWS2812.needUpdate = true;
			sem_post(ledSem);
			break;

		case OFF:

			gTimedWS2812.needUpdate = true;
			gCurrentWS2812.needUpdate = true;
			sem_post(ledSem);
			break;
	}
	
}


// 동시 접근 처리?