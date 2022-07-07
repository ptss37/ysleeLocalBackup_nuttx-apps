//printf("<YS> File : %s, Function : %s, Line : %d\n", __FILE__, __FUNCTION__, __LINE__);

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
#include <nuttx/board.h>
#include <sched.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>
#include <semaphore.h>
#include <s32k1xx_pin.h>



#include <ws2812.h>
#include "s32k1xx_lpspi.h"
#include "main.h"

/****************************************************************************
 * Defines
 ****************************************************************************/

/****************************************************************************
 * Types
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
// static int ledTaskFunc(int argc, char *argv[]);

static int virtualBMSTaskFunc(int argc, char *argv[]);

static int ledTaskFunc(int argc, char *argv[]);
/****************************************************************************
 * main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
	// 변수 선언
	int ret;

	// SPI 초기화
	ledSPI = s32k1xx_lpspibus_initialize(LPSPI1);
	if(ledSPI == NULL)
	{
		printf("<YS> myled앱에서 SPI 장치 초기화 실패\n");
		return -1;
	}
	
	// WS2812 장치 등록
	ret = ws2812_leds_register(CONFIG_EXAMPLES_WS2812_DEVNAME, ledSPI, LED_NUMBER);
 	if (ret < 0)
	{
		fprintf(stderr, "ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_WS2812_DEVNAME, errno); 
		return -1;
	}
	
	// WS2812 Open
	fd = open(CONFIG_EXAMPLES_WS2812_DEVNAME, O_RDWR);
	if (fd < 0)
	{
		fprintf(stderr, "ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_WS2812_DEVNAME, errno); 
		return -1;
	}

	// 뮤텍스 초기화
	pthread_mutex_init(&gLEDTaskLock, NULL);

	// Task 생성
	ret = task_create("led", DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, ledTaskFunc, NULL);
	if (ret < 0)
	{
		printf("<YS> interpreter Task 생성 실패\n");
		return -1;
	}

	ret = task_create("test", DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, virtualBMSTaskFunc, argv);
	if (ret < 0)
	{
		printf("<YS> virtualBMS Task 생성 실패\n");
		return -1;
	}

	return 0;
}

static int virtualBMSTaskFunc(int argc, char *argv[])
{
	int cmd;
	
	board_button_initialize();

	board_button_irq(BUTTON_SW2, sw2ISR, NULL);

	// ret = gpio_registerISR(BCC_FAULT, BCCFaultPinISR, 1);
	// if(ret)
	// {
	// 	printf("<YS> 에러 : 버튼 인터럽트 등록 실패!\n");
	// }

	if (argc > 3)
	{
		cmd = atoi(argv[2]);

		switch (cmd)
		{
			case CHARGING:
				usleep(atoi(argv[3])*1000*1000);
				led_setLED(CHARGING);
				
			break;
			
			case LED_OFF:
				usleep(atoi(argv[3])*1000*1000);
				led_setLED(LED_OFF);
			break;

			case SELF_DISCHARGE:
				usleep(atoi(argv[3])*1000*1000);
				led_setLED(SELF_DISCHARGE);
			break;

			case FAULT:
				usleep(atoi(argv[3])*1000*1000);
				led_setLED(FAULT);
			break;

			case BATT_REM:
				usleep(atoi(argv[3])*1000*1000);
				led_setLED(BATT_REM);
			break;

			case NO_REQUEST:
				printf("<YS> BATT_REM(1), FAULT(2), SELF_DISCHARGE(3), LED_OFF(4), CHARGING(5)\n");
			break;
		}
	} 
	
	while(1)
	{
		usleep(1*1000*1000);
	}
	

	return 0;
}

static int ledTaskFunc(int argc, char *argv[])
{	
	struct sigevent blinkNoti, timeoutNoti;
	struct sigaction actBlink, actTimeout;
	int ret;

	//LED Pattern 초기화
	initLEDPattern();
	
	// Signal 초기화
	memset(&actBlink, 0, sizeof(struct sigaction));
	actBlink.sa_sigaction = blinkSignalHandlerFunc;
	actBlink.sa_flags = SA_SIGINFO;

	sigemptyset(&actBlink.sa_mask);

	ret = sigaction(SIGALRM, &actBlink, NULL);
	if( ret != 0)
	{
		printf("<YS> blink시그널액션 등록 실패\n");
	}

	memset(&actTimeout, 0, sizeof(struct sigaction));
	actTimeout.sa_sigaction = timeoutSignalHandlerFunc;
	actTimeout.sa_flags = SA_SIGINFO;

	sigemptyset(&actTimeout.sa_mask);

	ret = sigaction(SIGPIPE, &actTimeout, NULL);
	if(ret != 0)
	{
		printf("<YS> blink시그널액션 등록 실패\n");
	}

	// Timer 초기화
	blinkNoti.sigev_notify            = SIGEV_SIGNAL;
	blinkNoti.sigev_signo             = SIGALRM;
	blinkNoti.sigev_value.sival_int   = 0;

	ret = timer_create(CLOCK_REALTIME, &blinkNoti, &blinkTimerId);
	if(ret != OK)
	{
		printf("sigev_thread_test: timer_create failed, errno=%d\n", errno);
		return -1;
	}

	timeoutNoti.sigev_notify            = SIGEV_SIGNAL;
	timeoutNoti.sigev_signo             = SIGPIPE;
	timeoutNoti.sigev_value.sival_int   = 0;

	ret = timer_create(CLOCK_REALTIME, &timeoutNoti, &timeoutTimerId);
	if(ret != OK)
	{
		printf("sigev_thread_test: timer_create failed, errno=%d\n", errno);
		return -1;
	}

	ledTaskInitialized = true;
	led_setLED(LED_OFF);

	while(1)
	{
		if( pthread_mutex_lock(&gLEDTaskLock) == -1 )
		{
			printf("<YS> 에러! : gLEDTaskLock 뮤텍스 락 실패\n");
			return -1;
		}

		switch (gRequest)
		{
			case CHARGING:
			case LED_OFF:
			case SELF_DISCHARGE:
			case FAULT:
				
				if(gMainState != gRequest)
				{
					gOldState = gMainState = gRequest;
				
					gRequest = NO_REQUEST;
					
					if( pthread_mutex_unlock(&gLEDTaskLock) == -1 )
					{
						printf("<YS> 에러! : gTaskControlLock 뮤텍스 언락 실패\n");
						return -1;
					}

					disarmTimer(blinkTimerId, blinkTimerSpec);
					
					disarmTimer(timeoutTimerId, timeoutTimerSpec);
					
					setLEDPattern(gMainState);

					setTimer(gLEDPattern);

					packLEDPattern(gRGBBuf, gLEDPattern);

					pushBuffer(gRGBBuf);
				}
				else
				{
					gRequest = NO_REQUEST;

					if( pthread_mutex_unlock(&gLEDTaskLock) == -1 )
					{
						printf("<YS> 에러! : gTaskControlLock 뮤텍스 언락 실패\n");
						return -1;
					}
				}

			break;

			case BATT_REM:

				gMainState = gRequest;
			
				gRequest = NO_REQUEST;

				if( pthread_mutex_unlock(&gLEDTaskLock) == -1 )
				{
					printf("<YS> 에러! : gTaskControlLock 뮤텍스 언락 실패\n");
					return -1;
				}

				disarmTimer(blinkTimerId, blinkTimerSpec);
				
				disarmTimer(timeoutTimerId, timeoutTimerSpec);
				
				setLEDPattern(gMainState);

				setTimer(gLEDPattern);

				packLEDPattern(gRGBBuf, gLEDPattern);

				pushBuffer(gRGBBuf);

			break;
		
			case NO_REQUEST:

				if( pthread_mutex_unlock(&gLEDTaskLock) == -1 )
				{
					printf("<YS> 에러! : gTaskControlLock 뮤텍스 언락 실패\n");
					return -1;
				}
				usleep(0.1*1000*1000);
			
			break;
		}
	}
	
	printf("<YS> ledTask 종료됨 빠져나옴\n");
	
	return -1;
}

static void blinkSignalHandlerFunc(int signo, siginfo_t *siginfo, void *arg)
{
	for( int i = 0 ; i < LED_NUMBER ; ++i)
	{
		if(gLEDPattern.blink[i] == true)
		{	
			gRGBBuf[i] = (uint32_t)( ( COLOR_TO_RGB(gLEDPattern.color[i], BRIGHTNESS_DEFAULT) )*gLEDPattern.nextToggle );
		}
	}

	pushBuffer(gRGBBuf);

	printf("<YS> Toggle! ^오^\n");

	gLEDPattern.nextToggle = !gLEDPattern.nextToggle;
	if(gLEDPattern.nextToggle == true)
	{
		armTimerOnce(blinkTimerId, blinkTimerSpec, gLEDPattern.onTimeMs);
	}
	else
	{
		armTimerOnce(blinkTimerId, blinkTimerSpec, gLEDPattern.offTimeMs);
	}
}

static void timeoutSignalHandlerFunc(int signo, siginfo_t *siginfo, void *arg)
{
	if( pthread_mutex_lock(&gLEDTaskLock) == -1 )
	{
		printf("<YS> 에러! : gTaskControlLock 뮤텍스 락 실패\n");
	}
		
	if(gRequest == NO_REQUEST)
	{
		gRequest = gOldState;
	}
	printf("  ㅜ오ㅜ\n");

	if( pthread_mutex_unlock(&gLEDTaskLock) == -1 )
	{
		printf("<YS> 에러! : gTaskControlLock 뮤텍스 락 실패\n");
	}
}

int led_setLED(ledState_t r)
{	
	if(ledTaskInitialized == true)
	{
		if( pthread_mutex_lock(&gLEDTaskLock) == -1 )
		{
			printf("<YS> 에러! : gTaskControlLock 뮤텍스 락 실패\n");
			return -1;
		}
		
		gRequest = r;	
		if( pthread_mutex_unlock(&gLEDTaskLock) == -1 )
		{
			printf("<YS> 에러! : gTaskControlLock 뮤텍스 락 실패\n");
			return -1;
		}
		
		return 0;
	}
	else
	{
		printf("<YS> Error : ledTask 초기화 안됐음\n");
		return -1;
	}
}

void setLEDPattern(ledState_t r)
{	
	gLEDPattern = ledPattern[r];
}

void initLEDPattern(void)
{
	for(int i = 0 ; i < LED_NUMBER ; ++i )
	{
		ledPattern[NO_REQUEST].color[i] = OFF;
		ledPattern[NO_REQUEST].blink[i] = false;
	}
	ledPattern[LED_OFF].onTimeMs = 0;
	ledPattern[LED_OFF].offTimeMs = 0;
	ledPattern[LED_OFF].durationMs = CONTINUOUS;
	ledPattern[LED_OFF].nextToggle = 0;

	for(int i = 0 ; i < LED_NUMBER ; ++i )
	{
		ledPattern[CHARGING].color[i] = GREEN;
		ledPattern[CHARGING].blink[i] = false;
	}
	ledPattern[CHARGING].onTimeMs = 500;
	ledPattern[CHARGING].offTimeMs = 500;
	ledPattern[CHARGING].durationMs = CONTINUOUS;
	ledPattern[CHARGING].nextToggle = 0;

	for(int i = 0 ; i < LED_NUMBER ; ++i )
	{
		ledPattern[BATT_REM].color[i] = GREEN;
		ledPattern[BATT_REM].blink[i] = false;
	}
	ledPattern[BATT_REM].onTimeMs = 250;
	ledPattern[BATT_REM].offTimeMs = 250;
	ledPattern[BATT_REM].durationMs = 3000;
	ledPattern[BATT_REM].nextToggle = 0;

	for(int i = 0 ; i < LED_NUMBER ; ++i )
	{
		ledPattern[FAULT].color[i] = OFF;
		ledPattern[FAULT].blink[i] = false;
	}
	ledPattern[FAULT].blink[0] = true;
	ledPattern[FAULT].color[0] = RED;
	ledPattern[FAULT].onTimeMs = 500;
	ledPattern[FAULT].offTimeMs = 500;
	ledPattern[FAULT].durationMs = CONTINUOUS;
	ledPattern[FAULT].nextToggle = 0;

	for(int i = 0 ; i < LED_NUMBER ; ++i )
	{
		ledPattern[SELF_DISCHARGE].color[i] = BLUE;
		ledPattern[SELF_DISCHARGE].blink[i] = true;
	}
	ledPattern[SELF_DISCHARGE].onTimeMs = 500;
	ledPattern[SELF_DISCHARGE].offTimeMs = 500;
	ledPattern[SELF_DISCHARGE].durationMs = CONTINUOUS;
	ledPattern[SELF_DISCHARGE].nextToggle = 0;
	

	//임시 배터리 상태
	ledPattern[CHARGING].color[3] = OFF;
	ledPattern[CHARGING].blink[3] = false;
	ledPattern[CHARGING].color[2] = GREEN;
	ledPattern[CHARGING].blink[2] = true;

	ledPattern[BATT_REM].color[3] = OFF;
	ledPattern[BATT_REM].blink[3] = false;
	ledPattern[BATT_REM].color[2] = GREEN;
	ledPattern[BATT_REM].blink[2] = true;


}

int disarmTimer(timer_t tid, struct itimerspec ts)
{
	int ret;

	ts.it_value.tv_sec     = 0;
	ts.it_value.tv_nsec    = 0;

	ret = timer_settime(tid, 0, &ts, NULL);
	if(ret < 0 )
	{
		printf("<YS> 에러 : Timer Disarm 실패\n");
		return -1;
	}
	return 0;
}

int setTimer(ledPattern_t lp)
{
	int ret;
	
	ret = armTimerOnce(blinkTimerId, blinkTimerSpec, lp.offTimeMs);
	if(ret < 0)
	{
		printf("<YS> 에러! : armTimerOnce(blink) 실패\n");
		return -1;
	}

	ret = armTimerOnce(timeoutTimerId, timeoutTimerSpec, lp.durationMs);
	if(ret < 0)
	{
		printf("<YS> 에러! : armTimerOnce(blink) 실패\n");
		return -1;
	}
	
	return 0;
}

int armTimerOnce(timer_t tid, struct itimerspec ts, uint32_t ms)
{
	int ret;
	ts.it_value.tv_sec = (time_t)MSEC_TO_SEC(ms);
	ts.it_value.tv_nsec = (long)MSEC_TO_NSEC(ms);
	ts.it_interval.tv_sec = 0;
	ts.it_interval.tv_nsec = 0;

	ret = timer_settime(tid, 0, &ts, NULL);
	if(ret < 0 )
	{
		printf("<YS> 에러 : Timer Disarm 실패\n");
		return -1;
	}
	
	return 0;
	
}

void packLEDPattern(uint32_t * buf, ledPattern_t lp)
{
	for(int i = 0; i < LED_NUMBER ; ++i)
	{
		buf[i] = (uint32_t)COLOR_TO_RGB(lp.color[i],BRIGHTNESS_DEFAULT);
	}
}

void pushBuffer(uint32_t* buf)
{
	if(lseek(fd, 0, SEEK_SET))
	{

	}
	if(write(fd, buf, 4*LED_NUMBER))
	{
		
	}
}

int sw2ISR(int irq, FAR void *context, FAR void *arg)
{
	if (board_buttons() == 0)
	{
		led_setLED(BATT_REM);
	}
	return 0;
}

// int gpio_registerISR(pinEnum_t pin, _sa_handler_t  pinISRHandler, bool num)
// {
// 	int signalNumber = SIGUSR1;
// 	int lvRetValue = -1, lvError = 0;
// 	int fd = -1;
// 	int lvErrorCode;
// 	char devPath[] = "/dev/gpint1";
// 	struct sigevent notify;
// 	_sa_handler_t sigRetValue;

// 	fd = open(devPath, O_RDWR);
// 	if(fd < 0)
// 	{
// 		printf("<YS> ㅇ러 : ")
// 	}
	
	

// 	// check if the signalnumber needs to change
// 	if(num)
// 	{
// 	// change the number
// 	signalNumber = SIGUSR2;
// 	}

// 	// get the file descriptor
// 	lvFd = setNGetFileDescriptior(false, pin, -1);

// 	// check for errors
// 	if(lvFd < 0)
// 	{
// 	lvError += ERROR_OPEN_DEV;
// 	cli_printfError("GPIO ERROR: Failed to get file descriptor %s: %d\n", pin, lvFd);
// 	}

// 	// check if it all went ok
// 	if(!lvError)
// 	{

// 	//cli_printf("setting notify %d!\n", pin);
// 	// set the notify signal
// 	notify.sigev_notify = SIGEV_SIGNAL;
// 	notify.sigev_signo  = signalNumber;//pin;

// 	// register the interrupt 
// 	lvRetValue = ioctl(lvFd, GPIOC_REGISTER, (unsigned long)&notify);
// 	if (lvRetValue < 0)
// 	{
// 		lvErrorCode = errno;
// 		cli_printfError("GPIO ERROR: Failed to register interrupt for %d: %d\n",
// 			pin, lvErrorCode);

// 		// set the error value
// 		lvError += ERROR_REGISTER_INT;
// 	}

// 	}

// 	// check if it all went ok
// 	if(!lvError)
// 	{
// 	// register the handler
// 	sigRetValue = signal(signalNumber, pinISRHandler);

// 	// check for errors
// 	if(sigRetValue == SIG_ERR)
// 	{
// 		// check errno for the error
// 		lvErrorCode = errno;
// 		cli_printfError("GPIO ERROR: Failed to register signal to ISR pin: %d sig: %d err:%d\n", pin, (pin+GPIO_SIG_OFFSET), lvErrorCode);

// 		// set the errorvalue
// 		lvError += ERROR_REGISTER_ISR;
// 	}
// 	}

// 	// return
// 	return lvRetValue;
// }
// static int ledTaskFunc(int argc, char *argv[])
// {
// 	int ret = 0;
// 	struct timespec waitTime;

// 	// 루프 시작 
// 	while(1)
// 	{
// 		// Task 제어
// 		// 세마포어 있을 때 까지 Task 정지
// 		ret = sem_wait(&gLEDSem);
// 		if(ret != 0)
// 		{
// 			printf("<YS> myled앱에서 sem_wait()가 세마포어 없이 빠져나옴\n", ret);
// 		}

// 		// 세마포어 있을 경우 계속 실행을 위해 세마포어 증가
// 		ret = sem_post(&gLEDSem);
// 		if(ret != 0)
// 		{
// 			printf("<YS> 에러! : myled앱에서 sem_post\n", ret);
// 		}		

// 		if(gCurrentWS2812.blink == true)
// 		{
// 			sem_timedwait(&gLEDWaitSem, &waitTime);

// 			for( int i = 0 ; i < LED_NUMBER ; ++i )
// 			{
// 				// 해당 LED RGB 업데이트
// 				gRGBBuf[i] = (uint32_t)((COLOR_TO_RGB(gCurrentWS2812.led[i].color,BRIGHTNESS_DEFAULT))*nextToggle);
// 			}
// 			// 해당 LED nextToggle 업데이트
// 			nextToggle = !nextToggle;	
// 			// WS2812 set
// 			pushBuffer(gRGBBuf);
// 			// 다음토글시간 계산
// 			calculateToggleTime();
// 		}

		
// 		/*
// 		// Task 초기화
// 		if( pthread_mutex_lock(&gTaskControlLock) == -1 )
// 		{
// 			printf("<YS> 에러! : gTaskControlLock 뮤텍스 락 실패\n");
// 			return -1;
// 		}

// 		if( (gRequest > NO_REQUEST) && (gRequest >= gMainState) )
// 		{
// 			gMainState = gRequest;

// 			if(gRequest == 2)
// 			{
// 				gCurrentWS2812 = gTimedWS2812;
// 				gRequest = STEADY;				
// 			}
// 			else
// 			{
// 				gCurrentWS2812 = gSteadyWS2812;
// 				gRequest = NO_REQUEST;
// 			}

// 			if (pthread_mutex_unlock(&gTaskControlLock) == -1)
// 			{
// 				printf("<YS> 에러! : gTaskControlLock 뮤텍스 언락 실패\n");
// 				return -1;
// 			}

// 			// LED 색깔 초기화
// 			for(int i = 0; i < LED_NUMBER ; ++i)
// 			{
// 				gRGBBuf[i] = (uint32_t)COLOR_TO_RGB(gCurrentWS2812.led[i].color,BRIGHTNESS_DEFAULT);
				
// 			}
// 			pushBuffer(gRGBBuf);

// 			// 토글비트 초기화
// 			nextToggle = false;

// 			// TimeStamp 초기화
// 			// Timed종료시간 계산 
// 			if(clock_gettime(CLOCK_REALTIME, &gTimedEndTime) == -1)
// 			{
// 				printf("<YS> 에러! : get startTime 실패\n");
// 				return -1;
// 			}
			
// 			addMsToClock(gCurrentWS2812.durationMs, &gTimedEndTime);
			
// 			// 최초 토글시간 계산
// 			if(gCurrentWS2812.blink == true)
// 			{
// 				if(calculateToggleTime() == -1)
// 					{
// 						printf("<YS> 에러! : get toggleTime 실패\n");
// 					}
// 			}
// 		}
// 		else
// 		{
// 			if (pthread_mutex_unlock(&gTaskControlLock) == -1)
// 			{
// 				printf("<YS> 에러! : gTaskControlLock 뮤텍스 언락 실패\n");
// 				return -1;
// 			}
// 		}

// 		// Blink 제어
// 		// 각 LED 마다,
// 		if(gCurrentWS2812.blink == true)
// 		{
// 			ret = checkNeedToggle();
// 			if(ret == -1)
// 			{
// 				printf("<YS> 에러! : 현재시간 get 실패\n");
// 			}
// 			else if (ret == 1)
// 			{
// 				for( int i = 0 ; i < LED_NUMBER ; ++i )
// 				{
// 					// 해당 LED RGB 업데이트
// 					gRGBBuf[i] = (uint32_t)((COLOR_TO_RGB(gCurrentWS2812.led[i].color,BRIGHTNESS_DEFAULT))*nextToggle);
// 				}
// 				// 해당 LED nextToggle 업데이트
// 				nextToggle = !nextToggle;	
// 				// WS2812 set
// 				pushBuffer(gRGBBuf);
// 				// 다음토글시간 계산
// 				calculateToggleTime();
// 			}
// 		}

// 		//usleep(1000*1000*0.05); // 디버그용 딜레이
// 		// 다음 State 판단
// 		if( pthread_mutex_lock(&gTaskControlLock) == -1 )
// 		{
// 			printf("<YS> 에러! : gTaskControlLock 뮤텍스 락 실패\n");
// 			return -1;
// 		}

// 		switch ( gMainState )
// 		{
// 			case TIMED:
				
// 				// 현재시간 get
// 				if(clock_gettime(CLOCK_REALTIME, &gCurrentTime) == -1)
// 				{
// 					printf("<YS> 에러! : get startTime 실패\n");
// 					return -1;
// 				}
				
// 				// MainState 바꿔야할지 비교
// 				if( (checkTimeout(gCurrentTime, gTimedEndTime)==true) && (gRequest < 2) )
// 				{
// 					gMainState = STEADY;
// 				}
// 				break;
			
// 			case STEADY:

// 				if( (gCurrentWS2812.blink==false) && (gRequest == NO_REQUEST) )
// 				{
					
// 					do
// 					{
// 						sem_wait(&gLEDSem);

// 						if( sem_getvalue(&gLEDSem, &ret) == -1 )
// 						{
// 							printf("<YS> 에러! : get startTime 실패\n");
// 							return -1;
// 						}

// 					} while( ret > 0 );
// 				}
// 				break;

// 			case NO_REQUEST:
// 				printf("<YS> 에러! : MainState = NO_REQUREST\n");
// 				return -1;
// 		}

// 		if( pthread_mutex_unlock(&gTaskControlLock) == -1 )
// 		{
// 			printf("<YS> 에러! : gTaskControlLock 뮤텍스 언락 실패\n");
// 			return -1;
// 		}
// 		*/
// 	}
	
// 	printf("<YS> 에러 : myled task가 루프 빠져나옴\n");
// 	close(fd);
	
// 	return -1;
// }


// int calculateToggleTime(void)
// {
// 	int ret;

// 	if(clock_gettime(CLOCK_REALTIME, &gWaitTime) == -1)
// 	{
// 		printf("<YS> 에러! : get gCurrentTime 실패\n");
// 		return -1;
// 	}
// 	else if(nextToggle)
// 	{
// 		addMsToClock(gCurrentWS2812.onTimeMs, &gWaitTime);
// 		return 0;
// 	}
// 	else
// 	{
// 		addMsToClock(gCurrentWS2812.offTimeMs, &gWaitTime);
// 		return 0;
// 	}
	
// 	/*
// 	if(clock_gettime(CLOCK_REALTIME, &gToggleTime) == -1)
// 	{
// 		printf("<YS> 에러! : get toggleTime 실패\n");
// 		return -1;
// 	}
// 	else
// 	{	
// 		// printf("<YS> 계산전 nextToggle time\n");
// 		// printf("<YS> LED%d toggleTime = %d, ", ledIndex, gCurrentWS2812.LEDs[ledIndex].toggleTime.tv_sec);
// 		// printf("%d\n", gCurrentWS2812.LEDs[ledIndex].toggleTime.tv_nsec);
		
// 		addMsToClock(gCurrentWS2812.togglePeriodMs, &gCurrentWS2812.LEDs.toggleTime );
		
// 		// printf("<YS> 계산 후 nextToggle time\n");
// 		// printf("<YS> LED%d toggleTime = %d, ", ledIndex, gCurrentWS2812.LEDs[ledIndex].toggleTime.tv_sec);
// 		// printf("%d\n", gCurrentWS2812.LEDs[ledIndex].toggleTime.tv_nsec);
// 		return 0;
// 	}
// 	*/
// }

// int calculateToggleTime(void)
// {
// 	int ret;

// 	if(clock_gettime(CLOCK_REALTIME, &gDurationTime) == -1)
// 	{
// 		printf("<YS> 에러! : get gCurrentTime 실패\n");
// 		return -1;
// 	}
// 	else
// 	{
// 		addMsToClock(gCurrentWS2812.durationMs, &gDurationTime);
// 		return 0;
// 	}

// }

// int checkNeedToggle(int ledIndex)
// {

// 	if(clock_gettime(CLOCK_REALTIME, &gCurrentTime) == -1)
// 	{
// 		printf("<YS> 에러! : get gCurrentTime 실패\n");
// 		return -1;
// 	}
// 	else if( checkTimeout(gCurrentTime, gCurrentWS2812.LEDs[ledIndex].toggleTime) )
// 	{
// 		// printf("<YS> 현재시간 = %d, ", c.tv_sec);
// 		// printf("%d\n", c.tv_nsec);

// 		// printf("<YS> ^오^\n");
// 		return 1;
// 	}
// 	else
// 	{	
// 		// printf("<YS> 현재시간 = %d, ", c.tv_sec);
// 		// printf("%d\n", c.tv_nsec);

// 		// printf("<YS> ㅜ0ㅜ\n");
// 		return 0;
// 	}
// }

// int checkTimeout(struct timespec c, struct timespec w)
// {
// 	if( (c.tv_sec > w.tv_sec) || ( (c.tv_sec == w.tv_sec) && (c.tv_nsec >= w.tv_nsec) ) )
// 	{
// 		return 1;
// 	}
// 	else
// 	{	
// 		return 0;
// 	}
// }

// void addMsToClock(int ms, struct timespec* c)
// {
// 	c->tv_sec += ((time_t)ms / 1000) + ((c->tv_nsec + (((time_t)ms % 1000) *1000*1000)) / (MAX_NSEC+1));
// 	c->tv_nsec = (c->tv_nsec + (((time_t)ms % 1000) *1000*1000)) % (MAX_NSEC+1);
// }





// int setSteadyLED(uint8_t* c, bool* b, uint32_t ont, uint32_t offt)
// {	
// 	int ret = 0;

// 	if( pthread_mutex_lock(&gTaskControlLock) == -1 )
// 	{
// 		printf("<YS> 에러! : gTaskControlLock 뮤텍스 락 실패\n");
// 		return -1;
// 	}
	
// 	gSteadyWS2812.blink = false;

// 	for(int i = 0; i < LED_NUMBER ; ++i)
// 	{
// 		gSteadyWS2812.led[i].color = c[i];
// 		gSteadyWS2812.led[i].blink = b[i];
// 		gSteadyWS2812.blink |= b[i];
// 	}

// 	gSteadyWS2812.onTimeMs = ont;
// 	gSteadyWS2812.offTimeMs = offt;

// 	if( sem_getvalue(&gLEDSem, &ret) == -1 )
// 	{
// 		printf("<YS> 에러! :  sem_getvalue(ledSem) 실패\n");
// 		return -1;
// 	}

// 	if( ret < 1)
// 	{
// 		sem_post(&gLEDSem);
// 	}

// 	if(gRequest < 1 )
// 	{
// 		gRequest = STEADY;
// 	}

// 	if( pthread_mutex_unlock(&gTaskControlLock) == -1 )
// 	{
// 		printf("<YS> 에러! : gTaskControlLock 뮤텍스 언락 실패\n");
// 		return -1;
// 	}

// 	return 0;
// }

// int setTimedLED(uint8_t* c, bool* b, uint32_t ont,  uint32_t offt, uint32_t d)
// {	
// 	int ret;

// 	if( pthread_mutex_lock(&gTaskControlLock) == -1 )
// 	{
// 		printf("<YS> 에러! : gTaskControlLock 뮤텍스 락 실패\n");
// 		return -1;
// 	}

// 	gTimedWS2812.blink = false;

// 	for(int i = 0; i < LED_NUMBER ; ++i)
// 	{
// 		gTimedWS2812.led[i].color = c[i];
// 		gTimedWS2812.led[i].blink = b[i];
// 		gTimedWS2812.blink |= b[i];
// 	}

// 	gTimedWS2812.onTimeMs = ont;
// 	gTimedWS2812.offTimeMs = offt;
// 	gTimedWS2812.durationMs = d;

// 	if( sem_getvalue(&gLEDSem, &ret) == -1 )
// 	{
// 		printf("<YS> 에러! :  sem_getvalue(ledSem) 실패\n");
// 		return -1;
// 	}

// 	if( ret < 1)
// 	{
// 		sem_post(&gLEDSem);
// 	}

// 	if(gRequest < 2 )
// 	{
// 		gRequest = TIMED;
// 	}

// 	if( pthread_mutex_unlock(&gTaskControlLock) == -1 )
// 	{
// 		printf("<YS> 에러! : gTaskControlLock 뮤텍스 언락 실패\n");
// 		return -1;
// 	}

// 	return 0;
// }


// 동시 접근 처리?

//printf("<YS> File : %s, Function : %s, Line : %d\n", __FILE__, __FUNCTION__, __LINE__);