/****************************************************************************
 * examples/pwm/pwm_main.c
 *
 *   Copyright (C) 2011-2012, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modifyification, are permitted provided that the following conditions
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
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/timers/pwm.h>

#include "pwm.h"
#include "../arch/arm/src/common/arm_arch.h"
#include "../arch/arm/src/stm32/hardware/stm32f10xxx_rcc.h"
#include "../arch/arm/src/stm32/hardware/stm32f10xxx_memorymap.h"
#include "../arch/arm/src/stm32/hardware/stm32_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DUTY_RATE         0x0088

/* Configuration ************************************************************/

#ifdef CONFIG_PWM_MULTICHAN
#  if CONFIG_PWM_NCHANNELS > 1
#    if CONFIG_EXAMPLES_PWM_CHANNEL1 == CONFIG_EXAMPLES_PWM_CHANNEL2
#      error "Channel numbers must be unique"
#    endif
#  endif
#  if CONFIG_PWM_NCHANNELS > 2
#    if CONFIG_EXAMPLES_PWM_CHANNEL1 == CONFIG_EXAMPLES_PWM_CHANNEL3 || \
        CONFIG_EXAMPLES_PWM_CHANNEL2 == CONFIG_EXAMPLES_PWM_CHANNEL3
#      error "Channel numbers must be unique"
#    endif
#  endif
#  if CONFIG_PWM_NCHANNELS > 3
#    if CONFIG_EXAMPLES_PWM_CHANNEL1 == CONFIG_EXAMPLES_PWM_CHANNEL4 || \
        CONFIG_EXAMPLES_PWM_CHANNEL2 == CONFIG_EXAMPLES_PWM_CHANNEL4 || \
        CONFIG_EXAMPLES_PWM_CHANNEL3 == CONFIG_EXAMPLES_PWM_CHANNEL4
#      error "Channel numbers must be unique"
#    endif
#  endif
#endif
/****************************************************************************
 * Private Types
 ****************************************************************************/
struct pwm_state_s
{
  bool      initialized;
  FAR char *devpath;
#ifdef CONFIG_PWM_MULTICHAN
  uint8_t   channels[CONFIG_PWM_NCHANNELS];
  uint8_t   duties[CONFIG_PWM_NCHANNELS];
#else
  uint8_t   duty;
#endif
  uint32_t  freq;
#ifdef CONFIG_PWM_PULSECOUNT
  uint32_t  count;
#endif
  int       duration;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct pwm_state_s g_pwmstate;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_devpath
 ****************************************************************************/

static void pwm_devpath(FAR struct pwm_state_s *pwm, FAR const char *devpath)
{
  /* Get rid of any old device path */

  if (pwm->devpath)
    {
      free(pwm->devpath);
    }

  /* Then set-up the new device path by copying the string */

  pwm->devpath = strdup(devpath);
}

/****************************************************************************
 * Name: pwm_help
 ****************************************************************************/

static void pwm_help(FAR struct pwm_state_s *pwm)
{
#ifdef CONFIG_PWM_MULTICHAN
  uint8_t channels[CONFIG_PWM_NCHANNELS] =
  {
    CONFIG_EXAMPLES_PWM_CHANNEL1,
#if CONFIG_PWM_NCHANNELS > 1
    CONFIG_EXAMPLES_PWM_CHANNEL2,
#endif
#if CONFIG_PWM_NCHANNELS > 2
    CONFIG_EXAMPLES_PWM_CHANNEL3,
#endif
#if CONFIG_PWM_NCHANNELS > 3
    CONFIG_EXAMPLES_PWM_CHANNEL4,
#endif
  };
  uint8_t duties[CONFIG_PWM_NCHANNELS] =
  {
    CONFIG_EXAMPLES_PWM_DUTYPCT1,
#if CONFIG_PWM_NCHANNELS > 1
    CONFIG_EXAMPLES_PWM_DUTYPCT2,
#endif
#if CONFIG_PWM_NCHANNELS > 2
    CONFIG_EXAMPLES_PWM_DUTYPCT3,
#endif
#if CONFIG_PWM_NCHANNELS > 3
    CONFIG_EXAMPLES_PWM_DUTYPCT4,
#endif
  };
  int i;
#endif

  printf("Usage: pwm [OPTIONS]\n");
  printf("\nArguments are \"sticky\".  For example, once the PWM frequency is\n");
  printf("specified, that frequency will be re-used until it is changed.\n");
  printf("\n\"sticky\" OPTIONS include:\n");
  printf("  [-p devpath] selects the PWM device.  "
         "Default: %s Current: %s\n",
         CONFIG_EXAMPLES_PWM_DEVPATH, pwm->devpath ? pwm->devpath : "NONE");
  printf("  [-f frequency] selects the pulse frequency.  "
         "Default: %d Hz Current: %u Hz\n",
         CONFIG_EXAMPLES_PWM_FREQUENCY, pwm->freq);
#ifdef CONFIG_PWM_MULTICHAN
  printf("  [[-c channel1] [[-c channel2] ...]] selects the channel number for each channel.  ");
  printf("Default:");
  for (i = 0; i < CONFIG_PWM_MULTICHAN; i++)
    {
      printf(" %d", channels[i]);
    }
  printf("Current:");
  for (i = 0; i < CONFIG_PWM_MULTICHAN; i++)
    {
      printf(" %d", pwm->channels[i]);
    }
  printf("  [[-d duty1] [[-d duty2] ...]] selects the pulse duty as a percentage.  ");
  printf("Default:");
  for (i = 0; i < CONFIG_PWM_MULTICHAN; i++)
    {
      printf(" %d %%", duties[i]);
    }
  printf("Current:");
  for (i = 0; i < CONFIG_PWM_MULTICHAN; i++)
    {
      printf(" %d %%", pwm->duties[i]);
    }
#else
  printf("  [-d duty] selects the pulse duty as a percentage.  "
         "Default: %d %% Current: %d %%\n",
         CONFIG_EXAMPLES_PWM_DUTYPCT, pwm->duty);
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  printf("  [-n count] selects the pulse count.  "
         "Default: %d Current: %u\n",
         CONFIG_EXAMPLES_PWM_PULSECOUNT, pwm->count);
#endif
  printf("  [-t duration] is the duration of the pulse train in seconds.  "
         "Default: %d Current: %d\n",
         CONFIG_EXAMPLES_PWM_DURATION, pwm->duration);
  printf("  [-h] shows this message and exits\n");
}

/****************************************************************************
 * Name: arg_string
 ****************************************************************************/

static int arg_string(FAR char **arg, FAR char **value)
{
  FAR char *ptr = *arg;

  if (ptr[2] == '\0')
    {
      *value = arg[1];
      return 2;
    }
  else
    {
      *value = &ptr[2];
      return 1;
    }
}

/****************************************************************************
 * Name: arg_decimal
 ****************************************************************************/

static int arg_decimal(FAR char **arg, FAR long *value)
{
  FAR char *string;
  int ret;

  ret = arg_string(arg, &string);
  *value = strtol(string, NULL, 10);
  return ret;
}

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

static void parse_args(FAR struct pwm_state_s *pwm, int argc, FAR char **argv)
{
  FAR char *ptr;
  FAR char *str;
  long value;
  int index;
  int nargs;
#ifdef CONFIG_PWM_MULTICHAN
  int nchannels = 0;
  int nduties   = 0;
#endif

  for (index = 1; index < argc; )
    {
      ptr = argv[index];
      if (ptr[0] != '-')
        {
          printf("Invalid options format: %s\n", ptr);
          exit(0);
        }

      switch (ptr[1])
        {
          case 'f':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 1)
              {
                printf("Frequency out of range: %ld\n", value);
                exit(1);
              }

            pwm->freq = (uint32_t)value;
            index += nargs;
            break;

#ifdef CONFIG_PWM_MULTICHAN
          case 'c':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 1 || value > 4)
              {
                printf("Channel out of range: %ld\n", value);
                exit(1);
              }

            if (nchannels < CONFIG_PWM_NCHANNELS)
              {
                nchannels++;
              }
            else
              {
                memmove(pwm->channels, pwm->channels+1, CONFIG_PWM_NCHANNELS-1);
              }

            pwm->channels[nchannels-1] = (uint8_t)value;
            index += nargs;
            break;
#endif

          case 'd':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 1 || value > 99)
              {
                printf("Duty out of range: %ld\n", value);
                exit(1);
              }

#ifdef CONFIG_PWM_MULTICHAN
            if (nduties < CONFIG_PWM_NCHANNELS)
              {
                nduties++;
              }
            else
              {
                memmove(pwm->duties, pwm->duties+1, CONFIG_PWM_NCHANNELS-1);
              }

            pwm->duties[nduties-1] = (uint8_t)value;
#else
            pwm->duty = (uint8_t)value;
#endif
            index += nargs;
            break;

#ifdef CONFIG_PWM_PULSECOUNT
          case 'n':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 0)
              {
                printf("Count must be non-negative: %ld\n", value);
                exit(1);
              }

            pwm->count = (uint32_t)value;
            index += nargs;
            break;
#endif

          case 'p':
            nargs = arg_string(&argv[index], &str);
            pwm_devpath(pwm, str);
            index += nargs;
            break;

          case 't':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 1 || value > INT_MAX)
              {
                printf("Duration out of range: %ld\n", value);
                exit(1);
              }

            pwm->duration = (int)value;
            index += nargs;
            break;

          case 'h':
            pwm_help(pwm);
            exit(0);

          default:
            printf("Unsupported option: %s\n", ptr);
            pwm_help(pwm);
            exit(1);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
static void init_timers(void)
{
  
  
	// TIM1
	// RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // <YS> TIM1 clock Enable
  modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM1EN);

	// RCC->APB2RSTR |=  RCC_APB2RSTR_TIM1RST; // <YS> TIM1 reset
	// RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST; 
  modifyreg32(STM32_RCC_APB2RSTR, 0, RCC_APB2RSTR_TIM1RST);
  modifyreg32(STM32_RCC_APB2RSTR, RCC_APB2RSTR_TIM1RST, 0);

	// // TIM2
	// // RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // <YS> TIM1 clock Enable
  // modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB1ENR_TIM2EN);

	// // RCC->APB2RSTR |=  RCC_APB2RSTR_TIM1RST; // <YS> TIM1 reset
	// // RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST; 
  // modifyreg32(STM32_RCC_APB2RSTR, 0, RCC_APB1RSTR_TIM2RST);
  // modifyreg32(STM32_RCC_APB2RSTR, RCC_APB1RSTR_TIM2RST, 0);

  
	// Reload value
	// TIM1->ARR = 0xFFFF; // overflow 돼서 counter가 reload되는 되는 값.
  modifyreg16(STM32_TIM1_ARR, 0xffff, 0x00ff);
  // putreg16(0xFFFF, STM32_TIM2_ARR);

	// Left-aligned PWM, direction up (will be enabled later)
  // TIM1->CR1 = 0;
  modifyreg16(STM32_TIM1_CR1, 0xffff, 0);
  // putreg16(0, STM32_TIM2_CR1);

	//<YS> Output idle state 0( break 이후(MOD falling), 0 ), unbuffered updates(?)
	// TIM1->CR2 = 0;
  modifyreg16(STM32_TIM1_CR2, 0xffff, 0);

	/*
	 * OC channels
	 * TIM1 CC1, CC2, CC3 are used to control the FETs; TIM1 CC4 is not used.
	 * TIM2 CC2 is used to trigger the ADC conversion.
	 */
	// Phase A, phase B
  // <YS> preload Enable, PWM modifye 1 (1 -> 0)
	// TIM1->CCMR1 =
	// TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 |
	// TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
  modifyreg16(STM32_TIM1_CCMR1, 0xffff, ATIM_CCMR1_OC1PE | ( ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC1M_SHIFT ) |
		ATIM_CCMR1_OC2PE | ( ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC2M_SHIFT ));

	// Phase C
  // <YS> preload Enable, PWM modifye 1 (1 -> 0)
	// TIM1->CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
  modifyreg16(STM32_TIM1_CCMR2, 0xffff, ATIM_CCMR2_OC3PE | ( ATIM_CCMR_MODE_PWM1 << ATIM_CCMR2_OC3M_SHIFT ) );

  // ADC sync
	// TIM2->CCMR1 = TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;
  // putreg16( ATIM_CCMR1_OC2PE | ( ATIM_CCMR_MODE_PWM2 << ATIM_CCMR1_OC2M_SHIFT ) , STM32_TIM2_CCMR1);

	// OC polarity (no inversion, all disabled except ADC sync)
  // <YS> 모든 채널 disable
	// TIM1->CCER = 0;
  modifyreg16(STM32_TIM1_CCER, 0xffff, 0);
  // TIM2->CCER = TIM_CCER_CC2E;
  // putreg16(ATIM_CCER_CC2E, STM32_TIM1_CCER);

	/*
	 * Dead time generator setup.
	 * DTS clock divider set 0, hence fDTS = input clock.
	 * DTG bit 7 must be 0, otherwise it will change multiplier which is not supported yet.
	 * At 72 MHz one tick ~ 13.9 nsec, max 127 * 13.9 ~ 1.764 usec, which is large enough.
	 */
	// assert(isfinite(pwm_dead_time) && (pwm_dead_time > 0));
	// const float pwm_dead_time_ticks_float = pwm_dead_time / (1.f / PWM_TIMER_FREQUENCY);
	// assert(pwm_dead_time_ticks_float > 0);
	// assert(pwm_dead_time_ticks_float < (_pwm_top * 0.2f));

	// uint16_t dead_time_ticks = (uint16_t)pwm_dead_time_ticks_float;
  uint16_t dead_time_ticks = 10;
	// if (dead_time_ticks > 127) {
	// 	assert(0);
	// 	dead_time_ticks = 127;
	// }
	// printf("Motor: PWM dead time %u ticks\n", (unsigned)dead_time_ticks);

  //<YS> Main output enable / Auto Enable / dead time = dead_time_ticks * 13.9ns (at 72MHz)
	// TIM1->BDTR = TIM1_BDTR_AOE | TIM_BDTR_MOE | dead_time_ticks;
  modifyreg16(STM32_TIM1_BDTR, 0xffff,  ATIM_BDTR_AOE | ATIM_BDTR_MOE | (dead_time_ticks << ATIM_BDTR_DTG_SHIFT) );


  // TIM2->CCR2 = _adc_blanking_ticks;
  // putreg16(DUTY_RATE, STM32_TIM2_CCR2);

	// Timers are configured now but not started yet. Starting is tricky because of synchronization, see below.
  // Counter reinitialize(0) / CCPC가 1일때만 Enable update를 허락한다?
	// TIM1->EGR = TIM_EGR_UG | TIM_EGR_COMG;
  modifyreg16(STM32_TIM1_EGR, 0xffff, ATIM_EGR_UG | ATIM_EGR_COMG);
  // putreg16( ATIM_EGR_UG | ATIM_EGR_COMG, STM32_TIM2_EGR );
}

static void start_timers(void)
{
  // TIM1->CR2 |= TIM_CR2_MMS_0;                   // TIM1 is master
  modifyreg16(STM32_TIM1_CR2, 0, ATIM_CR2_MMS_ENABLE );
  // TIM2->SMCR = TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2; // TIM2 is slave
  // putreg16( ATIM_SMCR_TRIGGER, STM32_TIM2_SMCR );

	// TIM1->CR1 |= TIM_CR1_CEN;                     // Start all
  modifyreg16(STM32_TIM1_CR1, 0, ATIM_CR1_CEN);

	// Configure the synchronous reset - TIM1 master, TIM2 slave
	// TIM1->CR2 &= ~TIM_CR2_MMS;                    // Master, UG bit triggers TRGO
  modifyreg16(STM32_TIM1_CR2, ATIM_CR2_MMS_MASK, 0 );
  // putreg16( ATIM_SMCR_RESET, STM32_TIM2_SMCR );
	



  //<YS>

  // modifyreg16(STM32_TIM1_CCR1, 0x00f0, DUTY_RATE);
  // modifyreg16(STM32_TIM1_CCR2, 0x00f0, DUTY_RATE);
  // modifyreg16(STM32_TIM1_CCR3, 0x00f0, DUTY_RATE);

}

static inline void phase_reset_i(uint8_t phase)
{
	if (phase == 0)
  {
		// TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
    modifyreg16(STM32_TIM1_CCER, ATIM_CCER_CC1E | ATIM_CCER_CC1NE, 0 );
		// TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
    modifyreg16(STM32_TIM1_CCMR1, ATIM_CCMR1_OC1M_MASK, 0 );
	}
  else if (phase == 1)
  {
		// TIM1->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2NE);
    modifyreg16(STM32_TIM1_CCER, ATIM_CCER_CC2E | ATIM_CCER_CC2NE, 0 );
		// TIM1->CCMR1 &= ~TIM_CCMR1_OC2M;
    modifyreg16(STM32_TIM1_CCMR1, ATIM_CCMR1_OC2M_MASK, 0 );
	}
  else
  {
		// TIM1->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC3NE);
    modifyreg16(STM32_TIM1_CCER, ATIM_CCER_CC3E | ATIM_CCER_CC3NE, 0 );
		// TIM1->CCMR2 &= ~TIM_CCMR2_OC3M;
    modifyreg16(STM32_TIM1_CCMR2, ATIM_CCMR2_OC3M_MASK, 0 );
	}
}

static inline void phase_set_i(uint8_t phase, bool inverted)
{
  
	// The channel must be enabled in the last order when it is fully configured
	if (phase == 0)
  {
		// TIM1->CCR1 = pwm_val;
    //modifyreg16(STM32_TIM1_CCR1, 0xffff, DUTY_RATE);
		if (inverted)
    {
      //<YS> (0 -> 1)
			// TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;  // PWM modifye 2 inverted
      modifyreg16(STM32_TIM1_CCMR1, 0, ( ATIM_CCMR_MODE_PWM2 << ATIM_CCMR1_OC1M_SHIFT ) ); 
		}
    else
    {
      //<YS> (1 -> 0)
			// TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;                 // PWM modifye 1 non inverted
      modifyreg16(STM32_TIM1_CCMR1, 0, ( ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC1M_SHIFT ) ); 
		}
		// TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
    //modifyreg16(STM32_TIM1_CR2, ATIM_CR2_OIS1|ATIM_CR2_OIS1N, 0 );
    modifyreg16(STM32_TIM1_CCER, 0, (ATIM_CCER_CC1E | ATIM_CCER_CC1NE) );
    modifyreg16(STM32_TIM1_CCR1, 0xffff, DUTY_RATE);
    //modifyreg16(STM32_TIM1_EGR, 0, ATIM_EGR_UG | ATIM_EGR_COMG );
	}
  else if (phase == 1)
  {
		// TIM1->CCR2 = pwm_val;
    //modifyreg16(STM32_TIM1_CCR2, 0xffff, DUTY_RATE);
		if (inverted)
    {
      //<YS> (0 -> 1)
			// TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;
      modifyreg16(STM32_TIM1_CCMR1, 0,( ATIM_CCMR_MODE_PWM2 << ATIM_CCMR1_OC2M_SHIFT ) ); 
		}
    else
    {
      //<YS> (1 -> 0)
			// TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
      modifyreg16(STM32_TIM1_CCMR1, 0,( ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC2M_SHIFT ) ); 
		}
		// TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);
    //modifyreg16(STM32_TIM1_CR2, ATIM_CR2_OIS2|ATIM_CR2_OIS2N, 0 );
    modifyreg16(STM32_TIM1_CCER, 0, (ATIM_CCER_CC2E | ATIM_CCER_CC2NE) );
    modifyreg16(STM32_TIM1_CCR2, 0xffff, DUTY_RATE);
    //modifyreg16(STM32_TIM1_EGR, 0, ATIM_EGR_UG | ATIM_EGR_COMG );
	}
  else
  {
		// TIM1->CCR3 = pwm_val;
   //modifyreg16(STM32_TIM1_CCR3, 0xffff, DUTY_RATE);
		if (inverted)
    {
      //<YS> (0 -> 1)
			// TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0;
      modifyreg16(STM32_TIM1_CCMR2, 0, ( ATIM_CCMR_MODE_PWM2 << ATIM_CCMR2_OC3M_SHIFT ) ); 
		}
    else
    {
      //<YS> (1 -> 0)
			// TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
      modifyreg16(STM32_TIM1_CCMR2, 0,( ATIM_CCMR_MODE_PWM1 << ATIM_CCMR2_OC3M_SHIFT ) ); 
		}
		// TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);
    //modifyreg16(STM32_TIM1_CR2, ATIM_CR2_OIS3|ATIM_CR2_OIS3N, 0 );
    modifyreg16(STM32_TIM1_CCER, 0, (ATIM_CCER_CC3E | ATIM_CCER_CC3NE) );
    modifyreg16(STM32_TIM1_CCR3, 0xffff, DUTY_RATE);
    //modifyreg16(STM32_TIM1_EGR, 0, ATIM_EGR_UG | ATIM_EGR_COMG );
	}
}

/****************************************************************************
 * Name: pwm_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  struct pwm_info_s info;
  int fd;
  int ret;
#ifdef CONFIG_PWM_MULTICHAN
  int i;
  int j;
#endif

  /* Initialize the state data */

  if (!g_pwmstate.initialized)
    {
#ifdef CONFIG_PWM_MULTICHAN
      g_pwmstate.channels[0] = CONFIG_EXAMPLES_PWM_CHANNEL1;
      g_pwmstate.duties[0]   = CONFIG_EXAMPLES_PWM_DUTYPCT1;
#if CONFIG_PWM_NCHANNELS > 1
      g_pwmstate.channels[1] = CONFIG_EXAMPLES_PWM_CHANNEL2;
      g_pwmstate.duties[1]   = CONFIG_EXAMPLES_PWM_DUTYPCT2;
#endif
#if CONFIG_PWM_NCHANNELS > 2
      g_pwmstate.channels[2] = CONFIG_EXAMPLES_PWM_CHANNEL3;
      g_pwmstate.duties[2]   = CONFIG_EXAMPLES_PWM_DUTYPCT3;
#endif
#if CONFIG_PWM_NCHANNELS > 3
      g_pwmstate.channels[3] = CONFIG_EXAMPLES_PWM_CHANNEL4;
      g_pwmstate.duties[3]   = CONFIG_EXAMPLES_PWM_DUTYPCT4;
#endif
#else
      g_pwmstate.duty        = CONFIG_EXAMPLES_PWM_DUTYPCT;
#endif
      g_pwmstate.freq        = CONFIG_EXAMPLES_PWM_FREQUENCY;
      g_pwmstate.duration    = CONFIG_EXAMPLES_PWM_DURATION;
#ifdef CONFIG_PWM_PULSECOUNT
      g_pwmstate.count       = CONFIG_EXAMPLES_PWM_PULSECOUNT;
#endif
      g_pwmstate.initialized = true;
    }

  /* Parse the command line */

  parse_args(&g_pwmstate, argc, argv);

#ifdef CONFIG_PWM_MULTICHAN
  for (i = 0; i < CONFIG_PWM_MULTICHAN; i++)
    {
      for (j = i + 1; j < CONFIG_PWM_MULTICHAN; j++)
        {
          if (g_pwmstate.channels[j] == g_pwmstate.channels[i])
            {
              printf("pwm_main: channel numbers must be unique\n");
              goto errout;
            }
        }
    }
#endif

  /* Has a device been assigned? */

  if (!g_pwmstate.devpath)
    {
      /* No.. use the default device */

      pwm_devpath(&g_pwmstate, CONFIG_EXAMPLES_PWM_DEVPATH);
    }

  uint8_t phase = 1;
  
  fd = open(g_pwmstate.devpath, O_RDONLY);
  if (fd < 0)
  {
    printf("pwm_main: open %s failed: %d\n", g_pwmstate.devpath, errno);
    goto errout;
  }

  init_timers();
  
  start_timers();
  
  while(1)
  {
    // while(1)
    // {
    //   ret = ioctl(fd, PWMIOC_MYSET, 0);
    //   if (ret < 0)
    //   {
    //     printf("pwm_main: ioctl(PWMIOC_STOP) failed: %d\n", errno);
    //     goto errout_with_dev;
    //   }
    
    //   usleep(1000*100);
    // }

     phase_reset_i( ( phase   %3)     );
     phase_set_i  ( ((phase+1)%3) , 0 );
     phase_set_i  ( ((phase+2)%3) , 1 );


    printf("<YS> Counter = %d\n", getreg16(STM32_TIM1_CNT));

    phase++;
    if( phase == 3 ) { phase = 0; }

    usleep(1000*1000);
  }

errout_with_dev:
  close(fd);
errout:
  fflush(stdout);
  return ERROR;
}





//   struct pwm_info_s info;
//   int fd;
//   int ret;
// #ifdef CONFIG_PWM_MULTICHAN
//   int i;
//   int j;
// #endif

//   // <YS>
//   uint32_t delay = 0;


//   /* Initialize the state data */

//   if (!g_pwmstate.initialized)
//     {
// #ifdef CONFIG_PWM_MULTICHAN
//       g_pwmstate.channels[0] = CONFIG_EXAMPLES_PWM_CHANNEL1;
//       g_pwmstate.duties[0]   = CONFIG_EXAMPLES_PWM_DUTYPCT1;
// #if CONFIG_PWM_NCHANNELS > 1
//       g_pwmstate.channels[1] = CONFIG_EXAMPLES_PWM_CHANNEL2;
//       g_pwmstate.duties[1]   = CONFIG_EXAMPLES_PWM_DUTYPCT2;
// #endif
// #if CONFIG_PWM_NCHANNELS > 2
//       g_pwmstate.channels[2] = CONFIG_EXAMPLES_PWM_CHANNEL3;
//       g_pwmstate.duties[2]   = CONFIG_EXAMPLES_PWM_DUTYPCT3;
// #endif
// #if CONFIG_PWM_NCHANNELS > 3
//       g_pwmstate.channels[3] = CONFIG_EXAMPLES_PWM_CHANNEL4;
//       g_pwmstate.duties[3]   = CONFIG_EXAMPLES_PWM_DUTYPCT4;
// #endif
// #else
//       g_pwmstate.duty        = CONFIG_EXAMPLES_PWM_DUTYPCT;
// #endif
//       g_pwmstate.freq        = CONFIG_EXAMPLES_PWM_FREQUENCY;
//       g_pwmstate.duration    = CONFIG_EXAMPLES_PWM_DURATION;
// #ifdef CONFIG_PWM_PULSECOUNT
//       g_pwmstate.count       = CONFIG_EXAMPLES_PWM_PULSECOUNT;
// #endif
//       g_pwmstate.initialized = true;
//     }

//   /* Parse the command line */

//   parse_args(&g_pwmstate, argc, argv);

// #ifdef CONFIG_PWM_MULTICHAN
//   for (i = 0; i < CONFIG_PWM_MULTICHAN; i++)
//     {
//       for (j = i + 1; j < CONFIG_PWM_MULTICHAN; j++)
//         {
//           if (g_pwmstate.channels[j] == g_pwmstate.channels[i])
//             {
//               printf("pwm_main: channel numbers must be unique\n");
//               goto errout;
//             }
//         }
//     }
// #endif

//   /* Has a device been assigned? */

//   if (!g_pwmstate.devpath)
//     {
//       /* No.. use the default device */

//       pwm_devpath(&g_pwmstate, CONFIG_EXAMPLES_PWM_DEVPATH);
//     }

//   /* Open the PWM device for reading */

//   fd = open(g_pwmstate.devpath, O_RDONLY);2
//   if (fd < 0)
//     {
//       printf("pwm_main: open %s failed: %d\n", g_pwmstate.devpath, errno);
//       goto errout;
//     }

//   /* Configure the characteristics of the pulse train */

//   info.frequency = g_pwmstate.freq;

// #ifdef CONFIG_PWM_MULTICHAN
//   printf("pwm_main: starting output with frequency: %u",
//          info.frequency);

//   for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
//     {
//       info.channels[i].channel = g_pwmstate.channels[i];
//       info.channels[i].duty    = ((uint32_t)g_pwmstate.duties[i] << 16) / 100;
//       printf(" channel: %d duty: %08x",
//         info.channels[i].channel, info.channels[i].duty);
//     }

//   printf("\n");

// #else
//   info.duty      = ((uint32_t)g_pwmstate.duty << 16) / 100;
// #  ifdef CONFIG_PWM_PULSECOUNT
//   info.count     = g_pwmstate.count;

//   printf("pwm_main: starting output with frequency: %u duty: %08x count: %u\n",
//          info.frequency, info.duty, info.count);

// #  else
//   printf("pwm_main: starting output with frequency: %u duty: %08x\n",
//          info.frequency, info.duty);

// #  endif
// #endif

//   ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
//   if (ret < 0)
//     {
//       printf("pwm_main: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
//       goto errout_with_dev;
//     }

//   /* Then start the pulse train.  Since the driver was opened in blocking
//    * modifye, this call will block if the count value is greater than zero.
//    */

//   ret = ioctl(fd, PWMIOC_START, 0);
//   if (ret < 0)
//     {
//       printf("pwm_main: ioctl(PWMIOC_START) failed: %d\n", errno);
//       goto errout_with_dev;
//     }

//   /* It a non-zero count was not specified, then wait for the selected
//    * duration, then stop the PWM output.
//    */

// #ifdef CONFIG_PWM_PULSECOUNT
//   if (info.count == 0)
// #endif
//     {
//       /* Wait for the specified duration */

//       sleep(g_pwmstate.duration);

//       /* Then stop the pulse train */

//       printf("pwm_main: stopping output\n");

//       ret = ioctl(fd, PWMIOC_STOP, 0);
//       if (ret < 0)
//         {
//           printf("pwm_main: ioctl(PWMIOC_STOP) failed: %d\n", errno);
//           goto errout_with_dev;
//         }
//     }

//   close(fd);
//   fflush(stdout);
//   return OK;

// errout_with_dev:
//   close(fd);
// errout:
//   fflush(stdout);
//   return ERROR;
// }
