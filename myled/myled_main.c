/****************************************************************************
 * examples/apa102/apa102_main.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Copyright (c) 2015-2017 Pololu Corporation.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#include <arch/board/board.h>
#include <sched.h>

#include <nuttx/leds/ws2812.h>
#include "s32k1xx_lpspi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_EXAMPLES_WS2812_DEVNAME "/dev/leddrv0"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#define NUM_LEDS  8
#define US_DELAY  1000*20

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * apa102_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  struct spi_dev_s *myspi;
  FAR const char dpath[] = CONFIG_EXAMPLES_WS2812_DEVNAME;
  int fd;
  int ret;
  uint32_t mybuf[8] = {0x00050000, 0x00000500, 0x00000005, 0x00000505, 0x00050500, 0x00050005, 0x00000000, 0x00050505};
  uint16_t numberl = NUM_LEDS;
  uint32_t mybuf2[8];

  myspi = s32k1xx_lpspibus_initialize(1);

  ws2812_leds_register(dpath, myspi,numberl);
  
  fd = open(CONFIG_EXAMPLES_WS2812_DEVNAME, O_RDWR);

  if (fd < 0)
  {
    fprintf(stderr, "ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_WS2812_DEVNAME, errno); 
    return -1;
  }
  
  while(1)
  {
     for (int i = 0; i < 8 ; ++i)
     {
       mybuf2[i] = mybuf[i];
     }
     for (int i = 0; i < 7 ; ++i)
     {
       mybuf[i+1] = mybuf2[i];
     }
     mybuf[0] = mybuf2[7];

    ret = lseek(fd, 0, SEEK_SET);
    ret = write(fd, mybuf, 4*NUM_LEDS);

    if (ret < 0)
    {
      _err("ERROR: write LED Strip failed: %d\n", errno);
      
    }
    usleep(US_DELAY);

  }

  close(fd);

  return 0;
}
