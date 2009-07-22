/**
  UIO DMA library.
  Copyright (C) 2009 Qualcomm Inc. All rights reserved.
  Written by Max Krasnyansky <maxk@qualcommm.com>
 
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:
 
  1. Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
  
  3. Neither the name of the QUALCOMM Incorporated nor the
  names of any contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY QUALCOMM AND ANY OTHER CONTRIBUTORS 
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL QUALCOMM 
  AND CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED 
  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT(INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#define _GNU_SOURCE

#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <signal.h>
#include <errno.h>
#include <getopt.h>
#include <sched.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/user.h>

#include <net/ethernet.h>

#include "uio-dma/uio-dma.h"

static int areasize  = 10 * 1024 * 1024;
static int areacount = 10;

static int terminated;
static void sig_int(int sig)
{
	terminated = 1;
}

static void do_test()
{
	struct uio_dma_area   *area[areacount];
	int fd, i, err;

	fd = uio_dma_open();
	if (fd < 0) {
		perror("UIO DMA open failed");
		exit(1);
	}

	// alloc/free single area
	for (i=0; i<areacount; i++) {
		area[i] = uio_dma_alloc(fd, areasize, 0, UIO_DMA_MASK(64), 0);
		if (!area[i]) {
			perror("UIO alloc failed");
			exit(1);
		}

		printf("DMA area%d: addr %p size %u offset %llu\n", i,
			area[i]->addr, area[i]->size, area[i]->mmap_offset);
		
		memset(area[i]->addr, 0, area[i]->size);

		if (uio_dma_free(fd, area[i]) < 0) {
			perror("UIO free failed");
			exit(1);
		}
	}
	puts("");

	// alloc/free multiple areas
	for (i=0; i<areacount; i++) {
		area[i] = uio_dma_alloc(fd, areasize, 0, UIO_DMA_MASK(64), 0);
		if (!area[i]) {
			perror("UIO alloc failed");
			exit(1);
		}

		printf("DMA area%d: addr %p size %u offset %llu\n", i,
			area[i]->addr, area[i]->size, area[i]->mmap_offset);
		
		memset(area[i]->addr, 0, area[i]->size);
	}
	for (i=0; i<areacount; i++) {
		if (uio_dma_free(fd, area[i]) < 0) {
			perror("UIO free failed");
			exit(1);
		}
	}
	uio_dma_close(fd);
}

static const char *main_help =
	"UIO DMA API tester\n"
	"Usage:\n"
	"\tuio-dma-api-test [options]\n"
	"\tOptions:\n"
	"\t\t--size | -s <N>     Size of the DMA area to allocate\n"
	"\t\t--count | -c <N>    Number of areas to allocate\n"
	"\t\t--help | -h         Display this help screen\n";

static struct option main_long_opts[] = {
	{"help", 0, 0, 'h'},
	{"size", 1, 0, 's'},
	{"count", 1, 0, 'c'},
	{0, 0, 0, 0}
};

static char main_short_opts[] = "hs:c:";

int main(int argc, char *argv[])
{
	int  opt;

	// Parse command line options
	while ((opt = getopt_long(argc, argv, main_short_opts, main_long_opts, NULL)) != -1) {
		switch (opt) {
		case 's':
			areasize  = atoi(optarg);
			break;

		case 'c':
			areacount = atoi(optarg);
			break;

		case 'h':
		default:
			printf(main_help);
			exit(1);
		}
	}

	argc -= optind;
	argv += optind;
	optind = 0;

	{
		struct sigaction sa = { { 0 } };

		sa.sa_handler = sig_int;
		sigaction(SIGINT, &sa, NULL);

		sa.sa_handler = sig_int;
		sigaction(SIGTERM, &sa, NULL);
	}

	do_test();

	return 0;
}
