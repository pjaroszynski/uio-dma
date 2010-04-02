/**
  UIO DMA library.
  Copyright (C) 2009 Qualcomm Inc. All rights reserved.
  Written by Max Krasnyansky <maxk@qualcommm.com>

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
 
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

#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/mman.h>
#include <sys/user.h>

#include "uio-dma-ioctl.h"
#include "uio-dma.h"

static inline int is_po2(unsigned long n)
{
        return (n & (n - 1)) == 0;
}

static unsigned int ulog2(unsigned long n)
{
	unsigned int i;
	for (i = 0; n != 0; n >>= 1, ++i);
	return i - 1;
}

static inline unsigned long roundup_po2(unsigned long n)
{
	return 1UL << (ulog2(n - 1) + 1);
}

int uio_dma_open()
{
	return open("/dev/uio-dma", O_RDWR);
}

void uio_dma_close(int fd)
{
	close(fd);
}

struct uio_dma_area *uio_dma_alloc(int fd, unsigned int size,
		unsigned int cache, uint64_t dma_mask, unsigned int memnode)
{
	struct uio_dma_area *da;
	struct uio_dma_alloc_req areq;
	struct uio_dma_free_req freq;
	unsigned int chunk_size;
	int err;

	da = malloc(sizeof(*da));
	if (!da)
		return NULL;

	areq.cache = cache;
	areq.dma_mask = dma_mask;
	areq.memnode  = memnode;
	areq.mmap_offset = 0;
	areq.flags = 0;

	/* Try allocating smallest number of chunks.
 	 * We only allocate power of two sized chunks so that we could
 	 * avoid division in uio_dma_addr() function. */
	err = 0;
	for (chunk_size = roundup_po2(size); chunk_size; chunk_size >>= 1) {
		areq.chunk_size  = chunk_size;
		areq.chunk_count = (size + chunk_size - 1) / chunk_size;
		err = ioctl(fd, UIO_DMA_ALLOC, (unsigned long) &areq);
		if (!err)
			break;
	}
	if (err) {
		free(da);
		return NULL;
	}

	if (!is_po2(areq.chunk_size)) {
		errno = -EILSEQ;
		goto failed;
	}

	da->size = areq.chunk_size * areq.chunk_count;
	da->chunk_count = areq.chunk_count;
	da->chunk_size  = areq.chunk_size;
	da->mmap_offset = areq.mmap_offset;
	da->addr = mmap(NULL, da->size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, da->mmap_offset);
        if (da->addr == MAP_FAILED)
		goto failed;

	return da;

failed:
	/* We need to free the area we just requested from the kernel. */
	err = errno;
	freq.mmap_offset = da->mmap_offset;
	ioctl(fd, UIO_DMA_FREE, (unsigned long) &freq);
	free(da);
	errno = err;
	return NULL;
}

int uio_dma_free(int fd, struct uio_dma_area *da)
{
	struct uio_dma_free_req freq;
	int err;

	munmap(da->addr, da->size);

	freq.mmap_offset = da->mmap_offset;
	if (ioctl(fd, UIO_DMA_FREE, (unsigned long) &freq) < 0)
		return -1;
	free(da);
	return 0;
}

struct uio_dma_mapping *uio_dma_map(int fd, struct uio_dma_area *area,
		unsigned int devid, unsigned int dir)
{
	struct uio_dma_mapping *m;
	struct {
		struct uio_dma_map_req req;
		uint64_t dmaddr[area->chunk_count];
	} mreq;

	int err, i;

	m = malloc(sizeof(*m) + sizeof(uint64_t) * area->chunk_count);
	if (!m)
		return NULL;

	mreq.req.mmap_offset = area->mmap_offset;
	mreq.req.devid = devid;
	mreq.req.direction = dir;
	mreq.req.chunk_count = area->chunk_count;
	mreq.req.flags = 0;
	for (i=0; i < area->chunk_count; i++)
		mreq.dmaddr[i] = 0;

	err = ioctl(fd, UIO_DMA_MAP, (unsigned long) &mreq);
	if (err)
		goto failed;

	m->devid = devid;
	m->direction = dir;
	m->size = area->size;
	m->chunk_count = area->chunk_count;
	m->chunk_shift = ulog2(area->chunk_size);
	m->mmap_offset = area->mmap_offset;
	m->addr = area->addr;
	for (i=0; i < m->chunk_count; i++)
		m->dmaddr[i] = mreq.dmaddr[i];

	return m;

failed:
	err = errno;
	free(m);
	errno = err;
	return NULL;
}

int uio_dma_unmap(int fd, struct uio_dma_mapping *m)
{
	struct uio_dma_unmap_req ureq;

	ureq.mmap_offset = m->mmap_offset;
	ureq.devid = m->devid;
	ureq.direction = m->direction;
	ureq.flags = 0;
	if (ioctl(fd, UIO_DMA_UNMAP, (unsigned long) &ureq) < 0)
		return -1;
	free(m);
	return 0;
}
