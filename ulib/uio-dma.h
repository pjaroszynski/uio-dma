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

/**
 * @file uio-dma.h
 * UIO-DMA - DMA support for UIO
 */

#ifndef UIO_DMA_H
#define UIO_DMA_H

#include <stdint.h>
#include <sys/types.h>
#include <sys/user.h>
#include <sys/uio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Macro for construction DMA masks
 * @param n number of non-zero bits 
 */
#define UIO_DMA_MASK(n) (((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))

/* Caching modes */
enum {
	UIO_DMA_CACHE_DEFAULT = 0,
	UIO_DMA_CACHE_DISABLE,
	UIO_DMA_CACHE_WRITECOMBINE
};

/* DMA mapping direction */
enum {
	UIO_DMA_BIDIRECTIONAL = 0,
	UIO_DMA_TODEVICE,
	UIO_DMA_FROMDEVICE
};

/**
 * UIO DMA area.
 */
struct uio_dma_area {
	uint8_t      *addr;
	unsigned long mmap_offset;
	unsigned long size;
	unsigned long chunk_size;
	unsigned int  chunk_count;
};

/**
 * UIO DMA mapping.
 */
struct uio_dma_mapping {
	uint8_t      *addr;
	unsigned long mmap_offset;
	unsigned long size;
	unsigned int  chunk_count;
	unsigned int  chunk_shift;
	uint32_t      devid;
	uint8_t       direction;
	uint64_t      dmaddr[0];
};

/**
 * Open and initialize UIO DMA file descriptor. 
 * @return file descriptor
 */
int uio_dma_open();

/**
 * Close UIO DMA file descriptor.
 * @param fd file descriptor
 */
void uio_dma_close(int fd);

/**
 * Allocate new DMA area.
 * @param fd UIO DMA file descriptor
 * @param size of the area
 * @param cache caching mode
 * @param dma_mask mask of the DMA address range 
 * @param memnode memory node to allocate the memory area on
 */
struct uio_dma_area *uio_dma_alloc(int fd, unsigned int size,
		unsigned int cache, uint64_t dma_mask, unsigned int memnode);
/**
 * Free DMA area.
 * @param fd UIO DMA file descriptor
 * @param pointer to @see uio_dma_area
 */
int uio_dma_free(int fd, struct uio_dma_area *a);

/**
 * Map DMA area to a device.
 * @param fd UIO DMA file descriptor
 * @param pointer to @see uio_dma_area
 * @param devid uio dma device id
 * @param dir direction
 * @return pointer to new @see uio_dma_mapping
 */
struct uio_dma_mapping * uio_dma_map(int fd, struct uio_dma_area *area, uint32_t devid, unsigned int dir);

/**
 * Unmap DMA area from a device.
 * @param fd UIO DMA file descriptor
 * @param pointer to @see uio_dma_mapping
 * @return 0 on success, <0 otherwise (sets errno)
 */
int uio_dma_unmap(int fd, struct uio_dma_mapping *m);

/**
 * Map user-space region to the DMA address. Region must belong to the specified 
 * dma mapping. Use this function only when size of the memory region is guaranteed
 * to be smaller than the chunk_size.
 * @param m pointer to @see uio_dma_mapping 
 * @param addr pointer to the user-space memory region
 * @param len length of the memory region
 * @return dma_addr or 0 if address could not be mapped
 */
static inline uint64_t uio_dma_addr(struct uio_dma_mapping *m, uint8_t *addr, unsigned int len)
{
	unsigned long chunk, offset, chunk_size;

	chunk_size = (1 << m->chunk_shift);
	if (len > chunk_size || addr < m->addr || (addr + len) > (m->addr + m->size))
		return 0;

	offset = addr - m->addr;
	chunk  = offset >> m->chunk_shift;
	offset = offset & (chunk_size - 1);

	return m->dmaddr[chunk] + offset;
}

#ifdef __cplusplus
}
#endif

#endif /* UIO_DMA_H */
