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

#ifndef UIO_DMA_IOCTL_H
#define UIO_DMA_IOCTL_H

#include <stdint.h>

/* ioctl defines */

#define UIO_DMA_ALLOC _IOW('U', 200, int)
struct uio_dma_alloc_req {
	uint64_t dma_mask;
	uint16_t memnode;
	uint16_t cache;
	uint32_t flags;
	uint32_t chunk_size;
	uint32_t chunk_count;
	uint64_t mmap_offset;
};

#define UIO_DMA_FREE  _IOW('U', 201, int)
struct uio_dma_free_req {
	uint64_t mmap_offset;
};

#define UIO_DMA_MAP   _IOW('U', 202, int)
struct uio_dma_map_req {
	uint64_t mmap_offset;
	uint32_t flags;
	uint32_t devid;
	uint8_t  direction;
	uint32_t chunk_count;
	uint32_t chunk_size;
	uint64_t dmaddr[0];
};

#define UIO_DMA_UNMAP _IOW('U', 203, int)
struct uio_dma_unmap_req {
	uint64_t mmap_offset;
	uint32_t devid;
	uint32_t flags;
	uint8_t  direction;
};

#endif /* UIO_DMA_IOCTL_H */
