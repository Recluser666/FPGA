#ifndef ADAS_DMA_UAPI_H
#define ADAS_DMA_UAPI_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define ADAS_DMA_MAX_BUFS 8

struct adas_dma_info {
	__u32 nbufs;
	__u32 bufsize;
	__u32 ff_pages;
	__u32 reserved;
	__u64 dma_addr[ADAS_DMA_MAX_BUFS];
};

struct adas_dma_buf_arg {
	__u32 idx;
	__u32 reserved;
};

#define ADAS_DMA_IOC_MAGIC	'A'

#define ADAS_DMA_GET_INFO	_IOR(ADAS_DMA_IOC_MAGIC, 0x01, struct adas_dma_info)

#define ADAS_DMA_SYNC_CPU	_IOW(ADAS_DMA_IOC_MAGIC, 0x02, struct adas_dma_buf_arg)

#define ADAS_DMA_SYNC_DEV	_IOW(ADAS_DMA_IOC_MAGIC, 0x03, struct adas_dma_buf_arg)

#endif
