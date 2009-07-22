/*
   UIO-DMA kernel back-end 
   Copyright (C) 2009 Qualcomm Inc. All rights reserved.
   Written by Max Krasnyansky <maxk@qualcommm.com>

   The UIO-DMA is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The UIO-DMA is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, write to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/sysctl.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/file.h>
#include <asm/io.h>

#include "uio-dma.h"

#ifdef DBG
#define UIO_DMA_DBG(args...) printk(KERN_DEBUG "uio-dma: " args)
#else
#define UIO_DMA_DBG(args...)
#endif

#define UIO_DMA_INFO(args...) printk(KERN_INFO "uio-dma: " args)
#define UIO_DMA_ERR(args...)  printk(KERN_ERR  "uio-dma: " args)

#define VERSION "1.0"

char uio_dma_driver_name[]    = "uio-dma";
char uio_dma_driver_string[]  = "UIO DMA kernel back-end";
char uio_dma_driver_version[] = VERSION;
char uio_dma_copyright[]      = "Copyright (c) 2009 Qualcomm Inc. Written by Max Krasnyansky <maxk@qualcomm.com>";

/* DMA area. Attached to a context. */
struct uio_dma_area {
	struct list_head  list;
	atomic_t          refcount;
	unsigned long     mmap_offset;
	unsigned long     size;
	unsigned int      chunk_size;
	unsigned int      chunk_count;
	uint8_t           cache;
	void             *addr[0];
};

/* DMA device. Attached to a context */
struct uio_dma_device {
	struct list_head  list;
	struct device    *dev;
	unsigned int      id;
	struct list_head  mappings;
};

/* DMA mapping. Attached to a device. */
struct uio_dma_mapping {
	struct list_head     list;
	struct uio_dma_area *area;
	unsigned int         direction;
	uint64_t             dmaddr[0];
};

/* UIO DMA context. Belongs to an fd. */
struct uio_dma_context {
	struct mutex      mutex;
	struct list_head  areas;
	struct list_head  devices;
	atomic_t          refcount;
};

static int uio_dma_open(struct inode *inode, struct file * file);
static void uio_dma_mapping_del(struct uio_dma_context *uc, 
		struct uio_dma_device *ud, struct uio_dma_mapping *m);

/* ---- Context ---- */

static struct uio_dma_context * uio_dma_context_get(struct uio_dma_context *uc)
{
	atomic_inc(&uc->refcount);
	return uc;
}

static void uio_dma_context_put(struct uio_dma_context *uc)
{
	if (atomic_dec_and_test(&uc->refcount)) {
		UIO_DMA_DBG("deleted context %p\n", uc);
		kfree(uc);
	}
}

/* ---- Devices ---- */

/* 
 * Add new device to the context.
 * Context must be locked.
 */
static int uio_dma_dev_add(struct uio_dma_context *uc, struct device *dev)
{
	struct uio_dma_device *ud;
	struct list_head *last;
	int id;

	/* Find first available id */
	id = 0;
	last = &uc->devices;
	list_for_each_entry(ud, &uc->devices, list) {
		if (id != ud->id)
			break;
		last = &ud->list;
		id++;
	}

	ud = kzalloc(sizeof(*ud), GFP_KERNEL);
	if (!ud)
		return -ENOMEM;

	INIT_LIST_HEAD(&ud->mappings);
	ud->id  = id;
	ud->dev = get_device(dev);
	list_add(&ud->list, last);

	UIO_DMA_DBG("added device. context %p dev #%d %s\n", uc, id, dev->bus_id);
	return id;
}

static struct uio_dma_device * uio_dma_dev_lookup(struct uio_dma_context *uc, int id)
{
	struct uio_dma_device *ud;
	list_for_each_entry(ud, &uc->devices, list) {
		if (ud->id == id)
			return ud;
	}
	return NULL;
}

static int uio_dma_dev_del(struct uio_dma_context *uc, int id)
{
	struct uio_dma_mapping *m, *n;
	struct uio_dma_device *ud = uio_dma_dev_lookup(uc, id);
	if (!ud) {
		UIO_DMA_DBG("remove bad devid. context %p dev #%d\n", uc, id);
		return -ENOENT;
	}

	/* Delete all mappings */
	list_for_each_entry_safe(m, n, &ud->mappings, list)
		uio_dma_mapping_del(uc, ud, m);

	list_del(&ud->list);
	put_device(ud->dev);

	UIO_DMA_DBG("removed device. context %p dev #%d %s\n", uc, id, ud->dev->bus_id);

	kfree(ud);
	return 0;
}

int uio_dma_register_device(int fd, struct device *dev, struct uio_dma_context **ctx)
{
	struct uio_dma_context *uc;
        struct file *file;
        int id;

	*ctx = NULL;

        file = fget(fd);
        if (!file) 
		return -EBADF;

	if (file->f_op->open != uio_dma_open) {
		fput(file);
		return -EBADF;
	}

	uc = file->private_data;
	if (!uc)
		return -EBADFD;

	mutex_lock(&uc->mutex);

	id = uio_dma_dev_add(uc, dev);

	mutex_unlock(&uc->mutex);

	if (id > -1)
		*ctx = uio_dma_context_get(uc);

        fput(file);

        return id;
}
EXPORT_SYMBOL(uio_dma_register_device);

int uio_dma_unregister_device(struct uio_dma_context *uc, int devid)
{
        int r;

	mutex_lock(&uc->mutex);

	r = uio_dma_dev_del(uc, devid);

	mutex_unlock(&uc->mutex);

	if (r > -1)
		uio_dma_context_put(uc);

        return r;
}
EXPORT_SYMBOL(uio_dma_unregister_device);


/* ---- Areas ---- */ 
static inline struct page *__last_page(void *addr, unsigned long size)
{
	return virt_to_page(addr + (PAGE_SIZE << get_order(size)) - 1);
}

static void uio_dma_area_free(struct uio_dma_context *uc, struct uio_dma_area *area)
{
	struct page *page, *last;
	int i;

	UIO_DMA_DBG("area free. context %p mmap_offset %lu\n", uc, area->mmap_offset);

	for (i=0; i < area->chunk_count; i++) {
		last = __last_page(area->addr[i], area->chunk_size);
		for (page = virt_to_page(area->addr[i]); page <= last; page++)
			ClearPageReserved(page);

		free_pages((unsigned long) area->addr[i], get_order(area->chunk_size));
	}

	kfree(area);
}

static struct uio_dma_area *uio_dma_area_alloc(struct uio_dma_context *uc,
		uint64_t dma_mask, unsigned int memnode, unsigned int cache,
		unsigned int chunk_size, unsigned int chunk_count)
{
	struct uio_dma_area *area;
	struct page *page, *last;
	int i, gfp;

	area = kzalloc(sizeof(*area) + sizeof(void *) * chunk_count, GFP_KERNEL);
	if (!area)
		return NULL;

	UIO_DMA_DBG("area alloc. context %p area %p chunk_size %u chunk_count %u\n", 
		uc, area, chunk_size, chunk_count);

	gfp = GFP_KERNEL | __GFP_NOWARN;
	if (dma_mask < DMA_64BIT_MASK) {
		if (dma_mask < DMA_32BIT_MASK)
			gfp |= GFP_DMA;
		else
			gfp |= GFP_DMA32;
	}

	atomic_set(&area->refcount, 1);

	area->chunk_size  = chunk_size;
	area->chunk_count = chunk_count;
	area->size = chunk_size * chunk_count;
	area->cache = cache;

	for (i=0; i < chunk_count; i++) {
		page = alloc_pages_node(memnode, gfp, get_order(chunk_size));
		if (!page) {
			area->chunk_count = i;
			uio_dma_area_free(uc, area);
			return NULL;
		}

		area->addr[i] = page_address(page);

		last = __last_page(area->addr[i], chunk_size);
		for (; page <= last; page++)
			SetPageReserved(page);
	}

	return area;
}

static struct uio_dma_area *uio_dma_area_lookup(struct uio_dma_context *uc, uint64_t offset)
{
	struct uio_dma_area *area;

	UIO_DMA_DBG("area lookup. context %p offset %llu\n", uc, (unsigned long long) offset);

	list_for_each_entry(area, &uc->areas, list) {
		if (area->mmap_offset == offset)
			return area;
	}

	return NULL;
}

static struct uio_dma_area * uio_dma_area_get(struct uio_dma_context *uc, struct uio_dma_area *area)
{
	atomic_inc(&area->refcount);
	return area;
}

static void uio_dma_area_put(struct uio_dma_context *uc, struct uio_dma_area *area)
{
	if (atomic_dec_and_test(&area->refcount)) {
		UIO_DMA_DBG("deleted area %p\n", area);
		uio_dma_area_free(uc, area);
	}
}

/* ---- Mappings ---- */

static void uio_dma_mapping_del(struct uio_dma_context *uc, struct uio_dma_device *ud, struct uio_dma_mapping *m)
{
	unsigned int i;

	UIO_DMA_DBG("mapping del. context %p device %s mapping %p area %p\n", uc, ud->dev->bus_id, m, m->area);

	for (i=0; i < m->area->chunk_count; i++)
 		dma_unmap_single(ud->dev, m->dmaddr[i], m->area->chunk_size, m->direction);
	list_del(&m->list);
	uio_dma_area_put(uc, m->area);
	kfree(m);
}

static int uio_dma_mapping_add(struct uio_dma_context *uc, struct uio_dma_device *ud,
					struct uio_dma_area *area, unsigned int dir,
					struct uio_dma_mapping **map)
{
	struct uio_dma_mapping *m;
	int i, n, err;

	m = kzalloc(sizeof(*m) + sizeof(dma_addr_t) * area->chunk_count, GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	UIO_DMA_DBG("maping add. context %p device %s area %p chunk_size %u chunk_count %u\n",
			uc, ud->dev->bus_id, area, area->chunk_size, area->chunk_count);

	m->area      = uio_dma_area_get(uc, area);
	m->direction = dir;
	for (i=0; i < area->chunk_count; i++) {
        	m->dmaddr[i] = dma_map_single(ud->dev, area->addr[i], area->chunk_size, dir);
        	if (!m->dmaddr[i]) {
			err = -EBADSLT;
			goto failed;
		}
		UIO_DMA_DBG("maped. context %p device %s area %p chunk #%u dmaddr %llx\n",
				uc, ud->dev->bus_id, area, i,
				(unsigned long long) m->dmaddr[i]);
	}

	list_add(&m->list, &ud->mappings);

	*map = m;
	return 0;

failed:
	for (n = 0; n < i; n++)
 		dma_unmap_single(ud->dev, m->dmaddr[n], m->area->chunk_size, dir);
	kfree(m);
	uio_dma_area_put(uc, area);
	return err;
}

static struct uio_dma_mapping *uio_dma_mapping_lookup(struct uio_dma_context *uc, 
		struct uio_dma_device *ud, uint64_t offset)
{
	struct uio_dma_mapping *m;

	UIO_DMA_DBG("mapping lookup. context %p device %s offset %llu\n", 
				uc, ud->dev->bus_id, (unsigned long long) offset);

	list_for_each_entry(m, &ud->mappings, list) {
		if (m->area->mmap_offset == offset)
			return m;
	}

	return NULL;
}

/* ---- User interface ---- */

/*
 * Make sure new area (offset & size) does not everlap with
 * existing areas and return list_head to append new area to.
 */
static struct list_head *append_to(struct uio_dma_context *uc, uint64_t *offset, unsigned int size)
{
	unsigned long start, end, astart, aend;
	struct uio_dma_area *area;
	struct list_head *last;

	start = *offset;
	end   = start + size;

	UIO_DMA_DBG("adding area. context %p start %lu end %lu\n", uc, start, end);

	last  = &uc->areas;

	list_for_each_entry(area, &uc->areas, list) {
		astart = area->mmap_offset;
		aend   = astart + area->size;

		UIO_DMA_DBG("checking area. context %p start %lu end %lu\n", uc, astart, aend);

		/* Since the list is sorted we know at this point that 
 		 * new area goes before this one. */
		if (end <= astart)
			break;

		last = &area->list;

		if ((start >= astart && start < aend) ||
				(end > astart && end <= aend)) {
			/* Found overlap. Set start to the end of the current
 			 * area and keep looking. */
			start = aend;
			end   = start + size;
			continue;
		}
	}

	*offset = start;
	return last;
}

static int uio_dma_cmd_alloc(struct uio_dma_context *uc, void __user *argp)
{
	struct uio_dma_alloc_req req;
	struct uio_dma_area *area;
	struct list_head *where;
	unsigned long size;

	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;

	if (!req.chunk_size || !req.chunk_count)
		return -EINVAL; 

	req.chunk_size = roundup(req.chunk_size, PAGE_SIZE);
	size = req.chunk_size * req.chunk_count;

	UIO_DMA_DBG("alloc req enter. context %p offset %llu chunk_size %u chunk_count %u (total %lu)\n",
		uc, (unsigned long long) req.mmap_offset, req.chunk_size, req.chunk_count, size);

	where = append_to(uc, &req.mmap_offset, size);
	if (!where)
		return -EBUSY;

	area = uio_dma_area_alloc(uc, req.dma_mask, req.memnode, req.cache, req.chunk_size, req.chunk_count);
	if (!area)
		return -ENOMEM;

	/* Add to the context */
	area->mmap_offset = req.mmap_offset;
	list_add(&area->list, where);

	if (copy_to_user(argp, &req, sizeof(req))) {
		list_del(&area->list);
		uio_dma_area_put(uc, area);
		return EFAULT;
	}

	UIO_DMA_DBG("alloc req exit. context %p offset %llu size %lu mask %llx node %u\n", uc,
		(unsigned long long) area->mmap_offset, area->size,
		(unsigned long long) req.dma_mask, req.memnode);
	return 0;
}

static int uio_dma_cmd_free(struct uio_dma_context *uc, void __user *argp)
{
	struct uio_dma_free_req req;
	struct uio_dma_area *area;

	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;

	UIO_DMA_DBG("free req. context %p offset %llu\n", uc, req.mmap_offset);

	area = uio_dma_area_lookup(uc, req.mmap_offset);
	if (!area)
		return -ENOENT;

	list_del(&area->list);
	uio_dma_area_put(uc, area);

	return 0;
}

static int uio_dma_cmd_map(struct uio_dma_context *uc, void __user *argp)
{
	struct uio_dma_map_req req;
	struct uio_dma_mapping *m;
	struct uio_dma_device *ud;
	struct uio_dma_area *area;
	int err;

	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;

	UIO_DMA_DBG("map req. context %p offset %llu devid %u\n", uc, req.mmap_offset, req.devid);

	ud = uio_dma_dev_lookup(uc, req.devid);
	if (!ud)
		return -ENODEV;

	m = uio_dma_mapping_lookup(uc, ud, req.mmap_offset);
	if (m)
		return -EALREADY;

	area = uio_dma_area_lookup(uc, req.mmap_offset);
	if (!area)
		return -ENOENT;

	if (req.chunk_count < area->chunk_count)
		return -EINVAL;

	err = uio_dma_mapping_add(uc, ud, area, req.direction, &m);
	if (err)
		return err;

	req.chunk_count = area->chunk_count;
	req.chunk_size  = area->chunk_size;
		
	if (copy_to_user(argp, &req, sizeof(req)))
		goto fault;

	/* Copy dma addresses */
	if (copy_to_user(argp + sizeof(req), m->dmaddr, sizeof(uint64_t) * area->chunk_count))
		goto fault;

	return 0;

fault:
	uio_dma_mapping_del(uc, ud, m);
	return EFAULT;
}

static int uio_dma_cmd_unmap(struct uio_dma_context *uc, void __user *argp)
{
	struct uio_dma_unmap_req req;
	struct uio_dma_device *ud;
	struct uio_dma_mapping *m;

	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;

	UIO_DMA_DBG("map req. context %p offset %llu devid %u\n", uc, req.mmap_offset, req.devid);

	ud = uio_dma_dev_lookup(uc, req.devid);
	if (!ud)
		return -ENODEV;

	m = uio_dma_mapping_lookup(uc, ud, req.mmap_offset);
	if (!m)
		return -ENOENT;

	uio_dma_mapping_del(uc, ud, m);
	return 0;
}

static long uio_dma_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct uio_dma_context *uc = file->private_data;
	void __user * argp = (void __user *) arg;
	int err;

	UIO_DMA_DBG("ioctl. context %p cmd %d arg %lu\n", uc, cmd, arg);
	if (!uc)
		return -EBADFD;

	mutex_lock(&uc->mutex);

	switch (cmd) {
	case UIO_DMA_ALLOC:
		err = uio_dma_cmd_alloc(uc, argp);
		break;

	case UIO_DMA_MAP:
		err = uio_dma_cmd_map(uc, argp);
		break;

	case UIO_DMA_UNMAP:
		err = uio_dma_cmd_unmap(uc, argp);
		break;

	case UIO_DMA_FREE:
		err = uio_dma_cmd_free(uc, argp);
		break;

	default:
		err = -EINVAL;
		break;
	};

	mutex_unlock(&uc->mutex);

	return err;
}

static int uio_dma_close(struct inode *inode, struct file *file)
{
	struct uio_dma_context *uc = file->private_data;
	struct uio_dma_area *area, *n;
	if (!uc)
		return 0;

	UIO_DMA_DBG("closed context %p\n", uc);

	list_for_each_entry_safe(area, n, &uc->areas, list)
		uio_dma_area_put(uc, area);

	file->private_data = NULL;
	uio_dma_context_put(uc);
	return 0;
}

static int uio_dma_open(struct inode *inode, struct file * file)
{
	struct uio_dma_context *uc;
	
	/* Allocate new context */
	uc = kmalloc(sizeof(*uc), GFP_KERNEL);
	if (!uc)
		return -ENOMEM;
	memset(uc, 0, sizeof(*uc));

	mutex_init(&uc->mutex);
	INIT_LIST_HEAD(&uc->areas);
	INIT_LIST_HEAD(&uc->devices);
	atomic_set(&uc->refcount, 1);

	file->private_data = uc;

	UIO_DMA_DBG("created context %p\n", uc);
	return 0;
}

static unsigned int uio_dma_poll(struct file *file, poll_table *wait)
{  
	return -ENOSYS;
}

static ssize_t uio_dma_read(struct file * file, char __user * buf, 
			    size_t count, loff_t *pos)
{
	return -ENOSYS;
}

static ssize_t uio_dma_write(struct file * file, const char __user * buf,
			     size_t count, loff_t *pos)
{
	return -ENOSYS;
}

static void uio_dma_vm_open(struct vm_area_struct *vma)
{
}

static void uio_dma_vm_close(struct vm_area_struct *vma)
{
}

static int uio_dma_vm_fault(struct vm_area_struct *area, 
			struct vm_fault *fdata)
{
	return VM_FAULT_SIGBUS;
}

static struct vm_operations_struct uio_dma_mmap_ops = {
	.open   = uio_dma_vm_open,
	.close  = uio_dma_vm_close,
	.fault  = uio_dma_vm_fault
};

static int uio_dma_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct uio_dma_context *uc = file->private_data;
	struct uio_dma_area *area;

	unsigned long start  = vma->vm_start;
	unsigned long size   = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff * PAGE_SIZE;
	unsigned long pfn;
	int i;

	if (!uc)
		return -ENODEV;

	UIO_DMA_DBG("mmap. context %p start %lu size %lu offset %lu\n", uc, start, size, offset);

	// Find an area that matches the offset and mmap it.
	area = uio_dma_area_lookup(uc, offset);	
	if (!area)
		return -ENOENT;

	// We do not do partial mappings, sorry
	if (area->size != size)
		return -EOVERFLOW;

	switch (area->cache) {
	case UIO_DMA_CACHE_DISABLE:
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		break;

	case UIO_DMA_CACHE_WRITECOMBINE:
		#ifdef pgprot_writecombine
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
		#endif
		break;

	default:
		/* Leave as is */
		break;
	}

	for (i=0; i < area->chunk_count; i++) {
		pfn = page_to_pfn(virt_to_page(area->addr[i]));
		if (remap_pfn_range(vma, start, pfn, area->chunk_size, vma->vm_page_prot))
			return -EIO;

		start += area->chunk_size;
	}

	vma->vm_ops = &uio_dma_mmap_ops;
	return 0;
}

static struct file_operations uio_dma_fops = {
	.owner	 = THIS_MODULE,
	.llseek  = no_llseek,
	.read	 = uio_dma_read,
	.write	 = uio_dma_write,
	.poll	 = uio_dma_poll,
	.open	 = uio_dma_open,
	.release = uio_dma_close,
	.mmap	 = uio_dma_mmap,
	.unlocked_ioctl = uio_dma_ioctl,
};

static struct miscdevice uio_dma_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "uio-dma",
	.fops = &uio_dma_fops,
};

static int __init uio_dma_init_module(void)
{
	int err;

	printk(KERN_INFO "%s - version %s\n", uio_dma_driver_string, uio_dma_driver_version);
	printk(KERN_INFO "%s\n", uio_dma_copyright);

	err = misc_register(&uio_dma_miscdev);
	if (err) {
		UIO_DMA_ERR("failed to register misc device\n");
		return err;
	}

	return err;
}

static void __exit uio_dma_exit_module(void)
{
	misc_deregister(&uio_dma_miscdev);
}

module_init(uio_dma_init_module);
module_exit(uio_dma_exit_module);

/* ---- */

MODULE_AUTHOR("Max Krasnyansky <maxk@qualcomm.com>");
MODULE_DESCRIPTION("uio-dma kernel backend");
MODULE_LICENSE("GPL");
MODULE_VERSION(VERSION);
