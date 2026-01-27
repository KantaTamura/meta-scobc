// SPDX-License-Identifier: GPL-2.0-only
/*
 * sc_mipi_csi.c
 *
 * Copyright (c) 2025 Space Cubics, Inc.
 *
 * This driver provides support for a custom FPGA-based MIPI CSI-2 receiver
 * The receiver captures camera frames via DMA and exposes them through the V4L2 framework.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/ktime.h>
#include <linux/bitops.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/fwnode.h>
#include <linux/interrupt.h>
#include <linux/videodev2.h>
#include <linux/dma-direct.h>
#include <linux/platform_device.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-subdev.h>
#include <media/media-entity.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

MODULE_DESCRIPTION(
	"V4L2 driver for MIPI CSI-2 camera receiver on 4th.ai Autonomous Driving Board");
MODULE_AUTHOR("KantaTamura <kanta@spacecubics.com>");
MODULE_LICENSE("GPL v2");

/* register ofsets */
#define REG_MIPI_INT_STATUS 0x030
#define REG_MIPI_INT_ENABLE 0x034

#define REG_DMA_CTRL 0x200
#define REG_DMA_STATUS 0x204
#define REG_DMA_FB0_ADDR 0x210
#define REG_DMA_FB1_ADDR 0x214
#define REG_DMA_FB2_ADDR 0x218
#define REG_DMA_FB3_ADDR 0x21C

/* interrupt bits */
#define INT_DMA_DONE BIT(0)

/* DMA control bits */
#define DMA_CTRL_MAX_BURST_MASK GENMASK(23, 16)
#define DMA_CTRL_NUM_FB_MASK GENMASK(9, 8)
#define DMA_CTRL_ENABLE BIT(0)
#define DMA_CTRL_MAX_BURST(v) FIELD_PREP(DMA_CTRL_MAX_BURST_MASK, (v))
#define DMA_CTRL_NUM_FB(v) FIELD_PREP(DMA_CTRL_NUM_FB_MASK, (v))

/* DMA status bits */
#define DMA_STATUS_ACTIVE_FB_MASK GENMASK(1, 0)

#define MAX_FRAME_BUFFER 4

struct rover_mipi_csi_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct rover_mipi_csi {
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct vb2_queue queue;
	struct list_head buf_list;
	struct rover_mipi_csi_buffer *fb_slot[MAX_FRAME_BUFFER];
	bool dma_running;
	u8 enable_fb_num;
	u8 fb_count;
	bool streaming;
	u32 sequence;
	int irq;

	struct mutex lock;
	spinlock_t qlock;

	void __iomem *base;

	struct v4l2_async_notifier notifier;
	struct v4l2_subdev *subdev;
	struct media_pad pad;
	struct media_device mdev;
	unsigned int subdev_src_pad;

	struct v4l2_format cur_fmt;
};

static inline u32 __rover_read(struct rover_mipi_csi *ctx, u32 reg)
{
	return ioread32(ctx->base + reg);
}

static inline void __rover_write(struct rover_mipi_csi *ctx, u32 reg, u32 val)
{
	iowrite32(val, ctx->base + reg);
}

static inline void __rover_update_bits(struct rover_mipi_csi *ctx, u32 reg,
				       u32 mask, u32 val)
{
	u32 tmp = __rover_read(ctx, reg);
	tmp &= ~mask;
	tmp |= val & mask;
	__rover_write(ctx, reg, tmp);
}

static inline void __rover_set_bits(struct rover_mipi_csi *ctx, u32 reg,
				    u32 mask)
{
	__rover_update_bits(ctx, reg, mask, mask);
}

static inline void __rover_clear_bits(struct rover_mipi_csi *ctx, u32 reg,
				      u32 mask)
{
	__rover_update_bits(ctx, reg, mask, 0);
}

static inline void __rover_w1c(struct rover_mipi_csi *ctx, u32 reg, u32 mask)
{
	__rover_write(ctx, reg, mask);
}

static inline void rover_irq_enable(struct rover_mipi_csi *ctx)
{
	__rover_set_bits(ctx, REG_MIPI_INT_ENABLE, INT_DMA_DONE);

	/* read back to ensure the write is posted */
	__rover_read(ctx, REG_MIPI_INT_ENABLE);

	dev_dbg(ctx->dev, "register: IRQ enabled\n");
}

static inline void rover_irq_disable(struct rover_mipi_csi *ctx)
{
	__rover_clear_bits(ctx, REG_MIPI_INT_ENABLE, INT_DMA_DONE);

	/* read back to ensure the write is posted */
	__rover_read(ctx, REG_MIPI_INT_ENABLE);

	dev_dbg(ctx->dev, "register: IRQ disabled\n");
}

static inline void rover_irq_clear_dma_done(struct rover_mipi_csi *ctx)
{
	__rover_w1c(ctx, REG_MIPI_INT_STATUS, INT_DMA_DONE);
	dev_dbg(ctx->dev, "register: DMA_DONE cleared\n");
}

static inline void rover_dma_config(struct rover_mipi_csi *ctx, u8 max_burst,
				    u8 num_fb, bool enable)
{
	u32 val = DMA_CTRL_MAX_BURST(max_burst) | DMA_CTRL_NUM_FB(num_fb) |
		  (enable ? DMA_CTRL_ENABLE : 0);
	__rover_write(ctx, REG_DMA_CTRL, val);
	wmb();

	/* read back to ensure the write is posted */
	__rover_read(ctx, REG_DMA_CTRL);

	dev_dbg(ctx->dev,
		"register: DMA configured (max_burst=%u, num_fb=%u, %s)\n",
		max_burst, num_fb, enable ? "enabled" : "disabled");
}

static inline void rover_dma_enable(struct rover_mipi_csi *ctx)
{
	__rover_set_bits(ctx, REG_DMA_CTRL, DMA_CTRL_ENABLE);
	wmb();

	/* read back to ensure the write is posted */
	__rover_read(ctx, REG_DMA_CTRL);

	dev_dbg(ctx->dev, "register: DMA enabled\n");
}

static inline void rover_dma_disable(struct rover_mipi_csi *ctx)
{
	__rover_clear_bits(ctx, REG_DMA_CTRL, DMA_CTRL_ENABLE);
	wmb();

	/* read back to ensure the write is posted */
	__rover_read(ctx, REG_DMA_CTRL);

	dev_dbg(ctx->dev, "register: DMA disabled\n");
}

static inline u8 rover_dma_active_fb(struct rover_mipi_csi *ctx)
{
	return __rover_read(ctx, REG_DMA_STATUS) & DMA_STATUS_ACTIVE_FB_MASK;
}

static inline void rover_write_fb_addr(struct rover_mipi_csi *ctx, u32 fb_idx,
				       phys_addr_t addr)
{
	u32 off;

	switch (fb_idx) {
	case 0:
		off = REG_DMA_FB0_ADDR;
		break;
	case 1:
		off = REG_DMA_FB1_ADDR;
		break;
	case 2:
		off = REG_DMA_FB2_ADDR;
		break;
	case 3:
		off = REG_DMA_FB3_ADDR;
		break;
	default:
		dev_err(ctx->dev, "Invalid framebuffer index: %d\n", fb_idx);
		return;
	}

	__rover_write(ctx, off, (u32)(addr));
	wmb();

	/* read back to ensure the write is posted */
	__rover_read(ctx, off);

	dev_dbg(ctx->dev, "register: FB%d address set to 0x%08x\n", fb_idx,
		(u32)addr);
}

#if defined(SC_ROVER_MIPI_CSI_DEBUG)
static void __rover_debug_dump_context(struct rover_mipi_csi *ctx)
{
	unsigned long flags;

	dev_info(ctx->dev, "context summary:\n");
	dev_info(
		ctx->dev,
		"\tstreaming=%d dma_running=%d enable_fb_num=%u fb_count=%u sequence=%d\n",
		ctx->streaming, ctx->dma_running, ctx->enable_fb_num,
		ctx->fb_count, ctx->sequence);

	spin_lock_irqsave(&ctx->qlock, flags);
	for (int i = 0; i < MAX_FRAME_BUFFER; i++) {
		struct rover_mipi_csi_buffer *buf = ctx->fb_slot[i];
		dma_addr_t dma;
		phys_addr_t pa;

		if (buf) {
			dma = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf,
							    0);
			pa = dma_to_phys(ctx->dev, dma);
			if (pa == DMA_MAPPING_ERROR) {
				dev_err(ctx->dev,
					"dma_to_phys failed for dma=%p\n",
					&dma);
			}
			dev_info(
				ctx->dev,
				"\tfb_slot[%d] = buf=%p dma=%pad phy=%pa idx=%u\n",
				i, buf, &dma, &pa, buf->vb.vb2_buf.index);
		} else {
			dev_info(ctx->dev, "\tfb_slot[%d] = NULL\n", i);
		}
	}
	if (!list_empty(&ctx->buf_list)) {
		struct rover_mipi_csi_buffer *p;
		list_for_each_entry(p, &ctx->buf_list, list) {
			dev_info(ctx->dev, "\tbuf_list -> buf=%p idx=%u\n", p,
				 p->vb.vb2_buf.index);
		}
	} else {
		dev_info(ctx->dev, "\tbuf_list is empty\n");
	}
	spin_unlock_irqrestore(&ctx->qlock, flags);
}

static void __rover_debug_dump_hardware(struct rover_mipi_csi *ctx)
{
	unsigned long flags;
	u32 reg_irq_enable, reg_dma_control, reg_fb_status;
	u32 reg_fb0_addr, reg_fb1_addr, reg_fb2_addr, reg_fb3_addr;

	spin_lock_irqsave(&ctx->qlock, flags);
	reg_irq_enable = __rover_read(ctx, REG_MIPI_INT_ENABLE);
	reg_dma_control = __rover_read(ctx, REG_DMA_CTRL);
	reg_fb_status = __rover_read(ctx, REG_DMA_STATUS);

	reg_fb0_addr = __rover_read(ctx, REG_DMA_FB0_ADDR);
	reg_fb1_addr = __rover_read(ctx, REG_DMA_FB1_ADDR);
	reg_fb2_addr = __rover_read(ctx, REG_DMA_FB2_ADDR);
	reg_fb3_addr = __rover_read(ctx, REG_DMA_FB3_ADDR);
	spin_unlock_irqrestore(&ctx->qlock, flags);

	u32 irq_en = reg_irq_enable & BIT(0);
	u32 dma_en = reg_dma_control & BIT(0);
	u32 num_fb = FIELD_GET(DMA_CTRL_NUM_FB_MASK, (reg_dma_control));
	u32 max_burst = FIELD_GET(DMA_CTRL_MAX_BURST_MASK, (reg_dma_control));
	u32 fb_status = reg_fb_status & DMA_STATUS_ACTIVE_FB_MASK;

	dev_info(ctx->dev, "hardware summary:\n");
	dev_info(ctx->dev,
		 "\tirq_en=%d dma_en=%d num_fb=%d max_burst=%d fb_status=%d\n",
		 irq_en, dma_en, num_fb, max_burst, fb_status);
	dev_info(ctx->dev, "\tfb0=0x%08x fb1=0x%08x fb2=0x%08x fb3=0x%08x\n",
		 reg_fb0_addr, reg_fb1_addr, reg_fb2_addr, reg_fb3_addr);
}

#define rover_debug_dump_full(ctx, tag)                                \
	do {                                                           \
		dev_info(ctx->dev, "========== %s ==========\n", tag); \
		__rover_debug_dump_context(ctx);                       \
		__rover_debug_dump_hardware(ctx);                      \
		dev_info(ctx->dev, "========================\n");      \
	} while (0)
#else
#define rover_debug_dump_full(ctx, tag) \
	do {                            \
	} while (0)
#endif /* defined(SC_ROVER_MIPI_CSI_DEBUG) */

static int rover_prime_ring(struct rover_mipi_csi *ctx)
{
	unsigned long flags;
	unsigned int filled = 0;

	spin_lock_irqsave(&ctx->qlock, flags);
	if (ctx->fb_count == 0) {
		for (int i = 0; i < MAX_FRAME_BUFFER; i++) {
			ctx->fb_slot[i] = NULL;
		}
	} else if (!ctx->fb_slot[0]) {
		// If a previous slot remains, it will be packed to the front.
		// NOTE: `active_slot` is initialized to 0 after DMA off due to this behavior.
		int dst = 0;
		for (int i = 0; i < ctx->enable_fb_num; i++) {
			if (!ctx->fb_slot[i]) {
				continue;
			}
			if (i != dst) {
				ctx->fb_slot[dst] = ctx->fb_slot[i];
				ctx->fb_slot[i] = NULL;
			}
			dst++;
		}
	}
	spin_unlock_irqrestore(&ctx->qlock, flags);

	// assign buffer queued in bf_list to fb_slot
	// NOTE: irq activation resets active_slot to 0
	for (int slot = 0; slot < ctx->enable_fb_num; slot++) {
		struct rover_mipi_csi_buffer *buf;
		dma_addr_t dma;
		phys_addr_t pa;

		spin_lock_irqsave(&ctx->qlock, flags);
		if (ctx->fb_slot[slot]) {
			spin_unlock_irqrestore(&ctx->qlock, flags);
			filled++;
			continue;
		} else if (list_empty(&ctx->buf_list)) {
			spin_unlock_irqrestore(&ctx->qlock, flags);
			break;
		}
		buf = list_first_entry(&ctx->buf_list,
				       struct rover_mipi_csi_buffer, list);
		list_del(&buf->list);
		spin_unlock_irqrestore(&ctx->qlock, flags);

		dma = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
		pa = dma_to_phys(ctx->dev, dma);
		if (pa == DMA_MAPPING_ERROR) {
			dev_err(ctx->dev, "dma_to_phys failed for dma=%p\n",
				&dma);
			return -EFAULT;
		}

		rover_write_fb_addr(ctx, slot, pa);

		spin_lock_irqsave(&ctx->qlock, flags);
		ctx->fb_slot[slot] = buf;
		spin_unlock_irqrestore(&ctx->qlock, flags);

		filled++;
	}

	spin_lock_irqsave(&ctx->qlock, flags);
	ctx->fb_count = filled;
	spin_unlock_irqrestore(&ctx->qlock, flags);

	// Start DMA after all slots are filled
	if (ctx->enable_fb_num && ctx->fb_count == ctx->enable_fb_num) {
		rover_dma_config(ctx, 15, ctx->enable_fb_num - 1, true);
		ctx->dma_running = true;
	} else {
		rover_dma_disable(ctx);
		ctx->dma_running = false;
	}

	dev_dbg(ctx->dev, "DMA ring primed: enable_fb_num=%u (filled=%u)\n",
		ctx->enable_fb_num, filled);
	return 0;
}

static int rover_buffer_init(struct vb2_buffer *vb)
{
	struct rover_mipi_csi_buffer *buf = container_of(
		to_vb2_v4l2_buffer(vb), struct rover_mipi_csi_buffer, vb);
	INIT_LIST_HEAD(&buf->list);
	return 0;
}

static int rover_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			     unsigned int *nplanes, unsigned int sizes[],
			     struct device *alloc_devs[])
{
	struct rover_mipi_csi *ctx = vb2_get_drv_priv(vq);
	u32 size = ctx->cur_fmt.fmt.pix.sizeimage;

	if (!size) {
		dev_err(ctx->dev, "Invalid sizeimage: %u\n", size);
		return -EINVAL;
	}

	if (*nplanes) {
		if (*nplanes != 1) {
			dev_err(ctx->dev, "Only one plane supported\n");
			return -EINVAL;
		}
		if (sizes[0] < size) {
			dev_err(ctx->dev, "Buffer size too small: %u < %u\n",
				sizes[0], size);
			return -EINVAL;
		}
	} else {
		*nplanes = 1;
		sizes[0] = size;
	}

	if (*nbuffers < 2) {
		*nbuffers = 2;
	}

	ctx->enable_fb_num = min_t(u8, MAX_FRAME_BUFFER, *nbuffers);

	return 0;
}

static int rover_buffer_prepare(struct vb2_buffer *vb)
{
	struct rover_mipi_csi *ctx = vb2_get_drv_priv(vb->vb2_queue);
	u32 size = ctx->cur_fmt.fmt.pix.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(ctx->dev, "Buffer size too small: %lu < %u\n",
			vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);

	return 0;
}

static void rover_buffer_queue(struct vb2_buffer *vb)
{
	struct rover_mipi_csi *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct rover_mipi_csi_buffer *buf = container_of(
		to_vb2_v4l2_buffer(vb), struct rover_mipi_csi_buffer, vb);
	unsigned long flags;
	u8 active_slot;

	spin_lock_irqsave(&ctx->qlock, flags);
	list_add_tail(&buf->list, &ctx->buf_list);
	spin_unlock_irqrestore(&ctx->qlock, flags);

	if (!ctx->streaming) {
		return;
	}

	active_slot = rover_dma_active_fb(ctx);

	if (ctx->dma_running) {
		for (int i = 0; i < ctx->enable_fb_num; i++) {
			struct rover_mipi_csi_buffer *next;
			dma_addr_t dma;
			phys_addr_t pa;
			int slot = (active_slot + i) % ctx->enable_fb_num;

			spin_lock_irqsave(&ctx->qlock, flags);
			if (ctx->fb_slot[slot] || list_empty(&ctx->buf_list)) {
				spin_unlock_irqrestore(&ctx->qlock, flags);
				continue;
			}
			next = list_first_entry(&ctx->buf_list,
						struct rover_mipi_csi_buffer,
						list);
			list_del(&next->list);
			spin_unlock_irqrestore(&ctx->qlock, flags);

			dma = vb2_dma_contig_plane_dma_addr(&next->vb.vb2_buf,
							    0);
			pa = dma_to_phys(ctx->dev, dma);
			if (unlikely(pa == DMA_MAPPING_ERROR)) {
				dev_err(ctx->dev,
					"dma_to_phys failed for dma=%p\n",
					&dma);
				continue;
			}
			rover_write_fb_addr(ctx, slot, pa);

			spin_lock_irqsave(&ctx->qlock, flags);
			if (!ctx->fb_slot[slot]) {
				ctx->fb_slot[slot] = next;
				ctx->fb_count++;
			} else {
				list_add_tail(&next->list, &ctx->buf_list);
			}
			spin_unlock_irqrestore(&ctx->qlock, flags);
		}
	} else {
		rover_prime_ring(ctx);
	}

	rover_debug_dump_full(ctx, __func__);
}

static int rover_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct rover_mipi_csi *ctx = vb2_get_drv_priv(vq);
	int ret;

	/* enable IRQ and clear any pending */
	rover_irq_clear_dma_done(ctx);
	rover_irq_enable(ctx);

	ctx->sequence = 0;
	ctx->streaming = true;

	if (ctx->subdev) {
		ret = v4l2_subdev_call(ctx->subdev, video, s_stream, 1);
		if (ret && ret != -ENOIOCTLCMD) {
			goto err;
		}
	}

	ret = rover_prime_ring(ctx);
	if (ret) {
		goto err;
	}

	rover_debug_dump_full(ctx, __func__);

	dev_dbg(ctx->dev, "Streaming start: queued=%u enable_fb_num=%u\n",
		count, ctx->enable_fb_num);
	return 0;
err:
	ctx->streaming = false;
	rover_dma_disable(ctx);
	rover_irq_disable(ctx);
	return ret;
}

static void rover_stop_streaming(struct vb2_queue *vq)
{
	struct rover_mipi_csi *ctx = vb2_get_drv_priv(vq);
	struct rover_mipi_csi_buffer *buf, *tmp;
	unsigned long flags;

	rover_irq_disable(ctx);
	rover_dma_disable(ctx);
	synchronize_irq(ctx->irq);

	rover_debug_dump_full(ctx, __func__);

	spin_lock_irqsave(&ctx->qlock, flags);
	for (int i = 0; i < MAX_FRAME_BUFFER; i++) {
		if (ctx->fb_slot[i]) {
			vb2_buffer_done(&ctx->fb_slot[i]->vb.vb2_buf,
					VB2_BUF_STATE_ERROR);
			ctx->fb_slot[i] = NULL;
		}
	}
	list_for_each_entry_safe(buf, tmp, &ctx->buf_list, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	ctx->enable_fb_num = 4;
	ctx->fb_count = 0;
	ctx->dma_running = false;
	ctx->streaming = false;
	spin_unlock_irqrestore(&ctx->qlock, flags);

	if (ctx->subdev) {
		v4l2_subdev_call(ctx->subdev, video, s_stream, 0);
	}

	dev_dbg(ctx->dev, "Stopping streaming\n");
}

static const struct vb2_ops rover_queue_ops = {
	.buf_init = rover_buffer_init,
	.queue_setup = rover_queue_setup,
	.buf_prepare = rover_buffer_prepare,
	.buf_queue = rover_buffer_queue,
	.start_streaming = rover_start_streaming,
	.stop_streaming = rover_stop_streaming,
};

struct rover_fmt_map {
	u32 fourcc;
	u32 mbus;
};

static const struct rover_fmt_map rover_fmt_table[] = {
	{ V4L2_PIX_FMT_SBGGR10P, MEDIA_BUS_FMT_SBGGR10_1X10 },
	{ V4L2_PIX_FMT_SGBRG10P, MEDIA_BUS_FMT_SGBRG10_1X10 },
	{ V4L2_PIX_FMT_SGRBG10P, MEDIA_BUS_FMT_SGRBG10_1X10 },
	{ V4L2_PIX_FMT_SRGGB10P, MEDIA_BUS_FMT_SRGGB10_1X10 },
};

static const struct rover_fmt_map *rover_find_by_fourcc(u32 fourcc)
{
	for (size_t i = 0; i < ARRAY_SIZE(rover_fmt_table); i++)
		if (rover_fmt_table[i].fourcc == fourcc)
			return &rover_fmt_table[i];
	return NULL;
}

static void rover_fill_pix(struct v4l2_pix_format *pix, u32 width, u32 height,
			   u32 fourcc)
{
	const u32 bpp = 10; // RAW10
	u32 w = ALIGN(width, 4);
	u32 bpl = (w * bpp) / 8;

	pix->width = w;
	pix->height = height;
	pix->pixelformat = fourcc;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = bpl;
	pix->sizeimage = bpl * height;
}

static int rover_querycap(struct file *file, void *priv,
			  struct v4l2_capability *cap)
{
	struct rover_mipi_csi *ctx = video_drvdata(file);
	u32 device_caps = ctx->vdev.device_caps;

	strscpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	strscpy(cap->card, "Rover MIPI Capture", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(ctx->dev));

	cap->device_caps = device_caps;
	cap->capabilities = device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int rover_enum_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(rover_fmt_table)) {
		return -EINVAL;
	}

	f->pixelformat = rover_fmt_table[f->index].fourcc;

	return 0;
}

static int rover_try_fmt_vid_cap(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct rover_mipi_csi *ctx = video_drvdata(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	u32 w = clamp_t(u32, pix->width, 16, 4096);
	u32 h = clamp_t(u32, pix->height, 16, 4096);

	w = ALIGN(w, 4);

	u32 fourcc = pix->pixelformat;

	if (!rover_find_by_fourcc(fourcc)) {
		dev_err(ctx->dev, "Unsupported pixel format: 0x%08x\n", fourcc);
		return -EINVAL;
	}

	rover_fill_pix(pix, w, h, fourcc);

	return 0;
}

static int rover_set_fmt_vid_cap(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct rover_mipi_csi *ctx = video_drvdata(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev *subdev = ctx->subdev;
	int ret;

	if (vb2_is_busy(&ctx->queue)) {
		dev_err(ctx->dev, "Cannot set format while queue is busy\n");
		return -EBUSY;
	}

	ret = rover_try_fmt_vid_cap(file, priv, f);
	if (ret) {
		dev_err(ctx->dev, "Failed to try format: %d\n", ret);
		return ret;
	}

	if (subdev) {
		struct v4l2_subdev_format s = {
			.which = V4L2_SUBDEV_FORMAT_ACTIVE,
			.pad = ctx->subdev_src_pad,
		};
		struct rover_fmt_map const *fmt =
			rover_find_by_fourcc(pix->pixelformat);

		if (!fmt) {
			dev_err(ctx->dev, "Unsupported pixel format: 0x%08x\n",
				pix->pixelformat);
			return -EINVAL;
		}

		s.format.code = fmt->mbus;
		s.format.width = pix->width;
		s.format.height = pix->height;
		s.format.field = pix->field;

		ret = v4l2_subdev_call(subdev, pad, set_fmt, NULL, &s);
		if (ret && ret != -ENOIOCTLCMD) {
			dev_err(ctx->dev, "Failed to set subdev format: %d\n",
				ret);
			return ret;
		}

		rover_fill_pix(pix, s.format.width, s.format.height,
			       pix->pixelformat);
	}

	ctx->cur_fmt = *f;
	return 0;
}

static int rover_get_fmt_vid_cap(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct rover_mipi_csi *ctx = video_drvdata(file);
	*f = ctx->cur_fmt;
	return 0;
}

static const struct v4l2_file_operations rover_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.read = vb2_fop_read,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

static const struct v4l2_ioctl_ops rover_ioctl_ops = {
	.vidioc_querycap = rover_querycap,
	.vidioc_enum_fmt_vid_cap = rover_enum_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = rover_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = rover_set_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = rover_get_fmt_vid_cap,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_expbuf = vb2_ioctl_expbuf,

	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	// TODO: implementation `vidioc_enum_framesizes`, `vidioc_enum_frameintervals`
};

static irqreturn_t rover_irq_handler(int irq, void *dev_id)
{
	struct rover_mipi_csi *ctx = dev_id;
	struct rover_mipi_csi_buffer *done = NULL, *next = NULL;
	unsigned long flags;
	u8 active, done_idx;

	rover_irq_clear_dma_done(ctx);

	if (!ctx->dma_running) {
		dev_warn(ctx->dev, "Spurious IRQ: DMA not running\n");
		return IRQ_HANDLED;
	}

	active = rover_dma_active_fb(ctx);
	done_idx = (active + ctx->enable_fb_num - 1) % ctx->enable_fb_num;

	spin_lock_irqsave(&ctx->qlock, flags);
	done = ctx->fb_slot[done_idx];
	ctx->fb_slot[done_idx] = NULL;
	ctx->fb_count--;

	if (!list_empty(&ctx->buf_list)) {
		next = list_first_entry(&ctx->buf_list,
					struct rover_mipi_csi_buffer, list);
		list_del(&next->list);
	}
	spin_unlock_irqrestore(&ctx->qlock, flags);

	if (done) {
		done->vb.sequence = ctx->sequence++;
		done->vb.vb2_buf.timestamp = ktime_get_ns();
		vb2_buffer_done(&done->vb.vb2_buf, VB2_BUF_STATE_DONE);
	}

	if (next) {
		dma_addr_t dma =
			vb2_dma_contig_plane_dma_addr(&next->vb.vb2_buf, 0);
		phys_addr_t pa = dma_to_phys(ctx->dev, dma);
		if (likely(pa != DMA_MAPPING_ERROR)) {
			rover_write_fb_addr(ctx, done_idx, pa);

			spin_lock_irqsave(&ctx->qlock, flags);
			ctx->fb_slot[done_idx] = next;
			ctx->fb_count++;
			spin_unlock_irqrestore(&ctx->qlock, flags);
		} else {
			dev_err(ctx->dev, "dma_to_phys failed for dma=%p\n",
				&dma);
		}
	}

	spin_lock_irqsave(&ctx->qlock, flags);
	if (ctx->fb_count <= 1 || !ctx->fb_slot[active]) {
		dev_warn(
			ctx->dev,
			"stop DMA errors when the number of buffers falls below the threshold\n");
		rover_dma_disable(ctx);
		ctx->dma_running = false;
	}
	spin_unlock_irqrestore(&ctx->qlock, flags);

	rover_debug_dump_full(ctx, __func__);

	return IRQ_HANDLED;
}

static int rover_notify_bound(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *subdev,
			      struct v4l2_async_connection *asd)
{
	struct rover_mipi_csi *ctx =
		container_of(notifier, struct rover_mipi_csi, notifier);
	int ret, src_pad;

	if (ctx->subdev && ctx->subdev != subdev) {
		dev_warn(ctx->dev, "Sensor %s is already bound, rejecting %s\n",
			 ctx->subdev->name, subdev->name);
		return -EBUSY;
	}

	src_pad = media_entity_get_fwnode_pad(&subdev->entity, subdev->fwnode,
					      MEDIA_PAD_FL_SOURCE);
	if (src_pad < 0) {
		dev_err(ctx->dev,
			"Failed to get source pad for subdev %s: %d\n",
			subdev->name, src_pad);
		return src_pad;
	}

	ret = media_create_pad_link(
		&subdev->entity, src_pad, &ctx->vdev.entity, 0,
		MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(ctx->dev, "Failed to create link %s->%s: %d\n",
			subdev->name, ctx->vdev.name, ret);
		return ret;
	}

	ctx->subdev = subdev;
	ctx->subdev_src_pad = src_pad;

	dev_dbg(ctx->dev, "Sensor %s bound (src pad=%d)\n", subdev->name,
		src_pad);
	return 0;
}

static void rover_notify_unbind(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_connection *asd)
{
	struct rover_mipi_csi *ctx =
		container_of(notifier, struct rover_mipi_csi, notifier);

	media_entity_remove_links(&ctx->vdev.entity);
	if (ctx->subdev) {
		media_entity_remove_links(&ctx->subdev->entity);
	}

	if (ctx->subdev == subdev) {
		ctx->subdev = NULL;
	}

	dev_dbg(ctx->dev, "Sensor %s unbound\n",
		subdev ? subdev->name : "<none>");
}

static int rover_async_complete(struct v4l2_async_notifier *notifier)
{
	struct rover_mipi_csi *ctx =
		container_of(notifier, struct rover_mipi_csi, notifier);
	int ret;

	/* prepare /dev/v4l-subdevX */
	ret = v4l2_device_register_subdev_nodes(&ctx->v4l2_dev);
	if (ret) {
		dev_err(ctx->dev, "Failed to register subdev %s: %d\n",
			ctx->subdev ? ctx->subdev->name : "<none>", ret);
		return ret;
	}

	if (ctx->subdev) {
		dev_dbg(ctx->dev, "Subdevice %s registered\n",
			ctx->subdev->name);
	}

	return 0;
}

static const struct v4l2_async_notifier_operations rover_async_ops = {
	.bound = rover_notify_bound,
	.unbind = rover_notify_unbind,
	.complete = rover_async_complete,
};

static void rover_mdev_unregister(void *data)
{
	struct media_device *mdev = data;

	media_device_unregister(mdev);
	media_device_cleanup(mdev);
}

static void rover_v4l2_dev_unregister(void *data)
{
	struct v4l2_device *v4l2_dev = data;

	v4l2_device_unregister(v4l2_dev);
}

static void rover_vdev_unregister(void *data)
{
	struct video_device *vdev = data;

	video_unregister_device(vdev);
}

static void rover_media_entity_cleanup(void *data)
{
	struct media_entity *entity = data;

	media_entity_cleanup(entity);
}

static void rover_async_notifier_cleanup(void *data)
{
	struct v4l2_async_notifier *notifier = data;

	v4l2_async_nf_unregister(notifier);
	v4l2_async_nf_cleanup(notifier);
}

static int rover_init_vb2_and_entity(struct platform_device *pdev,
				     struct rover_mipi_csi *ctx)
{
	struct vb2_queue *queue = &ctx->queue;
	struct video_device *vdev = &ctx->vdev;
	int ret;

	/* setup vb2 queue */
	queue->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	queue->io_modes = VB2_MMAP | VB2_DMABUF;
	queue->dev = &pdev->dev;
	queue->drv_priv = ctx;
	queue->buf_struct_size = sizeof(struct rover_mipi_csi_buffer);
	queue->ops = &rover_queue_ops;
	queue->mem_ops = &vb2_dma_contig_memops;
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	queue->min_queued_buffers = 1;
	queue->lock = &ctx->lock;
	queue->gfp_flags = GFP_KERNEL;

	ret = vb2_queue_init(queue);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize vb2 queue: %d\n",
			ret);
		return ret;
	}

	/* setup format */
	ctx->cur_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	rover_fill_pix(&ctx->cur_fmt.fmt.pix, 1456, 1088,
		       V4L2_PIX_FMT_SBGGR10P);

	/* setup video device */
	strscpy(vdev->name, KBUILD_MODNAME, sizeof(vdev->name));
	vdev->release = video_device_release_empty;
	vdev->fops = &rover_fops;
	vdev->ioctl_ops = &rover_ioctl_ops;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	vdev->lock = &ctx->lock;
	vdev->queue = queue;
	vdev->v4l2_dev = &ctx->v4l2_dev;

	ctx->vdev.entity.function = MEDIA_ENT_F_IO_V4L;
	ctx->pad.flags = MEDIA_PAD_FL_SINK;
	ctx->streaming = false;
	ctx->dma_running = false;
	ctx->enable_fb_num = 4;
	ctx->fb_count = 0;
	ctx->sequence = 0;

	ret = media_entity_pads_init(&ctx->vdev.entity, 1, &ctx->pad);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to initialize media entity pads: %d\n", ret);
		return ret;
	}
	devm_add_action_or_reset(&pdev->dev, rover_media_entity_cleanup,
				 &ctx->vdev.entity);

	return 0;
}

static int rover_init_async_notifier(struct platform_device *pdev,
				     struct rover_mipi_csi *ctx)
{
	struct fwnode_handle *endpoint;
	struct v4l2_async_connection *asd;
	int ret;

	v4l2_async_nf_init(&ctx->notifier, &ctx->v4l2_dev);
	ctx->notifier.ops = &rover_async_ops;

	endpoint = fwnode_graph_get_endpoint_by_id(dev_fwnode(ctx->dev), 0, 0,
						   FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!endpoint) {
		dev_err(ctx->dev, "Failed to get endpoint\n");
		return -ENODEV;
	}

	asd = v4l2_async_nf_add_fwnode_remote(&ctx->notifier, endpoint,
					      struct v4l2_async_connection);
	fwnode_handle_put(endpoint);
	if (IS_ERR(asd)) {
		dev_err(ctx->dev, "Failed to add remote endpoint\n");
		return PTR_ERR(asd);
	}

	ret = v4l2_async_nf_register(&ctx->notifier);
	if (ret) {
		dev_err(ctx->dev, "Failed to register async notifier: %d\n",
			ret);
		return ret;
	}
	devm_add_action_or_reset(&pdev->dev, rover_async_notifier_cleanup,
				 &ctx->notifier);

	return 0;
}

static int rover_mipi_csi_probe(struct platform_device *pdev)
{
	struct rover_mipi_csi *ctx;
	struct resource *res;
	int irq;
	int ret = 0;

	/* Allocate driver context */
	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		dev_err(&pdev->dev,
			"Failed to allocate memory for Rover MIPI CSI device\n");
		return -ENOMEM;
	}
	ctx->dev = &pdev->dev;

	/* Initialize dirver context */
	INIT_LIST_HEAD(&ctx->buf_list);
	mutex_init(&ctx->lock);
	spin_lock_init(&ctx->qlock);

	/* Setup media device */
	media_device_init(&ctx->mdev);
	ctx->mdev.dev = &pdev->dev;
	strscpy(ctx->mdev.model, KBUILD_MODNAME, sizeof(ctx->mdev.model));

	ret = media_device_register(&ctx->mdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register media device: %d\n",
			ret);
		return ret;
	}
	devm_add_action_or_reset(&pdev->dev, rover_mdev_unregister, &ctx->mdev);

	/* Setup v4l2 device */
	ctx->v4l2_dev.mdev = &ctx->mdev;
	ret = v4l2_device_register(&pdev->dev, &ctx->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register V4L2 device: %d\n",
			ret);
		return ret;
	}
	devm_add_action_or_reset(&pdev->dev, rover_v4l2_dev_unregister,
				 &ctx->v4l2_dev);

	/* MMIO and IRQ */
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "Failed to set 32-bit DMA mask: %d\n", ret);
		return ret;
	}

	ctx->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(ctx->base)) {
		dev_err(&pdev->dev, "Failed to map device memory\n");
		return PTR_ERR(ctx->base);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get IRQ resource\n");
		return irq;
	}
	ret = devm_request_irq(&pdev->dev, irq, rover_irq_handler, 0,
			       KBUILD_MODNAME, ctx);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ %d: %d\n", irq, ret);
		return ret;
	}
	ctx->irq = irq;

	/* Setup vb2 and media entity */
	ret = rover_init_vb2_and_entity(pdev, ctx);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to initialize vb2 and media entity: %d\n", ret);
		return ret;
	}

	/* Setup v4l2 subdevice async notifier */
	ret = rover_init_async_notifier(pdev, ctx);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize async notifier: %d\n",
			ret);
		return ret;
	}

	/* Setup video device */
	video_set_drvdata(&ctx->vdev, ctx);

	ret = video_register_device(&ctx->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register video device: %d\n",
			ret);
		return ret;
	}
	devm_add_action_or_reset(&pdev->dev, rover_vdev_unregister, &ctx->vdev);

	platform_set_drvdata(pdev, ctx);

	dev_info(&pdev->dev, "Rover MIPI CSI-2 driver probed\n");
	return 0;
}

static void rover_mipi_csi_remove(struct platform_device *pdev)
{
	struct rover_mipi_csi *ctx = platform_get_drvdata(pdev);

	vb2_queue_release(&ctx->queue);

	dev_info(&pdev->dev, "Rover MIPI CSI-2 driver removed\n");
}

#ifdef CONFIG_OF
static const struct of_device_id of_rover_mipi_csi_match[] = {
	{
		.compatible = "sc,rover-mipi-csi",
	},
	{},
};
MODULE_DEVICE_TABLE(of, of_rover_mipi_csi_match);
#endif

static struct platform_driver rover_mipi_csi_driver = {
	.probe = rover_mipi_csi_probe,
	.remove = rover_mipi_csi_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = of_match_ptr(of_rover_mipi_csi_match),
		// TODO: Implement suspend/resume and runtime PM callbacks
	},
};

module_platform_driver(rover_mipi_csi_driver);
