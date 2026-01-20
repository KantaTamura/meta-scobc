// SPDX-License-Identifier: GPL-2.0-only
/*
 * rover_hls.c
 *
 * Copyright (c) 2025 Space Cubics, Inc.
 *
 * This driver provides support for a generic FPGA-based image processing accelerator
 * implemented on the 4th.ai Autonomous Driving Board with a Xilinx Zynq UltraScale+.
 * The accelerator receives input image buffers, performs hardware processing defined in HLS,
 * and delivers the processed frames back via DMA.
 * The driver exposes the accelerator as a V4L2 mem2mem device,
 * allowing applications to stream image data in and out through the standard V4L2 framework.
 */

#include <linux/of.h>
#include <linux/videodev2.h>
#include <linux/dma-direct.h>
#include <linux/platform_device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

MODULE_DESCRIPTION(
	"V4L2 mem2mem driver for HLS on 4th.ai Autonomous Driving Board");
MODULE_AUTHOR("KantaTamura <kanta@spacecubics.com>");
MODULE_LICENSE("GPL v2");

struct rover_hls {
	struct device *dev;
	struct video_device vdev;
	struct v4l2_device v4l2_dev;
	struct v4l2_m2m_dev *m2m_dev;

	struct mutex lock;
	spinlock_t qlock;

	void __iomem *base;
	bool has_mode; // if REG_HLS_MODE is available
	u32 mode;

	struct v4l2_ctrl_handler ctrl_hdl;
	struct v4l2_ctrl *ctrl_mode;
	struct mutex ctrl_lock;

	u32 max_width;
	u32 max_height;

	struct rover_hls_ctx *active;
};

struct rover_hls_ctx {
	struct rover_hls *hls;

	struct v4l2_fh fh;
	struct v4l2_m2m_ctx *m2m_ctx;

	struct v4l2_pix_format_mplane out_fmt;
	struct v4l2_pix_format_mplane cap_fmt;

	struct vb2_v4l2_buffer *src_buf;
	struct vb2_v4l2_buffer *dst_buf;
};

/* V4L2 Controls */
#define V4L2_CID_SC_CHANNEL_MODE (V4L2_CID_USER_BASE + 0x1000)

/* HW register offsets */
#define REG_HLS_CTRL 0x00
#define REG_HLS_GLOBAL_INT_ENABLE 0x04
#define REG_HLS_IP_INT_ENABLE 0x08
#define REG_HLS_IP_INT_STATUS 0x0C
#define REG_HLS_IMG_IN_LOW 0x10
#define REG_HLS_IMG_IN_HIGH 0x14
#define REG_HLS_IMG_OUT_LOW 0x1C
#define REG_HLS_IMG_OUT_HIGH 0x20
#define REG_HLS_DATA_MAX_IMAGE_SIZE 0x28
#define REG_HLS_CTRL_MAX_IMAGE_SIZE 0x2C
#define REG_HLS_DATA_INPUT_IMAGE_SIZE 0x38
#define REG_HLS_CTRL_INPUT_IMAGE_SIZE 0x3C
#define REG_HLS_DATA_OUTPUT_IMAGE_SIZE 0x48
#define REG_HLS_CTRL_OUTPUT_IMAGE_SIZE 0x4C
#define REG_HLS_MODE 0x58

/* Control Signals */
#define AP_START BIT(0)
#define AP_DONE BIT(1)
#define AP_IDLE BIT(2)
#define AP_READY BIT(3)
#define AP_CONTINUE BIT(4)
#define AUTO_START BIT(7)
#define INTERRUPT BIT(9)

/* Interrupt Signals */
#define GLOBAL_INT_ENABLE BIT(0)
#define AP_READY_INT_ENABLE BIT(1)
#define AP_DONE_INT_ENABLE BIT(0)
#define AP_READY_INT_STATUS BIT(1)
#define AP_DONE_INT_STATUS BIT(0)

/* image valid bit */
#define IMAGE_VALID BIT(0)

/* HW register operations */
static inline u32 __rover_read(struct rover_hls *ctx, u32 reg)
{
	return ioread32(ctx->base + reg);
}

static inline void __rover_write(struct rover_hls *ctx, u32 reg, u32 val)
{
	iowrite32(val, ctx->base + reg);
}

static inline void __rover_update_bits(struct rover_hls *ctx, u32 reg, u32 mask,
				       u32 val)
{
	u32 tmp = __rover_read(ctx, reg);
	tmp &= ~mask;
	tmp |= val & mask;
	__rover_write(ctx, reg, tmp);
}

static inline void __rover_set_bits(struct rover_hls *ctx, u32 reg, u32 mask)
{
	__rover_update_bits(ctx, reg, mask, mask);
}

static inline void __rover_clear_bits(struct rover_hls *ctx, u32 reg, u32 mask)
{
	__rover_update_bits(ctx, reg, mask, 0);
}

static inline void __rover_w1c(struct rover_hls *ctx, u32 reg, u32 mask)
{
	__rover_write(ctx, reg, mask);
}

// NOTE: status_mask should be one of AP_DONE_INT_STATUS or AP_READY_INT_STATUS
static inline void rover_hls_irq_ack(struct rover_hls *ctx, u32 status_mask)
{
	u32 st = __rover_read(ctx, REG_HLS_IP_INT_STATUS);
	u32 ack = st & status_mask;

	if (ack)
		__rover_write(ctx, REG_HLS_IP_INT_STATUS, ack);
}

static inline void rover_hls_start(struct rover_hls *ctx)
{
	rover_hls_irq_ack(ctx, AP_DONE_INT_STATUS | AP_READY_INT_STATUS);
	wmb();
	__rover_set_bits(ctx, REG_HLS_CTRL, AP_START);
}

static inline void rover_hls_continue(struct rover_hls *ctx)
{
	__rover_set_bits(ctx, REG_HLS_CTRL, AP_CONTINUE);
}

static inline bool rover_hls_is_idle(struct rover_hls *ctx)
{
	return (__rover_read(ctx, REG_HLS_CTRL) & AP_IDLE) != 0;
}

static inline bool rover_hls_is_ready(struct rover_hls *ctx)
{
	return (__rover_read(ctx, REG_HLS_CTRL) & AP_READY) != 0;
}

static inline bool rover_hls_is_done(struct rover_hls *ctx)
{
	return (__rover_read(ctx, REG_HLS_CTRL) & AP_DONE) != 0;
}

// NOTE: This is a mode that enables hls to operate intermittently,
// but using the v4l2 file handler for operation appears to be more useful for handling multiple contexts.
// Therefore, it should generally not be used.
static inline void rover_hls_set_auto_restart(struct rover_hls *ctx, bool en)
{
	if (en) {
		__rover_set_bits(ctx, REG_HLS_CTRL, AUTO_START);
	} else {
		__rover_clear_bits(ctx, REG_HLS_CTRL, AUTO_START);
	}
}

static inline void rover_hls_global_irq_enable(struct rover_hls *ctx, bool en)
{
	if (en) {
		__rover_set_bits(ctx, REG_HLS_GLOBAL_INT_ENABLE,
				 GLOBAL_INT_ENABLE);
	} else {
		__rover_clear_bits(ctx, REG_HLS_GLOBAL_INT_ENABLE,
				   GLOBAL_INT_ENABLE);
	}
}

static inline void rover_hls_ip_irq_enable(struct rover_hls *ctx, bool en_done,
					   bool en_ready)
{
	u32 mask = 0;

	if (en_done) {
		mask |= AP_DONE_INT_ENABLE;
	}
	if (en_ready) {
		mask |= AP_READY_INT_ENABLE;
	}

	__rover_set_bits(ctx, REG_HLS_IP_INT_ENABLE, mask);
}

static inline void rover_hls_ip_irq_disable(struct rover_hls *ctx,
					    bool dis_done, bool dis_ready)
{
	u32 mask = 0;

	if (dis_done) {
		mask |= AP_DONE_INT_ENABLE;
	}
	if (dis_ready) {
		mask |= AP_READY_INT_ENABLE;
	}

	__rover_clear_bits(ctx, REG_HLS_IP_INT_ENABLE, mask);
}

static inline u32 rover_hls_irq_status(struct rover_hls *ctx)
{
	return __rover_read(ctx, REG_HLS_IP_INT_STATUS);
}

static inline void rover_hls_irq_enable_done(struct rover_hls *ctx)
{
	rover_hls_global_irq_enable(ctx, true);
	rover_hls_ip_irq_enable(ctx, true, false);
}

static inline void rover_hls_set_img_in_addr(struct rover_hls *ctx,
					     phys_addr_t addr)
{
	__rover_write(ctx, REG_HLS_IMG_IN_LOW, lower_32_bits(addr));
	__rover_write(ctx, REG_HLS_IMG_IN_HIGH, upper_32_bits(addr));
}

static inline void rover_hls_set_img_out_addr(struct rover_hls *ctx,
					      phys_addr_t addr)
{
	__rover_write(ctx, REG_HLS_IMG_OUT_LOW, lower_32_bits(addr));
	__rover_write(ctx, REG_HLS_IMG_OUT_HIGH, upper_32_bits(addr));
}

static inline int rover_hls_get_max_img_size(struct rover_hls *ctx, u32 *width,
					     u32 *height)
{
	if (__rover_read(ctx, REG_HLS_CTRL_MAX_IMAGE_SIZE) & IMAGE_VALID) {
		u32 val = __rover_read(ctx, REG_HLS_DATA_MAX_IMAGE_SIZE);
		*width = upper_16_bits(val);
		*height = lower_16_bits(val);
		return 0;
	} else {
		dev_warn(ctx->dev, "Max image size not valid\n");
		return -1;
	}
}

static inline int rover_hls_get_input_img_size(struct rover_hls *ctx,
					       u32 *width, u32 *height)
{
	if (__rover_read(ctx, REG_HLS_CTRL_INPUT_IMAGE_SIZE) & IMAGE_VALID) {
		u32 val = __rover_read(ctx, REG_HLS_DATA_INPUT_IMAGE_SIZE);
		*width = upper_16_bits(val);
		*height = lower_16_bits(val);
		return 0;
	} else {
		dev_warn(ctx->dev, "Input image size not valid\n");
		return -1;
	}
}

static inline int rover_hls_get_output_img_size(struct rover_hls *ctx,
						u32 *width, u32 *height)
{
	if (__rover_read(ctx, REG_HLS_CTRL_OUTPUT_IMAGE_SIZE) & IMAGE_VALID) {
		u32 val = __rover_read(ctx, REG_HLS_DATA_OUTPUT_IMAGE_SIZE);
		*width = upper_16_bits(val);
		*height = lower_16_bits(val);
		return 0;
	} else {
		dev_warn(ctx->dev, "Output image size not valid\n");
		return -1;
	}
}

static inline void rover_hls_set_mode(struct rover_hls *ctx)
{
	if (ctx->has_mode) {
		__rover_write(ctx, REG_HLS_MODE, ctx->mode);
	}
}

// NOTE: A utility function that instructs the HLS about the addresses for reading images and writing data,
// enables the `done` interrupt, and begins the operation
static inline void rover_hls_program_and_start(struct rover_hls *ctx,
					       phys_addr_t in_addr,
					       phys_addr_t out_addr)
{
	rover_hls_set_mode(ctx);
	rover_hls_set_img_in_addr(ctx, in_addr);
	rover_hls_set_img_out_addr(ctx, out_addr);
	rover_hls_irq_enable_done(ctx);
	rover_hls_start(ctx);
}

static inline void rover_hls_reset(struct rover_hls *ctx)
{
	// disable interrupts
	rover_hls_global_irq_enable(ctx, false);
	rover_hls_ip_irq_disable(ctx, true, true);
	// stop processing
	__rover_clear_bits(ctx, REG_HLS_CTRL,
			   AP_START | AP_CONTINUE | AUTO_START);
	// clear any pending interrupt status
	rover_hls_irq_ack(ctx, AP_DONE_INT_STATUS | AP_READY_INT_STATUS);
}

/* debug logging */
#if defined(SC_ROVER_HLS_DEBUG)
static void __rover_hls_debug_dump_context(struct rover_hls_ctx *ctx)
{
	unsigned long flags;
	struct rover_hls *hls = ctx->hls;
	unsigned int src_ready = v4l2_m2m_num_src_bufs_ready(ctx->m2m_ctx);
	unsigned int dst_ready = v4l2_m2m_num_dst_bufs_ready(ctx->m2m_ctx);

	dev_info(hls->dev, "context summary:\n");

	dev_info(hls->dev,
		 "\tOUTPUT: %ux%u fmt=%p4cc planes=%u bpl=%u size=%u\n",
		 ctx->out_fmt.width, ctx->out_fmt.height,
		 &ctx->out_fmt.pixelformat, ctx->out_fmt.num_planes,
		 ctx->out_fmt.plane_fmt[0].bytesperline,
		 ctx->out_fmt.plane_fmt[0].sizeimage);

	dev_info(hls->dev,
		 "\tCAPTURE: %ux%u fmt=%p4cc planes=%u bpl=%u size=%u\n",
		 ctx->cap_fmt.width, ctx->cap_fmt.height,
		 &ctx->cap_fmt.pixelformat, ctx->cap_fmt.num_planes,
		 ctx->cap_fmt.plane_fmt[0].bytesperline,
		 ctx->cap_fmt.plane_fmt[0].sizeimage);

	dev_info(hls->dev,
		 "\tm2m: src_ready=%u dst_ready=%u active=%p (this=%p)\n",
		 src_ready, dst_ready, hls->active, ctx);

	spin_lock_irqsave(&hls->qlock, flags);
	if (ctx->src_buf) {
		dma_addr_t src_addr = vb2_dma_contig_plane_dma_addr(
			&ctx->src_buf->vb2_buf, 0);
		phys_addr_t src_pa = dma_to_phys(hls->dev, src_addr);
		if (src_pa == DMA_MAPPING_ERROR) {
			dev_err(hls->dev, "dma_to_phys failed for dma=%p\n",
				&src_addr);
		}
		dev_info(hls->dev,
			 "\tcur_src: buf=%p idx=%u dma=%pad phy=%pa\n",
			 ctx->src_buf, ctx->src_buf->vb2_buf.index, &src_addr,
			 &src_pa);
	} else {
		dev_info(hls->dev, "\tcur_src: NULL\n");
	}

	if (ctx->dst_buf) {
		dma_addr_t dst_addr = vb2_dma_contig_plane_dma_addr(
			&ctx->dst_buf->vb2_buf, 0);
		phys_addr_t dst_pa = dma_to_phys(hls->dev, dst_addr);
		if (dst_pa == DMA_MAPPING_ERROR) {
			dev_err(hls->dev, "dma_to_phys failed for dma=%p\n",
				&dst_addr);
		}
		dev_info(hls->dev,
			 "\tcur_dst: buf=%p idx=%u dma=%pad phy=%pa\n",
			 ctx->dst_buf, ctx->dst_buf->vb2_buf.index, &dst_addr,
			 &dst_pa);
	} else {
		dev_info(hls->dev, "\tcur_dst: NULL\n");
	}
	spin_unlock_irqrestore(&hls->qlock, flags);
}

static void __rover_hls_debug_dump_hardware(struct rover_hls_ctx *ctx)
{
	struct rover_hls *hls = ctx->hls;
	unsigned long flags;
	u32 reg_ctrl, reg_gie, reg_ier, reg_isr;
	u32 in_lo, in_hi, out_lo, out_hi;
	u64 in_addr, out_addr;
	u32 max_width, max_height;
	u32 in_width, in_height;
	u32 out_width, out_height;
	u32 mode;

	spin_lock_irqsave(&hls->qlock, flags);
	reg_ctrl = __rover_read(hls, REG_HLS_CTRL);
	reg_gie = __rover_read(hls, REG_HLS_GLOBAL_INT_ENABLE);
	reg_ier = __rover_read(hls, REG_HLS_IP_INT_ENABLE);
	reg_isr = __rover_read(hls, REG_HLS_IP_INT_STATUS);

	in_lo = __rover_read(hls, REG_HLS_IMG_IN_LOW);
	in_hi = __rover_read(hls, REG_HLS_IMG_IN_HIGH);
	out_lo = __rover_read(hls, REG_HLS_IMG_OUT_LOW);
	out_hi = __rover_read(hls, REG_HLS_IMG_OUT_HIGH);

	mode = __rover_read(hls, REG_HLS_MODE);
	spin_unlock_irqrestore(&hls->qlock, flags);

	in_addr = ((u64)in_hi << 32) | in_lo;
	out_addr = ((u64)out_hi << 32) | out_lo;

	dev_info(hls->dev, "hardware summary:\n");
	dev_info(
		hls->dev,
		"\tCTRL=0x%08x [start=%d done=%d idle=%d ready=%d cont=%d auto=%d irq(latched)=%d]\n",
		reg_ctrl, !!(reg_ctrl & AP_START), !!(reg_ctrl & AP_DONE),
		!!(reg_ctrl & AP_IDLE), !!(reg_ctrl & AP_READY),
		!!(reg_ctrl & AP_CONTINUE), !!(reg_ctrl & AUTO_START),
		!!(reg_ctrl & INTERRUPT));

	dev_info(
		hls->dev,
		"\tGIE=0x%08x IER=0x%08x ISR=0x%08x [GIE=%d, done_en=%d ready_en=%d, done_st=%d ready_st=%d]\n",
		reg_gie, reg_ier, reg_isr, !!(reg_gie & GLOBAL_INT_ENABLE),
		!!(reg_ier & AP_DONE_INT_ENABLE),
		!!(reg_ier & AP_READY_INT_ENABLE),
		!!(reg_isr & AP_DONE_INT_STATUS),
		!!(reg_isr & AP_READY_INT_STATUS));

	dev_info(hls->dev, "\tIMG_IN = 0x%016llx (hi=0x%08x lo=0x%08x)\n",
		 (unsigned long long)in_addr, in_hi, in_lo);
	dev_info(hls->dev, "\tIMG_OUT= 0x%016llx (hi=0x%08x lo=0x%08x)\n",
		 (unsigned long long)out_addr, out_hi, out_lo);

	if (rover_hls_get_max_img_size(hls, &max_width, &max_height) == 0) {
		dev_info(hls->dev, "\tMAX_IMG_SIZE= %ux%u\n", max_width,
			 max_height);
	} else {
		dev_info(hls->dev, "\tMAX_IMG_SIZE= invalid\n");
	}

	if (rover_hls_get_input_img_size(hls, &in_width, &in_height) == 0) {
		dev_info(hls->dev, "\tINPUT_IMG_SIZE= %ux%u\n", in_width,
			 in_height);
	} else {
		dev_info(hls->dev, "\tINPUT_IMG_SIZE= invalid\n");
	}

	if (rover_hls_get_output_img_size(hls, &out_width, &out_height) == 0) {
		dev_info(hls->dev, "\tOUTPUT_IMG_SIZE= %ux%u\n", out_width,
			 out_height);
	} else {
		dev_info(hls->dev, "\tOUTPUT_IMG_SIZE= invalid\n");
	}

	dev_info(hls->dev, "\tMODE=0x%08x (has_mode=%d cur_mode=0x%08x)\n",
		 mode, hls->has_mode, hls->mode);
}

#define rover_hls_debug_dump_full(ctx, tag)                              \
	do {                                                             \
		dev_info((ctx)->hls->dev, "========== %s ==========\n",  \
			 (tag));                                         \
		__rover_hls_debug_dump_context((ctx));                   \
		__rover_hls_debug_dump_hardware((ctx));                  \
		dev_info((ctx)->hls->dev, "========================\n"); \
	} while (0)

#else
#define rover_hls_debug_dump_full(ctx, tag) \
	do {                                \
	} while (0)
#endif /* defined(SC_ROVER_HLS_DEBUG) */

/* V4L2 ioctl operations */
static int rover_querycap(struct file *file, void *priv,
			  struct v4l2_capability *cap)
{
	struct rover_hls *ctx = video_drvdata(file);
	u32 device_caps = ctx->vdev.device_caps;

	strscpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	strscpy(cap->card, "Rover HLS Driver", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(ctx->dev));

	cap->device_caps = device_caps;
	cap->capabilities = device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

struct rover_hls_fmt {
	u32 pixelformat;
	const char *name;
	unsigned int bpp;
	u8 num_planes;
	bool is_yuv;
	bool is_bayer;
};

static const struct rover_hls_fmt rover_hls_fmts[] = {
	/* Bayer (unpacked 16bpp) */
	{ V4L2_PIX_FMT_SBGGR10, "Bayer BGGR10", 16, 1, .is_yuv = false,
	  .is_bayer = true },
	{ V4L2_PIX_FMT_SGBRG10, "Bayer GBRG10", 16, 1, .is_yuv = false,
	  .is_bayer = true },
	{ V4L2_PIX_FMT_SGRBG10, "Bayer GRBG10", 16, 1, .is_yuv = false,
	  .is_bayer = true },
	{ V4L2_PIX_FMT_SRGGB10, "Bayer RGGB10", 16, 1, .is_yuv = false,
	  .is_bayer = true },

	/* Packed RGB/BGR */
	{ V4L2_PIX_FMT_RGB24, "RGB24", 24, 1, .is_yuv = false,
	  .is_bayer = false },
	{ V4L2_PIX_FMT_BGR24, "BGR24", 24, 1, .is_yuv = false,
	  .is_bayer = false },

	/* YUV422 packed (1plane) */
	{ V4L2_PIX_FMT_YUYV, "YUYV 4:2:2", 16, 1, .is_yuv = true,
	  .is_bayer = false },
	{ V4L2_PIX_FMT_UYVY, "UYVY 4:2:2", 16, 1, .is_yuv = true,
	  .is_bayer = false },

	/* Greyscale */
	{ V4L2_PIX_FMT_GREY, "Greyscale 8-bit", 8, 1, .is_yuv = false,
	  .is_bayer = false },
};

static inline u32 gcd_u32(u32 a, u32 b)
{
	return b ? gcd_u32(b, a % b) : a;
}

static inline u32 lcm_u32(u32 a, u32 b)
{
	return a / gcd_u32(a, b) * b;
}

static inline const struct rover_hls_fmt *rover_hls_find_fmt(u32 fourcc)
{
	for (size_t i = 0; i < ARRAY_SIZE(rover_hls_fmts); i++) {
		if (rover_hls_fmts[i].pixelformat == fourcc) {
			return &rover_hls_fmts[i];
		}
	}
	return NULL;
}

static void rover_fill_pix_mp(struct rover_hls_ctx *ctx,
			      struct v4l2_pix_format_mplane *pix, u32 w, u32 h,
			      const struct rover_hls_fmt *fmt)
{
	u32 width = clamp(w, 64, ctx->hls->max_width);
	u32 height = clamp(h, 64, ctx->hls->max_height);
	u32 align = 1, bpl;

	memset(pix, 0, sizeof(*pix));
	pix->width = width;
	pix->height = height;
	pix->field = V4L2_FIELD_NONE;
	pix->pixelformat = fmt->pixelformat;
	pix->num_planes = fmt->num_planes;

	if (fmt->pixelformat == V4L2_PIX_FMT_GREY) {
		pix->colorspace = V4L2_COLORSPACE_RAW;
		pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
		pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
		pix->xfer_func = V4L2_XFER_FUNC_NONE;
	} else if (fmt->is_bayer) {
		pix->colorspace = V4L2_COLORSPACE_RAW;
		pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
		pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
		pix->xfer_func = V4L2_XFER_FUNC_NONE;
	} else if (fmt->is_yuv) {
		pix->colorspace = V4L2_COLORSPACE_REC709;
		pix->ycbcr_enc = V4L2_YCBCR_ENC_709;
		pix->quantization = V4L2_QUANTIZATION_LIM_RANGE;
		pix->xfer_func = V4L2_XFER_FUNC_709;
	} else { /* RGB */
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
		pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
		pix->xfer_func = V4L2_XFER_FUNC_SRGB;
	}

	switch (fmt->pixelformat) {
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24: {
		const u32 cpp = DIV_ROUND_UP(fmt->bpp, 8);
		const u32 step = lcm_u32(align, cpp);
		bpl = roundup(width * cpp, step);
		pix->plane_fmt[0].bytesperline = bpl;
		pix->plane_fmt[0].sizeimage = bpl * height;
		break;
	}
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY: {
		const u32 cpp = DIV_ROUND_UP(fmt->bpp, 8);
		const u32 step = lcm_u32(align, cpp);
		bpl = roundup(width * cpp, step);
		pix->plane_fmt[0].bytesperline = bpl;
		pix->plane_fmt[0].sizeimage = bpl * height;
		break;
	}
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10: {
		bpl = ALIGN(DIV_ROUND_UP(width * fmt->bpp, 8), align);
		pix->plane_fmt[0].bytesperline = bpl;
		pix->plane_fmt[0].sizeimage = bpl * height;
		break;
	}
	case V4L2_PIX_FMT_GREY: {
		const u32 cpp = DIV_ROUND_UP(fmt->bpp, 8);
		const u32 step = lcm_u32(align, cpp);
		bpl = roundup(width * cpp, step);
		pix->plane_fmt[0].bytesperline = bpl;
		pix->plane_fmt[0].sizeimage = bpl * height;
		break;
	}
	default: {
		bpl = ALIGN(DIV_ROUND_UP(width * fmt->bpp, 8), align);
		pix->plane_fmt[0].bytesperline = bpl;
		pix->plane_fmt[0].sizeimage = bpl * height;
		break;
	}
	}
}

static int rover_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(rover_hls_fmts)) {
		return -EINVAL;
	}

	f->pixelformat = rover_hls_fmts[f->index].pixelformat;
	strscpy(f->description, rover_hls_fmts[f->index].name,
		sizeof(f->description));
	return 0;
}

static int rover_get_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct rover_hls_ctx *ctx =
		container_of(file->private_data, struct rover_hls_ctx, fh);

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		f->fmt.pix_mp = ctx->out_fmt;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		f->fmt.pix_mp = ctx->cap_fmt;
		break;
	default:
		dev_err(ctx->hls->dev, "Unsupported format type: %d\n",
			f->type);
		return -EINVAL;
	}
	return 0;
}

static int rover_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct rover_hls_ctx *ctx =
		container_of(file->private_data, struct rover_hls_ctx, fh);
	struct v4l2_pix_format_mplane *fmt_mp = &f->fmt.pix_mp;
	const struct rover_hls_fmt *fmt =
		rover_hls_find_fmt(fmt_mp->pixelformat);
	u32 width, height;

	if (!fmt) {
		// NOTE: default to first format if unsupported
		fmt = &rover_hls_fmts[0];
	}

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		if (rover_hls_get_input_img_size(ctx->hls, &width, &height) !=
		    0) {
			dev_err(ctx->hls->dev,
				"Failed to get input image size from hardware\n");
			return -EINVAL;
		}
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		if (rover_hls_get_output_img_size(ctx->hls, &width, &height) !=
		    0) {
			dev_err(ctx->hls->dev,
				"Failed to get input image size from hardware\n");
			return -EINVAL;
		}
		break;
	default:
		dev_err(ctx->hls->dev, "Unsupported format type: %d\n",
			f->type);
		return -EINVAL;
	}

	if (fmt_mp->width != width || fmt_mp->height != height) {
		dev_warn(ctx->hls->dev,
			"Requested size %ux%u does not match hardware size %ux%u. Adjusting.\n",
			fmt_mp->width, fmt_mp->height, width, height);
	}

	rover_fill_pix_mp(ctx, fmt_mp, width, height, fmt);
	return 0;
}

static int rover_set_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct rover_hls_ctx *ctx =
		container_of(file->private_data, struct rover_hls_ctx, fh);
	int ret;

	ret = rover_try_fmt(file, priv, f);
	if (ret) {
		dev_err(ctx->hls->dev, "Failed to set format: %d\n", ret);
		return ret;
	}

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		ctx->out_fmt = f->fmt.pix_mp;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		ctx->cap_fmt = f->fmt.pix_mp;
		break;
	default:
		dev_err(ctx->hls->dev, "Unsupported format type: %d\n",
			f->type);
		return -EINVAL;
	}
	return 0;
}

static const struct v4l2_ioctl_ops rover_ioctl_ops = {
	.vidioc_querycap = rover_querycap,

	.vidioc_enum_fmt_vid_cap = rover_enum_fmt,
	.vidioc_enum_fmt_vid_out = rover_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane = rover_get_fmt,
	.vidioc_g_fmt_vid_out_mplane = rover_get_fmt,
	.vidioc_try_fmt_vid_cap_mplane = rover_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = rover_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane = rover_set_fmt,
	.vidioc_s_fmt_vid_out_mplane = rover_set_fmt,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
};

/* vb2 queue operations */
static int rover_queue_setup(struct vb2_queue *vq, unsigned int *nbufs,
			     unsigned int *nplanes, unsigned int sizes[],
			     struct device *alloc_devs[])
{
	struct rover_hls_ctx *ctx = vb2_get_drv_priv(vq);

	if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		// OUTPUT (source)
		const struct v4l2_pix_format_mplane *f = &ctx->out_fmt;
		if (*nplanes && sizes[0] < f->plane_fmt[0].sizeimage) {
			dev_err(ctx->hls->dev,
				"Buffer size too small: %u < %u\n", sizes[0],
				f->plane_fmt[0].sizeimage);
			return -EINVAL;
		}

		*nplanes = 1;
		sizes[0] = f->plane_fmt[0].sizeimage;
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		// CAPTURE (destination)
		const struct v4l2_pix_format_mplane *f = &ctx->cap_fmt;
		if (*nplanes && sizes[0] < f->plane_fmt[0].sizeimage) {
			dev_err(ctx->hls->dev,
				"Buffer size too small: %u < %u\n", sizes[0],
				f->plane_fmt[0].sizeimage);
			return -EINVAL;
		}

		*nplanes = 1;
		sizes[0] = f->plane_fmt[0].sizeimage;
	} else {
		dev_err(ctx->hls->dev, "Unsupported queue type: %d\n",
			vq->type);
		return -EINVAL;
	}

	if (alloc_devs) {
		alloc_devs[0] = ctx->hls->dev;
	}

	if (*nbufs < 1) {
		*nbufs = 1;
	}

	return 0;
}

static int rover_buf_prepare(struct vb2_buffer *vb)
{
	struct rover_hls_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	const struct v4l2_pix_format_mplane *fmt;
	u32 size;

	if (vb->vb2_queue->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		fmt = &ctx->out_fmt;
	} else {
		fmt = &ctx->cap_fmt;
	}

	size = fmt->plane_fmt[0].sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(ctx->hls->dev, "Buffer size too small: %lu < %u\n",
			vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);
	return 0;
}

static void rover_buf_queue(struct vb2_buffer *vb)
{
	struct rover_hls_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	v4l2_m2m_buf_queue(ctx->m2m_ctx, vbuf);
}

static int rover_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct rover_hls_ctx *ctx = vb2_get_drv_priv(vq);

	// initialize HW
	rover_hls_irq_ack(ctx->hls, AP_DONE_INT_STATUS | AP_READY_INT_STATUS);

	rover_hls_debug_dump_full(ctx, __func__);
	return 0;
}

static void rover_stop_streaming(struct vb2_queue *vq)
{
	struct rover_hls_ctx *ctx = vb2_get_drv_priv(vq);
	struct rover_hls *hls = ctx->hls;
	struct v4l2_m2m_ctx *m2m_ctx = ctx->m2m_ctx;
	struct vb2_v4l2_buffer *vbuf;
	unsigned long flags;

	rover_hls_debug_dump_full(ctx, __func__);

	v4l2_m2m_job_finish(hls->m2m_dev, m2m_ctx);

	spin_lock_irqsave(&hls->qlock, flags);
	hls->active = NULL;
	ctx->src_buf = NULL;
	ctx->dst_buf = NULL;
	spin_unlock_irqrestore(&hls->qlock, flags);

	// reset HW
	rover_hls_reset(ctx->hls);

	if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		while ((vbuf = v4l2_m2m_src_buf_remove(m2m_ctx))) {
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
		}
	} else {
		while ((vbuf = v4l2_m2m_dst_buf_remove(m2m_ctx))) {
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
		}
	}
}

static const struct vb2_ops rover_vb2_ops = {
	.queue_setup = rover_queue_setup,
	.buf_prepare = rover_buf_prepare,
	.buf_queue = rover_buf_queue,
	.start_streaming = rover_start_streaming,
	.stop_streaming = rover_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int rover_hls_queue_init(void *priv, struct vb2_queue *src_vq,
				struct vb2_queue *dst_vq)
{
	struct rover_hls_ctx *ctx = priv;
	struct rover_hls *hls = ctx->hls;
	int ret;

	// OUTPUT (source)
	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &rover_vb2_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	src_vq->lock = &hls->lock;
	src_vq->dev = hls->dev;
	src_vq->min_queued_buffers = 1;
	ret = vb2_queue_init(src_vq);
	if (ret) {
		dev_err(hls->dev, "Failed to init source queue: %d\n", ret);
		return ret;
	}

	// CAPTURE (destination)
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &rover_vb2_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	dst_vq->lock = &hls->lock;
	dst_vq->dev = hls->dev;
	dst_vq->min_queued_buffers = 1;
	ret = vb2_queue_init(dst_vq);
	if (ret) {
		dev_err(hls->dev, "Failed to init dest queue: %d\n", ret);
		return ret;
	}

	return 0;
}

/* V4L2 file operations */
static int rover_open(struct file *file)
{
	struct rover_hls *hls = video_drvdata(file);
	struct rover_hls_ctx *ctx;
	const struct rover_hls_fmt *fmt = &rover_hls_fmts[0];

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		dev_err(hls->dev, "Failed to allocate file context\n");
		return -ENOMEM;
	}
	ctx->hls = hls;

	v4l2_fh_init(&ctx->fh, &hls->vdev);
	file->private_data = &ctx->fh;

	ctx->fh.m2m_ctx =
		v4l2_m2m_ctx_init(hls->m2m_dev, ctx, rover_hls_queue_init);
	ctx->m2m_ctx = ctx->fh.m2m_ctx;
	if (IS_ERR(ctx->m2m_ctx)) {
		dev_err(hls->dev, "Failed to initialize mem2mem context\n");
		v4l2_fh_del(&ctx->fh);
		v4l2_fh_exit(&ctx->fh);
		kfree(ctx);
		return PTR_ERR(ctx->m2m_ctx);
	}

	v4l2_fh_add(&ctx->fh);

	u32 in_width, in_height;
	u32 out_width, out_height;
	if (rover_hls_get_input_img_size(hls, &in_width, &in_height) != 0) {
		dev_err(hls->dev,
			"Failed to get max image size, using defaults\n");
		return -EIO;
	}
	if (rover_hls_get_output_img_size(hls, &out_width, &out_height) != 0) {
		dev_err(hls->dev,
			"Failed to get max image size, using defaults\n");
		return -EIO;
	}

	rover_fill_pix_mp(ctx, &ctx->out_fmt, in_width, in_height, fmt);
	rover_fill_pix_mp(ctx, &ctx->cap_fmt, out_width, out_height, fmt);

	return 0;
}

static int rover_release(struct file *file)
{
	struct v4l2_fh *fh = file->private_data;
	struct rover_hls_ctx *ctx = container_of(fh, struct rover_hls_ctx, fh);

	v4l2_m2m_ctx_release(ctx->m2m_ctx);

	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);

	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations rover_fops = {
	.owner = THIS_MODULE,
	.open = rover_open,
	.release = rover_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
	.poll = v4l2_m2m_fop_poll,
};

/* V4L2 mem2mem operations */
static void rover_device_run(void *priv)
{
	struct rover_hls_ctx *ctx = priv;
	struct rover_hls *hls = ctx->hls;
	struct vb2_v4l2_buffer *src, *dst;
	dma_addr_t src_addr, dst_addr;
	phys_addr_t src_pa, dst_pa;
	unsigned long flags;

	src = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	dst = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);

	if (!src || !dst) {
		dev_err(hls->dev, "Source or destination buffer is NULL\n");
		if (src) {
			v4l2_m2m_buf_done(src, VB2_BUF_STATE_ERROR);
		}
		if (dst) {
			v4l2_m2m_buf_done(dst, VB2_BUF_STATE_ERROR);
		}
		v4l2_m2m_job_finish(hls->m2m_dev, ctx->m2m_ctx);
		return;
	}

	src_addr = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0);
	src_pa = dma_to_phys(hls->dev, src_addr);
	if (src_pa == DMA_MAPPING_ERROR) {
		dev_err(hls->dev, "dma_to_phys failed for src dma=%p\n",
			&src_addr);
		v4l2_m2m_buf_done(src, VB2_BUF_STATE_ERROR);
		v4l2_m2m_buf_done(dst, VB2_BUF_STATE_ERROR);
		v4l2_m2m_job_finish(hls->m2m_dev, ctx->m2m_ctx);
		return;
	}

	dst_addr = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	dst_pa = dma_to_phys(hls->dev, dst_addr);
	if (dst_pa == DMA_MAPPING_ERROR) {
		dev_err(hls->dev, "dma_to_phys failed for dst dma=%p\n",
			&dst_addr);
		v4l2_m2m_buf_done(src, VB2_BUF_STATE_ERROR);
		v4l2_m2m_buf_done(dst, VB2_BUF_STATE_ERROR);
		v4l2_m2m_job_finish(hls->m2m_dev, ctx->m2m_ctx);
		return;
	}

	if (src_pa == dst_pa) {
		dev_err(hls->dev,
			"Source and destination addresses are the same\n");
		v4l2_m2m_buf_done(src, VB2_BUF_STATE_ERROR);
		v4l2_m2m_buf_done(dst, VB2_BUF_STATE_ERROR);
		v4l2_m2m_job_finish(hls->m2m_dev, ctx->m2m_ctx);
		return;
	}

	spin_lock_irqsave(&hls->qlock, flags);
	ctx->src_buf = src;
	ctx->dst_buf = dst;
	hls->active = ctx;
	spin_unlock_irqrestore(&hls->qlock, flags);

	rover_hls_program_and_start(ctx->hls, src_pa, dst_pa);
}

static const struct v4l2_m2m_ops rover_m2m_ops = {
	.device_run = rover_device_run,
};

static int rover_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct rover_hls *ctx =
		container_of(ctrl->handler, struct rover_hls, ctrl_hdl);

	switch (ctrl->id) {
	case V4L2_CID_SC_CHANNEL_MODE:
		ctx->mode = ctrl->val;
		rover_hls_set_mode(ctx);
		return 0;
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops rover_ctrl_ops = {
	.s_ctrl = rover_s_ctrl,
};

static irqreturn_t rover_irq_handler(int irq, void *dev_id)
{
	struct rover_hls *hls = dev_id;
	struct rover_hls_ctx *ctx;
	unsigned long flags;

	spin_lock_irqsave(&hls->qlock, flags);
	ctx = hls->active;
	spin_unlock_irqrestore(&hls->qlock, flags);

	if (!ctx) {
		dev_info(hls->dev, "Spurious IRQ: No active context\n");
		return IRQ_HANDLED;
	}

	rover_hls_debug_dump_full(ctx, "IRQ Handler Entry");

	// reset HW
	rover_hls_reset(ctx->hls);

	v4l2_m2m_buf_copy_metadata(ctx->src_buf, ctx->dst_buf, true);
	ctx->dst_buf->vb2_buf.planes[0].bytesused =
		ctx->cap_fmt.plane_fmt[0].sizeimage;

	v4l2_m2m_buf_done(ctx->src_buf, VB2_BUF_STATE_DONE);
	v4l2_m2m_buf_done(ctx->dst_buf, VB2_BUF_STATE_DONE);

	spin_lock_irqsave(&hls->qlock, flags);
	hls->active = NULL;
	ctx->src_buf = NULL;
	ctx->dst_buf = NULL;
	spin_unlock_irqrestore(&hls->qlock, flags);

	v4l2_m2m_job_finish(hls->m2m_dev, ctx->m2m_ctx);
	return IRQ_HANDLED;
}

static void rover_v4l2_dev_unregister(void *data)
{
	struct v4l2_device *v4l2_dev = data;

	v4l2_device_unregister(v4l2_dev);
}

static void rover_v4l2_m2m_dev_unregister(void *data)
{
	struct v4l2_m2m_dev *m2m_dev = data;

	v4l2_m2m_release(m2m_dev);
}

static void rover_ctrls_cleanup(void *data)
{
	struct rover_hls *hls = data;
	v4l2_ctrl_handler_free(&hls->ctrl_hdl);
}

static void rover_vdev_unregister(void *data)
{
	struct video_device *vdev = data;

	video_unregister_device(vdev);
}

static int rover_parse_of_property(struct platform_device *pdev,
				   struct rover_hls *ctx)
{
	struct resource *res;
	int ret, irq;
	u32 mode;

	/* Get and map MMIO resource */
	ctx->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(ctx->base)) {
		dev_err(&pdev->dev, "Failed to map device memory\n");
		return PTR_ERR(ctx->base);
	}

	/* Get IRQ resource */
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

	/* Read optional "sc,mode" property */
	ret = device_property_read_u32(&pdev->dev, "sc,mode", &mode);
	if (ret) {
		ctx->has_mode = false;
	} else {
		ctx->has_mode = true;
		ctx->mode = mode;
		dev_info(&pdev->dev, "optional mode: %u\n", ctx->mode);
	}

	/* Read max image size properties */
	ret = rover_hls_get_max_img_size(ctx, &ctx->max_width,
					 &ctx->max_height);
	if (ret != 0) {
		dev_err(&pdev->dev,
			"Failed to get max image size from HW: %d\n", ret);
		return ret;
	}

	return 0;
}

static int rover_init_v4l2_video_device(struct platform_device *pdev,
					struct rover_hls *ctx)
{
	struct video_device *vdev = &ctx->vdev;
	int ret;

	/* initialize v4l2 ioctl mutex */
	mutex_init(&ctx->lock);
	mutex_init(&ctx->ctrl_lock);
	spin_lock_init(&ctx->qlock);

	/* Setup v4l2 device */
	ret = v4l2_device_register(&pdev->dev, &ctx->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register V4L2 device: %d\n",
			ret);
		return ret;
	}
	devm_add_action_or_reset(&pdev->dev, rover_v4l2_dev_unregister,
				 &ctx->v4l2_dev);

	/* Setup v4l2 mem2mem device */
	ctx->m2m_dev = v4l2_m2m_init(&rover_m2m_ops);
	if (IS_ERR(ctx->m2m_dev)) {
		ret = PTR_ERR(ctx->m2m_dev);
		dev_err(&pdev->dev, "Failed to initialize mem2mem device: %d\n",
			ret);
		return ret;
	}
	devm_add_action_or_reset(&pdev->dev, rover_v4l2_m2m_dev_unregister,
				 &ctx->m2m_dev);

	/* Setup device controls */
	v4l2_ctrl_handler_init(&ctx->ctrl_hdl, 1);
	ctx->ctrl_hdl.lock = &ctx->ctrl_lock;
	{
		struct v4l2_ctrl_config cfg = {
			.ops = &rover_ctrl_ops,
			.id = V4L2_CID_SC_CHANNEL_MODE,
			.name = "hls_mode",
			.type = V4L2_CTRL_TYPE_INTEGER,
			.min = 0,
			.max = 100,
			.step = 1,
			.def = ctx->mode,
		};
		ctx->ctrl_mode =
			v4l2_ctrl_new_custom(&ctx->ctrl_hdl, &cfg, NULL);
	}

	ret = ctx->ctrl_hdl.error;
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize controls: %d\n", ret);
		v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
		return ret;
	}
	devm_add_action_or_reset(&pdev->dev, rover_ctrls_cleanup, ctx);

	ctx->vdev.ctrl_handler = &ctx->ctrl_hdl;

	/* setup video device */
	strscpy(vdev->name, KBUILD_MODNAME, sizeof(vdev->name));
	vdev->release = video_device_release_empty;
	vdev->fops = &rover_fops;
	vdev->ioctl_ops = &rover_ioctl_ops;
	vdev->lock = &ctx->lock;
	vdev->v4l2_dev = &ctx->v4l2_dev;
	vdev->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	vdev->vfl_dir = VFL_DIR_M2M;

	video_set_drvdata(&ctx->vdev, ctx);
	ret = video_register_device(&ctx->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register video device: %d\n",
			ret);
		return ret;
	}
	devm_add_action_or_reset(&pdev->dev, rover_vdev_unregister, &ctx->vdev);

	return 0;
}

static int rover_hls_probe(struct platform_device *pdev)
{
	struct rover_hls *ctx;
	int ret;

	/* Allocate driver context */
	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		dev_err(&pdev->dev,
			"Failed to allocate memory for Rover HLS device\n");
		return -ENOMEM;
	}
	ctx->dev = &pdev->dev;

	/* Parse DeviceTree Properties */
	ret = rover_parse_of_property(pdev, ctx);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to parse device tree properties: %d\n", ret);
		return ret;
	}

	/* Setup v4l2 video device */
	ret = rover_init_v4l2_video_device(pdev, ctx);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to initialize v4l2 video device: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, ctx);
	dev_info(&pdev->dev, "Rover HLS driver probed\n");
	return 0;
}

static void rover_hls_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "Rover HLS driver removed\n");
}

#ifdef CONFIG_OF
static const struct of_device_id of_rover_hls_match[] = {
	{
		.compatible = "sc,sc-hls",
	},
	{},
};
MODULE_DEVICE_TABLE(of, of_rover_hls_match);
#endif

static struct platform_driver rover_hls_driver = {
	.probe = rover_hls_probe,
	.remove = rover_hls_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = of_match_ptr(of_rover_hls_match),
	},
};

module_platform_driver(rover_hls_driver);
