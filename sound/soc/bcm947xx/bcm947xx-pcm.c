/*
 * ALSA PCM Interface for the Broadcom BCM947XX family of SOCs
 *
 * Copyright (C) 2014, Broadcom Corporation. All Rights Reserved.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * $Id: bcm947xx-pcm.c,v 1.2 2009-11-12 22:25:16 $
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <typedefs.h>
#include <bcmdevs.h>
#include <pcicfg.h>
#include <hndsoc.h>
#include <osl.h>
#include <bcmutils.h>
#include <siutils.h>
#include <sbhnddma.h>
#include <hnddma.h>
#include <i2s_core.h>

#include "bcm947xx-i2s.h"
#include "bcm947xx-pcm.h"


/* Be careful here... turning on prints can break everything, if you start seeing FIFO underflows
 * then it might be due to excessive printing
 */
#define BCM947XX_PCM_DEBUG 0
#if BCM947XX_PCM_DEBUG
#define DBG(x...) printk(KERN_ERR x)
#else
#define DBG(x...)
#endif

/* HND DMA has an empty slot to count.  ALSA will select the highest possible period
 * count that is a power of 2.  For example, if hnddma is configured for 128 descriptors,
 * only 127 will be available.  If 127 periods is reported to ALSA, then ALSA will choose
 * the nearest power of 2, which is 64 periods.
*/
#define BCM947XX_NR_PERIODS_MAX		(BCM947XX_NR_DMA_TXDS_MAX/2)

#define BCM947XX_PERIOD_BYTES_MAX	4096


static const struct snd_pcm_hardware bcm947xx_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
				  /*SNDRV_PCM_INFO_BLOCK_TRANSFER |*/
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_U16_LE |
				  SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S20_3LE |
				  SNDRV_PCM_FMTBIT_S24_LE |
				  SNDRV_PCM_FMTBIT_S24_3LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	.channels_min		= 2,
	.channels_max		= 2,
	.period_bytes_min	= 32,
	.period_bytes_max	= BCM947XX_PERIOD_BYTES_MAX,
	.periods_min		= 2,
	.periods_max		= BCM947XX_NR_PERIODS_MAX,
	.buffer_bytes_max	= BCM947XX_PERIOD_BYTES_MAX * BCM947XX_NR_PERIODS_MAX,
	.fifo_size			= 128, /* unused */
};

struct bcm947xx_runtime_data {
	spinlock_t lock;
	bcm947xx_i2s_info_t *snd_bcm;
	unsigned int dma_loaded;
	unsigned int dma_limit;
	unsigned int dma_period;
	dma_addr_t dma_start;
	dma_addr_t dma_pos;
	dma_addr_t dma_end;
	unsigned long bytes_pending;
	hnddma_t	*di[1];		/* hnddma handles, per fifo */
};



/* Acquire lock before calling me. */
static void bcm947xx_pcm_enqueue(struct snd_pcm_substream *substream)
{
	struct bcm947xx_runtime_data *brtd = substream->runtime->private_data;
	dma_addr_t pos = brtd->dma_pos;
	int ret;

//    DBG("%s dma_loaded: %d dma_limit: %d dma_period: %d\n", __FUNCTION__, brtd->dma_loaded, brtd->dma_limit, brtd->dma_period);
//    DBG("%s: pos %p - dma_start %p - dma_end %p\n", __FUNCTION__, (void *)pos, (void *)brtd->dma_start, (void *)brtd->dma_end);

	snd_BUG_ON(0 == brtd->bytes_pending);
	
	while (brtd->dma_loaded < brtd->dma_limit) {
		unsigned long len = brtd->dma_period;

		if ((pos & ~0xFFF) != (((pos+len - 1) & ~0xFFF))) {
			len = ((pos+len) & ~0xFFF) - pos;
		}

		if ((pos + len) > brtd->dma_end) {
			len  = brtd->dma_end - pos;
		}

		/* Not enough data to fill a period or DMA page, therefore bail until
		 * more data is written to the audio buffer.
		*/
		if (brtd->bytes_pending < len)
			break;

		ret = dma_txunframed(snd_bcm->di[0], (void *)pos, len, TRUE);

		if (ret == 0) {
			pos += len;
			brtd->dma_loaded++;
			brtd->bytes_pending -= len;
			if (pos >= brtd->dma_end)
				pos = brtd->dma_start;
		} else {
			DBG("%s dma_txunframed returned an error\n", __FUNCTION__);
			break;
		}
	}

	brtd->dma_pos = pos;
//	DBG("%s loaded: %p %lu\n", __FUNCTION__, brtd->dma_pos, brtd->dma_loaded);
}


struct snd_pcm_substream *my_stream;


irqreturn_t bcm947xx_i2s_isr(int irq, void *devid)
{
	uint32 intstatus = R_REG(snd_bcm->osh, &snd_bcm->regs->intstatus);
#if BCM947XX_PCM_DEBUG
	uint32 intmask = R_REG(snd_bcm->osh, &snd_bcm->regs->intmask);
#endif
	uint32 intstatus_new = 0;
	uint32 int_errmask = I2S_INT_DESCERR | I2S_INT_DATAERR | I2S_INT_DESC_PROTO_ERR |
	        I2S_INT_RCVFIFO_OFLOW | I2S_INT_XMTFIFO_UFLOW | I2S_INT_SPDIF_PAR_ERR;
	struct bcm947xx_runtime_data *brtd = my_stream->runtime->private_data;

//	DBG("%s enter\n", __FUNCTION__);

	if (intstatus & I2S_INT_XMT_INT) {
		/* reclaim descriptors that have been TX'd */
		spin_lock(&brtd->lock);
		dma_getnexttxp(snd_bcm->di[0], HNDDMA_RANGE_TRANSMITTED);
		spin_unlock(&brtd->lock);

		/* clear this bit by writing a "1" back, we've serviced this */
		intstatus_new |= I2S_INT_XMT_INT;
	}

	if (intstatus & int_errmask) {
		DBG("\n\n%s: Turning off all interrupts due to error\n", __FUNCTION__);
		DBG("%s: intstatus 0x%x intmask 0x%x\n", __FUNCTION__, intstatus, intmask);


		/* something bad happened, turn off all interrupts */
		W_REG(snd_bcm->osh, &snd_bcm->regs->intmask, 0);
	}

	snd_pcm_period_elapsed(my_stream);

	spin_lock(&brtd->lock);
	brtd->dma_loaded--;
	spin_unlock(&brtd->lock);

	W_REG(snd_bcm->osh, &snd_bcm->regs->intstatus, intstatus_new);

//	DBG("%s exit\n", __FUNCTION__);

	return IRQ_RETVAL(intstatus);
}


static int bcm947xx_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct bcm947xx_runtime_data *brtd;

	DBG("%s\n", __FUNCTION__);

	snd_soc_set_runtime_hwparams(substream, &bcm947xx_pcm_hardware);

	brtd = kzalloc(sizeof(struct bcm947xx_runtime_data), GFP_KERNEL);
	if (brtd == NULL) {
		return -ENOMEM;
	}
	brtd->snd_bcm = snd_bcm;

	spin_lock_init(&brtd->lock);

	runtime->private_data = brtd;

	/* probably should put this somewhere else, after setting up isr ??? */
	dma_txreset(snd_bcm->di[0]);
	dma_txinit(snd_bcm->di[0]);

#if BCM947XX_PCM_DEBUG
	DBG("%s: i2s devcontrol 0x%x devstatus 0x%x\n", __FUNCTION__,
	    R_REG(snd_bcm->osh, &snd_bcm->regs->devcontrol),
	    R_REG(snd_bcm->osh, &snd_bcm->regs->devstatus));
	DBG("%s: i2s intstatus 0x%x intmask 0x%x\n", __FUNCTION__,
	    R_REG(snd_bcm->osh, &snd_bcm->regs->intstatus),
	    R_REG(snd_bcm->osh, &snd_bcm->regs->intmask));
	DBG("%s: i2s control 0x%x\n", __FUNCTION__,
	    R_REG(snd_bcm->osh, &snd_bcm->regs->i2scontrol));
	DBG("%s: i2s clkdivider 0x%x txplayth 0x%x\n", __FUNCTION__,
	    R_REG(snd_bcm->osh, &snd_bcm->regs->clkdivider),
	    R_REG(snd_bcm->osh, &snd_bcm->regs->txplayth));
	DBG("%s: i2s stxctrl 0x%x\n", __FUNCTION__,
	    R_REG(snd_bcm->osh, &snd_bcm->regs->stxctrl));

	{
	uint32 temp;
	temp = R_REG(snd_bcm->osh, &snd_bcm->regs->fifocounter);
	DBG("%s: i2s txcnt 0x%x rxcnt 0x%x\n", __FUNCTION__,
	    (temp & I2S_FC_TX_CNT_MASK)>> I2S_FC_TX_CNT_SHIFT,
	    (temp & I2S_FC_RX_CNT_MASK)>> I2S_FC_RX_CNT_SHIFT);
	}
#endif


	return 0;
}

static int bcm947xx_pcm_close(struct snd_pcm_substream *substream)
{
	struct bcm947xx_runtime_data *brtd = substream->runtime->private_data;

	DBG("%s\n", __FUNCTION__);
	
	/* Turn off interrupts... */
	W_REG(snd_bcm->osh, &snd_bcm->regs->intmask,
	      R_REG(snd_bcm->osh, &snd_bcm->regs->intmask) & ~I2S_INT_XMT_INT);

#if BCM947XX_PCM_DEBUG
	{
		/* dump dma rings to console */
#if !defined(FIFOERROR_DUMP_SIZE)
#define FIFOERROR_DUMP_SIZE 8192
#endif
		char *tmp;
		struct bcmstrbuf b;
		if (snd_bcm->di[0] && (tmp = MALLOC(snd_bcm->osh, FIFOERROR_DUMP_SIZE))) {
			bcm_binit(&b, tmp, FIFOERROR_DUMP_SIZE);
			dma_dump(snd_bcm->di[0], &b, TRUE);
			printbig(tmp);
			MFREE(snd_bcm->osh, tmp, FIFOERROR_DUMP_SIZE);
		}
	}
#endif /* BCM947XX_PCM_DEBUG */

	/* reclaim all descriptors */
	dma_txreclaim(snd_bcm->di[0], HNDDMA_RANGE_ALL);

	if (brtd)
		kfree(brtd);
	else
		DBG("%s: called with brtd == NULL\n", __FUNCTION__);

	return 0;
}

static int bcm947xx_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct bcm947xx_runtime_data *brtd = runtime->private_data;
	//struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//struct bcm947xx_pcm_dma_params *dma = rtd->dai->cpu_dai->dma_data;
	unsigned long totbytes = params_buffer_bytes(params);

	int ret = 0;

	DBG("%s\n", __FUNCTION__);

	my_stream = substream;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totbytes;

	spin_lock_irq(&brtd->lock);
	/* Might be called multiple time, so clear out anything
	 * posted to the DMA.
	 */
	if (0 != brtd->dma_loaded) {
		dma_txreclaim(snd_bcm->di[0], HNDDMA_RANGE_ALL);
		dma_txreset(snd_bcm->di[0]);
		dma_txinit(snd_bcm->di[0]);
	}
	brtd->dma_limit = params_periods(params); //runtime->hw.periods_min;
	brtd->dma_period = params_period_bytes(params);
	/* Virtual address of our runtime buffer */
	brtd->dma_start = (dma_addr_t)runtime->dma_area;
	brtd->dma_loaded = 0;
	brtd->dma_pos = brtd->dma_start;
	brtd->dma_end = brtd->dma_start + totbytes;
	brtd->bytes_pending = 0;
	spin_unlock_irq(&brtd->lock);

#if BCM947XX_PCM_DEBUG
	{
	size_t buffer_size = params_buffer_size(params);
	size_t buffer_bytes = params_buffer_bytes(params);
	size_t period_size = params_period_size(params);
	size_t period_bytes = params_period_bytes(params);
	size_t periods = params_periods(params);
//	size_t tick_time = params_tick_time(params);

	DBG("%s: hw.periods_min %d dma_addr %p dma_bytes %d\n",
	    __FUNCTION__, runtime->hw.periods_min, (void *)runtime->dma_addr, runtime->dma_bytes);
	DBG("%s: buffer_size %d buffer_bytes %d\n", __FUNCTION__, buffer_size, buffer_bytes);
	DBG("%s: period_size %d period_bytes %d periods %d\n", __FUNCTION__, period_size, period_bytes, periods);
//	DBG("%s: periods 0x%x tick_time0x%x\n", __FUNCTION__, periods, tick_time);
	}
#endif

	return ret;
}


static int bcm947xx_pcm_hw_free(struct snd_pcm_substream *substream)
{
	//DBG("%s\n", __FUNCTION__);
	snd_pcm_set_runtime_buffer(substream, NULL);

	my_stream = NULL;

	return 0;
}

static int bcm947xx_pcm_prepare(struct snd_pcm_substream *substream)
{
	uint32 intmask = R_REG(snd_bcm->osh, &snd_bcm->regs->intmask);
	int ret = 0;

	DBG("%s\n", __FUNCTION__);


	/* Turn on Tx interrupt */
	W_REG(snd_bcm->osh, &snd_bcm->regs->intmask, intmask | I2S_INT_XMT_INT);

	return ret;
}

static int
bcm947xx_dma_getposition(dma_addr_t *src, dma_addr_t *dst)
{
	if (src) {
		*src = (dma_addr_t)dma_getpos(snd_bcm->di[0], DMA_TX);
	} else if (dst) {
		*dst = (dma_addr_t)dma_getpos(snd_bcm->di[0], DMA_RX);
	}

	return 0;
}

static snd_pcm_uframes_t
bcm947xx_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct bcm947xx_runtime_data *brtd = runtime->private_data;
	unsigned long res;
	dma_addr_t pos = 0;

	spin_lock(&brtd->lock);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		bcm947xx_dma_getposition(NULL, &pos);
	} else {
		bcm947xx_dma_getposition(&pos, NULL);
	}

	if ((void *)pos == NULL)
		res = 0; /* DMA not running? */
	else {
		res = pos - brtd->dma_start;
		//DBG("%s: pos %p - dma_start %p = 0x%x\n", __FUNCTION__, pos, brtd->dma_start, res);
	}

	spin_unlock(&brtd->lock);

	return bytes_to_frames(substream->runtime, res);
}

static int bcm947xx_pcm_copy(struct snd_pcm_substream *substream, int channel,
        snd_pcm_uframes_t pos, void *src, snd_pcm_uframes_t count)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct bcm947xx_runtime_data *brtd = my_stream->runtime->private_data;
	char *hwbuf = runtime->dma_area + frames_to_bytes(runtime, pos);

	if (copy_from_user(hwbuf, src, frames_to_bytes(runtime, count)))
		return -EFAULT;

	spin_lock_irq(&brtd->lock);
	brtd->bytes_pending += frames_to_bytes(runtime, count);
	bcm947xx_pcm_enqueue(substream);
	spin_unlock_irq(&brtd->lock);

	return 0;
}

struct snd_pcm_ops bcm947xx_pcm_ops = {
	.open		= bcm947xx_pcm_open,
	.close		= bcm947xx_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= bcm947xx_pcm_hw_params,
	.hw_free	= bcm947xx_pcm_hw_free,
	.prepare	= bcm947xx_pcm_prepare,
	.pointer	= bcm947xx_pcm_pointer,
	.copy		= bcm947xx_pcm_copy,
};


static int bcm947xx_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{

	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = bcm947xx_pcm_hardware.buffer_bytes_max;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = kmalloc(size, GFP_ATOMIC);
	DBG("%s: size %d @ 0x%p\n", __FUNCTION__, size, buf->area);

	if (!buf->area) {
		DBG("%s: dma_alloc failed\n", __FUNCTION__);
		return -ENOMEM;
	}
	buf->bytes = size;

	return 0;
}

static void bcm947xx_pcm_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	DBG("%s\n", __FUNCTION__);

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		kfree(buf->area);
		buf->area = NULL;
	}

	free_irq(snd_bcm->irq, snd_bcm);

}


static u64 bcm947xx_pcm_dmamask = DMA_BIT_MASK(32);

int bcm947xx_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
	struct snd_pcm *pcm)
{
	int ret = 0;

	DBG("%s\n", __FUNCTION__);

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &bcm947xx_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	if (dai->playback.channels_min) {
		ret = bcm947xx_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if ((request_irq(snd_bcm->irq,
	                 bcm947xx_i2s_isr, IRQF_SHARED, "i2s", snd_bcm)) < 0) {
		DBG("%s: request_irq failure\n", __FUNCTION__);
	}


 out:
	return ret;
}


struct snd_soc_platform bcm947xx_soc_platform = {
	.name		= "bcm947xx-audio",
	.pcm_ops 	= &bcm947xx_pcm_ops,
	.pcm_new	= bcm947xx_pcm_new,
	.pcm_free	= bcm947xx_pcm_free,
};

EXPORT_SYMBOL_GPL(bcm947xx_soc_platform);

static int __init bcm947xx_soc_platform_init(void)
{
	return snd_soc_register_platform(&bcm947xx_soc_platform);
}
module_init(bcm947xx_soc_platform_init);

static void __exit bcm947xx_soc_platform_exit(void)
{
	snd_soc_unregister_platform(&bcm947xx_soc_platform);
}
module_exit(bcm947xx_soc_platform_exit);

MODULE_LICENSE("GPL");
/* MODULE_AUTHOR(""); */
MODULE_DESCRIPTION("BCM947XX PCM module");
