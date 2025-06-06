// SPDX-License-Identifier: GPL-2.0-only
/*
 * Designware SPI core controller driver (refer pxa2xx_spi.c)
 *
 * Copyright (c) 2009, Intel Corporation.
 */

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/preempt.h>
#include <linux/highmem.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/bitfield.h>
#include <linux/time.h>

#include "spi-dw.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

/* Slave spi_device related */
struct chip_data {
	u32 cr0;
	u32 rx_sample_dly;	/* RX sample delay */
};

#ifdef CONFIG_DEBUG_FS

#define DW_SPI_DBGFS_REG(_name, _off)	\
{					\
	.name = _name,			\
	.offset = _off,			\
}

static const struct debugfs_reg32 dw_spi_dbgfs_regs[] = {
	DW_SPI_DBGFS_REG("CTRLR0", DW_SPI_CTRLR0),
	DW_SPI_DBGFS_REG("CTRLR1", DW_SPI_CTRLR1),
	DW_SPI_DBGFS_REG("SSIENR", DW_SPI_SSIENR),
	DW_SPI_DBGFS_REG("SER", DW_SPI_SER),
	DW_SPI_DBGFS_REG("BAUDR", DW_SPI_BAUDR),
	DW_SPI_DBGFS_REG("TXFTLR", DW_SPI_TXFTLR),
	DW_SPI_DBGFS_REG("RXFTLR", DW_SPI_RXFTLR),
	DW_SPI_DBGFS_REG("TXFLR", DW_SPI_TXFLR),
	DW_SPI_DBGFS_REG("RXFLR", DW_SPI_RXFLR),
	DW_SPI_DBGFS_REG("SR", DW_SPI_SR),
	DW_SPI_DBGFS_REG("IMR", DW_SPI_IMR),
	DW_SPI_DBGFS_REG("ISR", DW_SPI_ISR),
	DW_SPI_DBGFS_REG("DMACR", DW_SPI_DMACR),
	DW_SPI_DBGFS_REG("DMATDLR", DW_SPI_DMATDLR),
	DW_SPI_DBGFS_REG("DMARDLR", DW_SPI_DMARDLR),
	DW_SPI_DBGFS_REG("RX_SAMPLE_DLY", DW_SPI_RX_SAMPLE_DLY),
};

static int dw_spi_debugfs_init(struct dw_spi *dws)
{
	char name[32];

	snprintf(name, 32, "dw_spi%d", dws->master->bus_num);
	dws->debugfs = debugfs_create_dir(name, NULL);
	if (!dws->debugfs)
		return -ENOMEM;

	dws->regset.regs = dw_spi_dbgfs_regs;
	dws->regset.nregs = ARRAY_SIZE(dw_spi_dbgfs_regs);
	dws->regset.base = dws->regs;
	debugfs_create_regset32("registers", 0400, dws->debugfs, &dws->regset);

	return 0;
}

static void dw_spi_debugfs_remove(struct dw_spi *dws)
{
	debugfs_remove_recursive(dws->debugfs);
}

#else
static inline int dw_spi_debugfs_init(struct dw_spi *dws)
{
	return 0;
}

static inline void dw_spi_debugfs_remove(struct dw_spi *dws)
{
}
#endif /* CONFIG_DEBUG_FS */

void dw_spi_set_cs(struct spi_device *spi, bool enable)
{
	struct dw_spi *dws = spi_controller_get_devdata(spi->controller);
	bool cs_high = !!(spi->mode & SPI_CS_HIGH);

	/*
	 * DW SPI controller demands any native CS being set in order to
	 * proceed with data transfer. So in order to activate the SPI
	 * communications we must set a corresponding bit in the Slave
	 * Enable register no matter whether the SPI core is configured to
	 * support active-high or active-low CS level.
	 */
	if (cs_high == enable)
		dw_writel(dws, DW_SPI_SER, BIT(spi->chip_select));
	else
		dw_writel(dws, DW_SPI_SER, 0);
}
EXPORT_SYMBOL_GPL(dw_spi_set_cs);

/* Return the max entries we can fill into tx fifo */
static inline u32 tx_max(struct dw_spi *dws)
{
	u32 tx_room, rxtx_gap;

	tx_room = dws->fifo_len - dw_readl(dws, DW_SPI_TXFLR);

	/*
	 * Another concern is about the tx/rx mismatch, we
	 * though to use (dws->fifo_len - rxflr - txflr) as
	 * one maximum value for tx, but it doesn't cover the
	 * data which is out of tx/rx fifo and inside the
	 * shift registers. So a control from sw point of
	 * view is taken.
	 */
	rxtx_gap = dws->fifo_len - (dws->rx_len - dws->tx_len);

	return min3((u32)dws->tx_len, tx_room, rxtx_gap);
}

/* Return the max entries we should read out of rx fifo */
static inline u32 rx_max(struct dw_spi *dws)
{
	return min_t(u32, dws->rx_len, dw_readl(dws, DW_SPI_RXFLR));
}

static void dw_writer(struct dw_spi *dws)
{
	u32 max = tx_max(dws);
	u32 txw = 0;

	while (max--) {
		if (dws->tx) {
			if (dws->n_bytes == 1)
				txw = *(u8 *)(dws->tx);
			else if (dws->n_bytes == 2)
				txw = *(u16 *)(dws->tx);
			else
				txw = *(u32 *)(dws->tx);

			dws->tx += dws->n_bytes;
		}
		dw_write_io_reg(dws, DW_SPI_DR, txw);
		--dws->tx_len;
	}
}

static void dw_reader(struct dw_spi *dws)
{
	u32 max = rx_max(dws);
	u32 rxw;

	while (max--) {
		rxw = dw_read_io_reg(dws, DW_SPI_DR);
		if (dws->rx) {
			if (dws->n_bytes == 1)
				*(u8 *)(dws->rx) = rxw;
			else if (dws->n_bytes == 2)
				*(u16 *)(dws->rx) = rxw;
			else
				*(u32 *)(dws->rx) = rxw;

			dws->rx += dws->n_bytes;
		}
		--dws->rx_len;
	}
}

int dw_spi_check_status(struct dw_spi *dws, bool raw)
{
	u32 irq_status;
	int ret = 0;

	if (raw)
		irq_status = dw_readl(dws, DW_SPI_RISR);
	else
		irq_status = dw_readl(dws, DW_SPI_ISR);

	if (irq_status & SPI_INT_RXOI) {
		dev_err(&dws->master->dev, "RX FIFO overflow detected\n");
		ret = -EIO;
	}

	if (irq_status & SPI_INT_RXUI) {
		dev_err(&dws->master->dev, "RX FIFO underflow detected\n");
		ret = -EIO;
	}

	if (irq_status & SPI_INT_TXOI) {
		dev_err(&dws->master->dev, "TX FIFO overflow detected\n");
		ret = -EIO;
	}

	/* Generically handle the erroneous situation */
	if (ret) {
		spi_reset_chip(dws);
		if (dws->master->cur_msg)
			dws->master->cur_msg->status = ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(dw_spi_check_status);

static irqreturn_t dw_spi_transfer_handler(struct dw_spi *dws)
{
	u16 irq_status = dw_readl(dws, DW_SPI_ISR);

	if (dw_spi_check_status(dws, false)) {
		spi_finalize_current_transfer(dws->master);
		return IRQ_HANDLED;
	}

	/*
	 * Read data from the Rx FIFO every time we've got a chance executing
	 * this method. If there is nothing left to receive, terminate the
	 * procedure. Otherwise adjust the Rx FIFO Threshold level if it's a
	 * final stage of the transfer. By doing so we'll get the next IRQ
	 * right when the leftover incoming data is received.
	 */
	dw_reader(dws);
	if (!dws->rx_len) {
		spi_mask_intr(dws, 0xff);
		spi_finalize_current_transfer(dws->master);
	} else if (dws->rx_len <= dw_readl(dws, DW_SPI_RXFTLR)) {
		dw_writel(dws, DW_SPI_RXFTLR, dws->rx_len - 1);
	}

	/*
	 * Send data out if Tx FIFO Empty IRQ is received. The IRQ will be
	 * disabled after the data transmission is finished so not to
	 * have the TXE IRQ flood at the final stage of the transfer.
	 */
	if (irq_status & SPI_INT_TXEI) {
		dw_writer(dws);
		if (!dws->tx_len)
			spi_mask_intr(dws, SPI_INT_TXEI);
	}

	return IRQ_HANDLED;
}

static irqreturn_t dw_spi_enh_handler(struct dw_spi *dws)
{
        u16 irq_status = dw_readl(dws, DW_SPI_ISR);

        if (dw_spi_check_status(dws, false)) {
                spi_finalize_current_transfer(dws->master);
                return IRQ_HANDLED;
        }

        if (irq_status & SPI_INT_RXFI) {
                dw_reader(dws);
                if (dws->rx_len <= dw_readl(dws, DW_SPI_RXFTLR))
                        dw_writel(dws, DW_SPI_RXFTLR, dws->rx_len - 1);
        }

        if (irq_status & SPI_INT_TXEI){
                dw_writer(dws);
		}

		if (irq_status & SPI_INT_DONE){
			dw_readl(dws, DW_SPI_DONECR);
			dw_writel(dws, DW_SPI_DMACR, 0);
			spi_finalize_current_transfer(dws->master);
		}

        if (!dws->tx_len && dws->rx_len) {
                spi_mask_intr(dws, SPI_INT_TXEI);
        } else if (!dws->rx_len && !dws->tx_len) {
                spi_mask_intr(dws, 0xff);
                spi_finalize_current_transfer(dws->master);
        }
        return IRQ_HANDLED;
}
static irqreturn_t dw_spi_irq(int irq, void *dev_id)
{
	struct spi_controller *master = dev_id;
	struct dw_spi *dws = spi_controller_get_devdata(master);
	u16 irq_status = dw_readl(dws, DW_SPI_ISR) & 0x83f;

	if (!irq_status)
		return IRQ_NONE;

	if (!master->cur_msg && dws->transfer_handler ==
	    dw_spi_transfer_handler) {
		spi_mask_intr(dws, 0xff);
		return IRQ_HANDLED;
	}

	if ((dws->transfer_handler == dw_spi_enh_handler &&
			!dws->rx_len && !dws->tx_len)) {
			spi_mask_intr(dws, 0xff);
			spi_finalize_current_transfer(master);
			return IRQ_HANDLED;
	}
	return dws->transfer_handler(dws);
}

static u32 dw_spi_prepare_cr0(struct dw_spi *dws, struct spi_device *spi)
{
	u32 cr0 = 0;

	if (!(dws->caps & DW_SPI_CAP_DWC_SSI)) {
		/* CTRLR0[ 5: 4] Frame Format */
		cr0 |= SSI_MOTO_SPI << SPI_FRF_OFFSET;

		/*
		 * SPI mode (SCPOL|SCPH)
		 * CTRLR0[ 6] Serial Clock Phase
		 * CTRLR0[ 7] Serial Clock Polarity
		 */
		cr0 |= ((spi->mode & SPI_CPOL) ? 1 : 0) << SPI_SCOL_OFFSET;
		cr0 |= ((spi->mode & SPI_CPHA) ? 1 : 0) << SPI_SCPH_OFFSET;

		/* CTRLR0[11] Shift Register Loop */
		cr0 |= ((spi->mode & SPI_LOOP) ? 1 : 0) << SPI_SRL_OFFSET;
	} else {
		/* CTRLR0[ 7: 6] Frame Format */
		cr0 |= SSI_MOTO_SPI << DWC_SSI_CTRLR0_FRF_OFFSET;

		/*
		 * SPI mode (SCPOL|SCPH)
		 * CTRLR0[ 8] Serial Clock Phase
		 * CTRLR0[ 9] Serial Clock Polarity
		 */
		cr0 |= ((spi->mode & SPI_CPOL) ? 1 : 0) << DWC_SSI_CTRLR0_SCPOL_OFFSET;
		cr0 |= ((spi->mode & SPI_CPHA) ? 1 : 0) << DWC_SSI_CTRLR0_SCPH_OFFSET;

		/* CTRLR0[13] Shift Register Loop */
		cr0 |= ((spi->mode & SPI_LOOP) ? 1 : 0) << DWC_SSI_CTRLR0_SRL_OFFSET;

		if (dws->caps & DW_SPI_CAP_KEEMBAY_MST)
			cr0 |= DWC_SSI_CTRLR0_KEEMBAY_MST;
	}

	return cr0;
}

void dw_spi_update_config(struct dw_spi *dws, struct spi_device *spi,
			  struct dw_spi_cfg *cfg)
{
	struct chip_data *chip = spi_get_ctldata(spi);
	u32 cr0 = chip->cr0;
	u32 speed_hz, spi_ctrlr0;
	u16 clk_div;
	unsigned int hi_sys_config_addr = 0x91585000;
	static char *hi_sys_virt_addr;
	u32 data;


	/* CTRLR0[ 4/3: 0] Data Frame Size */
	cr0 |= (cfg->dfs - 1);

	if (!(dws->caps & DW_SPI_CAP_DWC_SSI))
		/* CTRLR0[ 9:8] Transfer Mode */
		cr0 |= cfg->tmode << SPI_TMOD_OFFSET;
	else
		/* CTRLR0[11:10] Transfer Mode */
		cr0 |= cfg->tmode << DWC_SSI_CTRLR0_TMOD_OFFSET;

	if (dws->caps & DW_SPI_CAP_EXT_SPI) {
		if (cfg->spi_frf) {
			cr0 |= FIELD_PREP(DW_HSSI_CTRLR0_SPI_FRF_MASK,\
				cfg->spi_frf);
			if(cfg->spi_frf == DW_SSI_CTRLR0_SPI_FRF_OCT_SPI){
				// hi_sys_virt_addr = ioremap(hi_sys_config_addr,0x400);
				// data = readl(hi_sys_virt_addr + 0x68);
				// data |= BIT(13);
				// data &= ~ (0xf << 14);
				// writel(data, hi_sys_virt_addr + 0x68);
				// iounmap(hi_sys_virt_addr);
				if(cfg->dtr == true){	
					chip->rx_sample_dly = 5;
				}
			}
		}
		else
			cr0 &= ~DW_HSSI_CTRLR0_SPI_FRF_MASK;
	}
	dw_writel(dws, DW_SPI_CTRLR0, cr0);

	if (cfg->tmode == SPI_TMOD_EPROMREAD || 
	    cfg->tmode == SPI_TMOD_RO || 
	    (cfg->tmode == SPI_TMOD_TO && 
		cfg->spi_frf != DW_SPI_CTRLR0_SPI_FRF_STD_SPI))
			dw_writel(dws, DW_SPI_CTRLR1, cfg->ndf ? cfg->ndf - 1 : 0);

	/* Note DW APB SSI clock divider doesn't support odd numbers */
	clk_div = (DIV_ROUND_UP(dws->max_freq, cfg->freq) + 1) & 0xfffe;
	speed_hz = dws->max_freq / clk_div;

	if (dws->current_freq != speed_hz) {
		spi_set_clk(dws, clk_div);
		dws->current_freq = speed_hz;
	}

	/* Update RX sample delay if required */
	if (dws->cur_rx_sample_dly != chip->rx_sample_dly) {
		dw_writel(dws, DW_SPI_RX_SAMPLE_DLY, chip->rx_sample_dly);
		dws->cur_rx_sample_dly = chip->rx_sample_dly;
	}

	if (cfg->spi_frf != DW_SPI_CTRLR0_SPI_FRF_STD_SPI) {
		spi_ctrlr0 = DW_SPI_SPI_CTRLR0_CLK_STRETCH_EN;
		spi_ctrlr0 |= FIELD_PREP(DW_SPI_SPI_CTRLR0_WAIT_CYCLE_MASK,
					 cfg->wait_c);
		spi_ctrlr0 |= FIELD_PREP(DW_SPI_SPI_CTRLR0_INST_L_MASK,
					 cfg->inst_l);
		spi_ctrlr0 |= FIELD_PREP(DW_SPI_SPI_CTRLR0_ADDR_L_MASK,
					 cfg->addr_l);
		spi_ctrlr0 |= cfg->trans_t;
		if(cfg->dtr == true){
			spi_ctrlr0 |= (0xb << 16);
		}
		dw_writel(dws, DW_SPI_SPI_CTRLR0, spi_ctrlr0);

		dw_writel(dws, DW_SPI_DDR_DRIVE_EDGE, 0);
	}

}
EXPORT_SYMBOL_GPL(dw_spi_update_config);

static void dw_spi_irq_setup(struct dw_spi *dws,
			     irqreturn_t (*t_handler)(struct dw_spi *))
{
	u32 level;
	u32 imask;
	volatile u32 dma_flag = 0;

	if(dw_readl(dws, DW_SPI_DMACR) & 0x4){
		dma_flag = 1;
	}

	/*
	 * Originally Tx and Rx data lengths match. Rx FIFO Threshold level
	 * will be adjusted at the final stage of the IRQ-based SPI transfer
	 * execution so not to lose the leftover of the incoming data.
	 */
	if(!dma_flag){
		level = min_t(unsigned int, dws->fifo_len / 2, dws->tx_len);
		dw_writel(dws, DW_SPI_TXFTLR, level);
	}
	/*
		* In enhanced mode if we are reading then tx_len is 0 as we
		* have nothing to transmit. Calculate DW_SPI_RXFTLR with
		* rx_len.
		*/
	if (t_handler == dw_spi_enh_handler)
			level = min_t(unsigned int, dws->fifo_len / 2, dws->rx_len);
	
	dw_writel(dws, DW_SPI_RXFTLR, level - 1);

	dws->transfer_handler = t_handler;

	if(dma_flag){
		imask = SPI_INT_TXOI | SPI_INT_RXUI | SPI_INT_RXOI | SPI_INT_DONE;
	}else {
		imask = SPI_INT_TXEI | SPI_INT_TXOI | SPI_INT_RXFI | SPI_INT_RXOI;
	}

	spi_umask_intr(dws, imask);
}

/*
 * The iterative procedure of the poll-based transfer is simple: write as much
 * as possible to the Tx FIFO, wait until the pending to receive data is ready
 * to be read, read it from the Rx FIFO and check whether the performed
 * procedure has been successful.
 *
 * Note this method the same way as the IRQ-based transfer won't work well for
 * the SPI devices connected to the controller with native CS due to the
 * automatic CS assertion/de-assertion.
 */
static int dw_spi_poll_transfer(struct dw_spi *dws,
				struct spi_transfer *transfer)
{
	struct spi_delay delay;
	u16 nbits;
	int ret;

	delay.unit = SPI_DELAY_UNIT_SCK;
	nbits = dws->n_bytes * BITS_PER_BYTE;

	do {
		dw_writer(dws);

		delay.value = nbits * (dws->rx_len - dws->tx_len);
		spi_delay_exec(&delay, transfer);

		dw_reader(dws);

		ret = dw_spi_check_status(dws, true);
		if (ret)
			return ret;
	} while (dws->rx_len);

	return 0;
}

static int dw_spi_transfer_one(struct spi_controller *master,
		struct spi_device *spi, struct spi_transfer *transfer)
{
	struct dw_spi *dws = spi_controller_get_devdata(master);
	struct dw_spi_cfg cfg = {
		.tmode = SPI_TMOD_TR,
		.dfs = transfer->bits_per_word,
		.freq = transfer->speed_hz,
		.spi_frf = DW_SPI_CTRLR0_SPI_FRF_STD_SPI,
	};
	int ret;

	dws->dma_mapped = 0;
	dws->n_bytes = DIV_ROUND_UP(transfer->bits_per_word, BITS_PER_BYTE);
	dws->tx = (void *)transfer->tx_buf;
	dws->tx_len = transfer->len / dws->n_bytes;
	dws->rx = transfer->rx_buf;
	dws->rx_len = dws->tx_len;

	/* Ensure the data above is visible for all CPUs */
	smp_mb();

	spi_enable_chip(dws, 0);

	dw_spi_update_config(dws, spi, &cfg);

	transfer->effective_speed_hz = dws->current_freq;

	/* Check if current transfer is a DMA transaction */
	if (master->can_dma && master->can_dma(master, spi, transfer))
		dws->dma_mapped = master->cur_msg_mapped;

	/* For poll mode just disable all interrupts */
	spi_mask_intr(dws, 0xff);

	if (dws->dma_mapped) {
		ret = dws->dma_ops->dma_setup(dws, transfer);
		if (ret)
			return ret;
	}

	spi_enable_chip(dws, 1);

	if (dws->dma_mapped)
		return dws->dma_ops->dma_transfer(dws, transfer);
	else if (dws->irq[0] == IRQ_NOTCONNECTED)
		return dw_spi_poll_transfer(dws, transfer);

	dw_spi_irq_setup(dws, dw_spi_transfer_handler);

	return 1;
}

static void dw_spi_handle_err(struct spi_controller *master,
		struct spi_message *msg)
{
	struct dw_spi *dws = spi_controller_get_devdata(master);

	if (dws->dma_mapped)
		dws->dma_ops->dma_stop(dws);

	spi_reset_chip(dws);
}

static int dw_spi_adjust_enh_mem_op_size(struct spi_mem *mem, struct spi_mem_op *op)
{
	op->data.nbytes = clamp_val(op->data.nbytes, 0, SPI_NDF_MASK + 1);

	return 0;
}

static int dw_spi_adjust_mem_op_size(struct spi_mem *mem, struct spi_mem_op *op)
{
	if (op->data.dir == SPI_MEM_DATA_IN)
		op->data.nbytes = clamp_val(op->data.nbytes, 0, SPI_NDF_MASK + 1);

	return 0;
}

static bool dw_spi_supports_enh_mem_op(struct spi_mem *mem,
                                       const struct spi_mem_op *op)
{
	if (op->addr.nbytes != 0 && op->addr.buswidth != 1 &&
		op->addr.buswidth != op->data.buswidth && op->cmd.buswidth != op->addr.buswidth)
			return false;
	if (op->cmd.buswidth != 1 && op->cmd.buswidth != op->addr.buswidth &&
		op->cmd.buswidth != op->data.buswidth && op->addr.buswidth != op->data.buswidth)
			return false;
	if (op->dummy.nbytes != 0 && op->data.dir == SPI_MEM_DATA_OUT)
			return false;
	if (op->dummy.nbytes != 0 && op->dummy.nbytes / op->dummy.buswidth > 4)
			return false;
	return spi_mem_default_supports_op(mem, op);
}

static bool dw_spi_supports_mem_op(struct spi_mem *mem,
				   const struct spi_mem_op *op)
{
	if (op->data.buswidth > 1 || op->addr.buswidth > 1 ||
	    op->dummy.buswidth > 1 || op->cmd.buswidth > 1)
		return false;

	return spi_mem_default_supports_op(mem, op);
}

static int dw_spi_init_mem_buf(struct dw_spi *dws, const struct spi_mem_op *op)
{
	unsigned int i, j, len;
	u8 *out;

	/*
	 * Calculate the total length of the EEPROM command transfer and
	 * either use the pre-allocated buffer or create a temporary one.
	 */
	len = op->cmd.nbytes + op->addr.nbytes + op->dummy.nbytes;
	if (op->data.dir == SPI_MEM_DATA_OUT)
		len += op->data.nbytes;

	if (len <= SPI_BUF_SIZE) {
		out = dws->buf;
	} else {
		out = kzalloc(len, GFP_KERNEL);
		if (!out)
			return -ENOMEM;
	}

	/*
	 * Collect the operation code, address and dummy bytes into the single
	 * buffer. If it's a transfer with data to be sent, also copy it into the
	 * single buffer in order to speed the data transmission up.
	 */
	for (i = 0; i < op->cmd.nbytes; ++i)
		out[i] = SPI_GET_BYTE(op->cmd.opcode, op->cmd.nbytes - i - 1);

	for (j = 0; j < op->addr.nbytes; ++i, ++j)
		out[i] = SPI_GET_BYTE(op->addr.val, op->addr.nbytes - j - 1);

	for (j = 0; j < op->dummy.nbytes; ++i, ++j)
		out[i] = 0x0;

	if (op->data.dir == SPI_MEM_DATA_OUT)
		memcpy(&out[i], op->data.buf.out, op->data.nbytes);

	dws->n_bytes = 1;
	dws->tx = out;
	dws->tx_len = len;
	if (op->data.dir == SPI_MEM_DATA_IN) {
		dws->rx = op->data.buf.in;
		dws->rx_len = op->data.nbytes;
	} else {
		dws->rx = NULL;
		dws->rx_len = 0;
	}

	return 0;
}

static void dw_spi_free_mem_buf(struct dw_spi *dws)
{
	if (dws->tx != dws->buf)
		kfree(dws->tx);
}

static int dw_spi_write_then_read(struct dw_spi *dws, struct spi_device *spi)
{
	u32 room, entries, sts;
	unsigned int len;
	u8 *buf;

	/*
	 * At initial stage we just pre-fill the Tx FIFO in with no rush,
	 * since native CS hasn't been enabled yet and the automatic data
	 * transmission won't start til we do that.
	 */
	len = min(dws->fifo_len, dws->tx_len);
	buf = dws->tx;
	while (len--)
		dw_write_io_reg(dws, DW_SPI_DR, *buf++);

	/*
	 * After setting any bit in the SER register the transmission will
	 * start automatically. We have to keep up with that procedure
	 * otherwise the CS de-assertion will happen whereupon the memory
	 * operation will be pre-terminated.
	 */
	len = dws->tx_len - ((void *)buf - dws->tx);
	dw_spi_set_cs(spi, false);
	while (len) {
		entries = readl_relaxed(dws->regs + DW_SPI_TXFLR);
		if (!entries) {
			dev_err(&dws->master->dev, "CS de-assertion on Tx\n");
			return -EIO;
		}
		room = min(dws->fifo_len - entries, len);
		for (; room; --room, --len)
			dw_write_io_reg(dws, DW_SPI_DR, *buf++);
	}

	/*
	 * Data fetching will start automatically if the EEPROM-read mode is
	 * activated. We have to keep up with the incoming data pace to
	 * prevent the Rx FIFO overflow causing the inbound data loss.
	 */
	len = dws->rx_len;
	buf = dws->rx;
	while (len) {
		entries = readl_relaxed(dws->regs + DW_SPI_RXFLR);
		if (!entries) {
			sts = readl_relaxed(dws->regs + DW_SPI_RISR);
			if (sts & SPI_INT_RXOI) {
				dev_err(&dws->master->dev, "FIFO overflow on Rx\n");
				return -EIO;
			}
			continue;
		}
		entries = min(entries, len);
		for (; entries; --entries, --len)
			*buf++ = dw_read_io_reg(dws, DW_SPI_DR);
	}

	return 0;
}

static inline bool dw_spi_ctlr_busy(struct dw_spi *dws)
{
	return dw_readl(dws, DW_SPI_SR) & SR_BUSY;
}

static int dw_spi_wait_mem_op_done(struct dw_spi *dws)
{
	int retry = SPI_WAIT_RETRIES;
	struct spi_delay delay;
	unsigned long ns, us;
	u32 nents;

	nents = dw_readl(dws, DW_SPI_TXFLR);
	ns = NSEC_PER_SEC / dws->current_freq * nents;
	ns *= dws->n_bytes * BITS_PER_BYTE;
	if (ns <= NSEC_PER_USEC) {
		delay.unit = SPI_DELAY_UNIT_NSECS;
		delay.value = ns;
	} else {
		us = DIV_ROUND_UP(ns, NSEC_PER_USEC);
		delay.unit = SPI_DELAY_UNIT_USECS;
		delay.value = clamp_val(us, 0, USHRT_MAX);
	}

	while (dw_spi_ctlr_busy(dws) && retry--)
		spi_delay_exec(&delay, NULL);

	if (retry < 0) {
		dev_err(&dws->master->dev, "Mem op hanged up\n");
		return -EIO;
	}

	return 0;
}

static void dw_spi_stop_mem_op(struct dw_spi *dws, struct spi_device *spi)
{
	spi_enable_chip(dws, 0);
	dw_spi_set_cs(spi, true);
	dw_writel(dws, DW_SPI_CTRLR0, 0);
	dw_writel(dws, DW_SPI_TXFTLR, 0);
	dw_writel(dws, DW_SPI_RXFTLR, 0);
	// spi_enable_chip(dws, 1);
}

/*
 * The SPI memory operation implementation below is the best choice for the
 * devices, which are selected by the native chip-select lane. It's
 * specifically developed to workaround the problem with automatic chip-select
 * lane toggle when there is no data in the Tx FIFO buffer. Luckily the current
 * SPI-mem core calls exec_op() callback only if the GPIO-based CS is
 * unavailable.
 */
static int dw_spi_exec_mem_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct dw_spi *dws = spi_controller_get_devdata(mem->spi->controller);
	struct dw_spi_cfg cfg = {0};
	unsigned long flags;
	int ret;
	int i;

	/*
	 * Collect the outbound data into a single buffer to speed the
	 * transmission up at least on the initial stage.
	 */
	ret = dw_spi_init_mem_buf(dws, op);
	if (ret)
		return ret;

	/*
	 * DW SPI EEPROM-read mode is required only for the SPI memory Data-IN
	 * operation. Transmit-only mode is suitable for the rest of them.
	 */
	cfg.dfs = 8;
	cfg.freq = clamp(mem->spi->max_speed_hz, 0U, dws->max_mem_freq);
	if (op->data.dir == SPI_MEM_DATA_IN) {
		cfg.tmode = SPI_TMOD_EPROMREAD;
		cfg.ndf = op->data.nbytes;
	} else {
		cfg.tmode = SPI_TMOD_TO;
	}

	spi_enable_chip(dws, 0);

	dw_spi_update_config(dws, mem->spi, &cfg);

	spi_mask_intr(dws, 0xff);

	spi_enable_chip(dws, 1);

	/*
	 * DW APB SSI controller has very nasty peculiarities. First originally
	 * (without any vendor-specific modifications) it doesn't provide a
	 * direct way to set and clear the native chip-select signal. Instead
	 * the controller asserts the CS lane if Tx FIFO isn't empty and a
	 * transmission is going on, and automatically de-asserts it back to
	 * the high level if the Tx FIFO doesn't have anything to be pushed
	 * out. Due to that a multi-tasking or heavy IRQs activity might be
	 * fatal, since the transfer procedure preemption may cause the Tx FIFO
	 * getting empty and sudden CS de-assertion, which in the middle of the
	 * transfer will most likely cause the data loss. Secondly the
	 * EEPROM-read or Read-only DW SPI transfer modes imply the incoming
	 * data being automatically pulled in into the Rx FIFO. So if the
	 * driver software is late in fetching the data from the FIFO before
	 * it's overflown, new incoming data will be lost. In order to make
	 * sure the executed memory operations are CS-atomic and to prevent the
	 * Rx FIFO overflow we have to disable the local interrupts so to block
	 * any preemption during the subsequent IO operations.
	 *
	 * Note. At some circumstances disabling IRQs may not help to prevent
	 * the problems described above. The CS de-assertion and Rx FIFO
	 * overflow may still happen due to the relatively slow system bus or
	 * CPU not working fast enough, so the write-then-read algo implemented
	 * here just won't keep up with the SPI bus data transfer. Such
	 * situation is highly platform specific and is supposed to be fixed by
	 * manually restricting the SPI bus frequency using the
	 * dws->max_mem_freq parameter.
	 */
	local_irq_save(flags);
	preempt_disable();

	ret = dw_spi_write_then_read(dws, mem->spi);

	local_irq_restore(flags);
	preempt_enable();

	/*
	 * Wait for the operation being finished and check the controller
	 * status only if there hasn't been any run-time error detected. In the
	 * former case it's just pointless. In the later one to prevent an
	 * additional error message printing since any hw error flag being set
	 * would be due to an error detected on the data transfer.
	 */
	if (!ret) {
		ret = dw_spi_wait_mem_op_done(dws);
		if (!ret)
			ret = dw_spi_check_status(dws, true);
	}

	dw_spi_stop_mem_op(dws, mem->spi);

	dw_spi_free_mem_buf(dws);

	return ret;
}

static void dw_spi_init_enh_mem_buf(struct dw_spi *dws, const struct spi_mem_op *op)
{
        unsigned int i, j;
        u8 *out;

        out = dws->buf;
        for (i = 0; i < SPI_BUF_SIZE; ++i)
                out[i] = 0;

        for (i = 0, j = op->cmd.nbytes; i < op->cmd.nbytes; ++i, --j){
                out[i] = SPI_GET_BYTE(op->cmd.opcode, op->cmd.nbytes - j);
		}

        for (j = op->addr.nbytes, i = dws->reg_io_width; j > 0; ++i, --j){
                out[i] = SPI_GET_BYTE(op->addr.val, op->addr.nbytes - j);
		}

        dws->n_bytes = 1;
        if (op->data.dir == SPI_MEM_DATA_IN) {
                dws->rx = op->data.buf.in;
				if(op->data.nbytes % dws->n_bytes == 0){
					dws->rx_len = op->data.nbytes / dws->n_bytes;
				}else{
					dws->rx_len = op->data.nbytes / dws->n_bytes + 1;
				}
				dws->tx = NULL;
                dws->tx_len = 0;
        } else if (op->data.dir == SPI_MEM_DATA_OUT) {
				if(op->data.nbytes % dws->n_bytes == 0){
                	dws->tx_len = op->data.nbytes / dws->n_bytes;
				}else{
					dws->tx_len = op->data.nbytes / dws->n_bytes + 1;
				}
                dws->tx = (void *)op->data.buf.out;
                dws->rx = NULL;
                dws->rx_len = 0;
        } else {
                dws->rx = NULL;
                dws->rx_len = 0;
                dws->tx = NULL;
                dws->tx_len = 0;
        }
}


static void dw_spi_enh_write_cmd_addr(struct dw_spi *dws, const struct spi_mem_op *op)
{
        void *buf = dws->buf;
        u32 txw;
		/* Send cmd as 32 bit value */
        if (buf) {
                txw = *(u32 *)(buf);
                dw_write_io_reg(dws, DW_SPI_DR, txw);
                buf += dws->reg_io_width;
                if (op->addr.nbytes) {
                        txw = *(u32 *)(buf);
                        dw_write_io_reg(dws, DW_SPI_DR, txw);
                        if (op->addr.nbytes > 4) {
                                /* address more than 32bit */
                                buf += dws->reg_io_width;
                                txw = *(u32 *)(buf);
                                dw_write_io_reg(dws, DW_SPI_DR, txw);
                        }
                }
        }
}

static void dw_spi_enh_write_dma_cmd_addr(struct dw_spi *dws, const struct spi_mem_op *op)
{
        void *buf = dws->buf;
        u32 txw;
		/* Send cmd as 32 bit value */
        if (buf) {
                txw = *(u32 *)(buf);
                dw_write_io_reg(dws, DW_SPI_SPIDR, txw);
                buf += dws->reg_io_width;
                if (op->addr.nbytes) {
                        txw = *(u32 *)(buf);
                        dw_write_io_reg(dws, DW_SPI_SPIAR, txw);
                }
        }
}

static void dw_spi_config_read_idma(struct dw_spi *dws, dma_addr_t buf, const struct spi_mem_op *op)
{
    spi_dmacr_u dmacr;
    spi_axiawlen_u axiawlen;
	unsigned char axi_id = 0x0;
	unsigned char axi_len = 15;
    ATW atw = BYTE_1;

    switch( (uint32_t)(uintptr_t)dws->rx % 8 )
    {
        case 0: atw = BYTE_8; break;
        case 4: atw = BYTE_4; break;
        case 2:
        case 6: atw = BYTE_2; break;
        case 1: 
        case 3:
        case 5:
        case 7:
        default: atw = BYTE_1; break;
    }

	dw_spi_enh_write_dma_cmd_addr(dws,op);

    axiawlen.axiawlen.awlen = axi_len;
	dw_writel(dws, DW_SPI_AXIAWLEN, axiawlen.data);

	dw_writel(dws, DW_SPI_AXIAR0, (unsigned long long)buf);
    dw_writel(dws, DW_SPI_AXIAR1, (unsigned long long)buf >> 32);

	dmacr.dmacr.idmae = 1;
    dmacr.dmacr.ainc = 1;
    dmacr.dmacr.atw = atw;
    dmacr.dmacr.aid = axi_id;
    dmacr.dmacr.aprot = 0;
    dmacr.dmacr.acache = 0;
	dw_writel(dws, DW_SPI_DMACR, dmacr.data);

	return;
}

static void dw_spi_config_write_idma(struct dw_spi *dws, dma_addr_t buf, const struct spi_mem_op *op)
{
    spi_dmacr_u dmacr;
    spi_axiarlen_u axiarlen;
	spi_txftlr_u txftlr;
	unsigned char axi_id = 0x0;
	unsigned char axi_len = 15;
    ATW atw = BYTE_1;
	unsigned int mailbox_config_addr = 0x91104000;
	static char *mailbox_virt_addr;
	unsigned int data;

	mailbox_virt_addr = ioremap(mailbox_config_addr,0x1000);
	data = readl(mailbox_virt_addr + 0x2b0);
	data |= 0xf;
	writel(data, mailbox_virt_addr + 0x2b0);
	// pr_err("QOS:%x",readl(mailbox_virt_addr + 0x2b0));
	iounmap(mailbox_virt_addr);	

    switch( (uint32_t)(uintptr_t)dws->tx % 8 )
    {
        case 0: atw = BYTE_8; break;
        case 4: atw = BYTE_4; break;
        case 2:
        case 6: atw = BYTE_2; break;
        case 1: 
        case 3:
        case 5:
        case 7:
        default: atw = BYTE_1; break;
    }

	dw_spi_enh_write_dma_cmd_addr(dws,op);

	txftlr.data = 0;
	if(op->data.nbytes < 64){
		if(op->cmd.dtr == true){
			if(op->data.nbytes % 2 ==0){
				txftlr.txftlr.txfthr = op->data.nbytes - 1;
			}else{
				txftlr.txftlr.txfthr = op->data.nbytes;
			}
		}else{
			txftlr.txftlr.txfthr = op->data.nbytes - 1;
		}
	}else{
		txftlr.txftlr.txfthr = op->data.nbytes / 2;	
	}
	if(txftlr.txftlr.txfthr > 0x7ff){
		txftlr.txftlr.txfthr = 0x7ff;
	}
	dw_writel(dws, DW_SPI_TXFTLR, txftlr.data);
	
    axiarlen.data = 0;
    axiarlen.axiarlen.arlen = axi_len;
	dw_writel(dws, DW_SPI_AXIARLEN, axiarlen.data);

	dw_writel(dws, DW_SPI_AXIAR0, (unsigned long long)buf);
    dw_writel(dws, DW_SPI_AXIAR1, (unsigned long long)buf >> 32);

	dmacr.data = 0;
    dmacr.dmacr.idmae = 1;
    dmacr.dmacr.ainc = 1;
    dmacr.dmacr.atw = atw;
    dmacr.dmacr.aid = axi_id;
    dmacr.dmacr.aprot = 0;
    dmacr.dmacr.acache = 0;
	dw_writel(dws, DW_SPI_DMACR, dmacr.data);
	
	return;
}

static int dw_spi_try_enh_mem_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
        struct spi_controller *ctlr = mem->spi->controller;
        struct dw_spi *dws = spi_controller_get_devdata(ctlr);
		struct dw_spi_cfg cfg;
		int ret = 0;
		unsigned long long ms = 0;
		dma_addr_t dma_handle;
		char *bouncebuf_addr;
		volatile int dma_read = 0,dma_write = 0,cpu_wr = 0;
		int i;

		if(op->cmd.buswidth == 8) {
			cfg.spi_frf = DW_SSI_CTRLR0_SPI_FRF_OCT_SPI;
		}else{
			switch (op->data.buswidth) {
			case 2:
					cfg.spi_frf = DW_SSI_CTRLR0_SPI_FRF_DUAL_SPI;
					break;
			case 4:
					cfg.spi_frf = DW_SSI_CTRLR0_SPI_FRF_QUAD_SPI;
					break;
			case 8:
					cfg.spi_frf = DW_SSI_CTRLR0_SPI_FRF_OCT_SPI;
					break;
			default:
					return dw_spi_exec_mem_op(mem, op);
			}
		}

		if(op->data.dir == SPI_MEM_DATA_OUT){
			dma_write = 1;
		}else if(op->data.dir == SPI_MEM_DATA_IN){
			dma_read = 1;
		}else{
			cpu_wr = 1;
		}

		if(!cpu_wr){
			if (dma_set_coherent_mask(&dws->master->dev, DMA_BIT_MASK(64))) {
				pr_err("mydev: No suitable DMA available\n");
			}
			if(op->cmd.dtr == true){
				if(op->data.nbytes % 2 == 0){
					bouncebuf_addr = dma_alloc_coherent(&dws->master->dev, op->data.nbytes, &dma_handle, GFP_DMA);
				}else{
					bouncebuf_addr = dma_alloc_coherent(&dws->master->dev, op->data.nbytes + 1, &dma_handle, GFP_DMA);
					bouncebuf_addr[op->data.nbytes] = 0xff;
				}
			}else{
				bouncebuf_addr = dma_alloc_coherent(&dws->master->dev, op->data.nbytes, &dma_handle, GFP_DMA);
			}
		}

        /* Collect cmd and addr into a single buffer */
        dw_spi_init_enh_mem_buf(dws, op);

		if(dma_write){
			memcpy(bouncebuf_addr,dws->tx,op->data.nbytes);
		}

        cfg.dfs = 8;
        cfg.freq = clamp(mem->spi->max_speed_hz, 0U, dws->max_mem_freq);
		if(op->data.nbytes % dws->n_bytes == 0){
			if(op->cmd.dtr == true){
				if(op->data.nbytes % 2 == 0){
       			cfg.ndf = op->data.nbytes / dws->n_bytes;
				}else{
				cfg.ndf = op->data.nbytes / dws->n_bytes + 1;
				}
			}else{
				cfg.ndf = op->data.nbytes / dws->n_bytes;
			}
		}else{
			cfg.ndf = op->data.nbytes / dws->n_bytes + 1;
		}
        if (op->data.dir == SPI_MEM_DATA_IN)
                cfg.tmode = SPI_TMOD_RO;
        else
                cfg.tmode = SPI_TMOD_TO;
        if ((op->data.buswidth == op->addr.buswidth &&
            op->data.buswidth == op->cmd.buswidth)||(op->cmd.buswidth == 8))
                cfg.trans_t = DW_SPI_SPI_CTRLR0_TRANS_TYPE_TT2;
		else if (op->data.buswidth == op->addr.buswidth)
                cfg.trans_t = DW_SPI_SPI_CTRLR0_TRANS_TYPE_TT1;
        else
                cfg.trans_t = DW_SPI_SPI_CTRLR0_TRANS_TYPE_TT0;

		if(op->cmd.dtr == true){
			cfg.dtr = true;
		}else{
			cfg.dtr = false;
		}

        cfg.addr_l = clamp(op->addr.nbytes * 2, 0, 0xf);
        if (op->cmd.nbytes > 1)
                cfg.inst_l = DW_SPI_SPI_CTRLR0_INST_L_INST_L16;
        else if (op->cmd.nbytes == 1)
                cfg.inst_l = DW_SPI_SPI_CTRLR0_INST_L_INST_L8;
        else
                cfg.inst_l = DW_SPI_SPI_CTRLR0_INST_L_INST_L0;

        cfg.wait_c = (op->dummy.nbytes * (BITS_PER_BYTE / op->dummy.buswidth));
		
		spi_enable_chip(dws, 0);
        
		dw_spi_update_config(dws, mem->spi, &cfg);

		spi_mask_intr(dws, 0xff);

		reinit_completion(&ctlr->xfer_completion);
		
		if(dma_read){
			dw_spi_config_read_idma(dws, dma_handle, op);
        }else if(dma_write){
			dw_spi_config_write_idma(dws, dma_handle, op);
        }

		dw_spi_irq_setup(dws, dw_spi_enh_handler);

		dw_spi_set_cs(mem->spi, false);
		spi_enable_chip(dws, 1);
		if(cpu_wr){
			dw_spi_enh_write_cmd_addr(dws, op);
		}
		
        /* Use timeout calculation from spi_transfer_wait() */
        ms = 8LL * MSEC_PER_SEC * (dws->rx_len ? dws->rx_len : dws->tx_len);
        do_div(ms, dws->current_freq);

        /*
         * Increase it twice and add 200 ms tolerance, use
         * predefined maximum in case of overflow.
         */
        ms = ms + 200;
        if (ms > UINT_MAX){
                ms = UINT_MAX;
		}
        ms = wait_for_completion_timeout(&ctlr->xfer_completion,
                                         msecs_to_jiffies(ms));
		if (dma_read) {
			memcpy(dws->rx,bouncebuf_addr,op->data.nbytes);
		}
		if(!cpu_wr) {
			dma_free_coherent(&dws->master->dev, op->data.nbytes, bouncebuf_addr, dma_handle);
		}

		dw_spi_stop_mem_op(dws, mem->spi);
        if (ms == 0){
            ret = -EIO;
			
			if(!cpu_wr){
				dw_writel(dws, DW_SPI_DMACR, 0);
			}
			if((op->data.dir == SPI_MEM_DATA_OUT)&&(dw_readl(dws, DW_SPI_RISR) & 0x80))
			{
				dw_readl(dws, DW_SPI_TXOICR);
			}
		}

        return ret;                  
}

static int dw_spi_exec_enh_mem_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
        struct spi_controller *ctlr = mem->spi->controller;
        struct dw_spi *dws = spi_controller_get_devdata(ctlr);
        int retry, ret = -EIO;

        for (retry = 0; retry < SPI_WAIT_RETRIES && ret != 0; retry++){
                ret = dw_spi_try_enh_mem_op(mem, op);

		}
        if (retry == SPI_WAIT_RETRIES)
                dev_err(&dws->master->dev, "Retry of enh_mem_op failed\n");

        return ret;
}


/*
 * Initialize the default memory operations if a glue layer hasn't specified
 * custom ones. Direct mapping operations will be preserved anyway since DW SPI
 * controller doesn't have an embedded dirmap interface. Note the memory
 * operations implemented in this driver is the best choice only for the DW APB
 * SSI controller with standard native CS functionality. If a hardware vendor
 * has fixed the automatic CS assertion/de-assertion peculiarity, then it will
 * be safer to use the normal SPI-messages-based transfers implementation.
 */
static void dw_spi_init_mem_ops(struct dw_spi *dws)
{
	if (!dws->mem_ops.exec_op && !(dws->caps & DW_SPI_CAP_CS_OVERRIDE) &&
	    !dws->set_cs) {
		if (dws->caps & DW_SPI_CAP_EXT_SPI) {
				dws->mem_ops.exec_op = dw_spi_exec_enh_mem_op;
				dws->mem_ops.supports_op = dw_spi_supports_enh_mem_op;
				dws->mem_ops.adjust_op_size = dw_spi_adjust_enh_mem_op_size;
		} else {
				dws->mem_ops.exec_op = dw_spi_exec_mem_op;
				dws->mem_ops.supports_op = dw_spi_supports_mem_op;
				dws->mem_ops.adjust_op_size = dw_spi_adjust_mem_op_size;
		}
		if (!dws->max_mem_freq)
			dws->max_mem_freq = dws->max_freq;
	}
}

/* This may be called twice for each spi dev */
static int dw_spi_setup(struct spi_device *spi)
{
	struct dw_spi *dws = spi_controller_get_devdata(spi->controller);
	struct chip_data *chip;

	/* Only alloc on first setup */
	chip = spi_get_ctldata(spi);
	if (!chip) {
		struct dw_spi *dws = spi_controller_get_devdata(spi->controller);
		u32 rx_sample_dly_ns;

		chip = kzalloc(sizeof(struct chip_data), GFP_KERNEL);
		if (!chip)
			return -ENOMEM;
		spi_set_ctldata(spi, chip);
		/* Get specific / default rx-sample-delay */
		if (device_property_read_u32(&spi->dev,
					     "rx-sample-delay-ns",
					     &rx_sample_dly_ns) != 0)
			/* Use default controller value */
			rx_sample_dly_ns = dws->def_rx_sample_dly_ns;
		chip->rx_sample_dly = DIV_ROUND_CLOSEST(rx_sample_dly_ns,
							NSEC_PER_SEC /
							dws->max_freq);
	}

	/*
	 * Update CR0 data each time the setup callback is invoked since
	 * the device parameters could have been changed, for instance, by
	 * the MMC SPI driver or something else.
	 */
	chip->cr0 = dw_spi_prepare_cr0(dws, spi);

	return 0;
}

static void dw_spi_cleanup(struct spi_device *spi)
{
	struct chip_data *chip = spi_get_ctldata(spi);

	kfree(chip);
	spi_set_ctldata(spi, NULL);
}

/* Restart the controller, disable all interrupts, clean rx fifo */
static void spi_hw_init(struct device *dev, struct dw_spi *dws)
{
	spi_reset_chip(dws);

	/*
	 * Try to detect the FIFO depth if not set by interface driver,
	 * the depth could be from 2 to 256 from HW spec
	 */
	if (!dws->fifo_len) {
		u32 fifo;

		for (fifo = 1; fifo < 256; fifo++) {
			dw_writel(dws, DW_SPI_TXFTLR, fifo);
			if (fifo != dw_readl(dws, DW_SPI_TXFTLR))
				break;
		}
		dw_writel(dws, DW_SPI_TXFTLR, 0);

		dws->fifo_len = (fifo == 1) ? 0 : fifo;
		dev_dbg(dev, "Detected FIFO size: %u bytes\n", dws->fifo_len);
	}

	/* enable HW fixup for explicit CS deselect for Amazon's alpine chip */
	if (dws->caps & DW_SPI_CAP_CS_OVERRIDE)
		dw_writel(dws, DW_SPI_CS_OVERRIDE, 0xF);
}

int dw_spi_add_host(struct device *dev, struct dw_spi *dws)
{
	struct spi_controller *master;
	int ret;
	int i;

	if (!dws)
		return -EINVAL;

	master = spi_alloc_master(dev, 0);
	if (!master)
		return -ENOMEM;

	dws->master = master;
	dws->dma_addr = (dma_addr_t)(dws->paddr + DW_SPI_DR);

	spi_controller_set_devdata(master, dws);

	/* Basic HW init */
	spi_hw_init(dev, dws);

	for (i = 0; i < 16; i++) {
		if (dws->irq[i] < 0)
			break;
		ret = request_irq(dws->irq[i], dw_spi_irq, IRQF_SHARED, dev_name(dev),
				master);
		if (ret < 0 && ret != -ENOTCONN) {
			dev_err(dev, "can not get IRQ\n");
			goto err_free_master;
		}
	}

	dw_spi_init_mem_ops(dws);

	master->use_gpio_descriptors = true;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LOOP;
	master->bits_per_word_mask =  SPI_BPW_RANGE_MASK(5, 32);
	master->bus_num = dws->bus_num;
	master->num_chipselect = dws->num_cs;
	master->setup = dw_spi_setup;
	master->cleanup = dw_spi_cleanup;
	if (dws->set_cs)
		master->set_cs = dws->set_cs;
	else
		master->set_cs = dw_spi_set_cs;
	master->transfer_one = dw_spi_transfer_one;
	master->handle_err = dw_spi_handle_err;
	if (dws->mem_ops.exec_op)
		master->mem_ops = &dws->mem_ops;
	master->max_speed_hz = dws->max_freq;
	master->dev.of_node = dev->of_node;
	master->dev.fwnode = dev->fwnode;
	master->flags = SPI_MASTER_GPIO_SS;
	master->auto_runtime_pm = true;

	/* Get default rx sample delay */
	device_property_read_u32(dev, "rx-sample-delay-ns",
				 &dws->def_rx_sample_dly_ns);

	if (dws->dma_ops && dws->dma_ops->dma_init) {
		ret = dws->dma_ops->dma_init(dev, dws);
		if (ret) {
			dev_warn(dev, "DMA init failed\n");
		} else {
			master->can_dma = dws->dma_ops->can_dma;
			master->flags |= SPI_CONTROLLER_MUST_TX;
		}
	}

	ret = spi_register_controller(master);
	if (ret) {
		dev_err(&master->dev, "problem registering spi master\n");
		goto err_dma_exit;
	}

	dw_spi_debugfs_init(dws);
	return 0;

err_dma_exit:
	if (dws->dma_ops && dws->dma_ops->dma_exit)
		dws->dma_ops->dma_exit(dws);
	spi_enable_chip(dws, 0);
	for (i = 0; i < 16; i++) {
		if (dws->irq[i] < 0)
			break;
		free_irq(dws->irq[i], master);
	}
err_free_master:
	spi_controller_put(master);
	return ret;
}
EXPORT_SYMBOL_GPL(dw_spi_add_host);

void dw_spi_remove_host(struct dw_spi *dws)
{
	int i;

	dw_spi_debugfs_remove(dws);

	spi_unregister_controller(dws->master);

	if (dws->dma_ops && dws->dma_ops->dma_exit)
		dws->dma_ops->dma_exit(dws);

	spi_shutdown_chip(dws);

	for (i = 0; i < 16; i++) {
		if (dws->irq[i] < 0)
			break;
		free_irq(dws->irq[i], dws->master);
	}
}
EXPORT_SYMBOL_GPL(dw_spi_remove_host);

int dw_spi_suspend_host(struct dw_spi *dws)
{
	int ret;

	ret = spi_controller_suspend(dws->master);
	if (ret)
		return ret;

	spi_shutdown_chip(dws);
	return 0;
}
EXPORT_SYMBOL_GPL(dw_spi_suspend_host);

int dw_spi_resume_host(struct dw_spi *dws)
{
	spi_hw_init(&dws->master->dev, dws);
	return spi_controller_resume(dws->master);
}
EXPORT_SYMBOL_GPL(dw_spi_resume_host);

MODULE_AUTHOR("Feng Tang <feng.tang@intel.com>");
MODULE_DESCRIPTION("Driver for DesignWare SPI controller core");
MODULE_LICENSE("GPL v2");
