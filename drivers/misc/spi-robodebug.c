/*
 * Broadcom RoboSwitch debug driver.
 * Copyright (C) 2012 The IMS Company.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/mii.h>

/* data structures **********************************************************/
struct robodebug_data {
	struct spi_device	*spi;

	/* currently selected register page */
	struct mutex	page_lock;
	int		page;

	/* mib statistics */
	struct mutex	mib_snapshot_lock;

	/* sysfs attribute values */
	int		reg_page;
	int		reg_addr;
	int		spi_debug;
};


/* SPI low-level I/O ********************************************************/
static int
robodebug_spi_xfer(struct robodebug_data *robodebug, u8 *tx, u8 *rx, int len)
{
	struct spi_message msg = { };
	struct spi_transfer xfer = {
		.rx_buf = rx,
		.tx_buf = tx,
		.len = len,
		.speed_hz = 2000000,
	};
	int ret;

	if (robodebug->spi_debug) {
		int i;

		printk(KERN_INFO "robodebug:");
		for (i = 0; i < len; i++)
			printk(" %.2x", tx[i]);
	}

	spi_message_init(&msg);

	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(robodebug->spi, &msg);

	if (robodebug->spi_debug) {
		if (ret == 0) {
			int i;

			printk(", reply");
			for (i = 0; i < len; i++)
				printk(" %.2x", rx[i]);
			printk("\n");
		} else {
			printk(", spi_sync: error %d\n", ret);
		}
	}

	return ret;
}

static int robodebug_read_reg(struct robodebug_data *robodebug,
			      int reg, int bits, u64 *_val)
{
	uint8_t msg[10];
	int ret;
	u64 val;

	msg[0] = 0x60;
	msg[1] = reg;
	msg[2] = 0x00;
	msg[3] = 0x00;
	msg[4] = 0x00;
	msg[5] = 0x00;
	msg[6] = 0x00;
	msg[7] = 0x00;
	msg[8] = 0x00;
	msg[9] = 0x00;

	ret = robodebug_spi_xfer(robodebug, msg, msg, 2 + (bits / 8));
	if (ret < 0)
		return ret;

	val = msg[2];
	if (bits > 8)
		val |= msg[3] << 8;
	if (bits > 16)
		val |= msg[4] << 16;
	if (bits > 24)
		val |= msg[5] << 24;
	if (bits > 32)
		val |= ((u64)msg[6]) << 32;
	if (bits > 40)
		val |= ((u64)msg[7]) << 40;
	if (bits > 48)
		val |= ((u64)msg[8]) << 48;
	if (bits > 56)
		val |= ((u64)msg[9]) << 56;
	*_val = val;

	return 0;
}

static int robodebug_fastread_reg(struct robodebug_data *robodebug,
				  int reg, int bits, u64 *_val)
{
	uint8_t msg[11];
	int ret;
	u64 val;

	msg[0] = 0x10;
	msg[1] = reg;
	msg[2] = 0x00;
	msg[3] = 0x00;
	msg[4] = 0x00;
	msg[5] = 0x00;
	msg[6] = 0x00;
	msg[7] = 0x00;
	msg[8] = 0x00;
	msg[9] = 0x00;
	msg[10] = 0x00;

	ret = robodebug_spi_xfer(robodebug, msg, msg, 3 + (bits / 8));
	if (ret < 0)
		return ret;

	if ((msg[2] & 0x03) == 0x00)
		return -ETIME;

	val = msg[3];
	if (bits > 8)
		val |= msg[4] << 8;
	if (bits > 16)
		val |= msg[5] << 16;
	if (bits > 24)
		val |= msg[6] << 24;
	if (bits > 32)
		val |= ((u64)msg[7]) << 32;
	if (bits > 40)
		val |= ((u64)msg[8]) << 40;
	if (bits > 48)
		val |= ((u64)msg[9]) << 48;
	if (bits > 56)
		val |= ((u64)msg[10]) << 56;
	*_val = val;

	return 0;
}

static int robodebug_write_reg(struct robodebug_data *robodebug,
			       int reg, int bits, u64 val)
{
	uint8_t msg[10];

	msg[0] = 0x61;
	msg[1] = reg;
	msg[2] = val & 0xff;
	msg[3] = (val >> 8) & 0xff;
	msg[4] = (val >> 16) & 0xff;
	msg[5] = (val >> 24) & 0xff;
	msg[6] = (val >> 32) & 0xff;
	msg[7] = (val >> 40) & 0xff;
	msg[8] = (val >> 48) & 0xff;
	msg[9] = (val >> 56) & 0xff;

	return robodebug_spi_xfer(robodebug, msg, msg, 2 + (bits / 8));
}


/* SPI low-level I/O ********************************************************/
#define ROBO_DATA_REG		0xf0
#define ROBO_STATUS_REG		0xfe
#define  ROBO_STATUS_SPIF	 0x80
#define  ROBO_STATUS_RACK	 0x20
#define ROBO_PAGE_REG		0xff

static int robodebug_spif_wait(struct robodebug_data *robodebug)
{
	int i;

	for (i = 0; i < 10; i++) {
		u64 val;
		int ret;

		ret = robodebug_read_reg(robodebug, ROBO_STATUS_REG, 8, &val);
		if (ret < 0)
			return ret;

		if ((val & (ROBO_STATUS_SPIF | ROBO_STATUS_RACK)) == 0)
			return 0;

		if ((val & ROBO_STATUS_RACK) == ROBO_STATUS_RACK)
			robodebug_read_reg(robodebug, ROBO_DATA_REG, 8, &val);

		msleep(1);
	}

	return -ETIMEDOUT;
}

static int robodebug_rack_wait(struct robodebug_data *robodebug)
{
	u8 spif_rack = ROBO_STATUS_SPIF | ROBO_STATUS_RACK;
	int i;

	for (i = 0; i < 10; i++) {
		u64 val;
		int ret;

		ret = robodebug_read_reg(robodebug, ROBO_STATUS_REG, 8, &val);
		if (ret < 0)
			return ret;

		if ((val & spif_rack) == spif_rack)
			return 0;

		msleep(1);
	}

	return -ETIMEDOUT;
}

static int robodebug_set_page(struct robodebug_data *robodebug, int page)
{
	int ret;

	ret = 0;
	if (robodebug->page != page) {
		ret = robodebug_write_reg(robodebug, ROBO_PAGE_REG, 8, page);
		if (!ret)
			robodebug->page = page;
	}

	return ret;
}

static int robodebug_read_page_reg(struct robodebug_data *robodebug,
				   int page, int reg, int bits, u64 *val)
{
	int ret;

	mutex_lock(&robodebug->page_lock);

	ret = robodebug_spif_wait(robodebug);
	if (ret == 0)
		ret = robodebug_set_page(robodebug, page);
	if (ret == 0)
		ret = robodebug_read_reg(robodebug, reg, 8, val);
	if (ret == 0)
		ret = robodebug_rack_wait(robodebug);
	if (ret == 0)
		ret = robodebug_read_reg(robodebug, ROBO_DATA_REG, bits, val);

	if (ret)
		pr_err("robodebug_read_page_reg: %d\n", ret);

	mutex_unlock(&robodebug->page_lock);

	return ret;
}

static int robodebug_fastread_page_reg(struct robodebug_data *robodebug,
				       int page, int reg, int bits, u64 *val)
{
	int ret;

	mutex_lock(&robodebug->page_lock);

	ret = robodebug_spif_wait(robodebug);
	if (ret == 0)
		ret = robodebug_set_page(robodebug, page);
	if (ret == 0)
		ret = robodebug_fastread_reg(robodebug, reg, bits, val);

	mutex_unlock(&robodebug->page_lock);

	if (ret == -ETIME) {
		printk(KERN_WARNING "robodebug_fastread_page_reg: fast read "
				    "failed, falling back to slow read\n");
		ret = robodebug_read_page_reg(robodebug, page, reg, bits, val);
	}

	return ret;
}

static int robodebug_write_page_reg(struct robodebug_data *robodebug,
				    int page, int reg, int bits, u64 val)
{
	int ret;

	mutex_lock(&robodebug->page_lock);

	ret = robodebug_spif_wait(robodebug);
	if (ret >= 0)
		ret = robodebug_set_page(robodebug, page);
	if (ret >= 0)
		ret = robodebug_write_reg(robodebug, reg, bits, val);

	mutex_unlock(&robodebug->page_lock);

	return ret;
}


/* misc functions ***********************************************************/
static ssize_t
robodebug_switch_model(struct robodebug_data *robodebug, char *buf, int len)
{
	int ret;
	u64 id;
	u64 rev;

	ret = robodebug_read_page_reg(robodebug, 0x02, 0x30, 8, &id);
	if (ret < 0)
		return ret;

	ret = robodebug_read_page_reg(robodebug, 0x02, 0x40, 8, &rev);
	if (ret < 0)
		return ret;

	if (id == 0x95)
		return snprintf(buf, len, "Broadcom BCM5395 (%d)", (int)rev);

	return snprintf(buf, len, "Unknown (ID: %d, Rev: %d)", (int)id, (int)rev);
}


/* init functions ***********************************************************/
static int
robodebug_switch_init(struct robodebug_data *robodebug)
{
	int ret;
	
	ret = robodebug_write_page_reg(robodebug, 0x00, 0x2f, 8, 0x00);
	if (ret < 0)
		return ret;
	return 0;
}


/* sysfs handling ***********************************************************/
static ssize_t atu_flush_set(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);
	u64 val;
	int ret;
	int i;
	u64 val2;

	ret = robodebug_read_page_reg(robodebug, 0x00, 0x88, 8, &val);
	if (ret < 0)
		return ret;

	ret = robodebug_write_page_reg(robodebug, 0x00, 0x88, 8, val | 0xa2);
	if (ret < 0)
		return ret;

	for (i = 0; i < 100; i++) {
		ret = robodebug_read_page_reg(robodebug, 0x00, 0x88, 8, &val2);
		if (ret < 0)
			return ret;

		if ((val2 & 0x80) == 0x00)
			break;

		msleep(1);
	}

	if (i == 100)
		return -ETIMEDOUT;

	val &= ~0x80;
	if (val != val2) {
		ret = robodebug_write_page_reg(robodebug, 0x00, 0x88, 8, val);
		if (ret < 0)
			return ret;
	}

	return count;
}

static DEVICE_ATTR(atu_flush, S_IWUSR, NULL, atu_flush_set);

static int
port_restart_aneg(struct robodebug_data *robodebug, int port, int count)
{
	u64 val;
	int ret;

	ret = robodebug_read_page_reg(robodebug, 0x10 + port, 0x00, 16, &val);
	if (ret < 0)
		return ret;

	val |= BMCR_ANENABLE | BMCR_ANRESTART;
	val &= ~BMCR_ISOLATE;

	ret = robodebug_write_page_reg(robodebug, 0x10 + port, 0x00, 16, val);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t port0_restart_aneg_set(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return port_restart_aneg(robodebug, 0, count);
}

static DEVICE_ATTR(port0_restart_aneg, S_IWUSR, NULL, port0_restart_aneg_set);

static ssize_t port1_restart_aneg_set(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return port_restart_aneg(robodebug, 1, count);
}

static DEVICE_ATTR(port1_restart_aneg, S_IWUSR, NULL, port1_restart_aneg_set);

static ssize_t port2_restart_aneg_set(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return port_restart_aneg(robodebug, 2, count);
}

static DEVICE_ATTR(port2_restart_aneg, S_IWUSR, NULL, port2_restart_aneg_set);

static ssize_t port3_restart_aneg_set(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return port_restart_aneg(robodebug, 3, count);
}

static DEVICE_ATTR(port3_restart_aneg, S_IWUSR, NULL, port3_restart_aneg_set);

static ssize_t port4_restart_aneg_set(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return port_restart_aneg(robodebug, 4, count);
}

static DEVICE_ATTR(port4_restart_aneg, S_IWUSR, NULL, port4_restart_aneg_set);

#define STATS32(off) \
({ \
	u64 val; \
	int ret; \
 \
	ret = robodebug_fastread_page_reg(robodebug, 0x71, off, 32, &val); \
	if (ret < 0) { \
		mutex_unlock(&robodebug->mib_snapshot_lock); \
		return ret; \
	} \
 \
	val; \
})

#define STATS64(off) \
({ \
	u64 val; \
	int ret; \
 \
	ret = robodebug_fastread_page_reg(robodebug, 0x71, off, 64, &val); \
	if (ret < 0) { \
		mutex_unlock(&robodebug->mib_snapshot_lock); \
		return ret; \
	} \
 \
	val; \
})

static ssize_t
port_statistics_show(struct robodebug_data *robodebug, char *buf, int port)
{
	int ret;
	int i;
	u64 tx_octets;
	u32 tx_drop_pkts;
	u32 tx_q0_pkts;
	u32 tx_broadcast_pkts;
	u32 tx_multicast_pkts;
	u32 tx_unicast_pkts;
	u32 tx_collisions;
	u32 tx_single_collision;
	u32 tx_multi_collision;
	u32 tx_deferred_transmit;
	u32 tx_late_collision;
	u32 tx_excessive_collision;
	u32 tx_frame_in_discards;
	u32 tx_pause_pkts;
	u32 tx_q1_pkts;
	u32 tx_q2_pkts;
	u32 tx_q3_pkts;
	u64 rx_octets;
	u32 rx_undersize_pkts;
	u32 rx_pause_pkts;
	u32 rx_pkts_64_octets;
	u32 rx_pkts_64_to_127_octets;
	u32 rx_pkts_128_to_255_octets;
	u32 rx_pkts_256_to_511_octets;
	u32 rx_pkts_512_to_1023_octets;
	u32 rx_pkts_1024_to_1522_octets;
	u32 rx_oversize_pkts;
	u32 rx_jabbers;
	u32 rx_alignment_errors;
	u32 rx_fcs_errors;
	u64 rx_good_octets;
	u32 rx_drop_pkts;
	u32 rx_unicast_pkts;
	u32 rx_multicast_pkts;
	u32 rx_broadcast_pkts;
	u32 rx_sa_changes;
	u32 rx_fragments;
	u32 rx_excess_size_disc;
	u32 rx_symbol_error;
	u32 rx_discard;

    if (!robodebug) 
        return -ENODATA;

	mutex_lock(&robodebug->mib_snapshot_lock);

	ret = robodebug_write_page_reg(robodebug, 0x70, 0x00, 8, 0x80 | port);
	if (ret < 0) {
		mutex_unlock(&robodebug->mib_snapshot_lock);
		return ret;
	}

	for (i = 0; i < 100; i++) {
		u64 val;

		ret = robodebug_read_page_reg(robodebug, 0x70, 0x00, 8, &val);
		if (ret < 0) {
			mutex_unlock(&robodebug->mib_snapshot_lock);
			return ret;
		}

		if ((val & 0x80) == 0x00)
			break;

		msleep(1);
	}

	if (i == 100) {
		mutex_unlock(&robodebug->mib_snapshot_lock);
		return -ETIMEDOUT;
	}

	tx_octets = STATS64(0x00);
	tx_drop_pkts = STATS32(0x08);
	tx_q0_pkts = STATS32(0x0c);
	tx_broadcast_pkts = STATS32(0x10);
	tx_multicast_pkts = STATS32(0x14);
	tx_unicast_pkts = STATS32(0x18);
	tx_collisions = STATS32(0x1c);
	tx_single_collision = STATS32(0x20);
	tx_multi_collision = STATS32(0x24);
	tx_deferred_transmit = STATS32(0x28);
	tx_late_collision = STATS32(0x2c);
	tx_excessive_collision = STATS32(0x30);
	tx_frame_in_discards = STATS32(0x34);
	tx_pause_pkts = STATS32(0x38);
	tx_q1_pkts = STATS32(0x3c);
	tx_q2_pkts = STATS32(0x40);
	tx_q3_pkts = STATS32(0x44);
	rx_octets = STATS64(0x50);
	rx_undersize_pkts = STATS32(0x58);
	rx_pause_pkts = STATS32(0x5c);
	rx_pkts_64_octets = STATS32(0x60);
	rx_pkts_64_to_127_octets = STATS32(0x64);
	rx_pkts_128_to_255_octets = STATS32(0x68);
	rx_pkts_256_to_511_octets = STATS32(0x6c);
	rx_pkts_512_to_1023_octets = STATS32(0x70);
	rx_pkts_1024_to_1522_octets = STATS32(0x74);
	rx_oversize_pkts = STATS32(0x78);
	rx_jabbers = STATS32(0x7c);
	rx_alignment_errors = STATS32(0x80);
	rx_fcs_errors = STATS32(0x84);
	rx_good_octets = STATS64(0x88);
	rx_drop_pkts = STATS32(0x90);
	rx_unicast_pkts = STATS32(0x94);
	rx_multicast_pkts = STATS32(0x98);
	rx_broadcast_pkts = STATS32(0x9c);
	rx_sa_changes = STATS32(0xa0);
	rx_fragments = STATS32(0xa4);
	rx_excess_size_disc = STATS32(0xa8);
	rx_symbol_error = STATS32(0xac);
	rx_discard = STATS32(0xc0);

	mutex_unlock(&robodebug->mib_snapshot_lock);

	return sprintf(buf,
		"Statistics for port %d:\n"
		"     tx_octets: %Lu\n"
		"     tx_drop_pkts: %u\n"
		"     tx_q0_pkts: %u\n"
		"     tx_broadcast_pkts: %u\n"
		"     tx_multicast_pkts: %u\n"
		"     tx_unicast_pkts: %u\n"
		"     tx_collisions: %u\n"
		"     tx_single_collision: %u\n"
		"     tx_multi_collision: %u\n"
		"     tx_deferred_transmit: %u\n"
		"     tx_late_collision: %u\n"
		"     tx_excessive_collision: %u\n"
		"     tx_frame_in_discards: %u\n"
		"     tx_pause_pkts: %u\n"
		"     tx_q1_pkts: %u\n"
		"     tx_q2_pkts: %u\n"
		"     tx_q3_pkts: %u\n"
		"     rx_octets: %Lu\n"
		"     rx_undersize_pkts: %u\n"
		"     rx_pause_pkts: %u\n"
		"     rx_pkts_64_octets: %u\n"
		"     rx_pkts_64_to_127_octets: %u\n"
		"     rx_pkts_128_to_255_octets: %u\n"
		"     rx_pkts_256_to_511_octets: %u\n"
		"     rx_pkts_512_to_1023_octets: %u\n"
		"     rx_pkts_1024_to_1522_octets: %u\n"
		"     rx_oversize_pkts: %u\n"
		"     rx_jabbers: %u\n"
		"     rx_alignment_errors: %u\n"
		"     rx_fcs_errors: %u\n"
		"     rx_good_octets: %Lu\n"
		"     rx_drop_pkts: %u\n"
		"     rx_unicast_pkts: %u\n"
		"     rx_multicast_pkts: %u\n"
		"     rx_broadcast_pkts: %u\n"
		"     rx_sa_changes: %u\n"
		"     rx_fragments: %u\n"
		"     rx_excess_size_disc: %u\n"
		"     rx_symbol_error: %u\n"
		"     rx_discard: %u\n",
			port,
			tx_octets,
			tx_drop_pkts,
			tx_q0_pkts,
			tx_broadcast_pkts,
			tx_multicast_pkts,
			tx_unicast_pkts,
			tx_collisions,
			tx_single_collision,
			tx_multi_collision,
			tx_deferred_transmit,
			tx_late_collision,
			tx_excessive_collision,
			tx_frame_in_discards,
			tx_pause_pkts,
			tx_q1_pkts,
			tx_q2_pkts,
			tx_q3_pkts,
			rx_octets,
			rx_undersize_pkts,
			rx_pause_pkts,
			rx_pkts_64_octets,
			rx_pkts_64_to_127_octets,
			rx_pkts_128_to_255_octets,
			rx_pkts_256_to_511_octets,
			rx_pkts_512_to_1023_octets,
			rx_pkts_1024_to_1522_octets,
			rx_oversize_pkts,
			rx_jabbers,
			rx_alignment_errors,
			rx_fcs_errors,
			rx_good_octets,
			rx_drop_pkts,
			rx_unicast_pkts,
			rx_multicast_pkts,
			rx_broadcast_pkts,
			rx_sa_changes,
			rx_fragments,
			rx_excess_size_disc,
			rx_symbol_error,
			rx_discard);
}

static ssize_t port0_statistics_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return port_statistics_show(robodebug, buf, 0);
}

static DEVICE_ATTR(port0_statistics, S_IRUSR, port0_statistics_show, NULL);

static ssize_t port1_statistics_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return port_statistics_show(robodebug, buf, 1);
}

static DEVICE_ATTR(port1_statistics, S_IRUSR, port1_statistics_show, NULL);

static ssize_t port2_statistics_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return port_statistics_show(robodebug, buf, 2);
}

static DEVICE_ATTR(port2_statistics, S_IRUSR, port2_statistics_show, NULL);

static ssize_t port3_statistics_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return port_statistics_show(robodebug, buf, 3);
}

static DEVICE_ATTR(port3_statistics, S_IRUSR, port3_statistics_show, NULL);

static ssize_t port4_statistics_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return port_statistics_show(robodebug, buf, 4);
}

static DEVICE_ATTR(port4_statistics, S_IRUSR, port4_statistics_show, NULL);

static ssize_t
port_status_show(struct robodebug_data *robodebug, char *buf, int port)
{
	int ret;
	u64 link;
	u64 speed;
	u64 duplex;
	u64 pause;

    if (!robodebug)
        return -ENODATA;

	ret = robodebug_read_page_reg(robodebug, 0x01, 0x00, 16, &link);
	if (ret < 0)
		return ret;

	link = (link >> port) & 1;
	if (!link)
		return sprintf(buf, "Settings for port %d:\n"
				    "\tLink detected: no\n", port);

	ret = robodebug_read_page_reg(robodebug, 0x01, 0x04, 32, &speed);
	if (ret < 0)
		return ret;

	speed = (speed >> (2 * port)) & 3;

	ret = robodebug_read_page_reg(robodebug, 0x01, 0x08, 16, &duplex);
	if (ret < 0)
		return ret;

	duplex = (duplex >> port) & 1;

	ret = robodebug_read_page_reg(robodebug, 0x01, 0x0a, 32, &pause);
	if (ret < 0)
		return ret;

	pause = pause >> port;
	pause = ((pause >> 8) & 2) | (pause & 1);

	return sprintf(buf, "Settings for port %d:\n\tSpeed: %sMb/s\n"
			    "\tDuplex: %s\n\tLink detected: yes\n"
			    "\tPause frame status: %s\n",
			port,
			(speed == 0) ? "10" :
			 (speed == 1) ? "100" :
			 (speed == 2) ? "1000" : "???",
			duplex ? "Full" : "Half",
			(pause == 0) ? "off" :
			 (pause == 1) ? "TX only" :
			 (pause == 2) ? "RX only" :
			 (pause == 3) ? "TX and RX" : "unknown");
}

static ssize_t port0_status_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return port_status_show(robodebug, buf, 0);
}

static DEVICE_ATTR(port0_status, S_IRUSR, port0_status_show, NULL);

static ssize_t port1_status_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return port_status_show(robodebug, buf, 1);
}

static DEVICE_ATTR(port1_status, S_IRUSR, port1_status_show, NULL);

static ssize_t port2_status_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return port_status_show(robodebug, buf, 2);
}

static DEVICE_ATTR(port2_status, S_IRUSR, port2_status_show, NULL);

static ssize_t port3_status_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return port_status_show(robodebug, buf, 3);
}

static DEVICE_ATTR(port3_status, S_IRUSR, port3_status_show, NULL);

static ssize_t port4_status_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return port_status_show(robodebug, buf, 4);
}

static DEVICE_ATTR(port4_status, S_IRUSR, port4_status_show, NULL);

static ssize_t reg_addr_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return sprintf(buf, "%x\n", robodebug->reg_addr);
}

static ssize_t reg_addr_set(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);
	unsigned int val;
	int ret;

	ret = sscanf(buf, "%x", &val);
	if (ret < 1)
		return -EINVAL;

	robodebug->reg_addr = val;

	return count;
}

static DEVICE_ATTR(reg_addr, S_IRUSR | S_IWUSR, reg_addr_show, reg_addr_set);

static ssize_t reg_data8_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);
	int ret;
	u64 val;

	ret = robodebug_read_page_reg(robodebug, robodebug->reg_page,
				      robodebug->reg_addr, 8, &val);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%x\n", (unsigned int)val);
}

static ssize_t reg_data8_set(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);
	unsigned int val;
	int ret;

	ret = sscanf(buf, "%x", &val);
	if (ret < 1)
		return -EINVAL;

	ret = robodebug_write_page_reg(robodebug, robodebug->reg_page,
				       robodebug->reg_addr, 8, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(reg_data8, S_IRUSR | S_IWUSR, reg_data8_show, reg_data8_set);

static ssize_t reg_data16_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);
	int ret;
	u64 val;

	ret = robodebug_read_page_reg(robodebug, robodebug->reg_page,
				      robodebug->reg_addr, 16, &val);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%x\n", (unsigned int)val);
}

static ssize_t reg_data16_set(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);
	unsigned int val;
	int ret;

	ret = sscanf(buf, "%x", &val);
	if (ret < 1)
		return -EINVAL;

	ret = robodebug_write_page_reg(robodebug, robodebug->reg_page,
				       robodebug->reg_addr, 16, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(reg_data16, S_IRUSR | S_IWUSR,
		   reg_data16_show, reg_data16_set);

static ssize_t reg_data32_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);
	int ret;
	u64 val;

	ret = robodebug_read_page_reg(robodebug, robodebug->reg_page,
				      robodebug->reg_addr, 32, &val);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%x\n", (unsigned int)val);
}

static ssize_t reg_data32_set(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);
	unsigned int val;
	int ret;

	ret = sscanf(buf, "%x", &val);
	if (ret < 1)
		return -EINVAL;

	ret = robodebug_write_page_reg(robodebug, robodebug->reg_page,
				       robodebug->reg_addr, 32, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(reg_data32, S_IRUSR | S_IWUSR,
		   reg_data32_show, reg_data32_set);

static ssize_t reg_data64_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);
	int ret;
	u64 val;

	ret = robodebug_read_page_reg(robodebug, robodebug->reg_page,
				      robodebug->reg_addr, 64, &val);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%Lx\n", val);
}

static ssize_t reg_data64_set(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);
	u64 val;
	int ret;

	ret = sscanf(buf, "%Lx", &val);
	if (ret < 1)
		return -EINVAL;

	ret = robodebug_write_page_reg(robodebug, robodebug->reg_page,
				       robodebug->reg_addr, 64, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(reg_data64, S_IRUSR | S_IWUSR,
		   reg_data64_show, reg_data64_set);

static ssize_t reg_page_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return sprintf(buf, "%x\n", robodebug->reg_page);
}

static ssize_t reg_page_set(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);
	unsigned int val;
	int ret;

	ret = sscanf(buf, "%x", &val);
	if (ret < 1)
		return -EINVAL;

	robodebug->reg_page = val;

	return count;
}

static DEVICE_ATTR(reg_page, S_IRUSR | S_IWUSR, reg_page_show, reg_page_set);

static ssize_t reset_set(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);
	int ret;

	ret = robodebug_write_page_reg(robodebug, 0x00, 0x79, 8, 0x90);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(reset, S_IWUSR, NULL, reset_set);

static ssize_t revision_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return robodebug_switch_model(robodebug, buf, PAGE_SIZE);
}

static DEVICE_ATTR(revision, S_IRUSR, revision_show, NULL);

static ssize_t rmon_reset_set(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);
	u64 val;
	int ret;

	ret = robodebug_read_page_reg(robodebug, 0x02, 0x00, 8, &val);
	if (ret < 0)
		return ret;

	ret = robodebug_write_page_reg(robodebug, 0x02, 0x00, 8, val | 0x01);
	if (ret < 0)
		return ret;

	ret = robodebug_write_page_reg(robodebug, 0x02, 0x00, 8, val & ~0x01);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(rmon_reset, S_IWUSR, NULL, rmon_reset_set);

static ssize_t spi_debug_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	return sprintf(buf, "%d\n", robodebug->spi_debug);
}

static ssize_t spi_debug_set(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct robodebug_data *robodebug = spi_get_drvdata(spi);
	unsigned int val;
	int ret;

	ret = sscanf(buf, "%d", &val);
	if (ret < 1)
		return -EINVAL;

	robodebug->spi_debug = val;

	return count;
}

static DEVICE_ATTR(spi_debug, S_IRUSR | S_IWUSR, spi_debug_show, spi_debug_set);

static struct attribute *robodebug_attributes[] = {
	&dev_attr_atu_flush.attr,
	&dev_attr_port0_restart_aneg.attr,
	&dev_attr_port0_statistics.attr,
	&dev_attr_port0_status.attr,
	&dev_attr_port1_restart_aneg.attr,
	&dev_attr_port1_statistics.attr,
	&dev_attr_port1_status.attr,
	&dev_attr_port2_restart_aneg.attr,
	&dev_attr_port2_statistics.attr,
	&dev_attr_port2_status.attr,
	&dev_attr_port3_restart_aneg.attr,
	&dev_attr_port3_statistics.attr,
	&dev_attr_port3_status.attr,
	&dev_attr_port4_restart_aneg.attr,
	&dev_attr_port4_statistics.attr,
	&dev_attr_port4_status.attr,
	&dev_attr_reg_addr.attr,
	&dev_attr_reg_data8.attr,
	&dev_attr_reg_data16.attr,
	&dev_attr_reg_data32.attr,
	&dev_attr_reg_data64.attr,
	&dev_attr_reg_page.attr,
	&dev_attr_reset.attr,
	&dev_attr_revision.attr,
	&dev_attr_rmon_reset.attr,
	&dev_attr_spi_debug.attr,
	NULL,
};

static const struct attribute_group robodebug_group = {
	.attrs = robodebug_attributes,
};


/* probe ********************************************************************/
static int robodebug_probe(struct spi_device *spi)
{
	struct robodebug_data *robodebug;
	int ret;
	char model[128];

	dev_info(&spi->dev, "probe\n");

	robodebug = kzalloc(sizeof(*robodebug), GFP_KERNEL);
	if (!robodebug)
		return -ENOMEM;

	robodebug->spi = spi;
	mutex_init(&robodebug->page_lock);
	robodebug->page = -1;
	mutex_init(&robodebug->mib_snapshot_lock);
#ifdef CONFIG_SPI_ROBODEBUG_SPI_DEBUG_DEFAULT_ON
	robodebug->spi_debug = 1;
#endif

	spi_set_drvdata(spi, robodebug);

	ret = robodebug_switch_model(robodebug, model, sizeof(model));
	if (ret < 0) {
		dev_err(&spi->dev, "robodebug_switch_model: %d\n", ret);
		goto error;
	}

	dev_info(&spi->dev, "detected a %s switch\n", model);

	ret = robodebug_switch_init(robodebug);
	if (ret < 0) {
		dev_err(&spi->dev, "robodebug_switch_init: %d\n", ret);
		goto error;
	}

	ret = sysfs_create_group(&spi->dev.kobj, &robodebug_group);
	if (ret) {
		dev_err(&spi->dev, "sysfs_create_group: %d\n", ret);
		goto error;
	}

	return 0;

error:
	mutex_destroy(&robodebug->page_lock);
	mutex_destroy(&robodebug->mib_snapshot_lock);
	kfree(robodebug);
	return ret;
}

static int robodebug_remove(struct spi_device *spi)
{
	struct robodebug_data *robodebug = spi_get_drvdata(spi);

	sysfs_remove_group(&spi->dev.kobj, &robodebug_group);

	mutex_destroy(&robodebug->page_lock);
	mutex_destroy(&robodebug->mib_snapshot_lock);
	kfree(robodebug);

	dev_info(&spi->dev, "remove\n");

	return 0;
}

static struct spi_driver robodebug_spi_driver = {
	.driver = {
		.name =		"robodebug",
		.owner =	THIS_MODULE,
	},
	.probe =	robodebug_probe,
	.remove =	robodebug_remove,
};

static int __init robodebug_init(void)
{
	return spi_register_driver(&robodebug_spi_driver);
}
module_init(robodebug_init);

static void __exit robodebug_exit(void)
{
	spi_unregister_driver(&robodebug_spi_driver);
}
module_exit(robodebug_exit);

MODULE_AUTHOR("The IMS Company");
MODULE_DESCRIPTION("Broadcom RoboSwitch debug driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:robodebug");
