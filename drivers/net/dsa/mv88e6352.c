/*
 * net/dsa/mv88e6352.c - Marvell 88e6352 switch chip support
 *
 * Copyright (c) 2014 Guenter Roeck
 *
 * Derived from mv88e6123_61_65.c
 * Copyright (c) 2008-2009 Marvell Semiconductor
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <net/dsa.h>
#include "mv88e6xxx.h"

static char *mv88e6352_probe(struct device *host_dev, int sw_addr)
{
	struct mii_bus *bus = dsa_host_dev_to_mii_bus(host_dev);
	int ret;

	if (bus == NULL)
		return NULL;

	ret = __mv88e6xxx_reg_read(bus, sw_addr, REG_PORT(0), PORT_SWITCH_ID);
	if (ret >= 0) {
		if ((ret & 0xfff0) == PORT_SWITCH_ID_6172)
			return "Marvell 88E6172";
		if ((ret & 0xfff0) == PORT_SWITCH_ID_6176)
			return "Marvell 88E6176";
		if (ret == PORT_SWITCH_ID_6352_A0)
			return "Marvell 88E6352 (A0)";
		if (ret == PORT_SWITCH_ID_6352_A1)
			return "Marvell 88E6352 (A1)";
		if ((ret & 0xfff0) == PORT_SWITCH_ID_6352)
			return "Marvell 88E6352";
	}

	return NULL;
}

static int mv88e6352_setup_global(struct dsa_switch *ds)
{
	u32 upstream_port = dsa_upstream_port(ds);
	int ret;
	u32 reg;

	ret = mv88e6xxx_setup_global(ds);
	if (ret)
		return ret;

	/* Discard packets with excessive collisions,
	 * mask all interrupt sources, enable PPU (bit 14, undocumented).
	 */
	REG_WRITE(REG_GLOBAL, GLOBAL_CONTROL,
		  GLOBAL_CONTROL_PPU_ENABLE | GLOBAL_CONTROL_DISCARD_EXCESS);

	/* Configure the upstream port, and configure the upstream
	 * port as the port to which ingress and egress monitor frames
	 * are to be sent.
	 */
	reg = upstream_port << GLOBAL_MONITOR_CONTROL_INGRESS_SHIFT |
		upstream_port << GLOBAL_MONITOR_CONTROL_EGRESS_SHIFT |
		upstream_port << GLOBAL_MONITOR_CONTROL_ARP_SHIFT;
	REG_WRITE(REG_GLOBAL, GLOBAL_MONITOR_CONTROL, reg);

	/* Disable remote management for now, and set the switch's
	 * DSA device number.
	 */
	REG_WRITE(REG_GLOBAL, 0x1c, ds->index & 0x1f);

	return 0;
}

#ifdef CONFIG_NET_DSA_HWMON

static int mv88e6352_get_temp(struct dsa_switch *ds, int *temp)
{
	int ret;

	*temp = 0;

	ret = mv88e6xxx_phy_page_read(ds, 0, 6, 27);
	if (ret < 0)
		return ret;

	*temp = (ret & 0xff) - 25;

	return 0;
}

static int mv88e6352_get_temp_limit(struct dsa_switch *ds, int *temp)
{
	int ret;

	*temp = 0;

	ret = mv88e6xxx_phy_page_read(ds, 0, 6, 26);
	if (ret < 0)
		return ret;

	*temp = (((ret >> 8) & 0x1f) * 5) - 25;

	return 0;
}

static int mv88e6352_set_temp_limit(struct dsa_switch *ds, int temp)
{
	int ret;

	ret = mv88e6xxx_phy_page_read(ds, 0, 6, 26);
	if (ret < 0)
		return ret;
	temp = clamp_val(DIV_ROUND_CLOSEST(temp, 5) + 5, 0, 0x1f);
	return mv88e6xxx_phy_page_write(ds, 0, 6, 26,
					(ret & 0xe0ff) | (temp << 8));
}

static int mv88e6352_get_temp_alarm(struct dsa_switch *ds, bool *alarm)
{
	int ret;

	*alarm = false;

	ret = mv88e6xxx_phy_page_read(ds, 0, 6, 26);
	if (ret < 0)
		return ret;

	*alarm = !!(ret & 0x40);

	return 0;
}
#endif /* CONFIG_NET_DSA_HWMON */

/* sysfs attributes */

/* switch registers */

static ssize_t reg_device_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dsa_switch_tree *dst = platform_get_drvdata(pdev);
	struct dsa_switch *ds = dst->ds[0];
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);

	return sprintf(buf, "%x\n", ps->reg_device);
}

static ssize_t reg_device_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dsa_switch_tree *dst = platform_get_drvdata(pdev);
	struct dsa_switch *ds = dst->ds[0];
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	ps->reg_device = val;

	return count;
}

static DEVICE_ATTR_RW(reg_device);

static ssize_t reg_addr_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dsa_switch_tree *dst = platform_get_drvdata(pdev);
	struct dsa_switch *ds = dst->ds[0];
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);

	return sprintf(buf, "%x\n", ps->reg_addr);
}

static ssize_t reg_addr_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dsa_switch_tree *dst = platform_get_drvdata(pdev);
	struct dsa_switch *ds = dst->ds[0];
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	ps->reg_addr = val;

	return count;
}

static DEVICE_ATTR_RW(reg_addr);

static ssize_t reg_data_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dsa_switch_tree *dst = platform_get_drvdata(pdev);
	struct dsa_switch *ds = dst->ds[0];
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);

	return sprintf(buf, "%x\n", REG_READ(ps->reg_device, ps->reg_addr));
}

static ssize_t reg_data_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dsa_switch_tree *dst = platform_get_drvdata(pdev);
	struct dsa_switch *ds = dst->ds[0];
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	REG_WRITE(ps->reg_device, ps->reg_addr, val);

	return count;
}

static DEVICE_ATTR_RW(reg_data);

/* phy register access */

static ssize_t phy_device_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dsa_switch_tree *dst = platform_get_drvdata(pdev);
	struct dsa_switch *ds = dst->ds[0];
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);

	return sprintf(buf, "%x\n", ps->phy_device);
}

static ssize_t phy_device_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dsa_switch_tree *dst = platform_get_drvdata(pdev);
	struct dsa_switch *ds = dst->ds[0];
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	ps->phy_device = val;

	return count;
}

static DEVICE_ATTR_RW(phy_device);

static ssize_t phy_page_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dsa_switch_tree *dst = platform_get_drvdata(pdev);
	struct dsa_switch *ds = dst->ds[0];
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);

	return sprintf(buf, "%x\n", ps->phy_page);
}

static ssize_t phy_page_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dsa_switch_tree *dst = platform_get_drvdata(pdev);
	struct dsa_switch *ds = dst->ds[0];
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	ps->phy_page = val;

	return count;
}

static DEVICE_ATTR_RW(phy_page);

static ssize_t phy_addr_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dsa_switch_tree *dst = platform_get_drvdata(pdev);
	struct dsa_switch *ds = dst->ds[0];
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);

	return sprintf(buf, "%x\n", ps->phy_addr);
}

static ssize_t phy_addr_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dsa_switch_tree *dst = platform_get_drvdata(pdev);
	struct dsa_switch *ds = dst->ds[0];
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	/* Don't let user write into page register directly */
	if (val == 0x16)
		return -EINVAL;

	ps->phy_addr = val;

	return count;
}

static DEVICE_ATTR_RW(phy_addr);

static ssize_t phy_data_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dsa_switch_tree *dst = platform_get_drvdata(pdev);
	struct dsa_switch *ds = dst->ds[0];
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	int val;

	if (ps->phy_page || ps->phy_addr >= 0x10) {
		val = mv88e6xxx_phy_page_read(ds, ps->phy_device, ps->phy_page,
					      ps->phy_addr);
	} else {
		val = REG_READ(REG_PORT(ps->phy_device), ps->phy_addr);
	}

	return sprintf(buf, "%x\n", val);
}

static ssize_t phy_data_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dsa_switch_tree *dst = platform_get_drvdata(pdev);
	struct dsa_switch *ds = dst->ds[0];
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (ps->phy_page || ps->phy_addr >= 0x10) {
		mv88e6xxx_phy_page_write(ds, ps->phy_device, ps->phy_page,
					 ps->phy_addr, val);
	} else {
		REG_WRITE(REG_PORT(ps->phy_device), ps->phy_addr, val);
	}

	return count;
}

static DEVICE_ATTR_RW(phy_data);

/* Switch type and revision */

static ssize_t revision_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dsa_switch_tree *dst = platform_get_drvdata(pdev);
	struct dsa_switch *ds = dst->ds[0];

	return sprintf(buf, "%s\n", ds->name);
}

static DEVICE_ATTR_RO(revision);

static struct attribute *mv88e6352_attributes[] = {
	&dev_attr_reg_device.attr,
	&dev_attr_reg_addr.attr,
	&dev_attr_reg_data.attr,
	&dev_attr_phy_device.attr,
	&dev_attr_phy_page.attr,
	&dev_attr_phy_addr.attr,
	&dev_attr_phy_data.attr,
	&dev_attr_revision.attr,
	NULL
};

const struct attribute_group mv88e6352_group = {
	.attrs = mv88e6352_attributes,
};

static int mv88e6352_setup(struct dsa_switch *ds)
{
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	int ret;

	ret = mv88e6xxx_setup_common(ds);
	if (ret < 0)
		return ret;

	ps->num_ports = 7;

	mutex_init(&ps->eeprom_mutex);

	ret = sysfs_create_group(&ds->parent->kobj, &mv88e6352_group);
	if (ret)
		return ret;

	if (dsa_is_unmanaged(ds))
		return 0;

	ret = mv88e6xxx_switch_reset(ds, true);
	if (ret < 0)
		return ret;

	ret = mv88e6352_setup_global(ds);
	if (ret < 0)
		return ret;

	return mv88e6xxx_setup_ports(ds);
}

static int mv88e6352_read_eeprom_word(struct dsa_switch *ds, int addr)
{
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	int ret;

	mutex_lock(&ps->eeprom_mutex);

	ret = mv88e6xxx_reg_write(ds, REG_GLOBAL2, 0x14,
				  0xc000 | (addr & 0xff));
	if (ret < 0)
		goto error;

	ret = mv88e6xxx_eeprom_busy_wait(ds);
	if (ret < 0)
		goto error;

	ret = mv88e6xxx_reg_read(ds, REG_GLOBAL2, 0x15);
error:
	mutex_unlock(&ps->eeprom_mutex);
	return ret;
}

static int mv88e6352_get_eeprom(struct dsa_switch *ds,
				struct ethtool_eeprom *eeprom, u8 *data)
{
	int offset;
	int len;
	int ret;

	offset = eeprom->offset;
	len = eeprom->len;
	eeprom->len = 0;

	eeprom->magic = 0xc3ec4951;

	ret = mv88e6xxx_eeprom_load_wait(ds);
	if (ret < 0)
		return ret;

	if (offset & 1) {
		int word;

		word = mv88e6352_read_eeprom_word(ds, offset >> 1);
		if (word < 0)
			return word;

		*data++ = (word >> 8) & 0xff;

		offset++;
		len--;
		eeprom->len++;
	}

	while (len >= 2) {
		int word;

		word = mv88e6352_read_eeprom_word(ds, offset >> 1);
		if (word < 0)
			return word;

		*data++ = word & 0xff;
		*data++ = (word >> 8) & 0xff;

		offset += 2;
		len -= 2;
		eeprom->len += 2;
	}

	if (len) {
		int word;

		word = mv88e6352_read_eeprom_word(ds, offset >> 1);
		if (word < 0)
			return word;

		*data++ = word & 0xff;

		offset++;
		len--;
		eeprom->len++;
	}

	return 0;
}

static int mv88e6352_eeprom_is_readonly(struct dsa_switch *ds)
{
	int ret;

	ret = mv88e6xxx_reg_read(ds, REG_GLOBAL2, 0x14);
	if (ret < 0)
		return ret;

	if (!(ret & 0x0400))
		return -EROFS;

	return 0;
}

static int mv88e6352_write_eeprom_word(struct dsa_switch *ds, int addr,
				       u16 data)
{
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	int ret;

	mutex_lock(&ps->eeprom_mutex);

	ret = mv88e6xxx_reg_write(ds, REG_GLOBAL2, 0x15, data);
	if (ret < 0)
		goto error;

	ret = mv88e6xxx_reg_write(ds, REG_GLOBAL2, 0x14,
				  0xb000 | (addr & 0xff));
	if (ret < 0)
		goto error;

	ret = mv88e6xxx_eeprom_busy_wait(ds);
error:
	mutex_unlock(&ps->eeprom_mutex);
	return ret;
}

static int mv88e6352_set_eeprom(struct dsa_switch *ds,
				struct ethtool_eeprom *eeprom, u8 *data)
{
	int offset;
	int ret;
	int len;

	if (eeprom->magic != 0xc3ec4951)
		return -EINVAL;

	ret = mv88e6352_eeprom_is_readonly(ds);
	if (ret)
		return ret;

	offset = eeprom->offset;
	len = eeprom->len;
	eeprom->len = 0;

	ret = mv88e6xxx_eeprom_load_wait(ds);
	if (ret < 0)
		return ret;

	if (offset & 1) {
		int word;

		word = mv88e6352_read_eeprom_word(ds, offset >> 1);
		if (word < 0)
			return word;

		word = (*data++ << 8) | (word & 0xff);

		ret = mv88e6352_write_eeprom_word(ds, offset >> 1, word);
		if (ret < 0)
			return ret;

		offset++;
		len--;
		eeprom->len++;
	}

	while (len >= 2) {
		int word;

		word = *data++;
		word |= *data++ << 8;

		ret = mv88e6352_write_eeprom_word(ds, offset >> 1, word);
		if (ret < 0)
			return ret;

		offset += 2;
		len -= 2;
		eeprom->len += 2;
	}

	if (len) {
		int word;

		word = mv88e6352_read_eeprom_word(ds, offset >> 1);
		if (word < 0)
			return word;

		word = (word & 0xff00) | *data++;

		ret = mv88e6352_write_eeprom_word(ds, offset >> 1, word);
		if (ret < 0)
			return ret;

		offset++;
		len--;
		eeprom->len++;
	}

	return 0;
}

static void mv88e6352_port_flush(struct dsa_switch *ds, int port)
{
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);

	netdev_dbg(ds->ports[port], "flush fid %d\n", ps->fid[port]);

	set_bit(ps->fid[port], &ps->fid_flush_mask);
	schedule_work(&ps->bridge_work);
}

/* Attributes attached to slave network devices */

static ssize_t idle_errors_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dsa_switch *ds = dsa_slave_switch(to_net_dev(dev));
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	int port = dsa_slave_port(to_net_dev(dev));
	int val;

	if (port < 0 || port >= ARRAY_SIZE(ps->idle_errors)) {
		dev_err(dev, "Bad port number %d\n", port);
		return -EINVAL;
	}

	val = mv88e6xxx_phy_read(ds, port, 0x0a);
	ps->idle_errors[port] += val & 0xff;

	return sprintf(buf, "%d\n", ps->idle_errors[port]);
}

static DEVICE_ATTR_RO(idle_errors);

static ssize_t receive_errors_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct dsa_switch *ds = dsa_slave_switch(to_net_dev(dev));
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	int port = dsa_slave_port(to_net_dev(dev));
	int val;

	if (port < 0 || port >= ARRAY_SIZE(ps->receive_errors)) {
		dev_err(dev, "Bad port number %d\n", port);
		return -EINVAL;
	}

	val = mv88e6xxx_phy_read(ds, port, 0x15);
	ps->receive_errors[port] += val;

	return sprintf(buf, "%d\n", ps->receive_errors[port]);
}

static DEVICE_ATTR_RO(receive_errors);

static int cable_length_read(struct dsa_switch *ds, int port)
{
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	int repeat, i, pair, ret = 0, val;
	int length = 0;

	mutex_lock(&ps->phy_mutex);
	_mv88e6xxx_phy_write_indirect(ds, port, 0x16, 0xff);

	for (repeat = 0; repeat < 10; repeat++) {
		for (pair = 0; pair < 4; pair++) {
			_mv88e6xxx_phy_write_indirect(ds, port, 0x10, 0x1118 | pair);
			for (i = 0; i < 100; i++) {
				val = _mv88e6xxx_phy_read_indirect(ds, port, 0x10);
				if (val & 0x8000)
					break;
				usleep_range(100, 200);
			}
			if (i == 100) {
				ret = -ETIMEDOUT;
				goto error;
			}
			length += _mv88e6xxx_phy_read_indirect(ds, port, 0x12);
		}
	}
	ret = length / 40;
error:
	_mv88e6xxx_phy_write_indirect(ds, port, 0x16, 0x0);
	mutex_unlock(&ps->phy_mutex);
	return ret;
}

static ssize_t cable_length_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct dsa_switch *ds = dsa_slave_switch(to_net_dev(dev));
	int port = dsa_slave_port(to_net_dev(dev));
	int val;

	val = REG_READ(REG_PORT(port), 0x00);
	/* Return 0 if link is not up at 1000 mbps */
	if ((val & 0x0b00) != 0x0a00)
		return sprintf(buf, "0\n");

	return sprintf(buf, "%d\n", cable_length_read(ds, port));
}

static DEVICE_ATTR_RO(cable_length);

static ssize_t link_down_count_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dsa_switch *ds = dsa_slave_switch(to_net_dev(dev));
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	int port = dsa_slave_port(to_net_dev(dev));

	if (port < 0 || port >= ARRAY_SIZE(ps->link_down_count)) {
		dev_err(dev, "Bad port number %d\n", port);
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", ps->link_down_count[port]);
}

static DEVICE_ATTR_RO(link_down_count);

static struct attribute *mv88e6352_port_attrs[] = {
	&dev_attr_idle_errors.attr,
	&dev_attr_receive_errors.attr,
	&dev_attr_cable_length.attr,
	&dev_attr_link_down_count.attr,
	NULL
};

const struct attribute_group mv88e6352_port_group = {
	.attrs = mv88e6352_port_attrs,
};

struct dsa_switch_driver mv88e6352_switch_driver = {
	.tag_protocol		= DSA_TAG_PROTO_EDSA,
	.priv_size		= sizeof(struct mv88e6xxx_priv_state),
	.probe			= mv88e6352_probe,
	.setup			= mv88e6352_setup,
	.set_addr		= mv88e6xxx_set_addr_indirect,
	.phy_read		= mv88e6xxx_phy_read_indirect,
	.phy_write		= mv88e6xxx_phy_write_indirect,
	.poll_link		= mv88e6xxx_poll_link,
	.get_strings		= mv88e6xxx_get_strings,
	.get_ethtool_stats	= mv88e6xxx_get_ethtool_stats,
	.get_sset_count		= mv88e6xxx_get_sset_count,
	.set_eee		= mv88e6xxx_set_eee,
	.get_eee		= mv88e6xxx_get_eee,
#ifdef CONFIG_NET_DSA_HWMON
	.get_temp		= mv88e6352_get_temp,
	.get_temp_limit		= mv88e6352_get_temp_limit,
	.set_temp_limit		= mv88e6352_set_temp_limit,
	.get_temp_alarm		= mv88e6352_get_temp_alarm,
#endif
	.get_eeprom		= mv88e6352_get_eeprom,
	.set_eeprom		= mv88e6352_set_eeprom,
	.get_regs_len		= mv88e6xxx_get_regs_len,
	.get_regs		= mv88e6xxx_get_regs,
	.port_join_bridge	= mv88e6xxx_join_bridge,
	.port_leave_bridge	= mv88e6xxx_leave_bridge,
	.port_stp_update	= mv88e6xxx_port_stp_update,
	.fdb_add		= mv88e6xxx_port_fdb_add,
	.fdb_del		= mv88e6xxx_port_fdb_del,
	.fdb_getnext		= mv88e6xxx_port_fdb_getnext,
	.port_vlan_add		= mv88e6xxx_port_vlan_add,
	.port_vlan_del		= mv88e6xxx_port_vlan_del,
	.port_flush		= mv88e6352_port_flush,
	.sysfs_group		= &mv88e6352_port_group,
};

MODULE_ALIAS("platform:mv88e6352");
MODULE_ALIAS("platform:mv88e6172");
