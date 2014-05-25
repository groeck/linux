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

	ret = __mv88e6xxx_reg_read(bus, sw_addr, REG_PORT(0), 0x03);
	if (ret >= 0) {
		if ((ret & 0xfff0) == 0x1760)
			return "Marvell 88E6176";
		if (ret == 0x3521)
			return "Marvell 88E6352 (A0)";
		if (ret == 0x3522)
			return "Marvell 88E6352 (A1)";
		if ((ret & 0xfff0) == 0x3520)
			return "Marvell 88E6352";
	}

	return NULL;
}

static int mv88e6352_switch_reset(struct dsa_switch *ds)
{
	unsigned long timeout;
	int ret;
	int i;

	/* Set all ports to the disabled state. */
	for (i = 0; i < 7; i++) {
		ret = REG_READ(REG_PORT(i), 0x04);
		REG_WRITE(REG_PORT(i), 0x04, ret & 0xfffc);
	}

	/* Wait for transmit queues to drain. */
	usleep_range(2000, 4000);

	/* Reset the switch. Keep PPU active (bit 14, undocumented).
	 * The PPU needs to be active to support indirect phy register
	 * accesses through global registers 0x18 and 0x19.
	 */
	REG_WRITE(REG_GLOBAL, 0x04, 0xc000);

	/* Wait up to one second for reset to complete. */
	timeout = jiffies + 1 * HZ;
	while (time_before(jiffies, timeout)) {
		ret = REG_READ(REG_GLOBAL, 0x00);
		if ((ret & 0x8800) == 0x8800)
			break;
		usleep_range(1000, 2000);
	}
	if (time_after(jiffies, timeout))
		return -ETIMEDOUT;

	return 0;
}

static int mv88e6352_setup_global(struct dsa_switch *ds)
{
	int ret;
	int i;

	/* Discard packets with excessive collisions,
	 * mask all interrupt sources, enable PPU (bit 14, undocumented).
	 */
	REG_WRITE(REG_GLOBAL, 0x04, 0x6000);

	/* Set the default address aging time to 5 minutes, and
	 * enable address learn messages to be sent to all message
	 * ports.
	 */
	REG_WRITE(REG_GLOBAL, 0x0a, 0x0148);

	/* Configure the priority mapping registers. */
	ret = mv88e6xxx_config_prio(ds);
	if (ret < 0)
		return ret;

	/* Configure the upstream port, and configure the upstream
	 * port as the port to which ingress and egress monitor frames
	 * are to be sent.
	 */
	REG_WRITE(REG_GLOBAL, 0x1a, (dsa_upstream_port(ds) * 0x1110));

	/* Disable remote management for now, and set the switch's
	 * DSA device number.
	 */
	REG_WRITE(REG_GLOBAL, 0x1c, ds->index & 0x1f);

	/* Send all frames with destination addresses matching
	 * 01:80:c2:00:00:2x to the CPU port.
	 */
	REG_WRITE(REG_GLOBAL2, 0x02, 0xffff);

	/* Send all frames with destination addresses matching
	 * 01:80:c2:00:00:0x to the CPU port.
	 */
	REG_WRITE(REG_GLOBAL2, 0x03, 0xffff);

	/* Disable the loopback filter, disable flow control
	 * messages, disable flood broadcast override, disable
	 * removing of provider tags, disable ATU age violation
	 * interrupts, disable tag flow control, force flow
	 * control priority to the highest, and send all special
	 * multicast frames to the CPU at the highest priority.
	 */
	REG_WRITE(REG_GLOBAL2, 0x05, 0x00ff);

	/* Program the DSA routing table. */
	for (i = 0; i < 32; i++) {
		int nexthop = 0x1f;

		if (i != ds->index && i < ds->dst->pd->nr_chips)
			nexthop = ds->pd->rtable[i] & 0x1f;

		REG_WRITE(REG_GLOBAL2, 0x06, 0x8000 | (i << 8) | nexthop);
	}

	/* Clear all trunk masks. */
	for (i = 0; i < 8; i++)
		REG_WRITE(REG_GLOBAL2, 0x07, 0x8000 | (i << 12) | 0x7f);

	/* Clear all trunk mappings. */
	for (i = 0; i < 16; i++)
		REG_WRITE(REG_GLOBAL2, 0x08, 0x8000 | (i << 11));

	/* Disable ingress rate limiting by resetting all ingress
	 * rate limit registers to their initial state.
	 */
	for (i = 0; i < 7; i++)
		REG_WRITE(REG_GLOBAL2, 0x09, 0x9000 | (i << 8));

	/* Initialise cross-chip port VLAN table to reset defaults. */
	REG_WRITE(REG_GLOBAL2, 0x0b, 0x9000);

	/* Clear the priority override table. */
	for (i = 0; i < 16; i++)
		REG_WRITE(REG_GLOBAL2, 0x0f, 0x8000 | (i << 8));

	/* @@@ initialise AVB (22/23) watchdog (27) sdet (29) registers */

	return 0;
}

static int mv88e6352_setup_port(struct dsa_switch *ds, int p)
{
	int addr = REG_PORT(p);
	u16 val;

	/* MAC Forcing register: don't force link, speed, duplex
	 * or flow control state to any particular values on physical
	 * ports, but force the CPU port and all DSA ports to 1000 Mb/s
	 * full duplex.
	 */
	if (dsa_is_cpu_port(ds, p) || ds->dsa_port_mask & (1 << p))
		REG_WRITE(addr, 0x01, 0x003e);
	else
		REG_WRITE(addr, 0x01, 0x0003);

	/* Do not limit the period of time that this port can be
	 * paused for by the remote end or the period of time that
	 * this port can pause the remote end.
	 */
	REG_WRITE(addr, 0x02, 0x0000);

	/* Port Control: disable Drop-on-Unlock, disable Drop-on-Lock,
	 * disable Header mode, enable IGMP/MLD snooping, disable VLAN
	 * tunneling, determine priority by looking at 802.1p and IP
	 * priority fields (IP prio has precedence), and set STP state
	 * to Forwarding.
	 *
	 * If this is the CPU link, use DSA or EDSA tagging depending
	 * on which tagging mode was configured.
	 *
	 * If this is a link to another switch, use DSA tagging mode.
	 *
	 * If this is the upstream port for this switch, enable
	 * forwarding of unknown unicasts and multicasts.
	 */
	val = 0x0433;
	if (dsa_is_cpu_port(ds, p)) {
		if (ds->dst->tag_protocol == DSA_TAG_PROTO_EDSA)
			val |= 0x3300;
		else
			val |= 0x0100;
	}
	if (ds->dsa_port_mask & (1 << p))
		val |= 0x0100;
	if (p == dsa_upstream_port(ds))
		val |= 0x000c;
	REG_WRITE(addr, 0x04, val);

	/* Port Control 1: disable trunking.  Also, if this is the
	 * CPU port, enable learn messages to be sent to this port.
	 */
	REG_WRITE(addr, 0x05, dsa_is_cpu_port(ds, p) ? 0x8000 : 0x0000);

	/* Port Control 2: don't force a good FCS, set the maximum
	 * frame size to 10240 bytes, don't let the switch add or
	 * strip 802.1q tags, don't discard tagged or untagged frames
	 * on this port, do a destination address lookup on all
	 * received packets as usual, disable ARP mirroring and don't
	 * send a copy of all transmitted/received frames on this port
	 * to the CPU.
	 */
	REG_WRITE(addr, 0x08, 0x2080);

	/* Egress rate control: disable egress rate control. */
	REG_WRITE(addr, 0x09, 0x0001);

	/* Egress rate control 2: disable egress rate control. */
	REG_WRITE(addr, 0x0a, 0x0000);

	/* Port Association Vector: when learning source addresses
	 * of packets, add the address to the address database using
	 * a port bitmap that has only the bit for this port set and
	 * the other bits clear.
	 */
	REG_WRITE(addr, 0x0b, 1 << p);

	/* Port ATU control: disable limiting the number of address
	 * database entries that this port is allowed to use.
	 */
	REG_WRITE(addr, 0x0c, 0x0000);

	/* Priority Override: disable DA, SA and VTU priority override. */
	REG_WRITE(addr, 0x0d, 0x0000);

	/* Port Ethertype: use the Ethertype DSA Ethertype value. */
	REG_WRITE(addr, 0x0f, ETH_P_EDSA);

	/* Tag Remap: use an identity 802.1p prio -> switch prio
	 * mapping.
	 */
	REG_WRITE(addr, 0x18, 0x3210);

	/* Tag Remap 2: use an identity 802.1p prio -> switch prio
	 * mapping.
	 */
	REG_WRITE(addr, 0x19, 0x7654);

	return mv88e6xxx_setup_port_common(ds, p);
}

#ifdef CONFIG_NET_DSA_HWMON

static int mv88e6352_phy_page_read(struct dsa_switch *ds,
				   int port, int page, int reg)
{
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	int ret;

	mutex_lock(&ps->phy_mutex);
	ret = mv88e6xxx_phy_write_indirect(ds, port, 0x16, page);
	if (ret < 0)
		goto error;
	ret = mv88e6xxx_phy_read_indirect(ds, port, reg);
error:
	mv88e6xxx_phy_write_indirect(ds, port, 0x16, 0x0);
	mutex_unlock(&ps->phy_mutex);
	return ret;
}

static int mv88e6352_phy_page_write(struct dsa_switch *ds,
				    int port, int page, int reg, int val)
{
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	int ret;

	mutex_lock(&ps->phy_mutex);
	ret = mv88e6xxx_phy_write_indirect(ds, port, 0x16, page);
	if (ret < 0)
		goto error;

	ret = mv88e6xxx_phy_write_indirect(ds, port, reg, val);
error:
	mv88e6xxx_phy_write_indirect(ds, port, 0x16, 0x0);
	mutex_unlock(&ps->phy_mutex);
	return ret;
}

static int mv88e6352_get_temp(struct dsa_switch *ds, int *temp)
{
	int ret;

	*temp = 0;

	ret = mv88e6352_phy_page_read(ds, 0, 6, 27);
	if (ret < 0)
		return ret;

	*temp = (ret & 0xff) - 25;

	return 0;
}

static int mv88e6352_get_temp_limit(struct dsa_switch *ds, int *temp)
{
	int ret;

	*temp = 0;

	ret = mv88e6352_phy_page_read(ds, 0, 6, 26);
	if (ret < 0)
		return ret;

	*temp = (((ret >> 8) & 0x1f) * 5) - 25;

	return 0;
}

static int mv88e6352_set_temp_limit(struct dsa_switch *ds, int temp)
{
	int ret;

	ret = mv88e6352_phy_page_read(ds, 0, 6, 26);
	if (ret < 0)
		return ret;
	temp = clamp_val(DIV_ROUND_CLOSEST(temp, 5) + 5, 0, 0x1f);
	return mv88e6352_phy_page_write(ds, 0, 6, 26,
					(ret & 0xe0ff) | (temp << 8));
}

static int mv88e6352_get_temp_alarm(struct dsa_switch *ds, bool *alarm)
{
	int ret;

	*alarm = false;

	ret = mv88e6352_phy_page_read(ds, 0, 6, 26);
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
		val = mv88e6352_phy_page_read(ds, ps->phy_device, ps->phy_page,
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
		mv88e6352_phy_page_write(ds, ps->phy_device, ps->phy_page,
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
	int i;

	ret = mv88e6xxx_setup_common(ds);
	if (ret < 0)
		return ret;

	mutex_init(&ps->eeprom_mutex);

	ps->id = REG_READ(REG_PORT(0), 0x03) & 0xfff0;

	ret = sysfs_create_group(&ds->parent->kobj, &mv88e6352_group);
	if (ret)
		return ret;

	if (dsa_is_unmanaged(ds))
		return 0;

	ret = mv88e6352_switch_reset(ds);
	if (ret < 0)
		return ret;

	/* @@@ initialise vtu and atu */

	ret = mv88e6352_setup_global(ds);
	if (ret < 0)
		return ret;

	for (i = 0; i < 7; i++) {
		ret = mv88e6352_setup_port(ds, i);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int mv88e6352_port_to_phy_addr(int port)
{
	if (port >= 0 && port <= 4)
		return port;
	return -EINVAL;
}

static int
mv88e6352_phy_read(struct dsa_switch *ds, int port, int regnum)
{
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	int addr = mv88e6352_port_to_phy_addr(port);
	int ret;

	if (addr < 0)
		return addr;

	mutex_lock(&ps->phy_mutex);
	ret = mv88e6xxx_phy_read_indirect(ds, addr, regnum);
	mutex_unlock(&ps->phy_mutex);

	return ret;
}

static int
mv88e6352_phy_write(struct dsa_switch *ds, int port, int regnum, u16 val)
{
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	int addr = mv88e6352_port_to_phy_addr(port);
	int ret;

	if (addr < 0)
		return addr;

	mutex_lock(&ps->phy_mutex);
	ret = mv88e6xxx_phy_write_indirect(ds, addr, regnum, val);
	mutex_unlock(&ps->phy_mutex);

	return ret;
}

static struct mv88e6xxx_hw_stat mv88e6352_hw_stats[] = {
	{ "in_good_octets", 8, 0x00, },
	{ "in_bad_octets", 4, 0x02, },
	{ "in_unicast", 4, 0x04, },
	{ "in_broadcasts", 4, 0x06, },
	{ "in_multicasts", 4, 0x07, },
	{ "in_pause", 4, 0x16, },
	{ "in_undersize", 4, 0x18, },
	{ "in_fragments", 4, 0x19, },
	{ "in_oversize", 4, 0x1a, },
	{ "in_jabber", 4, 0x1b, },
	{ "in_rx_error", 4, 0x1c, },
	{ "in_fcs_error", 4, 0x1d, },
	{ "out_octets", 8, 0x0e, },
	{ "out_unicast", 4, 0x10, },
	{ "out_broadcasts", 4, 0x13, },
	{ "out_multicasts", 4, 0x12, },
	{ "out_pause", 4, 0x15, },
	{ "excessive", 4, 0x11, },
	{ "collisions", 4, 0x1e, },
	{ "deferred", 4, 0x05, },
	{ "single", 4, 0x14, },
	{ "multiple", 4, 0x17, },
	{ "out_fcs_error", 4, 0x03, },
	{ "late", 4, 0x1f, },
	{ "hist_64bytes", 4, 0x08, },
	{ "hist_65_127bytes", 4, 0x09, },
	{ "hist_128_255bytes", 4, 0x0a, },
	{ "hist_256_511bytes", 4, 0x0b, },
	{ "hist_512_1023bytes", 4, 0x0c, },
	{ "hist_1024_max_bytes", 4, 0x0d, },
	{ "sw_in_discards", 4, 0x110, },
	{ "sw_in_filtered", 2, 0x112, },
	{ "sw_out_filtered", 2, 0x113, },
};

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

static void
mv88e6352_get_strings(struct dsa_switch *ds, int port, uint8_t *data)
{
	mv88e6xxx_get_strings(ds, ARRAY_SIZE(mv88e6352_hw_stats),
			      mv88e6352_hw_stats, port, data);
}

static void
mv88e6352_get_ethtool_stats(struct dsa_switch *ds, int port, uint64_t *data)
{
	mv88e6xxx_get_ethtool_stats(ds, ARRAY_SIZE(mv88e6352_hw_stats),
				    mv88e6352_hw_stats, port, data);
}

static int mv88e6352_get_sset_count(struct dsa_switch *ds)
{
	return ARRAY_SIZE(mv88e6352_hw_stats);
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

	val = mv88e6352_phy_read(ds, port, 0x0a);
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

	val = mv88e6352_phy_read(ds, port, 0x15);
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
	mv88e6xxx_phy_write_indirect(ds, port, 0x16, 0xff);

	for (repeat = 0; repeat < 10; repeat++) {
		for (pair = 0; pair < 4; pair++) {
			mv88e6xxx_phy_write_indirect(ds, port, 0x10, 0x1118 | pair);
			for (i = 0; i < 100; i++) {
				val = mv88e6xxx_phy_read_indirect(ds, port, 0x10);
				if (val & 0x8000)
					break;
				usleep_range(100, 200);
			}
			if (i == 100) {
				ret = -ETIMEDOUT;
				goto error;
			}
			length += mv88e6xxx_phy_read_indirect(ds, port, 0x12);
		}
	}
	ret = length / 40;
error:
	mv88e6xxx_phy_write_indirect(ds, port, 0x16, 0x0);
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

static ssize_t packet_generator_count_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct dsa_switch *ds = dsa_slave_switch(to_net_dev(dev));
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);

	return sprintf(buf, "%x\n", ps->packet_generator_count);
}

static ssize_t packet_generator_count_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct dsa_switch *ds = dsa_slave_switch(to_net_dev(dev));
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	ps->packet_generator_count = clamp_val(val, 1, 255);

	return count;
}

static DEVICE_ATTR_RW(packet_generator_count);

static ssize_t packet_generator_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct dsa_switch *ds = dsa_slave_switch(to_net_dev(dev));
	int port = dsa_slave_port(to_net_dev(dev));
	int ret;

	ret = mv88e6352_phy_page_read(ds, port, 0x06, 0x10);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%4x\n", ret);
}

static ssize_t packet_generator_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct dsa_switch *ds = dsa_slave_switch(to_net_dev(dev));
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	int port = dsa_slave_port(to_net_dev(dev));
	int pcount = clamp_val(ps->packet_generator_count, 1, 255);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val != 0 && val != 1)
		return -EINVAL;

	ret = mv88e6352_phy_page_write(ds, port, 0x06, 0x10,
				       (pcount << 8) | (val << 3) | 0x06);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR_RW(packet_generator);

static struct attribute *mv88e6352_port_attrs[] = {
	&dev_attr_idle_errors.attr,
	&dev_attr_receive_errors.attr,
	&dev_attr_cable_length.attr,
	&dev_attr_link_down_count.attr,
	&dev_attr_packet_generator_count.attr,
	&dev_attr_packet_generator.attr,
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
	.phy_read		= mv88e6352_phy_read,
	.phy_write		= mv88e6352_phy_write,
	.poll_link		= mv88e6xxx_poll_link,
	.get_strings		= mv88e6352_get_strings,
	.get_ethtool_stats	= mv88e6352_get_ethtool_stats,
	.get_sset_count		= mv88e6352_get_sset_count,
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
	.port_flush		= mv88e6352_port_flush,
	.sysfs_group		= &mv88e6352_port_group,
};

MODULE_ALIAS("platform:mv88e6352");
