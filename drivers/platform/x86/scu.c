/*
 * SCU board driver
 *
 * Copyright (c) 2012, 2014 Guenter Roeck <linux@roeck-us.net>
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/leds.h>
#include <linux/mdio-gpio.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/dmi.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/version.h>
#include <linux/platform_data/at24.h>
#include <linux/platform_data/pca953x.h>
#include <linux/lis3lv02d.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/netdevice.h>
#include <net/dsa.h>
#include <asm/processor.h>
#include <asm/byteorder.h>

#define SCU_EXT_GPIO_BASE(x)	(0 + (x) * 8)
#define SCU_EXT_GPIO(chip, x)	(SCU_EXT_GPIO_BASE(chip) + (x))

/* Front panel LEDs */
#define SCU_RD_LED_GPIO		SCU_EXT_GPIO(1, 0)
#define SCU_WLES_LED_GPIO	SCU_EXT_GPIO(1, 1)
#define SCU_LD_FAIL_LED_GPIO	SCU_EXT_GPIO(1, 2)
#define SCU_SW_LED_GPIO		SCU_EXT_GPIO(1, 3)	/* shared control with PIC */

#define SCU_SD_ACTIVE_1_GPIO	SCU_EXT_GPIO(2, 0)
#define SCU_SD_ERROR_1_GPIO	SCU_EXT_GPIO(2, 1)
#define SCU_SD_ACTIVE_2_GPIO	SCU_EXT_GPIO(2, 2)
#define SCU_SD_ERROR_2_GPIO	SCU_EXT_GPIO(2, 3)
#define SCU_SD_ACTIVE_3_GPIO	SCU_EXT_GPIO(2, 4)
#define SCU_SD_ERROR_3_GPIO	SCU_EXT_GPIO(2, 5)

#define SCU_SD_ACTIVE_4_GPIO	SCU_EXT_GPIO(3, 0)
#define SCU_SD_ERROR_4_GPIO	SCU_EXT_GPIO(3, 1)
#define SCU_SD_ACTIVE_5_GPIO	SCU_EXT_GPIO(3, 2)
#define SCU_SD_ERROR_5_GPIO	SCU_EXT_GPIO(3, 3)
#define SCU_SD_ACTIVE_6_GPIO	SCU_EXT_GPIO(3, 4)
#define SCU_SD_ERROR_6_GPIO	SCU_EXT_GPIO(3, 5)

struct __packed eeprom_data {
	unsigned short length;			/* 0 - 1 */
	unsigned char checksum;			/* 2 */
	unsigned char have_gsm_modem;		/* 3 */
	unsigned char have_cdma_modem;		/* 4 */
	unsigned char have_wifi_modem;		/* 5 */
	unsigned char have_rhdd;		/* 6 */
	unsigned char have_dvd;			/* 7 */
	unsigned char have_tape;		/* 8 */
	unsigned char have_humidity_sensor;	/* 9 */
	unsigned char have_fiber_channel;	/* 10 */
	unsigned char lru_part_number[11];	/* 11 - 21 Box Part Number */
	unsigned char lru_revision[7];		/* 22 - 28 Box Revision */
	unsigned char lru_serial_number[7];	/* 29 - 35 Box Serial Number */
	unsigned char lru_date_of_manufacture[7];
				/* 36 - 42 Box Date of Manufacture */
	unsigned char board_part_number[11];
				/* 43 - 53 Base Board Part Number */
	unsigned char board_revision[7];
				/* 54 - 60 Base Board Revision */
	unsigned char board_serial_number[7];
				/* 61 - 67 Base Board Serial Number */
	unsigned char board_date_of_manufacture[7];
				/* 68 - 74 Base Board Date of Manufacture */
	unsigned char dummy[21];/* 75 - 95 spare/filler */
};

enum scu_version { scu1, scu2, scu3 };

struct scu_data;

struct scu_platform_data {
	const char *board_type;
	const char *part_number;
	enum scu_version version;
	int eeprom_len;
	struct i2c_board_info *i2c_board_info;
	int num_i2c_board_info;
	struct spi_board_info *spi_board_info;
	int num_spi_board_info;
	void (*init)(struct scu_data *);
	void (*remove)(struct scu_data *);
};

struct scu_data {
	struct device *dev;			/* SCU platform device */
	struct net_device *netdev;		/* Ethernet device */
	struct platform_device *mdio_dev;	/* MDIO device */
	struct platform_device *dsa_dev;	/* DSA device */
	struct proc_dir_entry *rave_proc_dir;
	struct mutex write_lock;
	struct platform_device *leds_pdev[3];
	struct i2c_adapter *adapter;		/* I2C adapter */
	struct spi_master *master;		/* SPI master */
	struct i2c_client *client[10];		/* I2C clients */
	struct spi_device *spidev[1];		/* SPI devices */
	const struct scu_platform_data *pdata;
	bool have_write_magic;
	struct memory_accessor *macc;
	struct eeprom_data eeprom;
	bool eeprom_accessible;
	bool eeprom_valid;
};

#define SCU_EEPROM_LEN_EEPROM	36
#define SCU_EEPROM_LEN_GEN1	36
#define SCU_EEPROM_LEN_GEN2	75
#define SCU_EEPROM_LEN_GEN3	75		/* Preliminary */

#define SCU_PARTNUM_GEN1	"00-5001"
#define SCU_PARTNUM_GEN2	"00-5010"
#define SCU_PARTNUM_GEN3	"00-5013"

#define SCU_WRITE_MAGIC		5482328594ULL

/* sysfs */

unsigned char scu_get_checksum(unsigned char *ptr, int len)
{
	unsigned char checksum = 0;
	int i;

	for (i = 0; i < len; i++)
		checksum += ptr[i];
	return checksum;
}

static int scu_update_checksum(struct scu_data *data)
{
	unsigned char checksum;
	int ret;

	data->eeprom.checksum = 0;
	checksum = scu_get_checksum((unsigned char *)&data->eeprom,
				    data->pdata->eeprom_len);
	data->eeprom.checksum = ~checksum + 1;
	ret = data->macc->write(data->macc, &data->eeprom.checksum,
			0x300 + offsetof(struct eeprom_data, checksum),
			sizeof(data->eeprom.checksum));
	if (ret <= 0)
		return ret < 0 ? ret : -EIO;
	return 0;
}

static ssize_t board_type_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct scu_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", data->pdata->board_type);
}

static ssize_t attribute_magic_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct scu_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->have_write_magic);
}

static ssize_t attribute_magic_store(struct device *dev,
				     struct device_attribute *devattr,
				     const char *buf, size_t count)
{
	struct scu_data *data = dev_get_drvdata(dev);
	unsigned long long magic;
	int err;

	err = kstrtoull(buf, 10, &magic);
	if (err)
		return err;

	data->have_write_magic = magic == SCU_WRITE_MAGIC;

	return count;
}

static ssize_t scu_object_show(char *buf, char *data, int len)
{
	return snprintf(buf, PAGE_SIZE, "%.*s\n", len, data);
}

static ssize_t scu_object_store(struct scu_data *data, int offset,
				const char *in, char *out, ssize_t len)
{
	char buffer[12] = { 0 };
	char *cp;
	int ret;

	if (!data->have_write_magic)
		return -EACCES;

	strlcpy(buffer, in, sizeof(buffer));
	/* do not copy newline from input string */
	cp = strchr(buffer, '\n');
	if (cp)
		*cp = '\0';

	if (len > sizeof(buffer))
		len = sizeof(buffer);

	mutex_lock(&data->write_lock);
	strncpy(out, buffer, len);

	/* Write entire eeprom if it was marked invalid */
	if (!data->eeprom_valid) {
		offset = 0;
		len = data->pdata->eeprom_len;
		out = (char *)&data->eeprom;
	}

	ret = data->macc->write(data->macc, out, 0x300 + offset, len);
	if (ret <= 0) {
		data->eeprom_valid = false;
		if (ret == 0)
			ret = -EIO;
		goto error;
	}
	ret = scu_update_checksum(data);
	if (ret < 0) {
		data->eeprom_valid = false;
		goto error;
	}
	data->eeprom_valid = true;
error:
	mutex_unlock(&data->write_lock);
	return ret;
}

static ssize_t lru_part_number_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct scu_data *data = dev_get_drvdata(dev);

	return scu_object_show(buf, data->eeprom.lru_part_number,
			       sizeof(data->eeprom.lru_part_number));
}

static ssize_t lru_part_number_store(struct device *dev,
				     struct device_attribute *devattr,
				     const char *buf, size_t count)
{
	struct scu_data *data = dev_get_drvdata(dev);
	int ret;

	ret  = scu_object_store(data,
				offsetof(struct eeprom_data, lru_part_number),
				buf, data->eeprom.lru_part_number,
				sizeof(data->eeprom.lru_part_number));
	return ret < 0 ? ret : count;
}

static ssize_t lru_serial_number_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct scu_data *data = dev_get_drvdata(dev);

	return scu_object_show(buf, data->eeprom.lru_serial_number,
			       sizeof(data->eeprom.lru_serial_number));
}

static ssize_t lru_serial_number_store(struct device *dev,
				       struct device_attribute *devattr,
				       const char *buf, size_t count)
{
	struct scu_data *data = dev_get_drvdata(dev);
	int ret;

	ret = scu_object_store(data,
				offsetof(struct eeprom_data, lru_serial_number),
				buf, data->eeprom.lru_serial_number,
				sizeof(data->eeprom.lru_serial_number));
	return ret < 0 ? ret : count;
}

static ssize_t lru_revision_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct scu_data *data = dev_get_drvdata(dev);

	return scu_object_show(buf, data->eeprom.lru_revision,
			       sizeof(data->eeprom.lru_revision));
}

static ssize_t lru_revision_store(struct device *dev,
				  struct device_attribute *devattr,
				  const char *buf, size_t count)
{
	struct scu_data *data = dev_get_drvdata(dev);
	int ret;

	ret = scu_object_store(data,
				offsetof(struct eeprom_data, lru_revision),
				buf, data->eeprom.lru_revision,
				sizeof(data->eeprom.lru_revision));
	return ret < 0 ? ret : count;
}

static ssize_t lru_date_of_manufacture_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct scu_data *data = dev_get_drvdata(dev);

	return scu_object_show(buf, data->eeprom.lru_date_of_manufacture,
			       sizeof(data->eeprom.lru_date_of_manufacture));
}

static ssize_t lru_date_of_manufacture_store(struct device *dev,
					     struct device_attribute *devattr,
					     const char *buf, size_t count)
{
	struct scu_data *data = dev_get_drvdata(dev);
	int ret;

	ret = scu_object_store(data,
				offsetof(struct eeprom_data,
					 lru_date_of_manufacture),
				buf, data->eeprom.lru_date_of_manufacture,
				sizeof(data->eeprom.lru_date_of_manufacture));
	return ret < 0 ? ret : count;
}
static ssize_t board_part_number_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct scu_data *data = dev_get_drvdata(dev);

	return scu_object_show(buf, data->eeprom.board_part_number,
			       sizeof(data->eeprom.board_part_number));
}

static ssize_t board_part_number_store(struct device *dev,
				       struct device_attribute *devattr,
				       const char *buf, size_t count)
{
	struct scu_data *data = dev_get_drvdata(dev);
	int ret;

	ret = scu_object_store(data,
				offsetof(struct eeprom_data, board_part_number),
				buf, data->eeprom.board_part_number,
				sizeof(data->eeprom.board_part_number));
	return ret < 0 ? ret : count;
}

static ssize_t board_serial_number_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct scu_data *data = dev_get_drvdata(dev);

	return scu_object_show(buf, data->eeprom.board_serial_number,
			       sizeof(data->eeprom.board_serial_number));
}

static ssize_t board_serial_number_store(struct device *dev,
					 struct device_attribute *devattr,
					 const char *buf, size_t count)
{
	struct scu_data *data = dev_get_drvdata(dev);
	int ret;

	ret = scu_object_store(data,
			offsetof(struct eeprom_data, board_serial_number),
			buf, data->eeprom.board_serial_number,
			sizeof(data->eeprom.board_serial_number));
	return ret < 0 ? ret : count;
}

static ssize_t board_revision_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct scu_data *data = dev_get_drvdata(dev);

	return scu_object_show(buf, data->eeprom.board_revision,
			       sizeof(data->eeprom.board_revision));
}

static ssize_t board_revision_store(struct device *dev,
				   struct device_attribute *devattr,
				   const char *buf, size_t count)
{
	struct scu_data *data = dev_get_drvdata(dev);
	int ret;

	ret = scu_object_store(data,
				offsetof(struct eeprom_data, board_revision),
				buf, data->eeprom.board_revision,
				sizeof(data->eeprom.board_revision));
	return ret < 0 ? ret : count;
}

static ssize_t board_date_of_manufacture_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct scu_data *data = dev_get_drvdata(dev);

	return scu_object_show(buf, data->eeprom.board_date_of_manufacture,
			       sizeof(data->eeprom.board_date_of_manufacture));
}

static ssize_t board_date_of_manufacture_store(struct device *dev,
					      struct device_attribute *devattr,
					      const char *buf, size_t count)
{
	struct scu_data *data = dev_get_drvdata(dev);
	int ret;

	ret = scu_object_store(data,
				offsetof(struct eeprom_data,
					 board_date_of_manufacture),
				buf, data->eeprom.board_date_of_manufacture,
				sizeof(data->eeprom.board_date_of_manufacture));
	return ret < 0 ? ret : count;
}

static DEVICE_ATTR_RO(board_type);
static DEVICE_ATTR_RW(attribute_magic);
static DEVICE_ATTR_RW(lru_part_number);
static DEVICE_ATTR_RW(lru_serial_number);
static DEVICE_ATTR_RW(lru_revision);
static DEVICE_ATTR_RW(lru_date_of_manufacture);	/* SCU2/SCU3 only */
static DEVICE_ATTR_RW(board_part_number);
static DEVICE_ATTR_RW(board_serial_number);
static DEVICE_ATTR_RW(board_revision);
static DEVICE_ATTR_RW(board_date_of_manufacture);

static struct attribute *scu_base_attrs[] = {
	&dev_attr_board_type.attr,
	NULL,
};

static struct attribute_group scu_base_group = {
	.attrs = scu_base_attrs,
};

static struct attribute *scu_eeprom_attrs[] = {
	&dev_attr_attribute_magic.attr,
	&dev_attr_lru_part_number.attr,			/* 1 */
	&dev_attr_lru_serial_number.attr,
	&dev_attr_lru_revision.attr,
	&dev_attr_lru_date_of_manufacture.attr,		/* 4 */
	&dev_attr_board_part_number.attr,
	&dev_attr_board_serial_number.attr,
	&dev_attr_board_revision.attr,
	&dev_attr_board_date_of_manufacture.attr,
	NULL
};

static umode_t scu_attr_is_visible(struct kobject *kobj, struct attribute *attr,
				   int index)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct scu_data *data = dev_get_drvdata(dev);
	umode_t mode = attr->mode;

	/*
	 * If the eeprom has not been processed, disable its attributes.
	 * If it has been processed but is not accessible, disable
	 * write accesses to it.
	 */
	if (index >= 1 && !data->eeprom_accessible)
			mode &= S_IRUGO;

	if (index >= 4 && data->pdata->version == scu1)
		return 0;

	return mode;
}

static struct attribute_group scu_eeprom_group = {
	.attrs = scu_eeprom_attrs,
	.is_visible = scu_attr_is_visible,
};

/* platform data */

static struct gpio_led pca_gpio_leds1[] = {
	{			/* bit 0 */
	 .name = "scu_status:g:RD",
	 .gpio = SCU_RD_LED_GPIO,
	 .active_low = 1,
	 .default_trigger = "heartbeat",
	 .default_state = LEDS_GPIO_DEFSTATE_OFF,
	 },
	{			/* bit 1 */
	 .name = "scu_status:a:WLess",
	 .gpio = SCU_WLES_LED_GPIO,
	 .active_low = 1,
	 .default_trigger = "none",
	 .default_state = LEDS_GPIO_DEFSTATE_OFF,
	 },
	{			/* bit 2 */
	 .name = "scu_status:r:LDFail",
	 .gpio = SCU_LD_FAIL_LED_GPIO,
	 .active_low = 1,
	 .default_trigger = "none",
	 .default_state = LEDS_GPIO_DEFSTATE_OFF,
	 },
	{			/* bit 3 */
	 .name = "scu_status:a:SW",
	 .gpio = SCU_SW_LED_GPIO,
	 .active_low = 1,
	 .default_trigger = "none",
	 .default_state = LEDS_GPIO_DEFSTATE_OFF,
	 }
};

static struct gpio_led_platform_data pca_gpio_led_info1 = {
	.leds = pca_gpio_leds1,
	.num_leds = ARRAY_SIZE(pca_gpio_leds1),
};

static struct gpio_led pca_gpio_leds2[] = {
	{			/* bit 0 */
	 .name = "SD1:g:active",
	 .gpio = SCU_SD_ACTIVE_1_GPIO,
	 .active_low = 1,
	 .default_trigger = "none",
	 .default_state = LEDS_GPIO_DEFSTATE_OFF,
	 },
	{			/* bit 1 */
	 .name = "SD1:a:error",
	 .gpio = SCU_SD_ERROR_1_GPIO,
	 .active_low = 1,
	 .default_trigger = "none",
	 .default_state = LEDS_GPIO_DEFSTATE_OFF,
	 },
	{			/* bit 2 */
	 .name = "SD2:g:active",
	 .gpio = SCU_SD_ACTIVE_2_GPIO,
	 .active_low = 1,
	 .default_trigger = "none",
	 .default_state = LEDS_GPIO_DEFSTATE_OFF,
	 },
	{			/* bit 3 */
	 .name = "SD2:a:error",
	 .gpio = SCU_SD_ERROR_2_GPIO,
	 .active_low = 1,
	 .default_trigger = "none",
	 .default_state = LEDS_GPIO_DEFSTATE_OFF,
	 },
	{			/* bit 4 */
	 .name = "SD3:g:active",
	 .gpio = SCU_SD_ACTIVE_3_GPIO,
	 .active_low = 1,
	 .default_trigger = "none",
	 .default_state = LEDS_GPIO_DEFSTATE_OFF,
	 },
	{			/* bit 5 */
	 .name = "SD3:a:error",
	 .gpio = SCU_SD_ERROR_3_GPIO,
	 .active_low = 1,
	 .default_trigger = "none",
	 .default_state = LEDS_GPIO_DEFSTATE_OFF,
	 }
};

static struct gpio_led_platform_data pca_gpio_led_info2 = {
	.leds = pca_gpio_leds2,
	.num_leds = ARRAY_SIZE(pca_gpio_leds2),
};

static struct gpio_led pca_gpio_leds3[] = {
	{			/* bit 0 */
	 .name = "SD4:g:active",
	 .gpio = SCU_SD_ACTIVE_4_GPIO,
	 .active_low = 1,
	 .default_trigger = "none",
	 .default_state = LEDS_GPIO_DEFSTATE_OFF,
	 },
	{			/* bit 1 */
	 .name = "SD4:a:error",
	 .gpio = SCU_SD_ERROR_4_GPIO,
	 .active_low = 1,
	 .default_trigger = "none",
	 .default_state = LEDS_GPIO_DEFSTATE_OFF,
	 },
	{			/* bit 2 */
	 .name = "SD5:g:active",
	 .gpio = SCU_SD_ACTIVE_5_GPIO,
	 .active_low = 1,
	 .default_trigger = "none",
	 .default_state = LEDS_GPIO_DEFSTATE_OFF,
	 },
	{			/* bit 3 */
	 .name = "SD5:a:error",
	 .gpio = SCU_SD_ERROR_5_GPIO,
	 .active_low = 1,
	 .default_trigger = "none",
	 .default_state = LEDS_GPIO_DEFSTATE_OFF,
	 },
	{			/* bit 4 */
	 .name = "SD6:g:active",
	 .gpio = SCU_SD_ACTIVE_6_GPIO,
	 .active_low = 1,
	 .default_trigger = "none",
	 .default_state = LEDS_GPIO_DEFSTATE_OFF,
	 },
	{			/* bit 5 */
	 .name = "SD6:a:error",
	 .gpio = SCU_SD_ERROR_6_GPIO,
	 .active_low = 1,
	 .default_trigger = "none",
	 .default_state = LEDS_GPIO_DEFSTATE_OFF,
	 }
};

static struct gpio_led_platform_data pca_gpio_led_info3 = {
	.leds = pca_gpio_leds3,
	.num_leds = ARRAY_SIZE(pca_gpio_leds3),
};

static void __init pca_leds_register(struct device *parent,
				     struct scu_data *data)
{
	data->leds_pdev[0] =
		platform_device_register_data(parent, "leds-gpio", 1,
					      &pca_gpio_led_info1,
					      sizeof(pca_gpio_led_info1));
	data->leds_pdev[1] =
		platform_device_register_data(parent, "leds-gpio", 2,
					      &pca_gpio_led_info2,
					      sizeof(pca_gpio_led_info2));
	data->leds_pdev[2] =
		platform_device_register_data(parent, "leds-gpio", 3,
					      &pca_gpio_led_info3,
					      sizeof(pca_gpio_led_info3));
}

static void pca_leds_unregister(struct scu_data *data)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(data->leds_pdev); i++)
		platform_device_unregister(data->leds_pdev[i]);
}

static const char *pca9538_ext0_gpio_names[8] = {
	"pca9538_ext0:wireless_ena_1",
	"pca9538_ext0:wireless_ena_2",
	"pca9538_ext0:wireless_a_radio_disable",
	"pca9538_ext0:wireless_a_reset",
	"pca9538_ext0:in_spare_1",
	"pca9538_ext0:in_spare_2",
	"pca9538_ext0:wireless_b_radio_disable",
	"pca9538_ext0:wireless_b_reset",
};

static const char *pca9538_ext1_gpio_names[8] = {
	"pca9538_ext1:rd_led_on",
	"pca9538_ext1:wless_led_on",
	"pca9538_ext1:ld_fail_led_on",
	"pca9538_ext1:sw_led_on",
	"pca9538_ext1:discrete_out_1",
	"pca9538_ext1:discrete_out_2",
	"pca9538_ext1:discrete_out_3",
	"pca9538_ext1:discrete_out_4",
};

static const char *pca9538_ext2_gpio_names[8] = {
	"pca9538_ext2:sd_active_1",
	"pca9538_ext2:sd_error_1",
	"pca9538_ext2:sd_active_2",
	"pca9538_ext2:sd_error_2",
	"pca9538_ext2:sd_active_3",
	"pca9538_ext2:sd_error_3",
	"pca9538_ext2:hub_6_reset",
	"pca9538_ext2:hub_6_config_status",
};

static const char *pca9538_ext3_gpio_names[8] = {
	"pca9538_ext3:sd_active_4",
	"pca9538_ext3:sd_error_4",
	"pca9538_ext3:sd_active_5",
	"pca9538_ext3:sd_error_5",
	"pca9538_ext3:sd_active_6",
	"pca9538_ext3:sd_error_6",
	"pca9538_ext3:hub_2_reset",
	"pca9538_ext3:hub_2_config_status",
};

static const char *pca9557_gpio_names[8] = {
	"pca9557:port1_cd",
	"pca9557:port2_cd",
	"pca9557:port3_cd",
	"pca9557:port4_cd",
	"pca9557:port5_cd",
	"pca9557:port6_cd",
	"pca9557:spare1",
	"pca9557:spare2",
};

static int scu_gpio_common_setup(unsigned gpio_base, unsigned ngpio, u32 mask,
				 u32 is_input, u32 is_active, u32 active_low)
{
	int i;
	unsigned long flags;

	for (i = 0; i < ngpio; i++) {
		if (!(mask & (1 << i)))
			continue;
		flags = GPIOF_EXPORT_DIR_FIXED;
		if (is_input & (1 << i)) {
			flags |= GPIOF_DIR_IN;
		} else {
			flags |= GPIOF_DIR_OUT;
			if (is_active & (1 << i))
				flags |= GPIOF_INIT_HIGH;
		}
		if (active_low & (1 << i))
			flags |= GPIOF_ACTIVE_LOW;
		gpio_request_one(gpio_base + i, flags, NULL);
	}
	return 0;
}

static int pca9538_ext0_setup(struct i2c_client *client,
			      unsigned gpio_base, unsigned ngpio, void *context)
{
	scu_gpio_common_setup(gpio_base, ngpio, 0xff, 0x33, 0xcc, 0x00);
	return 0;
}

static int pca9538_ext1_setup(struct i2c_client *client,
			      unsigned gpio_base, unsigned ngpio, void *context)
{
	scu_gpio_common_setup(gpio_base, ngpio, 0xf0, 0x00, 0x00, 0x00);
	return 0;
}

static int pca9538_ext2_setup(struct i2c_client *client,
			      unsigned gpio_base, unsigned ngpio, void *context)
{
	scu_gpio_common_setup(gpio_base, ngpio, 0xc0, 0x80, 0x40, 0x00);
	return 0;
}

static int pca9538_ext3_setup(struct i2c_client *client,
			      unsigned gpio_base, unsigned ngpio, void *context)
{
	scu_gpio_common_setup(gpio_base, ngpio, 0xc0, 0x80, 0x40, 0x00);
	return 0;
}

static int pca9557_setup(struct i2c_client *client,
			 unsigned gpio_base, unsigned ngpio, void *context)
{
	scu_gpio_common_setup(gpio_base, ngpio, 0x3f, 0x3f, 0x00, 0x3f);
	return 0;
}

static void scu_gpio_common_teardown(unsigned gpio_base, int ngpio, u32 mask)
{
	int i;

	for (i = 0; i < ngpio; i++) {
		if (mask & (1 << i)) {
			gpio_unexport(gpio_base + i);
			gpio_free(gpio_base + i);
		}
	}
}

static int pca9538_ext0_teardown(struct i2c_client *client,
				 unsigned gpio_base, unsigned ngpio,
				 void *context)
{
	scu_gpio_common_teardown(gpio_base, ngpio, 0xff);
	return 0;
}

static int pca9538_ext1_teardown(struct i2c_client *client,
				 unsigned gpio_base, unsigned ngpio,
				 void *context)
{
	scu_gpio_common_teardown(gpio_base, ngpio, 0xf0);
	return 0;
}

static int pca9538_ext2_teardown(struct i2c_client *client,
				 unsigned gpio_base, unsigned ngpio,
				 void *context)
{
	scu_gpio_common_teardown(gpio_base, ngpio, 0xc0);
	return 0;
}

static int pca9538_ext3_teardown(struct i2c_client *client,
				 unsigned gpio_base, unsigned ngpio,
				 void *context)
{
	scu_gpio_common_teardown(gpio_base, ngpio, 0xc0);
	return 0;
}

static int pca9557_teardown(struct i2c_client *client,
			    unsigned gpio_base, unsigned ngpio,
			    void *context)
{
	scu_gpio_common_teardown(gpio_base, ngpio, 0x3f);
	return 0;
}

static int scu_gpiochip_match_name(struct gpio_chip *chip, void *data)
{
	return !strcmp(chip->label, data);
}

static struct gpio_chip *scu_find_chip_by_name(const char *name)
{
	return gpiochip_find((void *)name, scu_gpiochip_match_name);
}

static void pch_gpio_setup(struct scu_data *data)
{
	struct gpio_chip *chip = scu_find_chip_by_name("gpio_ich");
	struct mdio_gpio_platform_data mdio_pdata = { };

	if (chip) {
		mdio_pdata.mdc = chip->base + 17;	/* GPO1 */
		mdio_pdata.mdo = chip->base + 21;	/* GPO2 */
		mdio_pdata.mdo_active_low = true;
		mdio_pdata.mdio = chip->base + 2;	/* GPI2 */

		data->mdio_dev = platform_device_register_data(&platform_bus,
							       "mdio-gpio", 0,
							       &mdio_pdata,
							       sizeof(mdio_pdata));
		if (IS_ERR(data->mdio_dev)) {
			dev_err(data->dev, "Failed to register MDIO device\n");
			data->mdio_dev = NULL;
		}
		/* generic: 0-1, 3 (input), 16, 20 (output) */
		scu_gpio_common_setup(chip->base, 22, 0x11000b, 0x00000b, 0x0,
				      0x0);
	}
}

static void pch_gpio_teardown(struct scu_data *data)
{
	struct gpio_chip *chip = scu_find_chip_by_name("gpio_ich");

	if (chip) {
		platform_device_unregister(data->mdio_dev);
		scu_gpio_common_teardown(chip->base, 22, 0x30000b);
	}
}

static struct dsa_chip_data switch_chip_data = {
        .port_names[0]	= "cpu",
        .port_names[1]	= "netaux",
        .port_names[2]	= 0,	/* unused */
        .port_names[3]	= "netright",
        .port_names[4]	= "netleft",
        .port_names[5]	= 0,	/* unused */
};

static void scu_setup_ethernet_switch(struct scu_data *data)
{
	struct dsa_platform_data switch_data = {
        	.nr_chips = 1,
        	.chip = &switch_chip_data,
	};

	switch_data.netdev = &data->netdev->dev;
	switch_chip_data.mii_bus = &data->mdio_dev->dev;
	data->dsa_dev = platform_device_register_data(&platform_bus, "dsa",
						      0, &switch_data,
						      sizeof(switch_data));
	if (IS_ERR(data->dsa_dev)) {
		dev_err(data->dev, "Failed to register DSA device\n");
		data->dsa_dev = NULL;
	}
}

static void scu3_init(struct scu_data *data)
{
	pch_gpio_setup(data);
	if (data->mdio_dev)
		scu_setup_ethernet_switch(data);
}

static void scu3_remove(struct scu_data *data)
{
	pch_gpio_teardown(data);
	platform_device_unregister(data->dsa_dev);
}

static struct pca953x_platform_data scu_pca953x_pdata[] = {
	[0] = {.gpio_base = SCU_EXT_GPIO_BASE(0),
	       .irq_base = -1,
	       .setup = pca9538_ext0_setup,
	       .teardown = pca9538_ext0_teardown,
	       .names = pca9538_ext0_gpio_names},
	[1] = {.gpio_base = SCU_EXT_GPIO_BASE(1),
	       .irq_base = -1,
	       .setup = pca9538_ext1_setup,
	       .teardown = pca9538_ext1_teardown,
	       .names = pca9538_ext1_gpio_names},
	[2] = {.gpio_base = SCU_EXT_GPIO_BASE(2),
	       .irq_base = -1,
	       .setup = pca9538_ext2_setup,
	       .teardown = pca9538_ext2_teardown,
	       .names = pca9538_ext2_gpio_names},
	[3] = {.gpio_base = SCU_EXT_GPIO_BASE(3),
	       .irq_base = -1,
	       .setup = pca9538_ext3_setup,
	       .teardown = pca9538_ext3_teardown,
	       .names = pca9538_ext3_gpio_names},
	[4] = {.gpio_base = SCU_EXT_GPIO_BASE(4),
	       .irq_base = -1,
	       .setup = pca9557_setup,
	       .teardown = pca9557_teardown,
	       .names = pca9557_gpio_names},
};

static struct i2c_board_info scu_i2c_info_scu2[] = {
	{ I2C_BOARD_INFO("sc18is602", 0x28)},
};

#ifdef TESTING	/* removed from design */

static struct lis3lv02d_platform_data scu_lis3lv02d_data = {
	.click_flags = LIS3_CLICK_SINGLE_X | LIS3_CLICK_SINGLE_Y |
	    LIS3_CLICK_SINGLE_Z,
	/* Limits are 0.5g * value */
	.click_thresh_x = 8,
	.click_thresh_y = 8,
	.click_thresh_z = 10,
	/* Click must be longer than time limit */
	.click_time_limit = 9,
	/* Kind of debounce filter */
	.click_latency = 50,

	/* Limits for all axis. millig-value / 18 to get HW values */
	.wakeup_flags = LIS3_WAKEUP_X_HI | LIS3_WAKEUP_Y_HI,
	.wakeup_thresh = 800 / 18,
	.wakeup_flags2 = LIS3_WAKEUP_Z_HI,
	.wakeup_thresh2 = 900 / 18,

	.hipass_ctrl = LIS3_HIPASS1_DISABLE | LIS3_HIPASS2_DISABLE,

	.axis_x = LIS3_DEV_X,
	.axis_y = LIS3_INV_DEV_Y,
	.axis_z = LIS3_INV_DEV_Z,
	.st_min_limits = {-32, 3, 3},
	.st_max_limits = {-3, 32, 32},
};
#endif /* TESTING */

static struct i2c_board_info scu_i2c_info_scu3[] = {
#ifdef TESTING  /* removed from design */
	{ I2C_BOARD_INFO("lis3lv02d", 0x18),
		.platform_data = &scu_lis3lv02d_data},
#endif /* TESTING */
	{ I2C_BOARD_INFO("pca9557", 0x1b),
		.platform_data = &scu_pca953x_pdata[4],},
};

static struct spi_board_info scu_spi_info[] = {
	{
#ifdef TESTING
	 .modalias = "lm70",
	 .bus_num = 0,
	 .chip_select = 0,
	 .max_speed_hz = 1000000,
	 .mode = SPI_MODE_0,
#else
	 .modalias = "robodebug",
	 .bus_num = 0,
	 .chip_select = 0,
	 .max_speed_hz = 2000000,
	 .mode = SPI_MODE_3,
#endif
	},
};

static struct scu_platform_data scu_platform_data[] = {
	[scu1] = {
		.board_type = "SCU1 x86",
		.part_number = SCU_PARTNUM_GEN1,
		.version = scu1,
		.eeprom_len = SCU_EEPROM_LEN_GEN1,
		.i2c_board_info = scu_i2c_info_scu2,
		.num_i2c_board_info = ARRAY_SIZE(scu_i2c_info_scu2),
		.spi_board_info = scu_spi_info,
		.num_spi_board_info = ARRAY_SIZE(scu_spi_info),
	},
	[scu2] = {
		.board_type = "SCU2 x86",
		.part_number = SCU_PARTNUM_GEN2,
		.version = scu2,
		.eeprom_len = SCU_EEPROM_LEN_GEN2,
		.i2c_board_info = scu_i2c_info_scu2,
		.num_i2c_board_info = ARRAY_SIZE(scu_i2c_info_scu2),
		.spi_board_info = scu_spi_info,
		.num_spi_board_info = ARRAY_SIZE(scu_spi_info),
	},
	[scu3] = {
		.board_type = "SCU3 x86",
		.part_number = SCU_PARTNUM_GEN3,
		.version = scu3,
		.eeprom_len = SCU_EEPROM_LEN_GEN3,
		.i2c_board_info = scu_i2c_info_scu3,
		.num_i2c_board_info = ARRAY_SIZE(scu_i2c_info_scu3),
		.init = scu3_init,
		.remove = scu3_remove,
	},
};

static int scu_instantiate_i2c(struct scu_data *data, int base,
			       struct i2c_board_info *info, int count)
{
	int i;

	for (i = 0; i < count; i++) {
		data->client[base + i] = i2c_new_device(data->adapter, info);
		if (!data->client[base + i]) {
			/*
			 * Unfortunately this call does not tell us
			 * why it failed. Pick the most likely reason.
			 */
			return -EBUSY;
		}
		info++;
	}
	return 0;
}

static int scu_instantiate_spi(struct scu_data *data,
			       struct spi_board_info *info, int count)
{
	struct spi_master *master;
	int i;

	/* SPI bus number matches i2c bus number (set by sc18is602 driver) */
	master = spi_busnum_to_master(data->adapter->nr);
	if (!master) {
		dev_err(data->dev, "Failed to find SPI adapter\n");
		return -ENODEV;
	}
	data->master = master;

	for (i = 0; i < count; i++) {
		info->bus_num = master->bus_num;
		/* ignore errors */
		data->spidev[i] = spi_new_device(master, info);
		info++;
	}
	return 0;
}

/*
 * This is the callback function when a a specifc at24 eeprom is found.
 * Its reads out the eeprom contents via the read function passed back in via
 * struct memory_accessor. It then calls part_number_proc, serial_number_proc,
 * and dom_proc to populate the procfs entries for each specific field.
 */
static void populate_unit_info(struct memory_accessor *mem_accessor,
			       void *context)
{
	const struct scu_platform_data *pdata;
	struct scu_data *data = context;
	unsigned char *ptr;
	int i, len;

	data->macc = mem_accessor;

	ptr = (unsigned char *)&data->eeprom;
	/* Read Data structure from EEPROM */
	if (mem_accessor->read(mem_accessor, ptr, 0x300,
			       sizeof(data->eeprom)) <= 0) {
		dev_err(data->dev, "Failed to read eeprom data\n");
		goto error;
	}

	/* EEPROM is accessible, permit write access to it */
	data->eeprom_accessible = true;

	/* Sanity check */
	if (le16_to_cpu(data->eeprom.length) != SCU_EEPROM_LEN_EEPROM) {
		dev_err(data->dev,
			"Bad eeprom data length: Expected %d, got %d\n",
			SCU_EEPROM_LEN_EEPROM,
			le16_to_cpu(data->eeprom.length));
		goto error;
	}

	/* Update platform data based on part number retrieved from EEPROM */
	for (i = 0; i < ARRAY_SIZE(scu_platform_data); i++) {
		pdata = &scu_platform_data[i];
		if (!strncmp(data->eeprom.lru_part_number, pdata->part_number,
			     strlen(pdata->part_number))) {
			data->pdata = pdata;
			break;
		}
	}
	pdata = data->pdata;

	/*
	 * We know as much as we will ever find out about the platform.
	 * Instantiate additional I2C devices as well as SPI devices now,
	 * prior to validating the EEPROM checksum.
	 */
	if (pdata->i2c_board_info)
		scu_instantiate_i2c(data, 7, pdata->i2c_board_info,
				    pdata->num_i2c_board_info);

	if (pdata->spi_board_info)
		scu_instantiate_spi(data, pdata->spi_board_info,
				    pdata->num_spi_board_info);

	if (pdata->init)
		pdata->init(data);

	len = data->pdata->eeprom_len;

	/* Validate checksum */
	if (scu_get_checksum(ptr, len)) {
		dev_err(data->dev,
			"EEPROM data checksum error: expected 0, got 0x%x [len=%d]\n",
			scu_get_checksum(ptr, len), len);
		/* TBD: Do we want to clear the eeprom in this case ? */
		goto error_noclean;
	}

	/* Create sysfs attributes based on retrieved platform data */
	data->eeprom_valid = true;
	goto done;

error:
	memset(&data->eeprom, '\0', sizeof(data->eeprom));
	data->eeprom.length = cpu_to_le16(SCU_EEPROM_LEN_EEPROM);
error_noclean:
	data->eeprom_valid = false;
done:
	sysfs_create_group(&data->dev->kobj, &scu_eeprom_group);
}

static struct at24_platform_data at24c08 = {
	.byte_len = 1024,
	.page_size = 16,
	.setup = populate_unit_info,
};

static struct i2c_board_info scu_i2c_info_common[] = {
	{ I2C_BOARD_INFO("scu_pic", 0x20)},
	{ I2C_BOARD_INFO("at24", 0x54),
		.platform_data = &at24c08},
	{ I2C_BOARD_INFO("ds1682", 0x6b)},
	{ I2C_BOARD_INFO("pca9538", 0x70),
		.platform_data = &scu_pca953x_pdata[0],},
	{ I2C_BOARD_INFO("pca9538", 0x71),
		.platform_data = &scu_pca953x_pdata[1],},
	{ I2C_BOARD_INFO("pca9538", 0x72),
		.platform_data = &scu_pca953x_pdata[2],},
	{ I2C_BOARD_INFO("pca9538", 0x73),
		.platform_data = &scu_pca953x_pdata[3],},
};

static int scu_i2c_adap_name_match(struct device *dev, void *data)
{
	struct i2c_adapter *adap = i2c_verify_adapter(dev);

	if (!adap)
		return false;

	return !strcmp(adap->name, (char *)data);
}

static struct i2c_adapter *scu_find_i2c_adapter(char *name)
{
	struct device *dev;
	struct i2c_adapter *adap;

	dev = bus_find_device(&i2c_bus_type, NULL, name,
			      scu_i2c_adap_name_match);
	if (!dev)
		return NULL;

	adap = i2c_verify_adapter(dev);
	if (!adap)
		put_device(dev);

	return adap;
}

const char *scu_modules[] = {
	"kempld-core",
	"i2c-kempld",
	"spi-sc18is602",
	"lpc_ich",
	"gpio_ich",
	"mdio-gpio",
	"dsa",
	NULL
};

static void scu_request_modules(bool wait)
{
	struct module *m;
	int i;

	/*
	 * Try to load modules which we are going to need later on.
	 * Fail silently; if loading the module is not successful
	 * we'll bail out later on.
	 */
	for (i = 0; scu_modules[i]; i++) {
		mutex_lock(&module_mutex);
		m = find_module(scu_modules[i]);
		mutex_unlock(&module_mutex);
		if (!m) {
			if (wait)
				request_module(scu_modules[i]);
			else
				request_module_nowait(scu_modules[i]);
		}
	}
}

static int proc_board_type_show(struct seq_file *m, void *v)
{
	struct scu_data *data = (struct scu_data *)m->private;

	seq_printf(m, "%s\n", data->pdata->board_type);

	return 0;
}

static int scu_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_board_type_show, PDE_DATA(inode));
}

static const struct file_operations scu_proc_fops = {
	.owner = THIS_MODULE,
	.open = scu_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int scu_probe(struct platform_device *pdev)
{
	struct proc_dir_entry *rave_board_type;
	struct device *dev = &pdev->dev;
	struct i2c_adapter *adapter;
	struct scu_data *data;
	int i, ret;

	scu_request_modules(true);

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);

	data->pdata = dev_get_platdata(dev);
	if (!data->pdata)
		return -ENODEV;
	data->dev = dev;

	mutex_init(&data->write_lock);

	data->netdev = dev_get_by_name(&init_net, "eth0");
	if (!data->netdev)
		return -EPROBE_DEFER;

	/*
	 * The adapter driver should have been loaded by now.
	 * If not, try again later.
	 */
	adapter = scu_find_i2c_adapter("i2c-kempld");
	if (!adapter) {
		ret = -EPROBE_DEFER;
		goto error_put_net;
	}
	data->adapter = adapter;

	data->rave_proc_dir = proc_mkdir("rave", NULL);
	if (!data->rave_proc_dir) {
		ret = -ENODEV;
		goto error_put;
	}
	rave_board_type = proc_create_data("board_type", 0, data->rave_proc_dir,
					   &scu_proc_fops, data);
	if (rave_board_type == NULL) {
		ret = -ENODEV;
		goto error_remove;
	}

	at24c08.context = data;

	ret = scu_instantiate_i2c(data, 0, scu_i2c_info_common,
				  ARRAY_SIZE(scu_i2c_info_common));
	if (ret)
		goto error_i2c_client;

	pca_leds_register(dev, data);

	ret = sysfs_create_group(&dev->kobj, &scu_base_group);
	if (ret)
		goto error_group;

	return 0;

error_group:
	pca_leds_unregister(data);
error_i2c_client:
	for (i = 0; i < ARRAY_SIZE(data->client) && data->client[i]; i++)
		i2c_unregister_device(data->client[i]);
error_remove:
	proc_remove(data->rave_proc_dir);
error_put:
	put_device(&adapter->dev);
error_put_net:
	dev_put(data->netdev);
	return ret;
}

static int __exit scu_remove(struct platform_device *pdev)
{
	struct scu_data *data = platform_get_drvdata(pdev);
	int i;

	sysfs_remove_group(&pdev->dev.kobj, &scu_eeprom_group);
	sysfs_remove_group(&pdev->dev.kobj, &scu_base_group);

	if (data->pdata->remove)
		data->pdata->remove(data);

	pca_leds_unregister(data);

	for (i = 0; i < ARRAY_SIZE(data->spidev); i++)
		spi_unregister_device(data->spidev[i]);
	spi_master_put(data->master);

	for (i = 0; i < ARRAY_SIZE(data->client) && data->client[i]; i++)
		i2c_unregister_device(data->client[i]);

	proc_remove(data->rave_proc_dir);

	put_device(&data->adapter->dev);
	dev_put(data->netdev);

	return 0;
}

static struct platform_driver scu_driver = {
	.probe = scu_probe,
	.remove = __exit_p(scu_remove),
	.driver = {
		.name = "scu",
		.owner = THIS_MODULE,
	},
};

static struct platform_device *scu_pdev;

static int scu_create_platform_device(const struct dmi_system_id *id)
{
	struct scu_platform_data *pdata = id->driver_data;
	int ret;

	if (pdata == NULL)
		return -EINVAL;

	scu_pdev = platform_device_alloc("scu", -1);
	if (!scu_pdev)
		return -ENOMEM;

	ret = platform_device_add_data(scu_pdev, pdata, sizeof(*pdata));
	if (ret)
		goto err;

	ret = platform_device_add(scu_pdev);
	if (ret)
		goto err;

	return 0;
err:
	platform_device_put(scu_pdev);
	return ret;
}

static const struct dmi_system_id scu_device_table[] __initconst = {
	{
		.ident = "IMS SCU version 1, Core 2 Duo",
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "Kontron"),
			DMI_MATCH(DMI_BOARD_NAME, "PXT"),
		},
		.driver_data = &scu_platform_data[scu1],
		.callback = scu_create_platform_device,
	},
	{
		.ident = "IMS SCU version 2, Ivy Bridge",
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "Kontron"),
			DMI_MATCH(DMI_BOARD_NAME, "COMe-bSC6"),
		},
		.driver_data = &scu_platform_data[scu2],
		.callback = scu_create_platform_device,
	},
	{
		.ident = "IMS SCU version 2, Ivy Bridge",
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "Kontron"),
			DMI_MATCH(DMI_BOARD_NAME, "COMe-bIP2"),
		},
		.driver_data = &scu_platform_data[scu2],
		.callback = scu_create_platform_device,
	},
	{
		.ident = "IMS SCU version 2, Sandy Bridge",
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "Kontron"),
			DMI_MATCH(DMI_BOARD_NAME, "COMe-bSC2"),
		},
		.driver_data = &scu_platform_data[scu2],
		.callback = scu_create_platform_device,
	},
	{ }
};
MODULE_DEVICE_TABLE(dmi, scu_device_table);

static int __init scu_init(void)
{
	if (!dmi_check_system(scu_device_table))
		return -ENODEV;

	scu_request_modules(false);

	return platform_driver_register(&scu_driver);
}

static void __exit scu_exit(void)
{
	if (scu_pdev)
		platform_device_unregister(scu_pdev);

	platform_driver_unregister(&scu_driver);
}

module_init(scu_init);
module_exit(scu_exit);

MODULE_ALIAS("platform:scu");
MODULE_LICENSE("GPL");
