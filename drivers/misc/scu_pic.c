/*
 * scu_pic.c - thermal monitoring, fan control, and fault LED control.
 *
 * Copyright (C) 2012 Guenter Roeck
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/watchdog.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include "scu_pic.h"

static const unsigned short normal_i2c[] = { 0x20, I2C_CLIENT_END };

struct scu_pic_data {
	struct i2c_client *client;
	struct mutex i2c_lock;
	struct kref kref;
	struct list_head list;		/* member of scu_pic_data_list */

	/* hwmon */
	struct device *hwmon_dev;
	u8 version_major;
	u8 version_minor;
	u8 reset_reason;

	bool valid;			/* true if following fields are valid */
	unsigned long last_updated;	/* In jiffies */

	u8 fan[2];
	u8 pwm_state;	/* same for both fans, only one control available */
	u8 pwm;		/* for manual control, only off/on supported */
	s8 temp[2];

	/* wdt data */
	struct watchdog_device wdt_dev;
	struct timer_list wdt_timer;
	unsigned long wdt_lastping;

	/* led data */
	struct led_classdev cdev;
};

#define to_scu_pic_data_miscdev(d) container_of(d, struct scu_pic_data, wdt_miscdev)

/*
 * Global data pointer list with all scu_pic devices, so that we can find
 * our device data. When using misc_register there is no other method
 * to get to ones device data from the open fop.
 */
static LIST_HEAD(scu_pic_data_list);
/* Note this lock not only protects list access, but also data.kref access */
static DEFINE_MUTEX(scu_pic_data_mutex);

static void scu_pic_release_resources(struct kref *ref)
{
	struct scu_pic_data *data = container_of(ref, struct scu_pic_data, kref);
	kfree(data);
}

#define WDT_IN_USE			0
#define WDT_EXPECT_CLOSE		1

#define TEMP_FROM_REG(val)		((val) * 1000)

#define FAN_FROM_REG(reg, div)		((reg) && (reg) != 0xff ? \
					 (11250 * 60) / ((reg) * (div)) : 0)

#define SCU_PIC_WDT_TIMEOUT	300		/* 5 minutes */

static int nowayout = WATCHDOG_NOWAYOUT;

module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started");

static int scu_pic_xfer(struct i2c_adapter *adapter, u16 addr, unsigned short flags,
		    bool is_read, u8 command, u8 *data)
{
	unsigned char msgbuf0[4];
	unsigned char msgbuf1[2];
	int num = is_read ? 2 : 1;
	struct i2c_msg msg[2] = { { addr, flags, 2, msgbuf0 },
				  { addr, flags | I2C_M_RD, 1, msgbuf1 }
				};
	int status;

	msgbuf0[0] = addr;
	msgbuf0[1] = command;
	if (!is_read) {
		msgbuf0[2] = *data;
		msg[0].len = 3;
	}

	status = i2c_transfer(adapter, msg, num);
	if (status < 0)
		return status;

	if (is_read)
		*data = msgbuf1[0];

	return 0;
}

static int scu_pic_read_value(struct i2c_client *client, u8 reg)
{
	u8 data;
	int ret;

	ret = scu_pic_xfer(client->adapter, client->addr, client->flags, true,
		       reg, &data);
	if (ret)
		return ret;
	return data;
}

static int
scu_pic_write_value(struct i2c_client *client, u8 reg, u8 value)
{
	return scu_pic_xfer(client->adapter, client->addr, client->flags, false,
			reg, &value);
}

/* hardware monitoring */

static struct scu_pic_data * get_reset_reason(struct device *dev) 
{
	struct i2c_client *client = to_i2c_client(dev);
	struct scu_pic_data *data = i2c_get_clientdata(client);
	/*
	 * Get the reset reason from the PIC
     */
	data->reset_reason = scu_pic_read_value(client, I2C_GET_SCU_PIC_RESET_REASON);

	return data;
}


static ssize_t show_reset_reason(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct scu_pic_data *data = get_reset_reason(dev);
	return sprintf(buf, "%d\n", data->reset_reason);
}

static DEVICE_ATTR(reset_reason, S_IRUGO, show_reset_reason, NULL);


static struct scu_pic_data *scu_pic_update_device(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct scu_pic_data *data = i2c_get_clientdata(client);
	int reg;

	mutex_lock(&data->i2c_lock);

	/*
	 * Default ADC sampling rate is 1 second, so wait a little longer
	 * before updating data.
	 */
	if (time_after(jiffies, data->last_updated + HZ + 1) || !data->valid) {
		data->fan[0] = scu_pic_read_value(client, I2C_GET_SCU_PIC_FAN1_SPEED);
		data->fan[1] = scu_pic_read_value(client, I2C_GET_SCU_PIC_FAN2_SPEED);
		reg = scu_pic_read_value(client, I2C_GET_SCU_PIC_THERMAL_CONTROL_STATE);
		if (reg == 1) {		/* auto */
			data->pwm_state = 2;
			data->pwm = 255;
		} else {		/* manual */
			data->pwm_state = 1;
			/* Assume pwm is 0 if fans are off. */
			if (data->fan[0] == 0 && data->fan[1] == 0)
				data->pwm = 0;
			else
				data->pwm = 255;
		}
		data->temp[0] = scu_pic_read_value(client, I2C_GET_SCU_PIC_LOCAL_TEMP);
		data->temp[1] = scu_pic_read_value(client, I2C_GET_SCU_PIC_REMOTE_TEMP);

		data->last_updated = jiffies;
		data->valid = 1;
	}
	mutex_unlock(&data->i2c_lock);

	return data;
}

static ssize_t show_pwm(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct scu_pic_data *data = scu_pic_update_device(dev);
	return sprintf(buf, "%d\n", data->pwm);
}

static ssize_t set_pwm(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct scu_pic_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;
	if (val > 255)
		return -EINVAL;

	mutex_lock(&data->i2c_lock);
	/* Can not override automatic control, ignore write if enabled */
	if (data->pwm_state == 2)
		val = 255;
	data->pwm = val ? 255 : 0;
	scu_pic_write_value(client, I2C_SET_SCU_PIC_FAN_STATE, val ? 1 : 0);
	mutex_unlock(&data->i2c_lock);
	return count;
}

static DEVICE_ATTR(pwm1, S_IRUGO | S_IWUSR, show_pwm, set_pwm);
static DEVICE_ATTR(pwm2, S_IRUGO | S_IWUSR, show_pwm, set_pwm);

static ssize_t show_pwm_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct scu_pic_data *data = scu_pic_update_device(dev);
	return sprintf(buf, "%d\n", data->pwm_state);
}

static ssize_t set_pwm_enable(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct scu_pic_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;
	if (val == 0 || val > 2)
		return -EINVAL;

	mutex_lock(&data->i2c_lock);
	data->pwm_state = val;
	scu_pic_write_value(client, I2C_SET_SCU_PIC_THERMAL_CONTROL_STATE,
			val == 2 ? 1 : 0);
	/* Manual control turns off fans, reflect in pwm value. */
	data->pwm = val == 2 ? 255 : 0;
	mutex_unlock(&data->i2c_lock);
	return count;
}

static DEVICE_ATTR(pwm1_enable, S_IRUGO | S_IWUSR, show_pwm_enable,
		   set_pwm_enable);
static DEVICE_ATTR(pwm2_enable, S_IRUGO | S_IWUSR, show_pwm_enable,
		   set_pwm_enable);

static ssize_t show_fan(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct scu_pic_data *data = scu_pic_update_device(dev);
	int div = data->pwm_state == 2 ? 2 : 1;

	return sprintf(buf, "%d\n", FAN_FROM_REG(data->fan[nr], div));
}

static ssize_t show_fan_div(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct scu_pic_data *data = scu_pic_update_device(dev);
	int div = data->pwm_state == 2 ? 2 : 1;

	return sprintf(buf, "%d\n", div);
}

static SENSOR_DEVICE_ATTR(fan1_input, S_IRUGO, show_fan, NULL, 0);
static SENSOR_DEVICE_ATTR(fan2_input, S_IRUGO, show_fan, NULL, 1);
static SENSOR_DEVICE_ATTR(fan1_div, S_IRUGO, show_fan_div, NULL, 0);
static SENSOR_DEVICE_ATTR(fan2_div, S_IRUGO, show_fan_div, NULL, 1);

static ssize_t show_temp(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct scu_pic_data *data = scu_pic_update_device(dev);

	return sprintf(buf, "%d\n", TEMP_FROM_REG(data->temp[nr]));
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO,	show_temp, NULL, 0);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO,	show_temp, NULL, 1);

static ssize_t show_label(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	int nr = to_sensor_dev_attr(attr)->index;

	return sprintf(buf, "%s\n", nr ? "remote" : "local");
}

static SENSOR_DEVICE_ATTR(temp1_label, S_IRUGO,	show_label, NULL, 0);
static SENSOR_DEVICE_ATTR(temp2_label, S_IRUGO,	show_label, NULL, 1);

static ssize_t show_version(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct scu_pic_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%u.%u\n", data->version_major, data->version_minor);
}

static DEVICE_ATTR(version, S_IRUGO, show_version, NULL);

static struct i2c_client *scu_pic_reset_client;

/*
 * If linked into the kernel, the following function will replace
 * mach_reboot_fixups() in arch/x86/kernel/reboot.c.
 */
void mach_reboot_fixups(void)
{
	/* Don't bother about mutexes here */
	if (scu_pic_reset_client)
		scu_pic_write_value(scu_pic_reset_client, I2C_SET_SCU_PIC_RESET_HOST, 1);

	/* is 20 ms enough time ? */
	mdelay(20);
}

static ssize_t show_reset(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0\n");
}

static ssize_t set_reset(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;
	if (val != 1)
		return -EINVAL;

	mach_reboot_fixups();
	return count;
}

static DEVICE_ATTR(reset, S_IRUGO | S_IWUSR, show_reset, set_reset);

static struct attribute *scu_pic_attributes[] = {
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_fan1_div.dev_attr.attr,
	&sensor_dev_attr_fan2_input.dev_attr.attr,
	&sensor_dev_attr_fan2_div.dev_attr.attr,
	&dev_attr_pwm1.attr,
	&dev_attr_pwm1_enable.attr,
	&dev_attr_pwm2.attr,
	&dev_attr_pwm2_enable.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp1_label.dev_attr.attr,
	&sensor_dev_attr_temp2_label.dev_attr.attr,
	&dev_attr_version.attr,
	&dev_attr_reset.attr,
	&dev_attr_reset_reason.attr,
	NULL
};

static const struct attribute_group scu_pic_group = {
	.attrs = scu_pic_attributes,
};

static int scu_pic_hwmon_init(struct i2c_client *client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	int err;

	/* Register sysfs hooks */
	err = sysfs_create_group(&dev->kobj, &scu_pic_group);
	if (err)
		return err;

	data->hwmon_dev = hwmon_device_register(dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove;
	}
	return 0;

exit_remove:
	sysfs_remove_group(&dev->kobj, &scu_pic_group);
	return err;
}

static void scu_pic_hwmon_exit(struct i2c_client *client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &scu_pic_group);
}

/* led */

static void scu_pic_led_set(struct led_classdev *led_cdev,
			enum led_brightness brightness)
{
	struct scu_pic_data *data
		    = container_of(led_cdev, struct scu_pic_data, cdev);

	mutex_lock(&data->i2c_lock);
	scu_pic_write_value(data->client, I2C_SET_SCU_PIC_FAULT_LED_STATE,
			brightness);
	mutex_unlock(&data->i2c_lock);
}

static enum led_brightness scu_pic_led_get(struct led_classdev *led_cdev)
{
	struct scu_pic_data *data
		    = container_of(led_cdev, struct scu_pic_data, cdev);
	int ret;

	mutex_lock(&data->i2c_lock);
	ret = scu_pic_read_value(data->client, I2C_GET_SCU_PIC_FAULT_LED_STATE);
	mutex_unlock(&data->i2c_lock);
	return ret;
}

static int scu_pic_led_init(struct i2c_client *client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);

	data->cdev.name = "scu_status:r:Fault";
	data->cdev.brightness_set = scu_pic_led_set;
	data->cdev.brightness_get = scu_pic_led_get;
	data->cdev.max_brightness = 1;
	data->cdev.flags = LED_CORE_SUSPENDRESUME;
	return led_classdev_register(&client->dev, &data->cdev);
}

static void scu_pic_led_exit(struct i2c_client *client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);

	led_classdev_unregister(&data->cdev);
}

/* wdt */

static int scu_pic_wdt_ping(struct watchdog_device *wdev)
{
	struct scu_pic_data *data = watchdog_get_drvdata(wdev);

	mutex_lock(&data->i2c_lock);
	/* Any host communication resets watchdog */
	if (data->client)
		(void)scu_pic_read_value(data->client, I2C_GET_SCU_PIC_WDT_STATE);
	mod_timer(&data->wdt_timer,
		  jiffies + min_t(unsigned long, SCU_PIC_WDT_TIMEOUT / 2,
				  wdev->timeout) * HZ);
	data->wdt_lastping = jiffies;
	mutex_unlock(&data->i2c_lock);

	return 0;
}

static int scu_pic_wdt_start(struct watchdog_device *wdev)
{
	struct scu_pic_data *data = watchdog_get_drvdata(wdev);

	mutex_lock(&data->i2c_lock);
	if (data->client)
		scu_pic_write_value(data->client, I2C_SET_SCU_PIC_WDT_STATE, 1);
	mod_timer(&data->wdt_timer, jiffies + min_t(unsigned long,
						    SCU_PIC_WDT_TIMEOUT / 2,
						    wdev->timeout) * HZ);
	mutex_unlock(&data->i2c_lock);

	return 0;
}

static int scu_pic_wdt_stop(struct watchdog_device *wdev)
{
	struct scu_pic_data *data = watchdog_get_drvdata(wdev);

	mutex_lock(&data->i2c_lock);
	if (data->client)
		scu_pic_write_value(data->client, I2C_SET_SCU_PIC_WDT_STATE, 0);
	del_timer(&data->wdt_timer);
	mutex_unlock(&data->i2c_lock);

	return 0;
}

static int scu_pic_wdt_set_timeout(struct watchdog_device *wdev, unsigned int t)
{
	wdev->timeout = t;
	scu_pic_wdt_ping(wdev);

	return 0;
}

static void scu_pic_wdt_timerfunc(unsigned long d)
{
	struct watchdog_device *wdev = (struct watchdog_device *)d;
	struct scu_pic_data *data = watchdog_get_drvdata(wdev);

	if (time_after(jiffies, data->wdt_lastping + wdev->timeout * HZ)) {
	        pr_crit("Software watchdog timeout: Initiating system reboot.\n");
		emergency_restart();
	}
	scu_pic_wdt_ping(wdev);
}

static const struct watchdog_info scu_pic_wdt_ident = {
	.options	= WDIOF_MAGICCLOSE | WDIOF_KEEPALIVEPING |
			  WDIOF_SETTIMEOUT,
	.identity	= "SCU Pic Watchdog",
};

static const struct watchdog_ops scu_pic_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= scu_pic_wdt_start,
	.stop		= scu_pic_wdt_stop,
	.ping		= scu_pic_wdt_ping,
	.set_timeout	= scu_pic_wdt_set_timeout,
};

static int scu_pic_wdt_init(struct i2c_client *client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&scu_pic_data_mutex);

#if defined(CONFIG_SCU_PIC_MODULE)
	/* Disable watchdog at startup if running as module */
	scu_pic_write_value(data->client, I2C_SET_scu_pic_WDT_STATE, 0);
#endif

	watchdog_set_drvdata(&data->wdt_dev, data);

	data->wdt_timer.function = scu_pic_wdt_timerfunc;
	data->wdt_timer.data = (unsigned long)&data->wdt_dev;
	init_timer(&data->wdt_timer);

	data->wdt_dev.info = &scu_pic_wdt_ident;
	data->wdt_dev.ops = &scu_pic_wdt_ops;
	data->wdt_dev.timeout = SCU_PIC_WDT_TIMEOUT;
	data->wdt_dev.min_timeout = 1;
	data->wdt_dev.max_timeout = 0xffff;
	watchdog_set_nowayout(&data->wdt_dev, nowayout);
	data->wdt_dev.parent = client->dev.parent;

	ret = watchdog_register_device(&data->wdt_dev);
	if (ret)
		goto error;
	list_add(&data->list, &scu_pic_data_list);
error:
	mutex_unlock(&scu_pic_data_mutex);
	return 0;
}

static int scu_pic_wdt_exit(struct i2c_client *client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);

	del_timer(&data->wdt_timer);

	watchdog_unregister_device(&data->wdt_dev);
	mutex_lock(&scu_pic_data_mutex);
	list_del(&data->list);
	mutex_unlock(&scu_pic_data_mutex);

	/* Tell watchdog code that client is gone */
	mutex_lock(&data->i2c_lock);
	data->client = NULL;
	mutex_unlock(&data->i2c_lock);

	return 0;
}

/* device level functions */

static int scu_pic_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct scu_pic_data *data;
	int err;
	int major, minor;

	major = scu_pic_read_value(client, I2C_GET_SCU_PIC_FIRMWARE_REV_MAJOR);
	minor = scu_pic_read_value(client, I2C_GET_SCU_PIC_FIRMWARE_REV_MINOR);

	if (major < 0 || minor < 0) {
		dev_err(dev, "major %d minor %d\n", major, minor);
		return -ENODEV;
	}

	data = kzalloc(sizeof(struct scu_pic_data), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "out of memory\n");
		err = -ENOMEM;
		goto exit;
	}

	client->flags &= ~I2C_CLIENT_PEC;	/* PEC is not supported */

	i2c_set_clientdata(client, data);
	data->client = client;
	scu_pic_reset_client = client;

	data->version_major = major;
	data->version_minor = minor;

	mutex_init(&data->i2c_lock);
	kref_init(&data->kref);

	err = scu_pic_wdt_init(client);
	if (err) {
		dev_err(dev, "Failed to init watchdog (%d)\n", err);
		goto exit_free;
	}

	err = scu_pic_led_init(client);
	if (err) {
		dev_err(dev, "Failed to init leds (%d)\n", err);
		goto exit_wdt;
	}

	err = scu_pic_hwmon_init(client);
	if (err) {
		dev_err(dev, "Failed to init hwmon (%d)\n", err);
		goto exit_led;
	}

	dev_info(dev, "SCU PIC Firmware revision %d.%d\n", major, minor);

	return 0;

exit_led:
	scu_pic_led_exit(client);
exit_wdt:
	scu_pic_wdt_exit(client);
exit_free:
	kfree(data);
exit:
	return err;
}

static int scu_pic_remove(struct i2c_client *client)
{
	struct scu_pic_data *data = i2c_get_clientdata(client);

	scu_pic_reset_client = NULL;

	scu_pic_hwmon_exit(client);
	scu_pic_led_exit(client);
	scu_pic_wdt_exit(client);

	mutex_lock(&scu_pic_data_mutex);
	kref_put(&data->kref, scu_pic_release_resources);
	mutex_unlock(&scu_pic_data_mutex);

	return 0;
}

static const struct i2c_device_id scu_pic_id[] = {
	{ "scu_pic", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, scu_pic_id);

static struct i2c_driver scu_pic_driver = {
	.driver = {
		.name = "scu_pic",
	},
	.probe		= scu_pic_probe,
	.remove		= scu_pic_remove,
	.id_table	= scu_pic_id,
	.address_list	= normal_i2c,
};

module_i2c_driver(scu_pic_driver);

MODULE_AUTHOR("Guenter Roeck <linux@roeck-us.net>");
MODULE_DESCRIPTION("SCU PIC driver");
MODULE_LICENSE("GPL");
