/*
 * touch_hwif.c
 *
 * Copyright (c) 2015 LGE.
 *
 * author : hoyeon.jang@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/version.h>

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>
#include <touch_i2c.h>
#include <touch_spi.h>
#include <touch_hwif.h>
#include <linux/export.h>

/* -- gpio -- */
int touch_gpio_init(int pin, const char *name)
{
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s - pin:%d, name:%s\n", __func__, pin, name);

	if (gpio_is_valid(pin))
		ret = gpio_request(pin, name);

	return ret;
}
EXPORT_SYMBOL(touch_gpio_init);

int touch_gpio_direction_input(int pin)
{
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s - pin:%d\n", __func__, pin);

	if (gpio_is_valid(pin))
		ret = gpio_direction_input(pin);

	return ret;
}
EXPORT_SYMBOL(touch_gpio_direction_input);

int touch_gpio_direction_output(int pin, int value)
{
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s - pin:%d, value:%d\n", __func__, pin, value);

	if (gpio_is_valid(pin))
		ret = gpio_direction_output(pin, value);

	return ret;
}
EXPORT_SYMBOL(touch_gpio_direction_output);

/* -- power -- */
int touch_power_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	if (gpio_is_valid(ts->vcl_pin)) {
		ret = gpio_request(ts->vcl_pin, "touch-vcl");
	} else {
		ts->vcl = regulator_get(dev, "vcl");
		if (IS_ERR(ts->vcl))
			TOUCH_I("regulator \"vcl\" not exist\n");
	}

	if (gpio_is_valid(ts->vdd_pin)) {
		ret = gpio_request(ts->vdd_pin, "touch-vdd");
	} else {
		ts->vdd = regulator_get(dev, "vdd");
		if (IS_ERR(ts->vdd))
			TOUCH_I("regulator \"vdd\" not exist\n");
	}

	return ret;
}
EXPORT_SYMBOL(touch_power_init);

void touch_power_3_3_vcl(struct device *dev, int value)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	if (gpio_is_valid(ts->vcl_pin)) {
		ret = gpio_direction_output(ts->vcl_pin, value);
	} else if (!IS_ERR_OR_NULL(ts->vcl)) {
		if (value)
			ret = regulator_enable((struct regulator *)ts->vcl);
		else
			ret = regulator_disable((struct regulator *)ts->vcl);
	}

	if (ret)
		TOUCH_E("ret = %d\n", ret);
}
EXPORT_SYMBOL(touch_power_3_3_vcl);

void touch_power_1_8_vdd(struct device *dev, int value)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	int enabled = 0;

	TOUCH_TRACE();

	if (gpio_is_valid(ts->vdd_pin)) {
		ret = gpio_direction_output(ts->vdd_pin, value);
	} else if (!IS_ERR_OR_NULL(ts->vdd)) {
		enabled = regulator_is_enabled((struct regulator *)ts->vdd); // Regulator State Enabled : 1 , Disabled : 0
		if (value) {
			if (enabled)
				TOUCH_I("1_8_vdd_supply already enabled\n");
			else
				ret = regulator_enable((struct regulator *)ts->vdd);
		} else {
			if (enabled)
				ret = regulator_disable((struct regulator *)ts->vdd);
			else
				TOUCH_I("1_8_vdd_supply already disabled\n");
		}
	}

	if (ret)
		TOUCH_E("ret = %d\n", ret);
}
EXPORT_SYMBOL(touch_power_1_8_vdd);

int touch_bus_init(struct device *dev, int buf_size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	if (buf_size) {
		ts->tx_buf = devm_kzalloc(dev, buf_size, GFP_KERNEL | GFP_DMA);
		if (!ts->tx_buf)
			TOUCH_E("fail to allocate tx_buf\n");

		ts->rx_buf = devm_kzalloc(dev, buf_size, GFP_KERNEL | GFP_DMA);
		if (!ts->rx_buf)
			TOUCH_E("fail to allocate rx_buf\n");
	}

	ts->xfer = devm_kzalloc(dev, sizeof(struct touch_xfer_msg), GFP_KERNEL);
	if (!ts->xfer)
		TOUCH_E("fail to allocate xfer\n");

	ts->pinctrl.ctrl = devm_pinctrl_get(dev);

	if (IS_ERR_OR_NULL(ts->pinctrl.ctrl)) {
		if (PTR_ERR(ts->pinctrl.ctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		TOUCH_I("Target does not use pinctrl\n");
		ts->pinctrl.ctrl = NULL;
	} else {
		ts->pinctrl.active = pinctrl_lookup_state(ts->pinctrl.ctrl,
				"touch_pin_active");

		if (IS_ERR_OR_NULL(ts->pinctrl.active))
			TOUCH_E("cannot get pinctrl.active\n");

		ts->pinctrl.suspend = pinctrl_lookup_state(ts->pinctrl.ctrl,
				"touch_pin_sleep");

		if (IS_ERR_OR_NULL(ts->pinctrl.suspend))
			TOUCH_E("cannot get pinctrl.suspend\n");

		if (!IS_ERR_OR_NULL(ts->pinctrl.active)) {
			ret = pinctrl_select_state(ts->pinctrl.ctrl,
					ts->pinctrl.active);
			if (ret)
				TOUCH_I("cannot set pinctrl.active\n");
			else
				TOUCH_I("pinctrl set active\n");
		}
	}

	if (ts->bus_type == HWIF_SPI) {
		ret = spi_setup(to_spi_device(dev));

		if (ret < 0) {
			TOUCH_E("Failed to perform SPI setup\n");
			return -ENODEV;
		}
	}

	return ret;
}
EXPORT_SYMBOL(touch_bus_init);

int touch_bus_read(struct device *dev, struct touch_bus_msg *msg)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	if (ts->bus_type == HWIF_I2C)
		ret = touch_i2c_read(to_i2c_client(dev), msg);
	else if (ts->bus_type == HWIF_SPI) {
		if (atomic_read(&ts->state.pm) >= DEV_PM_SUSPEND) {
			TOUCH_E("bus_read when pm_suspend");
			return -EDEADLK;
		}
		ret = touch_spi_read(to_spi_device(dev), msg);
	}

	return ret;
}
EXPORT_SYMBOL(touch_bus_read);

int touch_bus_write(struct device *dev, struct touch_bus_msg *msg)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	if (ts->bus_type == HWIF_I2C)
		ret = touch_i2c_write(to_i2c_client(dev), msg);
	else if (ts->bus_type == HWIF_SPI) {
		if (atomic_read(&ts->state.pm) >= DEV_PM_SUSPEND) {
			TOUCH_E("bus_write when pm_suspend");
			return -EDEADLK;
		}
		ret = touch_spi_write(to_spi_device(dev), msg);
	}

	return ret;
}
EXPORT_SYMBOL(touch_bus_write);

int touch_bus_xfer(struct device *dev, struct touch_xfer_msg *xfer)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	if (ts->bus_type == HWIF_I2C) {
		TOUCH_E("Not implemented\n");
		ret = -1;
	} else if (ts->bus_type == HWIF_SPI) {
		ret = touch_spi_xfer(to_spi_device(dev), xfer);
	}

	return ret;
}
EXPORT_SYMBOL(touch_bus_xfer);

void touch_enable_irq_wake(unsigned int irq)
{
	TOUCH_TRACE();
	enable_irq_wake(irq);
}

void touch_disable_irq_wake(unsigned int irq)
{
	TOUCH_TRACE();
	disable_irq_wake(irq);
}

#define istate core_internal_state__do_not_mess_with_it
void touch_enable_irq(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	TOUCH_TRACE();
	if (desc) {
		if (desc->istate & IRQS_PENDING)
			TOUCH_D(BASE_INFO, "Remove pending irq(%d)\n", irq);
		desc->istate &= ~(IRQS_PENDING);
	}
	enable_irq(irq);
}

void touch_disable_irq(unsigned int irq)
{
	TOUCH_TRACE();

	disable_irq_nosync(irq);
}

int touch_request_irq(unsigned int irq, irq_handler_t handler,
		     irq_handler_t thread_fn,
		     unsigned long flags, const char *name, void *dev)
{
	return request_threaded_irq(irq, handler, thread_fn, flags, name, dev);
}

void touch_resend_irq(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	TOUCH_TRACE();
	if (desc) {
		if (desc->istate & IRQS_PENDING)
			TOUCH_D(BASE_INFO, "irq(%d) pending\n", irq);
#if !IS_ENABLED (CONFIG_LGE_TOUCH_DLKM)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
		check_irq_resend(desc);
#else
		check_irq_resend(desc, irq);
#endif
#endif
	}
}

int touch_check_irq(unsigned int irq, struct device *dev)
{
	struct irq_desc *desc = irq_to_desc(irq);
	struct irqaction *action;
	action = desc->action;
	if (desc == NULL || action == NULL) {
		TOUCH_I("irq is not connected");
		return -EPERM;
	} else {
		TOUCH_I("irq checked: %d", desc->irq_data.irq);
	}
	return 0;
}

void touch_free_irq(unsigned int irq, struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	if (touch_check_irq(irq, dev) == 0) {
		TOUCH_I("try to free irq");
		free_irq(irq, ts);
	}
}

void touch_set_irq_pending(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long flags;

	TOUCH_TRACE();

	TOUCH_D(BASE_INFO, "%s : irq=%d\n", __func__, irq);

	if (desc) {
		raw_spin_lock_irqsave(&desc->lock, flags);
		desc->istate |= IRQS_PENDING;
		raw_spin_unlock_irqrestore(&desc->lock, flags);
	}
}

int touch_check_boot_mode(struct device *dev)
{
	int ret = TOUCH_NORMAL_BOOT;
#ifdef CONFIG_LGE_USB_FACTORY
	bool factory_boot = false;
	struct touch_core_data *ts = to_touch_core(dev);
#endif

	TOUCH_TRACE();

#ifdef CONFIG_MACH_LGE
	if (lge_check_recoveryboot()) {
		ret = TOUCH_RECOVERY_MODE;
		return ret;
	}
#endif

#ifdef CONFIG_LGE_USB_FACTORY
	if (lge_get_laf_mode() == LGE_LAF_MODE_LAF) {
		ret = TOUCH_LAF_MODE;
		return ret;
	}

	if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) {
		ret = TOUCH_CHARGER_MODE;
		return ret;
	}

	factory_boot = lge_get_factory_boot();

	if (factory_boot) {
		switch (atomic_read(&ts->state.mfts)) {
		case MFTS_NONE:
			ret = TOUCH_MINIOS_AAT;
			break;
		case MFTS_FOLDER:
			ret = TOUCH_MINIOS_MFTS_FOLDER;
			break;
		case MFTS_FLAT:
			ret = TOUCH_MINIOS_MFTS_FLAT;
			break;
		case MFTS_CURVED:
			ret = TOUCH_MINIOS_MFTS_CURVED;
			break;
		case MFTS_DS_FLAT:
			ret = TOUCH_MINIOS_MFTS_DS_FLAT;
			break;
		default:
			ret = TOUCH_MINIOS_AAT;
			break;
		}
	}
#endif

	return ret;
}
EXPORT_SYMBOL(touch_check_boot_mode);

int touch_bus_device_init(struct touch_hwif *hwif, void *driver)
{
	TOUCH_TRACE();

	if (hwif->bus_type == HWIF_I2C)
		return touch_i2c_device_init(hwif, driver);
	else if (hwif->bus_type == HWIF_SPI)
		return touch_spi_device_init(hwif, driver);

	TOUCH_E("Unknown touch interface : %d\n", hwif->bus_type);

	return -ENODEV;
}
EXPORT_SYMBOL(touch_bus_device_init);

void touch_bus_device_exit(struct touch_hwif *hwif)
{
	TOUCH_TRACE();

	if (hwif->bus_type == HWIF_I2C)
		touch_i2c_device_exit(hwif);
	else if (hwif->bus_type == HWIF_SPI)
		touch_spi_device_exit(hwif);
}
EXPORT_SYMBOL(touch_bus_device_exit);
