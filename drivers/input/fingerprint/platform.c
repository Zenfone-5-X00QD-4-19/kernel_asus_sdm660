/*
 * platform indepent driver interface
 *
 * Coypritht (c) 2017 Goodix
 */
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

int gf_parse_dts(struct gf_dev *gf_dev)
{
	int rc = 0;

	/*get reset resource*/
	gf_dev->reset_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node,"fp-reset-gpio",0);
	if(!gpio_is_valid(gf_dev->reset_gpio)) {
		pr_info("RESET GPIO is invalid.\n");
		return -1;
	}

	rc = gpio_request(gf_dev->reset_gpio, "goodix_reset");
	if(rc) {
		dev_err(&gf_dev->spi->dev, "Failed to request RESET GPIO. rc = %d\n", rc);
		return -1;
	}
	gpio_direction_output(gf_dev->reset_gpio, 0);

	/*get irq resourece*/
	gf_dev->irq_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node,"fp-irq-gpio",0);
	printk("[GDX_FP] gf::irq_gpio:%d\n", gf_dev->irq_gpio);
	if(!gpio_is_valid(gf_dev->irq_gpio)) {
		pr_info("IRQ GPIO is invalid.\n");
		return -1;
	}

	rc = gpio_request(gf_dev->irq_gpio, "goodix_irq");
	if(rc) {
		dev_err(&gf_dev->spi->dev, "Failed to request IRQ GPIO. rc = %d\n", rc);
		return -1;
	}
	gpio_direction_input(gf_dev->irq_gpio);

	/*get vcc resourece*/
	gf_dev->osvcc_ze602kl_enable_pin = of_get_named_gpio(gf_dev->spi->dev.of_node,"fp-vcc-enable-gpio", 0);
	printk("[GDX_FP] gf::osvcc_ze602kl_enable_pin:%d\n", gf_dev->osvcc_ze602kl_enable_pin);
	if(!gpio_is_valid(gf_dev->osvcc_ze602kl_enable_pin)) {
		pr_info("osvcc_ze602kl_enable_pin GPIO is invalid.\n");
		return -1;
	}
	//gpio_direction_output(gf_dev->osvcc_ze602kl_enable_pin, 1);

	rc = gpio_request(gf_dev->osvcc_ze602kl_enable_pin, "vcc_ze602kl_enable_pin");
	if(rc) {
		dev_err(&gf_dev->spi->dev, "Failed to request osvcc_ze602kl_enable_pin GPIO. rc = %d\n", rc);
		return -1;
	}

	return 0;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
	pr_info("[info] %s\n", __func__);

	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
}

int gf_power_on(struct gf_dev *gf_dev)
{
	int rc = 0;

	if (gpio_is_valid(gf_dev->osvcc_ze602kl_enable_pin)) {
        gpio_direction_output(gf_dev->osvcc_ze602kl_enable_pin, 1);
        pr_info("set ze602kl_enable_pin 1\n");
    }

	msleep(15);

	if (gpio_is_valid(gf_dev->reset_gpio)) {
        gpio_direction_output(gf_dev->reset_gpio, 1);
        pr_info("set reset_gpio 1\n");
    }

	msleep(1);

	pr_info("---- power on ok ----\n");

	return rc;
}

int gf_power_off(struct gf_dev *gf_dev)
{
	int rc = 0;

	if (gpio_is_valid(gf_dev->osvcc_ze602kl_enable_pin)) {
        gpio_direction_output(gf_dev->osvcc_ze602kl_enable_pin, 0);
        pr_info("set ze602kl_enable_pin 0\n");
    }
    
	msleep(10);
	pr_info("---- power off ----\n");
	return rc;
}

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -1;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -1;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}
