// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAX14526 extcon driver to support MUIC
 */

#include <linux/devm-helpers.h>
#include <linux/delay.h>
#include <linux/extcon-provider.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

/* I2C addresses of MUIC internal registers */
#define	DEVICE_ID		0x00
#define MAX14526 		0x20

/* CONTROL_1 register masks */
#define	CONTROL_1		0x01
#define	ID_2P2			BIT(6)
#define	ID_620			BIT(5)
#define	ID_200			BIT(4)
#define	VLDO			BIT(3)
#define	SEMREN			BIT(2)
#define	ADC_EN			BIT(1)
#define	CP_EN			BIT(0)

/* CONTROL_2 register masks */
#define	CONTROL_2		0x02
#define	INTPOL			BIT(7)
#define	INT_EN			BIT(6)
#define	MIC_LP			BIT(5)
#define	CP_AUD			BIT(4)
#define	CHG_TYPE		BIT(1)
#define	USB_DET_DIS		BIT(0)

/* SW_CONTROL register masks */
#define	SW_CONTROL		0x03
#define	MIC_ON			0x40
#define DP			0x38
#define DM			0x07

/* DP and DM settings of SW_CONTROL */
#define DP_USB			0x00
#define	DP_UART			0x08
#define	DP_AUDIO		0x10
#define	DP_OPEN			0x38

#define DM_USB			0x00
#define	DM_UART			0x01
#define	DM_AUDIO		0x02
#define	DM_OPEN			0x07

/* Combined masks of SW_CONTROL register */
#define USB		DP_USB   | DM_USB 	/* 0x00 */
#define UART 		DP_UART  | DM_UART 	/* 0x09 */
#define AUDIO		DP_AUDIO | DM_AUDIO	/* 0x12 */
#define OPEN		DP_OPEN  | DM_OPEN 	/* 0x3f */

/* INT_STATUS register masks */
#define	INT_STAT		0x04
#define	CHGDET			BIT(7)
#define	MR_COMP			BIT(6)
#define	SENDEND			BIT(5)
#define	V_VBUS			BIT(4)
#define	IDNO			0x0f

/* STATUS register masks */
#define	STATUS			0x05
#define	DCPORT			BIT(7)
#define	CHPORT			BIT(6)
#define C1COMP      		BIT(0)

#define MAX14526_DATA_POLL_MSEC 1000

struct max14526_data {
	struct i2c_client *client;
	struct extcon_dev *edev;

	struct gpio_desc *usif_gpio;
	struct gpio_desc *dp2t_gpio;

	struct regulator *periph_supply;

	u8 last_state;
};

enum max14526_muic_modes {
	MAX14526_NONE = 0x0b,
	MAX14526_USB = 0x1b,
	MAX14526_CHG = 0x9b,
	MAX14526_OTG = 0x00, /* no power */
	MAX14526_OTG_Y = 0x10,
	MAX14526_MHL = 0x02, /* no power */
	MAX14526_MHL_CHG = 0x90,
};

static const unsigned int max14526_extcon_cable[] = {
	EXTCON_NONE,
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_CHG_USB_DCP,
	EXTCON_DISP_MHL,
};

static int max14526_i2c_read_byte(struct max14526_data *priv, u8 addr, u8 *value)
{
	u32 ret = i2c_smbus_read_byte_data(priv->client, addr);
	if (ret < 0) {
		dev_info(&priv->client->dev,
				  "i2c read failed from addr %04x, ret = %d\n", addr, ret);
		return ret;
	} else {
		*value = (u8)ret;
		return 0;
	}
}

static int max14526_i2c_write_byte(struct max14526_data *priv, u8 addr, u8 value)
{
	int ret = i2c_smbus_write_byte_data(priv->client, addr, value);
	if (ret)
		dev_info(&priv->client->dev, "i2c write failed to addr %04x, ret = %d\n", addr, ret);
	return ret;
}

static int max14526_ap_usb_mode(struct max14526_data *priv)
{
	struct device *dev = &priv->client->dev;
	int ret;

	/* Connect CP UART to AP UART */
	gpiod_set_value_cansleep(priv->usif_gpio, 0);
	dev_dbg(dev, "USIF SWITCH: CP UART is connected to AP\n");

	/* Connect CP UART to MUIC UART */
	gpiod_set_value_cansleep(priv->dp2t_gpio, 0);
	dev_dbg(dev, "DP2T SWITCH: CP UART is connected to MUIC UART\n");

	/* Enable USB Path */
	ret = max14526_i2c_write_byte(priv, SW_CONTROL, USB);
	if (ret)
		return ret;

	/* Enable 200K, Charger Pump and ADC */
	ret = max14526_i2c_write_byte(priv, CONTROL_1, ID_200 | ADC_EN | CP_EN);
	if (ret)
		return ret;

	dev_info(dev, "AP USB mode set\n");

	return 0;
}

static irqreturn_t max14526_interrupt(int irq, void *dev_id)
{
	struct max14526_data *priv = dev_id;
	struct device *dev = &priv->client->dev;
	u8 state;
	int ret, i;

	/* 
	 * Upon an MUIC IRQ (MUIC_INT_N falls),
	 * wait 70ms before reading INT_STAT and STATUS.
	 * After the reads, MUIC_INT_N returns to high
	 * (but the INT_STAT and STATUS contents will be held).
	 */
	mdelay(250);

	ret = max14526_i2c_read_byte(priv, INT_STAT, &state);
	if (ret)
		dev_err(dev, "failed to read MUIC state %d\n", ret);
	else
		dev_info(dev, "IRQ detected mode 0x%X\n", state);

	if (state == priv->last_state)
		return IRQ_HANDLED;

	/* Detach all devices before setting active */
	for (i = 0; i < ARRAY_SIZE(max14526_extcon_cable); i++)
		extcon_set_state_sync(priv->edev, max14526_extcon_cable[i], false);

	ret = regulator_is_enabled(priv->periph_supply);
	if (ret)
		regulator_disable(priv->periph_supply);

	switch (state) {
	case MAX14526_USB:
		ret = regulator_enable(priv->periph_supply);
		if (ret)
			dev_err(dev, "failed to enable peripheral regulator\n");
		extcon_set_state_sync(priv->edev, EXTCON_USB, true);
		break;

	case MAX14526_CHG:
		extcon_set_state_sync(priv->edev, EXTCON_CHG_USB_DCP, true);
		break;

	case MAX14526_OTG:
	case MAX14526_OTG_Y:
		extcon_set_state_sync(priv->edev, EXTCON_USB_HOST, true);
		break;

	case MAX14526_MHL:
	case MAX14526_MHL_CHG:
		extcon_set_state_sync(priv->edev, EXTCON_DISP_MHL, true);
		break;

	case MAX14526_NONE:
	default:
		extcon_set_state_sync(priv->edev, EXTCON_NONE, true);
		break;
	}

	return IRQ_HANDLED;
}

static int max14526_regulator_get(struct max14526_data *priv)
{
	struct device *dev = &priv->client->dev;
	int ret;

	priv->periph_supply = devm_regulator_get_optional(dev, "periph");
	if (IS_ERR(priv->periph_supply))
		return PTR_ERR(priv->periph_supply);

	/* Powercycle to avoid interrupt errors */
	ret = regulator_enable(priv->periph_supply);
	if (ret)
		return ret;

	mdelay(50);

	regulator_disable(priv->periph_supply);

	return 0;
}

static int max14526_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct max14526_data *priv;
	u8 dev_id;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;
	i2c_set_clientdata(client, priv);

	priv->usif_gpio = devm_gpiod_get_optional(dev, "usif",
						  GPIOD_OUT_LOW);
	if (IS_ERR(priv->usif_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->usif_gpio),
				     "failed to get usif GPIO\n");

	priv->dp2t_gpio = devm_gpiod_get_optional(dev, "dp2t",
						  GPIOD_OUT_LOW);
	if (IS_ERR(priv->dp2t_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->dp2t_gpio),
				     "failed to get DP2T GPIO\n");

	ret = max14526_regulator_get(priv);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to get peripheral regulator\n");

	/* Detect if MUIC version is supported */
	ret = max14526_i2c_read_byte(priv, DEVICE_ID, &dev_id);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read MUIC ID\n");

	if ((dev_id & 0xf0) == MAX14526) {
		dev_info(dev, "detected MAX14526 MUIC with id 0x%X\n", dev_id);
	} else {
		dev_err(dev, "MUIC vendor id 0x%X is not recognized\n", dev_id);
		return -EINVAL;
	}

	priv->edev = devm_extcon_dev_allocate(dev, max14526_extcon_cable);
	if (IS_ERR(priv->edev))
		return dev_err_probe(dev, (IS_ERR(priv->edev)),
				     "failed to allocate extcon device\n");

	ret = devm_extcon_dev_register(dev, priv->edev);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "failed to register extcon device\n");

	ret = max14526_ap_usb_mode(priv);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "failed to set AP USB mode\n");

	max14526_i2c_write_byte(priv, CONTROL_2, INT_EN);

	mdelay(250);

	ret = devm_request_threaded_irq(dev, client->irq,
					NULL, &max14526_interrupt,
					IRQF_ONESHOT | IRQF_SHARED,
					client->name, priv);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register IRQ\n");

	ret = max14526_i2c_read_byte(priv, INT_STAT, &priv->last_state);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read internal state\n");

	return 0;
}

static const struct of_device_id max14526_match[] = {
	{ .compatible = "maxim,max14526-muic" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, max14526_match);

static const struct i2c_device_id max14526_id[] = {
	{ "max14526_muic", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max14526_id);

static struct i2c_driver max14526_driver = {
	.driver = {
		.name = "max14526-muic",
		.of_match_table = max14526_match,
	},
	.probe = max14526_probe,
	.id_table = max14526_id,
};
module_i2c_driver(max14526_driver);

MODULE_AUTHOR("Svyatoslav Ryhel <clamor95@gmail.com>");
MODULE_DESCRIPTION("MAX14526 extcon driver to support MUIC");
MODULE_LICENSE("GPL");
