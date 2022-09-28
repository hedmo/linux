// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Maxim 8971 USB/Adapter Charger Driver
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/power_supply.h>

#include "max8971_charger.h"

static const char *max8971_manufacturer	= "Maxim Integrated";
static const char *max8971_model		= "MAX8971";

struct max8971_data {
	struct i2c_client *client;
	struct device *dev;
	struct power_supply *psy_mains;

	bool present;
};

enum max8971_charging_state {
	MAX8971_CHARGING_DEAD_BATTERY = 0x0,
	MAX8971_CHARGING_PREQUALIFICATION,
	MAX8971_CHARGING_FAST_CONST_CURRENT,
	MAX8971_CHARGING_FAST_CONST_VOLTAGE,
	MAX8971_CHARGING_TOP_OFF,
	MAX8971_CHARGING_DONE,
	MAX8971_CHARGING_TIMER_FAULT,
	MAX8971_CHARGING_SUSPENDED_THERMAL,
	MAX8971_CHARGING_OFF,
	MAX8971_CHARGING_THERMAL_LOOP,
};

enum max8971_health_state {
	MAX8971_HEALTH_UNKNOWN = 0x0,
	MAX8971_HEALTH_COLD,
	MAX8971_HEALTH_COOL,
	MAX8971_HEALTH_WARM,
	MAX8971_HEALTH_HOT,
	MAX8971_HEALTH_OVERHEAT,
};

/* Fast-Charge current limit, 250..1550 mA, 50 mA steps */
#define MAX8971_CHG_CC_STEP			  50000U
#define MAX8971_CHG_CC_MIN			 250000U
#define MAX8971_CHG_CC_MAX			1550000U

/* Input current limit, 250..1500 mA, 25 mA steps */
#define MAX8971_DCILMT_STEP			  25000U
#define MAX8971_DCILMT_MIN			 250000U
#define MAX8971_DCILMT_MAX			1500000U

static int max8971_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	dev_dbg(&client->dev, "writing value 0x%x to reg 0x%x\n", value, reg);
	return i2c_smbus_write_byte_data(client, reg, value);
}

static int max8971_read_reg(struct i2c_client *client, u8 reg)
{
	dev_dbg(&client->dev, "reading from reg 0x%x\n", reg);
	return i2c_smbus_read_byte_data(client, reg);
}

static int max8971_set_bits(struct i2c_client *client, u8 reg, u8 mask, u8 data)
{
	u8 value = 0;
	int ret;

	value = max8971_read_reg(client, reg);
	if (value < 0) {
		dev_err(&client->dev, "read of 0x%x failed with %d\n", reg, value);
		return ret;
	}

	value &= ~mask;
	value |= data;

	max8971_write_reg(client, MAX8971_REG_PROTCMD,
			MAX8971_CHGPROT_UNLOCKED);

	ret = max8971_write_reg(client, reg, value);

	max8971_write_reg(client, MAX8971_REG_PROTCMD,
			MAX8971_CHGPROT_LOCKED);

	return ret;
}

static int max8971_get_status(struct max8971_data *priv, int *val)
{
	struct i2c_client *client = priv->client;
	u8 regval;

	regval = max8971_read_reg(client, MAX8971_REG_DETAILS2);
	if (regval < 0)
		return regval;

	regval &= MAX8971_CHG_DTLS_MASK;

	switch (regval) {
	case MAX8971_CHARGING_DEAD_BATTERY:
	case MAX8971_CHARGING_PREQUALIFICATION:
	case MAX8971_CHARGING_FAST_CONST_CURRENT:
	case MAX8971_CHARGING_FAST_CONST_VOLTAGE:
	case MAX8971_CHARGING_TOP_OFF:
	case MAX8971_CHARGING_THERMAL_LOOP:
		*val = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case MAX8971_CHARGING_DONE:
		*val = POWER_SUPPLY_STATUS_FULL;
		break;
	case MAX8971_CHARGING_TIMER_FAULT:
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case MAX8971_CHARGING_OFF:
	case MAX8971_CHARGING_SUSPENDED_THERMAL:
		*val = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		*val = POWER_SUPPLY_STATUS_UNKNOWN;
	}

	return 0;
}

static int max8971_get_charge_type(struct max8971_data *priv, int *val)
{
	struct i2c_client *client = priv->client;
	u8 regval;

	regval = max8971_read_reg(client, MAX8971_REG_DETAILS2);
	if (regval < 0)
		return regval;

	regval &= MAX8971_CHG_DTLS_MASK;

	switch (regval) {
	case MAX8971_CHARGING_DEAD_BATTERY:
	case MAX8971_CHARGING_PREQUALIFICATION:
		*val = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case MAX8971_CHARGING_FAST_CONST_CURRENT:
	case MAX8971_CHARGING_FAST_CONST_VOLTAGE:
		*val = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case MAX8971_CHARGING_TOP_OFF:
	case MAX8971_CHARGING_THERMAL_LOOP:
		*val = POWER_SUPPLY_CHARGE_TYPE_STANDARD;
		break;
	case MAX8971_CHARGING_DONE:
	case MAX8971_CHARGING_TIMER_FAULT:
	case MAX8971_CHARGING_SUSPENDED_THERMAL:
	case MAX8971_CHARGING_OFF:
		*val = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	default:
		*val = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	return 0;
}

static int max8971_get_health(struct max8971_data *priv, int *val)
{
	struct i2c_client *client = priv->client;
	u8 regval;

	regval = max8971_read_reg(client, MAX8971_REG_DETAILS1);
	if (regval < 0)
		return regval;

	regval &= MAX8971_THM_DTLS_MASK;

	switch (regval) {
	case MAX8971_HEALTH_COLD:
		*val = POWER_SUPPLY_HEALTH_COLD;
		break;
	case MAX8971_HEALTH_COOL:
		*val = POWER_SUPPLY_HEALTH_COOL;
		break;
	case MAX8971_HEALTH_WARM:
		*val = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case MAX8971_HEALTH_HOT:
		*val = POWER_SUPPLY_HEALTH_HOT;
		break;
	case MAX8971_HEALTH_OVERHEAT:
		*val = POWER_SUPPLY_HEALTH_OVERHEAT;
		break;
	case MAX8971_HEALTH_UNKNOWN:
	default:
		*val = POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	return 0;
}

static int max8971_get_online(struct max8971_data *priv, int *val)
{
	struct i2c_client *client = priv->client;
	u8 regval;

	regval = max8971_read_reg(client, MAX8971_REG_CHG_STAT);
	if (regval < 0)
		return regval;

	regval = (regval & MAX8971_CHG_MASK) >> MAX8971_CHG_SHIFT;

	if (priv->present)
		*val = !regval;
	else
		*val = priv->present;

	return 0;
}

static int max8971_get_integer(struct max8971_data *priv, u8 reg, u8 mask,
				unsigned int clamp_min, unsigned int clamp_max,
				unsigned int mult, int *val)
{
	struct i2c_client *client = priv->client;
	unsigned int regval;
	
	regval = max8971_read_reg(client, reg);
	if (regval < 0)
		return regval;

	regval &= mask;

	*val = clamp_val(regval * mult, clamp_min, clamp_max);

	return 0;
}

static int max8971_set_integer(struct max8971_data *priv, u8 reg, u8 mask,
				unsigned int clamp_min, unsigned int clamp_max,
				unsigned int div, int val)
{
	struct i2c_client *client = priv->client;
	unsigned int regval;

	regval = clamp_val(val, clamp_min, clamp_max) / div;

	return max8971_set_bits(client, reg, mask, regval);
}

static int max8971_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct max8971_data *priv = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = max8971_get_status(priv, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = max8971_get_charge_type(priv, &val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = max8971_get_health(priv, &val->intval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = max8971_get_online(priv, &val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = priv->present;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = MAX8971_CHG_CC_MAX;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		ret = max8971_get_integer(priv, MAX8971_REG_FCHGCRNT,
					   MAX8971_CHG_CC_MASK,
					   MAX8971_CHG_CC_MIN,
					   MAX8971_CHG_CC_MAX,
					   MAX8971_CHG_CC_STEP,
					   &val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = max8971_get_integer(priv, MAX8971_REG_DCCRNT,
					   MAX8971_DCILMT_MASK,
					   MAX8971_DCILMT_MIN,
					   MAX8971_DCILMT_MAX,
					   MAX8971_DCILMT_STEP,
					   &val->intval);
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = max8971_model;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = max8971_manufacturer;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int max8971_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *val)
{
	struct max8971_data *priv = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		ret = max8971_set_integer(priv, MAX8971_REG_FCHGCRNT,
					   MAX8971_CHG_CC_MASK,
					   MAX8971_CHG_CC_MIN,
					   MAX8971_CHG_CC_MAX,
					   MAX8971_CHG_CC_STEP,
					   val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = max8971_set_integer(priv, MAX8971_REG_DCCRNT,
					   MAX8971_DCILMT_MASK,
					   MAX8971_DCILMT_MIN,
					   MAX8971_DCILMT_MAX,
					   MAX8971_DCILMT_STEP,
					   val->intval);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
};

static int max8971_property_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return true;
	default:
		return false;
	}
}

static enum power_supply_property max8971_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static const struct power_supply_desc max8971_mains_desc = {
	.name = "charger",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = max8971_properties,
	.num_properties = ARRAY_SIZE(max8971_properties),
	.get_property = max8971_get_property,
	.set_property = max8971_set_property,
	.property_is_writeable	= max8971_property_is_writeable,
};

static irqreturn_t max8971_interrupt(int irq, void *dev_id)
{
	struct max8971_data *priv = dev_id;
	struct i2c_client *client = priv->client;
	int ret, state;

	state = max8971_read_reg(client, MAX8971_REG_CHGINT);
	if (state < 0)
		dev_err(priv->dev, "interrupt reg read failed %d\n", state);

	ret = max8971_write_reg(client, MAX8971_REG_CHGINT_MASK,
				MAX8971_AICL_MASK);
	if (ret)
		dev_err(priv->dev, "failed to mask IRQ\n");

	/* set presence prop */
	priv->present = state & MAX8971_REG_CHG_MASK;

	/* update supply status */
	power_supply_changed(priv->psy_mains);

	return IRQ_HANDLED;
}

static char *max8971_supplied_to[] = {
	"battery",
};

static int max8971_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct max8971_data *priv;
	struct power_supply_config cfg = {};
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;
	priv->dev = dev;

	i2c_set_clientdata(client, priv);

	cfg.of_node = dev->parent->of_node;
	cfg.drv_data = priv;
	cfg.supplied_to = max8971_supplied_to;
	cfg.num_supplicants = ARRAY_SIZE(max8971_supplied_to);

	priv->psy_mains = devm_power_supply_register(dev, &max8971_mains_desc, &cfg);
	if (IS_ERR(priv->psy_mains))
		return dev_err_probe(dev, PTR_ERR(priv->psy_mains),
				     "Failed to register mains supply\n");

	ret = max8971_write_reg(client, MAX8971_REG_CHGINT_MASK,
				MAX8971_AICL_MASK);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to mask IRQ\n");

	ret = devm_request_threaded_irq(dev, client->irq,
					NULL, &max8971_interrupt,
					IRQF_ONESHOT | IRQF_SHARED,
					client->name, priv);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register IRQ %d\n", client->irq);

	return 0;
}

static const struct of_device_id max8971_match_ids[] = {
	{ .compatible = "maxim,max8971" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, max8971_match_ids);

static const struct i2c_device_id max8971_i2c_id[] = {
	{ "max8971", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max8971_i2c_id);

static struct i2c_driver max8971_driver = {
	.driver = {
		.name = "max8971-charger",
		.of_match_table = max8971_match_ids,
	},
	.probe = max8971_probe,
	.id_table = max8971_i2c_id,
};
module_i2c_driver(max8971_driver);

MODULE_AUTHOR("Svyatoslav Ryhel <clamor95@gmail.com>");
MODULE_DESCRIPTION("MAX8971 Charger Driver");
MODULE_LICENSE("GPL v2");
