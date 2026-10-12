// SPDX-License-Identifier: GPL-2.0-only
/*
 * NSING N32H7xx pin controller and GPIO driver.
 *
 * The N32H7xx has eleven GPIO ports (A..K) of sixteen pins each, and one
 * register block per port holds everything that is a property of a pin: the
 * mode (GPIO input, GPIO output, alternate function or analog), the
 * alternate-function number, the pull resistor, the slew rate and the drive
 * strength, plus the GPIO data register.  The pin controller and the GPIO
 * chips therefore describe the same hardware and live in one driver.
 *
 * The register layout and the set of peripheral signals a pin can carry are
 * properties of the series, so they live here.  The pin/signal table is
 * generated from the vendor function-remap database by
 * scripts/nsing/pinctrl/gen-pinfunc.py.  Which pin a board wires to which
 * signal is not a series property, so the board device tree names the pin and
 * the signal, and this driver turns that into an alternate-function number.
 *
 * Bindings
 * --------
 *	pinctrl: pinctrl@58032800 {
 *		compatible = "nsing,n32h7xx-pinctrl";
 *		reg = <0x58032800 0x2c00>;
 *		#address-cells = <1>;
 *		#size-cells = <1>;
 *
 *		gpioa: gpio@58032800 {
 *			reg = <0x58032800 0x400>;
 *			gpio-controller;
 *			#gpio-cells = <2>;
 *			gpio-ranges = <&pinctrl 0 0 16>;
 *		};
 *		... one child per port ...
 *	};
 *
 * A consumer names the pins it uses and the signal it wants on them:
 *
 *	&pinctrl {
 *		usart1_pins: usart1-pins {
 *			pins = "PA9", "PA10";
 *			function = "usart1";
 *			bias-pull-up;
 *		};
 *	};
 *
 *	&usart1 {
 *		pinctrl-names = "default";
 *		pinctrl-0 = <&usart1_pins>;
 *	};
 *
 * "pins" holds package pin names (PA0..PK15).  "function" is a signal name
 * ("usart1_tx"), a peripheral name ("usart1", which selects the signal that
 * peripheral has on each pin), the bare "gpio" or "analog", or an explicit
 * "af0".."af15".
 *
 * A group whose pins need different configuration nests one subnode per
 * subgroup instead of listing "pins" itself.  A peripheral name is rejected
 * as ambiguous where it names two different signals on one pin; the board
 * then has to spell the signal out.
 *
 * Pin configuration properties:
 *	bias-disable / bias-pull-up / bias-pull-down
 *	drive-strength = <0..3>	raw GPIO_DS encoding.  On a 3.3 V IO pad
 *				0 = 2 mA, 1 = 8 mA, 2 = 4 mA, 3 = 12 mA.
 *	slew-rate = <0|1>	0 = fast, 1 = slow
 *	nsing,hs-drive-strength-high
 *				on a high-speed pad, select the high-current
 *				driver bank instead of the low one.
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>

#include "../core.h"
#include "../pinconf.h"
#include "../pinmux.h"
#include "../pinctrl-utils.h"

#include "pinctrl-n32h7xx-pinfunc.h"

/* AHB5 GPIO block: eleven ports spaced 0x400 apart. */
#define N32H7XX_GPIO_BASE		0x58032800
#define N32H7XX_GPIO_PORT_STRIDE	0x400
#define N32H7XX_NPORTS			11
#define N32H7XX_PINS_PER_PORT		16

/* Per-port register offsets. */
#define N32H7XX_GPIO_PMODE		0x00
#define N32H7XX_GPIO_POTYPE		0x04
#define N32H7XX_GPIO_SR			0x08
#define N32H7XX_GPIO_PUPD		0x0c
#define N32H7XX_GPIO_PID		0x10
#define N32H7XX_GPIO_POD		0x14
#define N32H7XX_GPIO_AFL		0x20
#define N32H7XX_GPIO_AFH		0x24
#define N32H7XX_GPIO_DS			0x2c

/* AFIO block: the high-speed pad driver-current bank select bits. */
#define N32H7XX_AFIO_BASE		0x58032400
#define N32H7XX_AFIO_HS_DSN		0xe4
#define N32H7XX_AFIO_HS_DSP		0xf0

/* RCC AHB5 gates: the GPIO ports and the AFIO block must be clocked. */
#define N32H7XX_RCC_BASE		0x58030000
#define N32H7XX_RCC_AHB5EN1		0x0e4
#define N32H7XX_RCC_AHB5EN2		0x0e8
#define N32H7XX_AHB5EN1_GPIO		(BIT(31) | BIT(27) | BIT(23) | \
					 BIT(19) | BIT(15) | BIT(11) | \
					 BIT(7) | BIT(3))
#define N32H7XX_AHB5EN2_GPIO		(BIT(31) | BIT(27) | BIT(23))
#define N32H7XX_AHB5EN2_AFIO		BIT(3)

enum n32h7xx_pin_mode {
	N32H7XX_MODE_INPUT	= 0,
	N32H7XX_MODE_OUTPUT	= 1,
	N32H7XX_MODE_AF		= 2,
	N32H7XX_MODE_ANALOG	= 3,
};

enum n32h7xx_pin_pull {
	N32H7XX_PULL_NONE	= 0,
	N32H7XX_PULL_UP		= 1,
	N32H7XX_PULL_DOWN	= 2,
};

/* Driver-private pinconf parameter; custom parameters start after the enum. */
#define N32H7XX_PIN_CONFIG_HS_HIGH	(PIN_CONFIG_END + 1)

static const struct pinconf_generic_params n32h7xx_custom_params[] = {
	{ "nsing,hs-drive-strength-high", N32H7XX_PIN_CONFIG_HS_HIGH, 1 },
};

static const struct pin_config_item n32h7xx_custom_conf_items[] = {
	PCONFDUMP(N32H7XX_PIN_CONFIG_HS_HIGH, "high-current driver bank", NULL,
		  false),
};

/*
 * Pads that have a high-speed driver and can therefore be switched between
 * the low- and high-current banks.  The bit position in the AFIO registers is
 * the count of high-speed pads before this one, with PI0 sharing the word
 * boundary at bit 63.
 */
static const u16 n32h7xx_hs_pads[] = {
	0x00b0, 0xc060, 0x103d, 0xc707, 0xff83, 0xf83f, 0x8137, 0xffec, 0x06ff,
};
static const u8 n32h7xx_hs_base[] = {
	0, 3, 7, 13, 21, 32, 43, 50, 64,
};

struct n32h7xx_pinctrl;

struct n32h7xx_gpio_bank {
	struct gpio_chip chip;
	void __iomem *base;
	spinlock_t lock;
};

struct n32h7xx_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctl;
	struct pinctrl_desc desc;
	void __iomem *base;
	void __iomem *afio;
	unsigned int *group_pins;
	struct n32h7xx_gpio_bank *banks;
	unsigned int nbanks;
};

static int n32h7xx_pin_from_name(const char *name, unsigned int *pin)
{
	unsigned int i;

	for (i = 0; i < N32H7XX_NPINS; i++) {
		if (!strcmp(name, n32h7xx_pins[i].name)) {
			*pin = n32h7xx_pins[i].number;
			return 0;
		}
	}

	return -EINVAL;
}

/*
 * Resolve the alternate-function number that routes "signal" onto "pin".  An
 * exact signal name ("usart1_tx") wins over a peripheral name ("usart1"),
 * which matches the signal that peripheral has on the pin.
 *
 * A peripheral name is only accepted when every signal it matches on this pin
 * uses the same alternate function.  Several peripherals have pins that can
 * carry one of two different signals -- xspi1_dqs and xspi1_io3 both sit on
 * PA1, for example -- and quietly picking whichever comes first in the table
 * would mux the wrong function.  Such a pin has to name the signal exactly.
 *
 * Returns 0 on success, -ENOTUNIQ if the name matches several signals with
 * different alternate functions, and -EINVAL if the pin cannot carry it.
 */
static int n32h7xx_resolve_af(unsigned int pin, const char *signal, u8 *af)
{
	const struct n32h7xx_desc_pin *desc = &n32h7xx_pins[pin];
	size_t len = strlen(signal);
	bool found = false;
	unsigned int i;
	u8 val = 0;

	for (i = 0; i < desc->nfunctions; i++) {
		if (!strcmp(desc->functions[i].name, signal)) {
			*af = desc->functions[i].af;
			return 0;
		}
	}

	if (!len)
		return -EINVAL;

	for (i = 0; i < desc->nfunctions; i++) {
		const char *name = desc->functions[i].name;

		if (strncmp(name, signal, len) || name[len] != '_')
			continue;
		if (found && desc->functions[i].af != val)
			return -ENOTUNIQ;
		val = desc->functions[i].af;
		found = true;
	}

	if (!found)
		return -EINVAL;

	*af = val;
	return 0;
}

static void n32h7xx_pin_set_mux(void __iomem *base, unsigned int offset,
				unsigned int mode, unsigned int af)
{
	unsigned int shift = (offset % 8) * 4;
	unsigned int af_reg;
	u32 reg;

	/* The alternate-function number is used by AF output and by analog. */
	if (mode != N32H7XX_MODE_OUTPUT) {
		af_reg = offset < 8 ? N32H7XX_GPIO_AFL : N32H7XX_GPIO_AFH;
		reg = readl(base + af_reg);
		reg &= ~(0xfU << shift);
		reg |= (af & 0xfU) << shift;
		writel(reg, base + af_reg);
	}

	reg = readl(base + N32H7XX_GPIO_PMODE);
	reg &= ~(0x3U << (offset * 2));
	reg |= (mode & 0x3U) << (offset * 2);
	writel(reg, base + N32H7XX_GPIO_PMODE);
}

static void n32h7xx_pin_set_pull(void __iomem *base, unsigned int offset,
				 unsigned int pull)
{
	u32 reg = readl(base + N32H7XX_GPIO_PUPD);

	reg &= ~(0x3U << (offset * 2));
	reg |= (pull & 0x3U) << (offset * 2);
	writel(reg, base + N32H7XX_GPIO_PUPD);
}

/*
 * A high-speed pad is driven from one of two current banks selected by the
 * AFIO HS_DSN/HS_DSP bits.  The vendor GPIO_HS_IO_DC_* encodings that name a
 * low-bank current clear both bits; the two remaining encodings set them.
 * Only the low bank is selected unless the device tree asks otherwise.
 */
static void n32h7xx_hs_driver_bank(struct n32h7xx_pinctrl *pctl,
				   unsigned int pin, bool high)
{
	unsigned int port = pin / N32H7XX_PINS_PER_PORT;
	unsigned int offset = pin % N32H7XX_PINS_PER_PORT;
	unsigned int bit, reg, shift;
	u32 mask, val;

	if (port >= ARRAY_SIZE(n32h7xx_hs_pads))
		return;
	if (!(n32h7xx_hs_pads[port] & BIT(offset)))
		return;

	bit = n32h7xx_hs_base[port] +
	      hweight16(n32h7xx_hs_pads[port] & (BIT(offset) - 1));
	/* PI0 sits in the previous register word, at bit 63. */
	if (port == 8)
		bit--;

	reg = (bit / 32) * sizeof(u32);
	shift = bit % 32;
	mask = BIT(shift);

	val = readl(pctl->afio + N32H7XX_AFIO_HS_DSN + reg);
	writel(high ? val | mask : val & ~mask,
	       pctl->afio + N32H7XX_AFIO_HS_DSN + reg);

	val = readl(pctl->afio + N32H7XX_AFIO_HS_DSP + reg);
	writel(high ? val | mask : val & ~mask,
	       pctl->afio + N32H7XX_AFIO_HS_DSP + reg);
}

/* --- pin controller ----------------------------------------------------- */

static int n32h7xx_dt_subnode_to_map(struct pinctrl_dev *pctldev,
				     struct device_node *np,
				     struct pinctrl_map **map,
				     unsigned int *reserved_maps,
				     unsigned int *num_maps)
{
	struct n32h7xx_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	unsigned long *configs = NULL;
	unsigned int reserve = 0, num_configs = 0;
	const char *function = NULL;
	int npins, ret, i;

	npins = of_property_count_strings(np, "pins");
	if (npins < 0)
		/* Not a group node; the caller walks the parent's children. */
		return npins == -EINVAL ? 0 : npins;
	if (!npins)
		return -EINVAL;

	ret = of_property_read_string(np, "function", &function);
	if (ret && ret != -EINVAL)
		return ret;

	ret = pinconf_generic_parse_dt_config(np, pctldev, &configs,
					      &num_configs);
	if (ret)
		return ret;

	if (function) {
		for (i = 0; i < npins; i++) {
			const char *name;
			unsigned int pin;
			u8 af;

			ret = of_property_read_string_index(np, "pins", i,
							    &name);
			if (ret)
				goto out;
			if (n32h7xx_pin_from_name(name, &pin)) {
				dev_err(pctl->dev, "%pOF: unknown pin %s\n",
					np, name);
				ret = -EINVAL;
				goto out;
			}
			/*
			 * Catch a signal that cannot be routed to the pin now
			 * rather than silently leaving the pin unconfigured.
			 */
			if (!strcmp(function, "gpio") ||
			    !strcmp(function, "analog"))
				continue;
			if (!strncmp(function, "af", 2))
				continue;
			ret = n32h7xx_resolve_af(pin, function, &af);
			if (ret == -ENOTUNIQ) {
				dev_err(pctl->dev,
					"%pOF: %s is ambiguous on %s, name the exact signal\n",
					np, function, name);
				goto out;
			}
			if (ret) {
				dev_err(pctl->dev,
					"%pOF: %s cannot be routed to %s\n",
					np, function, name);
				ret = -EINVAL;
				goto out;
			}
		}
		reserve += npins;
	}
	if (num_configs)
		reserve += npins;

	ret = pinctrl_utils_reserve_map(pctldev, map, reserved_maps, num_maps,
					reserve);
	if (ret)
		goto out;

	for (i = 0; i < npins; i++) {
		const char *name;

		of_property_read_string_index(np, "pins", i, &name);

		if (function) {
			ret = pinctrl_utils_add_map_mux(pctldev, map,
							reserved_maps, num_maps,
							name, function);
			if (ret)
				goto out;
		}
		if (num_configs) {
			ret = pinctrl_utils_add_map_configs(pctldev, map,
					reserved_maps, num_maps, name, configs,
					num_configs, PIN_MAP_TYPE_CONFIGS_PIN);
			if (ret)
				goto out;
		}
	}

	ret = 0;
out:
	kfree(configs);
	return ret;
}

static int n32h7xx_dt_node_to_map(struct pinctrl_dev *pctldev,
				  struct device_node *np_config,
				  struct pinctrl_map **map,
				  unsigned int *num_maps)
{
	unsigned int reserved_maps = 0;
	int ret;

	*map = NULL;
	*num_maps = 0;

	/*
	 * The device tree core hands us the node named by pinctrl-N.  A group
	 * either names its own pins or holds one subnode per subgroup, so the
	 * two shapes are both valid and are tried in that order.
	 */
	if (of_find_property(np_config, "pins", NULL)) {
		ret = n32h7xx_dt_subnode_to_map(pctldev, np_config, map,
						&reserved_maps, num_maps);
		if (ret)
			goto err;
		return 0;
	}

	for_each_available_child_of_node_scoped(np_config, np) {
		ret = n32h7xx_dt_subnode_to_map(pctldev, np, map,
						&reserved_maps, num_maps);
		if (ret)
			goto err;
	}

	return 0;

err:
	pinctrl_utils_free_map(pctldev, *map, *num_maps);
	*map = NULL;
	*num_maps = 0;
	return ret;
}

static const struct pinctrl_ops n32h7xx_pctl_ops = {
	.get_groups_count	= pinctrl_generic_get_group_count,
	.get_group_name		= pinctrl_generic_get_group_name,
	.get_group_pins		= pinctrl_generic_get_group_pins,
	.dt_node_to_map		= n32h7xx_dt_node_to_map,
	.dt_free_map		= pinctrl_utils_free_map,
};

/* --- pinmux ------------------------------------------------------------- */

static int n32h7xx_set_mux(struct pinctrl_dev *pctldev, unsigned int function,
			   unsigned int group)
{
	struct n32h7xx_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	const char *fname = pinmux_generic_get_function_name(pctldev, function);
	unsigned int pin = pctl->group_pins[group];
	unsigned int mode;
	unsigned int af = 0;
	u8 val;

	if (!strcmp(fname, "gpio")) {
		mode = N32H7XX_MODE_INPUT;
	} else if (!strcmp(fname, "analog")) {
		mode = N32H7XX_MODE_ANALOG;
	} else if (!strncmp(fname, "af", 2) && !kstrtou8(fname + 2, 10, &val) &&
		   val <= 0xf) {
		af = val;
		mode = N32H7XX_MODE_AF;
	} else {
		if (n32h7xx_resolve_af(pin, fname, &val))
			return -EINVAL;
		af = val;
		mode = N32H7XX_MODE_AF;
	}

	n32h7xx_pin_set_mux(pctl->base + (pin / N32H7XX_PINS_PER_PORT) *
			    N32H7XX_GPIO_PORT_STRIDE,
			    pin % N32H7XX_PINS_PER_PORT, mode, af);

	return 0;
}

static const struct pinmux_ops n32h7xx_pmx_ops = {
	.get_functions_count	= pinmux_generic_get_function_count,
	.get_function_name	= pinmux_generic_get_function_name,
	.get_function_groups	= pinmux_generic_get_function_groups,
	.set_mux		= n32h7xx_set_mux,
	.strict			= true,
};

/* --- pin configuration -------------------------------------------------- */

static int n32h7xx_pinconf_get(struct pinctrl_dev *pctldev, unsigned int pin,
			       unsigned long *config)
{
	struct n32h7xx_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	void __iomem *base = pctl->base + (pin / N32H7XX_PINS_PER_PORT) *
			     N32H7XX_GPIO_PORT_STRIDE;
	unsigned int offset = pin % N32H7XX_PINS_PER_PORT;
	unsigned int param = pinconf_to_config_param(*config);
	unsigned int arg;

	switch (param) {
	case PIN_CONFIG_BIAS_PULL_UP:
	case PIN_CONFIG_BIAS_PULL_DOWN:
	case PIN_CONFIG_BIAS_DISABLE:
		arg = (readl(base + N32H7XX_GPIO_PUPD) >> (offset * 2)) & 0x3;
		if ((param == PIN_CONFIG_BIAS_PULL_UP &&
		     arg != N32H7XX_PULL_UP) ||
		    (param == PIN_CONFIG_BIAS_PULL_DOWN &&
		     arg != N32H7XX_PULL_DOWN) ||
		    (param == PIN_CONFIG_BIAS_DISABLE &&
		     arg != N32H7XX_PULL_NONE))
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		arg = (readl(base + N32H7XX_GPIO_DS) >> (offset * 2)) & 0x3;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		arg = (readl(base + N32H7XX_GPIO_POTYPE) >> offset) & 0x1;
		arg = (param == PIN_CONFIG_DRIVE_OPEN_DRAIN) ? arg : !arg;
		if (!arg)
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_SLEW_RATE:
		arg = (readl(base + N32H7XX_GPIO_SR) >> offset) & 0x1;
		break;
	case PIN_CONFIG_OUTPUT_ENABLE:
		arg = (readl(base + N32H7XX_GPIO_PMODE) >> (offset * 2)) & 0x3;
		if (arg != N32H7XX_MODE_OUTPUT)
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_LEVEL:
		arg = (readl(base + N32H7XX_GPIO_POD) >> offset) & 0x1;
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int n32h7xx_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
			       unsigned long *configs, unsigned int num_configs)
{
	struct n32h7xx_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	void __iomem *base = pctl->base + (pin / N32H7XX_PINS_PER_PORT) *
			     N32H7XX_GPIO_PORT_STRIDE;
	unsigned int offset = pin % N32H7XX_PINS_PER_PORT;
	unsigned int i;
	u32 reg;

	for (i = 0; i < num_configs; i++) {
		unsigned int param = pinconf_to_config_param(configs[i]);
		unsigned int arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_BIAS_PULL_UP:
			n32h7xx_pin_set_pull(base, offset, N32H7XX_PULL_UP);
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			n32h7xx_pin_set_pull(base, offset, N32H7XX_PULL_DOWN);
			break;
		case PIN_CONFIG_BIAS_DISABLE:
			n32h7xx_pin_set_pull(base, offset, N32H7XX_PULL_NONE);
			break;
		case PIN_CONFIG_DRIVE_STRENGTH:
			if (arg > 3)
				return -EINVAL;
			reg = readl(base + N32H7XX_GPIO_DS);
			reg &= ~(0x3U << (offset * 2));
			reg |= arg << (offset * 2);
			writel(reg, base + N32H7XX_GPIO_DS);
			n32h7xx_hs_driver_bank(pctl, pin, false);
			break;
		case PIN_CONFIG_SLEW_RATE:
			if (arg > 1)
				return -EINVAL;
			reg = readl(base + N32H7XX_GPIO_SR);
			reg &= ~BIT(offset);
			reg |= arg << offset;
			writel(reg, base + N32H7XX_GPIO_SR);
			break;
		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
			reg = readl(base + N32H7XX_GPIO_POTYPE);
			writel(reg | BIT(offset), base + N32H7XX_GPIO_POTYPE);
			break;
		case PIN_CONFIG_DRIVE_PUSH_PULL:
			reg = readl(base + N32H7XX_GPIO_POTYPE);
			writel(reg & ~BIT(offset), base + N32H7XX_GPIO_POTYPE);
			break;
		case N32H7XX_PIN_CONFIG_HS_HIGH:
			n32h7xx_hs_driver_bank(pctl, pin, arg != 0);
			break;
		case PIN_CONFIG_LEVEL:
			reg = readl(base + N32H7XX_GPIO_POD);
			reg &= ~BIT(offset);
			reg |= arg ? BIT(offset) : 0;
			writel(reg, base + N32H7XX_GPIO_POD);
			break;
		case PIN_CONFIG_OUTPUT_ENABLE:
			if (arg)
				n32h7xx_pin_set_mux(base, offset,
						    N32H7XX_MODE_OUTPUT, 0);
			break;
		default:
			return -ENOTSUPP;
		}
	}

	return 0;
}

static int n32h7xx_pinconf_group_get(struct pinctrl_dev *pctldev,
				     unsigned int selector, unsigned long *config)
{
	const unsigned int *pins;
	unsigned int npins;

	if (pinctrl_generic_get_group_pins(pctldev, selector, &pins, &npins))
		return -EINVAL;
	if (!npins)
		return -EINVAL;

	/* Groups are single-pin, so the first pin answers for the group. */
	return n32h7xx_pinconf_get(pctldev, pins[0], config);
}

static int n32h7xx_pinconf_group_set(struct pinctrl_dev *pctldev,
				     unsigned int selector,
				     unsigned long *configs,
				     unsigned int num_configs)
{
	const unsigned int *pins;
	unsigned int npins, i;

	if (pinctrl_generic_get_group_pins(pctldev, selector, &pins, &npins))
		return -EINVAL;

	for (i = 0; i < npins; i++) {
		int ret = n32h7xx_pinconf_set(pctldev, pins[i], configs,
					      num_configs);

		if (ret)
			return ret;
	}

	return 0;
}

static const struct pinconf_ops n32h7xx_pinconf_ops = {
	.is_generic		= true,
	.pin_config_get		= n32h7xx_pinconf_get,
	.pin_config_set		= n32h7xx_pinconf_set,
	.pin_config_group_get	= n32h7xx_pinconf_group_get,
	.pin_config_group_set	= n32h7xx_pinconf_group_set,
};

/* --- GPIO --------------------------------------------------------------- */

static int n32h7xx_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct n32h7xx_gpio_bank *bank = gpiochip_get_data(chip);

	return !!(readl(bank->base + N32H7XX_GPIO_PID) & BIT(offset));
}

static int n32h7xx_gpio_set(struct gpio_chip *chip, unsigned int offset,
			    int value)
{
	struct n32h7xx_gpio_bank *bank = gpiochip_get_data(chip);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&bank->lock, flags);
	reg = readl(bank->base + N32H7XX_GPIO_POD);
	if (value)
		reg |= BIT(offset);
	else
		reg &= ~BIT(offset);
	writel(reg, bank->base + N32H7XX_GPIO_POD);
	spin_unlock_irqrestore(&bank->lock, flags);

	return 0;
}

static int n32h7xx_gpio_get_direction(struct gpio_chip *chip,
				      unsigned int offset)
{
	struct n32h7xx_gpio_bank *bank = gpiochip_get_data(chip);
	u32 mode = (readl(bank->base + N32H7XX_GPIO_PMODE) >> (offset * 2)) & 0x3;

	return mode == N32H7XX_MODE_OUTPUT ? GPIO_LINE_DIRECTION_OUT :
					     GPIO_LINE_DIRECTION_IN;
}

static int n32h7xx_gpio_direction_input(struct gpio_chip *chip,
					unsigned int offset)
{
	struct n32h7xx_gpio_bank *bank = gpiochip_get_data(chip);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&bank->lock, flags);
	reg = readl(bank->base + N32H7XX_GPIO_PMODE);
	reg &= ~(0x3U << (offset * 2));
	reg |= N32H7XX_MODE_INPUT << (offset * 2);
	writel(reg, bank->base + N32H7XX_GPIO_PMODE);
	spin_unlock_irqrestore(&bank->lock, flags);

	return pinctrl_gpio_direction_input(chip, offset);
}

static int n32h7xx_gpio_direction_output(struct gpio_chip *chip,
					 unsigned int offset, int value)
{
	struct n32h7xx_gpio_bank *bank = gpiochip_get_data(chip);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&bank->lock, flags);
	/* Drive the level before switching the pad to output. */
	reg = readl(bank->base + N32H7XX_GPIO_POD);
	if (value)
		reg |= BIT(offset);
	else
		reg &= ~BIT(offset);
	writel(reg, bank->base + N32H7XX_GPIO_POD);

	reg = readl(bank->base + N32H7XX_GPIO_PMODE);
	reg &= ~(0x3U << (offset * 2));
	reg |= N32H7XX_MODE_OUTPUT << (offset * 2);
	writel(reg, bank->base + N32H7XX_GPIO_PMODE);
	spin_unlock_irqrestore(&bank->lock, flags);

	return pinctrl_gpio_direction_output(chip, offset);
}

static const struct gpio_chip n32h7xx_gpio_template = {
	.request		= gpiochip_generic_request,
	.free			= gpiochip_generic_free,
	.get			= n32h7xx_gpio_get,
	.set			= n32h7xx_gpio_set,
	.get_direction		= n32h7xx_gpio_get_direction,
	.direction_input	= n32h7xx_gpio_direction_input,
	.direction_output	= n32h7xx_gpio_direction_output,
};

/* --- function table ----------------------------------------------------- */

struct n32h7xx_function {
	const char *name;
	const char **groups;
	unsigned int ngroups;
	unsigned int cap;
};

static int n32h7xx_func_add_group(struct device *dev,
				  struct n32h7xx_function *fn,
				  const char *group)
{
	unsigned int i;

	for (i = 0; i < fn->ngroups; i++)
		if (!strcmp(fn->groups[i], group))
			return 0;

	if (fn->ngroups == fn->cap) {
		unsigned int cap = fn->cap ? fn->cap * 2 : 4;
		const char **groups;

		groups = devm_krealloc(dev, (void *)fn->groups,
				       cap * sizeof(*groups), GFP_KERNEL);
		if (!groups)
			return -ENOMEM;
		fn->groups = groups;
		fn->cap = cap;
	}

	fn->groups[fn->ngroups++] = group;

	return 0;
}

/*
 * Find the function called "name", creating it on first use.  Only the first
 * "len" characters are significant, which lets one routine look up both an
 * exact signal name and the peripheral name it starts with ("eth1_rmii_txd0"
 * and "eth1").  Returns the function index, or a negative errno.
 */
static int n32h7xx_func_lookup(struct device *dev,
			       struct n32h7xx_function **funcs,
			       unsigned int *nfuncs, unsigned int *cap,
			       const char *name, size_t len)
{
	struct n32h7xx_function *fn;
	unsigned int i;
	char *copy;

	for (i = 0; i < *nfuncs; i++) {
		if (!strncmp((*funcs)[i].name, name, len) &&
		    (*funcs)[i].name[len] == '\0')
			return i;
	}

	if (*nfuncs == *cap) {
		unsigned int ncap = *cap ? *cap * 2 : 64;

		fn = devm_krealloc(dev, *funcs, ncap * sizeof(*fn), GFP_KERNEL);
		if (!fn)
			return -ENOMEM;
		memset(fn + *cap, 0, (ncap - *cap) * sizeof(*fn));
		*funcs = fn;
		*cap = ncap;
	}

	if (len == strlen(name)) {
		/* Exact signal names already live in the static pin table. */
		(*funcs)[*nfuncs].name = name;
	} else {
		copy = devm_kzalloc(dev, len + 1, GFP_KERNEL);
		if (!copy)
			return -ENOMEM;
		memcpy(copy, name, len);
		(*funcs)[*nfuncs].name = copy;
	}

	return (*nfuncs)++;
}

/*
 * Every signal found in the series pin table becomes a function whose groups
 * are the pins that can carry it, so the device tree can name a signal
 * instead of an alternate-function number.
 */
static int n32h7xx_build_functions(struct n32h7xx_pinctrl *pctl)
{
	struct device *dev = pctl->dev;
	struct n32h7xx_function *funcs = NULL;
	const char **all;
	unsigned int nfuncs = 0, cap = 0, i, j;
	int ret;

	/* "gpio", "analog" and "af0".."af15" are usable on every pin. */
	all = devm_kcalloc(dev, N32H7XX_NPINS, sizeof(*all), GFP_KERNEL);
	if (!all)
		return -ENOMEM;
	for (i = 0; i < N32H7XX_NPINS; i++)
		all[i] = n32h7xx_pins[i].name;

	for (i = 0; i < N32H7XX_NPINS; i++) {
		const struct n32h7xx_desc_pin *pin = &n32h7xx_pins[i];

		for (j = 0; j < pin->nfunctions; j++) {
			const char *name = pin->functions[j].name;
			int sel;

			sel = n32h7xx_func_lookup(dev, &funcs, &nfuncs, &cap,
						  name, strlen(name));
			if (sel < 0)
				return sel;
			ret = n32h7xx_func_add_group(dev, &funcs[sel],
						     pin->name);
			if (ret)
				return ret;
		}

		/*
		 * A board may name the peripheral instead of the signal
		 * ("usart1" rather than "usart1_tx"), so every peripheral also
		 * gets a function whose groups are the pins it reaches.  Where
		 * one peripheral reaches a pin through two signals with
		 * different alternate functions the pin is still listed here;
		 * the map builder rejects that combination and asks the board
		 * for the exact signal instead.
		 */
		for (j = 0; j < pin->nfunctions; j++) {
			const char *name = pin->functions[j].name;
			const char *sep = strchr(name, '_');
			int sel;

			if (!sep || sep == name)
				continue;
			sel = n32h7xx_func_lookup(dev, &funcs, &nfuncs, &cap,
						  name, sep - name);
			if (sel < 0)
				return sel;
			ret = n32h7xx_func_add_group(dev, &funcs[sel],
						     pin->name);
			if (ret)
				return ret;
		}
	}

	for (i = 0; i < nfuncs; i++) {
		ret = pinmux_generic_add_function(pctl->pctl, funcs[i].name,
						  funcs[i].groups,
						  funcs[i].ngroups, NULL);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "cannot register function %s\n",
					     funcs[i].name);
	}

	for (i = 0; i <= 0xf; i++) {
		char name[8];

		snprintf(name, sizeof(name), "af%u", i);
		ret = pinmux_generic_add_function(pctl->pctl, name, all,
						  N32H7XX_NPINS, NULL);
		if (ret < 0)
			return dev_err_probe(dev, ret, "cannot register %s\n",
					     name);
	}

	ret = pinmux_generic_add_function(pctl->pctl, "gpio", all,
					  N32H7XX_NPINS, NULL);
	if (ret < 0)
		return ret;

	ret = pinmux_generic_add_function(pctl->pctl, "analog", all,
					  N32H7XX_NPINS, NULL);
	if (ret < 0)
		return ret;

	dev_dbg(dev, "%u functions\n", nfuncs + 18);
	return 0;
}

/* --- probe -------------------------------------------------------------- */

/*
 * The GPIO and AFIO register blocks are gated on AHB5.  The bootloader leaves
 * them on for the console, but this driver is the one that now owns the
 * block, so it opens the gates itself instead of relying on probe order.
 */
static int n32h7xx_enable_clocks(struct device *dev)
{
	void __iomem *rcc;

	rcc = devm_ioremap(dev, N32H7XX_RCC_BASE, 0x400);
	if (!rcc)
		return -ENOMEM;

	writel(readl(rcc + N32H7XX_RCC_AHB5EN1) | N32H7XX_AHB5EN1_GPIO,
	       rcc + N32H7XX_RCC_AHB5EN1);
	writel(readl(rcc + N32H7XX_RCC_AHB5EN2) | N32H7XX_AHB5EN2_GPIO |
	       N32H7XX_AHB5EN2_AFIO, rcc + N32H7XX_RCC_AHB5EN2);
	/* Make sure the write reached the bridge before the block is used. */
	readl(rcc + N32H7XX_RCC_AHB5EN2);

	return 0;
}

static int n32h7xx_pinctrl_add_bank(struct n32h7xx_pinctrl *pctl,
				    struct device_node *np, unsigned int index)
{
	struct n32h7xx_gpio_bank *bank = &pctl->banks[index];
	struct device *dev = pctl->dev;
	const char **names;
	unsigned int port = index;
	u32 reg;
	int ret;

	if (!of_property_read_u32(np, "reg", &reg) &&
	    reg >= N32H7XX_GPIO_BASE &&
	    (reg - N32H7XX_GPIO_BASE) % N32H7XX_GPIO_PORT_STRIDE == 0)
		port = (reg - N32H7XX_GPIO_BASE) / N32H7XX_GPIO_PORT_STRIDE;

	if (port >= N32H7XX_NPORTS)
		return dev_err_probe(dev, -EINVAL, "%pOF: bad GPIO bank reg\n",
				     np);

	names = devm_kcalloc(dev, N32H7XX_PINS_PER_PORT, sizeof(*names),
			     GFP_KERNEL);
	if (!names)
		return -ENOMEM;
	for (unsigned int j = 0; j < N32H7XX_PINS_PER_PORT; j++)
		names[j] = n32h7xx_pins[port * N32H7XX_PINS_PER_PORT + j].name;

	bank->base = pctl->base + port * N32H7XX_GPIO_PORT_STRIDE;
	spin_lock_init(&bank->lock);

	bank->chip = n32h7xx_gpio_template;
	bank->chip.label = devm_kasprintf(dev, GFP_KERNEL, "GPIO%c",
					  'A' + port);
	if (!bank->chip.label)
		return -ENOMEM;
	bank->chip.parent = dev;
	bank->chip.fwnode = of_fwnode_handle(np);
	bank->chip.base = -1;
	bank->chip.ngpio = N32H7XX_PINS_PER_PORT;
	bank->chip.names = names;

	ret = gpiochip_add_data(&bank->chip, bank);
	if (ret)
		return dev_err_probe(dev, ret, "cannot add GPIO%c\n",
				     'A' + port);

	return 0;
}

static int n32h7xx_pinctrl_add_banks(struct n32h7xx_pinctrl *pctl)
{
	struct device *dev = pctl->dev;
	unsigned int nbanks, index = 0;

	nbanks = gpiochip_node_count(dev);
	if (!nbanks)
		return dev_err_probe(dev, -ENODEV, "no GPIO bank nodes\n");
	if (nbanks > N32H7XX_NPORTS)
		return dev_err_probe(dev, -EINVAL, "too many GPIO banks\n");

	pctl->banks = devm_kcalloc(dev, nbanks, sizeof(*pctl->banks),
				   GFP_KERNEL);
	if (!pctl->banks)
		return -ENOMEM;
	pctl->nbanks = nbanks;

	for_each_available_child_of_node_scoped(dev->of_node, np) {
		int ret;

		if (!of_property_read_bool(np, "gpio-controller"))
			continue;

		ret = n32h7xx_pinctrl_add_bank(pctl, np, index);
		if (ret)
			return ret;
		index++;
	}

	if (index != nbanks)
		return dev_err_probe(dev, -EINVAL, "GPIO bank count mismatch\n");

	return 0;
}

static int n32h7xx_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct n32h7xx_pinctrl *pctl;
	struct pinctrl_pin_desc *descs;
	unsigned int i;
	int ret;

	pctl = devm_kzalloc(dev, sizeof(*pctl), GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;
	pctl->dev = dev;
	platform_set_drvdata(pdev, pctl);

	ret = n32h7xx_enable_clocks(dev);
	if (ret)
		return ret;

	pctl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pctl->base))
		return PTR_ERR(pctl->base);

	pctl->afio = devm_ioremap(dev, N32H7XX_AFIO_BASE, 0x400);
	if (!pctl->afio)
		return -ENOMEM;

	descs = devm_kcalloc(dev, N32H7XX_NPINS, sizeof(*descs), GFP_KERNEL);
	pctl->group_pins = devm_kcalloc(dev, N32H7XX_NPINS,
					sizeof(*pctl->group_pins), GFP_KERNEL);
	if (!descs || !pctl->group_pins)
		return -ENOMEM;

	for (i = 0; i < N32H7XX_NPINS; i++) {
		descs[i].number = n32h7xx_pins[i].number;
		descs[i].name = n32h7xx_pins[i].name;
		pctl->group_pins[i] = n32h7xx_pins[i].number;
	}

	pctl->desc.name = dev_name(dev);
	pctl->desc.owner = THIS_MODULE;
	pctl->desc.pins = descs;
	pctl->desc.npins = N32H7XX_NPINS;
	pctl->desc.pctlops = &n32h7xx_pctl_ops;
	pctl->desc.pmxops = &n32h7xx_pmx_ops;
	pctl->desc.confops = &n32h7xx_pinconf_ops;
	pctl->desc.num_custom_params = ARRAY_SIZE(n32h7xx_custom_params);
	pctl->desc.custom_params = n32h7xx_custom_params;
	pctl->desc.custom_conf_items = n32h7xx_custom_conf_items;

	pctl->pctl = devm_pinctrl_register(dev, &pctl->desc, pctl);
	if (IS_ERR(pctl->pctl))
		return dev_err_probe(dev, PTR_ERR(pctl->pctl),
				     "failed to register pin controller\n");

	/* One single-pin group per package pin, named after the pin. */
	for (i = 0; i < N32H7XX_NPINS; i++) {
		ret = pinctrl_generic_add_group(pctl->pctl,
						n32h7xx_pins[i].name,
						&pctl->group_pins[i], 1, NULL);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "cannot register group %s\n",
					     n32h7xx_pins[i].name);
	}

	ret = n32h7xx_build_functions(pctl);
	if (ret)
		return ret;

	ret = n32h7xx_pinctrl_add_banks(pctl);
	if (ret)
		return ret;

	dev_info(dev, "%d pins, %u GPIO banks\n", N32H7XX_NPINS, pctl->nbanks);

	return 0;
}

static const struct of_device_id n32h7xx_pinctrl_of_match[] = {
	{ .compatible = "nsing,n32h7xx-pinctrl" },
	{ }
};
MODULE_DEVICE_TABLE(of, n32h7xx_pinctrl_of_match);

static struct platform_driver n32h7xx_pinctrl_driver = {
	.probe	= n32h7xx_pinctrl_probe,
	.driver	= {
		.name		= "n32h7xx-pinctrl",
		.of_match_table	= n32h7xx_pinctrl_of_match,
		.suppress_bind_attrs = true,
	},
};

/*
 * The console UART is driver-registered from arch/arm/mach-nsing, which links
 * ahead of drivers/ and therefore probes before a normal platform driver
 * here would.  A device that finds no pin controller when it probes is not
 * retried (the device core only treats -EPROBE_DEFER as a reason to come
 * back), so its pinctrl-0 state would be dropped without a word and the pins
 * would keep whatever the bootloader left behind.  Registering at
 * arch_initcall gets in front of that: the device tree platform devices are
 * created by arch_initcall_sync, so every consumer probes after us.
 */
static int __init n32h7xx_pinctrl_init(void)
{
	return platform_driver_register(&n32h7xx_pinctrl_driver);
}
arch_initcall(n32h7xx_pinctrl_init);

MODULE_AUTHOR("NSING");
MODULE_DESCRIPTION("NSING N32H7xx pin controller and GPIO driver");
MODULE_LICENSE("GPL");
