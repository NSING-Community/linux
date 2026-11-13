# SPDX-License-Identifier: GPL-2.0-only
dtb-$(CONFIG_SOC_N32H7XX) += n32h787-hmi.dtb

# The board tree follows driver Kconfig symbols, so expose the generated
# autoconf.h and IS_ENABLED() helpers to its preprocessor.
$(obj)/n32h787-hmi.dtb: dtc_cpp_flags += \
	-I $(objtree)/include -I $(srctree)/include
