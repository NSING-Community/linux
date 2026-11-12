# SPDX-License-Identifier: GPL-2.0-only
dtb-$(CONFIG_SOC_N32H7XX) += n32h787-hmi.dtb

ifeq ($(CONFIG_SOC_N32H7XX),y)
extra-y += n32h787-hmi-camera.dtbo
extra-y += n32h787-hmi-cm4.dtbo
extra-y += n32h787-hmi-display.dtbo
targets += n32h787-hmi-camera.dtbo
targets += n32h787-hmi-cm4.dtbo
targets += n32h787-hmi-display.dtbo
endif

# The base tree carries phandles for feature overlays.
DTC_FLAGS_n32h787-hmi := -@
