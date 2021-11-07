#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

# Connectivity combo driver
# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE),)
    subdir-ccflags-y += -I$(srctree)/
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/clkbuf_v1
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/clkbuf_v1/$(MTK_PLATFORM)
    subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
    subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ifeq ($(CONFIG_MTK_PMIC_CHIP_MT6359),y)
    subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6359
endif
ifeq ($(CONFIG_MTK_PMIC_NEW_ARCH),y)
    subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/pmic/include
endif
    subdir-ccflags-y += -I$(srctree)/drivers/mmc/core
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/eccci/$(MTK_PLATFORM)
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/eccci/
    subdir-ccflags-y += -I$(srctree)/drivers/clk/mediatek/
    subdir-ccflags-y += -I$(srctree)/drivers/pinctrl/mediatek/

    # Do Nothing, move to standalone repo
    MODULE_NAME := connadp
    obj-y += $(MODULE_NAME).o
    $(MODULE_NAME)-objs += common/connectivity_build_in_adapter.o
    $(MODULE_NAME)-objs += common/wmt_build_in_adapter.o


    # Do build-in for Makefile checking
    # export CONFIG_WLAN_DRV_BUILD_IN=y

    ifeq ($(CONFIG_WLAN_DRV_BUILD_IN),y)
        #$(info $$CONFIG_MTK_COMBO_CHIP is [${CONFIG_MTK_COMBO_CHIP}])
        MTK_PLATFORM_ID := $(patsubst CONSYS_%,%,$(subst ",,$(CONFIG_MTK_COMBO_CHIP)))
        #$(info MTK_PLATFORM_ID is [${MTK_PLATFORM_ID}])

        # for gen4m options
        export CONFIG_MTK_COMBO_WIFI_HIF=axi
	export MTK_COMBO_CHIP=CONNAC2X2_SOC3_0
        export WLAN_CHIP_ID=6893
        export MTK_ANDROID_WMT=y
        export MTK_ANDROID_EMI=y
        export ADAPTOR_OPTS=$(MTK_COMBO_CHIP)
	export WIFI_IP_SET=1
	export WIFI_ECO_VER=1

        # Do build-in for xxx.c checking
        subdir-ccflags-y += -D MTK_WCN_REMOVE_KERNEL_MODULE
        subdir-ccflags-y += -D MTK_WCN_BUILT_IN_DRIVER
	obj-y += conninfra/
        obj-y += wlan/adaptor/
        obj-y += wlan/core/gen4m/

    endif

# Bluetooth
obj-$(CONFIG_MTK_COMBO_BT) += bt/mt66xx/connac2/

# GPS
obj-$(CONFIG_MTK_COMBO_GPS) += gps/

# Otherwise we were called directly from the command line;
# invoke the kernel build system.
else
    KERNELDIR ?= /lib/modules/$(shell uname -r)/build
    PWD  := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
endif
