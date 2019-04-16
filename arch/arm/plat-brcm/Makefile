ifneq ($(ROOTDIR),)
RELATED = $(BCM_REL_DIR)../../../
else
SHARED ?= ../../../../../../../src/shared
endif

AFLAGS_plat-head.o        := -DTEXT_OFFSET=$(TEXT_OFFSET)

ifeq ($(CONFIG_BUZZZ), y)
obj-$(CONFIG_BUZZZ) += buzzz.o
endif
obj-$(CONFIG_PLAT_MPCORE) += plat-head.o
obj-$(CONFIG_PLAT_MPCORE) += mpcore_core.o
obj-$(CONFIG_PLAT_MPCORE) += mpcore_gtimer.o
obj-$(CONFIG_PLAT_MPCORE) += wps_led.o
obj-$(CONFIG_PLAT_MPCORE) += mpcore_cache.o
obj-$(CONFIG_PLAT_MPCORE) += clock.o
obj-$(CONFIG_PLAT_MPCORE) += platsmp.o 
obj-$(CONFIG_PLAT_IPROC) += iproc_cru.o
obj-$(CONFIG_PLAT_BCM5301X) += bcm5301x.o bcm5301x_dmu.o
obj-$(CONFIG_PLAT_BCM5301X) += bcm5301x_pcie.o
obj-$(CONFIG_PLAT_NAND_JFFS2) += bcm5301x_nandc.o
obj-$(CONFIG_PLAT_BCM5301X) += nvram_linux.o
obj-$(CONFIG_PLAT_BCM5301X) += hndcoma.o
obj-$(CONFIG_PLAT_BCM5301X) += bcm5301x_pwm.o
obj-$(CONFIG_PLAT_BCM5301X) += $(addprefix $(RELATED)$(SHARED)/, hndarm_ca9.o)
obj-$(CONFIG_PLAT_BCM5301X) += $(addprefix $(RELATED)$(SHARED)/, hndsoc.o)
obj-$(CONFIG_PLAT_BCM5301X) += $(addprefix $(RELATED)$(SHARED)/, hndchipc.o)
obj-$(CONFIG_PLAT_BCM5301X) += $(addprefix $(RELATED)$(SHARED)/, nvram.o)
obj-$(CONFIG_PLAT_BCM5301X) += $(addprefix $(RELATED)$(SHARED)/, hndpci.o)
obj-$(CONFIG_PLAT_BCM5301X) += $(addprefix $(RELATED)$(SHARED)/, hndsflash.o)
obj-$(CONFIG_PLAT_BCM5301X) += $(addprefix $(RELATED)$(SHARED)/, spiflash.o)
obj-$(CONFIG_PLAT_BCM5301X) += $(addprefix $(RELATED)$(SHARED)/, nandcore.o)
obj-$(CONFIG_PLAT_BCM5301X) += $(addprefix $(RELATED)$(SHARED)/, hndnand.o)
obj-$(CONFIG_PLAT_BCM5301X) += $(addprefix $(RELATED)$(SHARED)/, linux_gpio.o)
obj-$(CONFIG_CACHE_L310) += cache-l310.o
