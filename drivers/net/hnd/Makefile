#
# Makefile for Broadcom Home Networking Division (HND) shared driver code
#
# Copyright (C) 2013, Broadcom Corporation. All Rights Reserved.
# 
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
# 
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
# SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
# OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
# CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# $Id: Makefile,v 1.5 2008-05-02 22:49:54 $
#

ifneq ($(ROUTER),)
RELATED = $(BCM_REL_DIR)../../../
else
SHARED   ?= ../../../../../../../src/shared
endif

WLSHARED := wlconfig_lx_shared
ifneq ($(ROOTDIR)$(SRCBASE),)
WLCFGDIR := $(ROOTDIR)$(SRCBASE)/wl/config
WLSYSDIR := $(ROOTDIR)$(SRCBASE)/src/wl/sys
else
WLCFGDIR := $(src)/../../../../../../../src/wl/config
WLSYSDIR := $(src)/../../../../../../../src/wl/sys
endif

ifeq ($(CONFIG_BUZZZ_FUNC),y)
subdir-ccflags-y += -O0 -fno-inline -finstrument-functions
endif # CONFIG_BUZZZ_FUNC

ifneq ($(wildcard $(WLCFGDIR)),)
include $(WLCFGDIR)/$(WLSHARED)
include $(WLCFGDIR)/wl.mk
endif

hnd-objs := $(addprefix $(RELATED)$(SHARED)/, $(sort $(WLFILES:.c=.o)))
HND_OBJS := $(addprefix $(src)/$(RELATED)$(SHARED)/, $(sort $(WLFILES:.c=.o)))
EXTRA_CFLAGS += $(WLFLAGS) -DBCM5354 -DLINUX_HYBRID_POSTPROCESSING_REMOVAL

# For the ARM platform...
ifneq ($(CONFIG_MACH_BRCM_NS),)
EXTRA_CFLAGS    += -DROBO_SRAB -DBCMDMASGLISTOSL
endif

ifneq ($(CONFIG_BCM947XX),y)
ifneq ($(CONFIG_PLAT_BCM5301X),y)
HND_OBJS += $(src)/$(RELATED)$(SHARED)/nvramstubs.o
hnd-objs += $(RELATED)$(SHARED)/nvramstubs.o
endif
endif

ifeq ($(CONFIG_ET_47XX),y)
HND_OBJS += $(src)/$(RELATED)$(SHARED)/bcmrobo.o
hnd-objs += $(RELATED)$(SHARED)/bcmrobo.o
else
ifeq ($(CONFIG_BCM57XX),y)
HND_OBJS += $(src)/$(RELATED)$(SHARED)/bcmrobo.o
hnd-objs += $(RELATED)$(SHARED)/bcmrobo.o
endif
endif

ifdef CONFIG_USB_RNDIS
EXTRA_CFLAGS += -DCONFIG_USBRNDIS_RETAIL
endif                                  

ifeq ($(BCMDBG_MEM),1)
EXTRA_CFLAGS += -DBCMDBG_MEM
endif

obj-$(CONFIG_HND) := hnd.o shared_ksyms.o

ifeq ($(wildcard $(src)/$(RELATED)$(SHARED)/bcmutils.c),)
hnd-objs += $(RELATED)$(SHARED)/bcmutils.o
endif

ifeq ($(wildcard $(src)/$(RELATED)$(SHARED)/bcmrobo.c),)
hnd-objs += $(RELATED)$(SHARED)/bcmrobo.o
endif

ifeq ($(wildcard $(src)/$(RELATED)$(SHARED)/hnddma.c),)
hnd-objs += $(RELATED)$(SHARED)/hnddma.o
endif

ifeq ($(wildcard $(src)/$(RELATED)$(SHARED)/linux_osl.c),)
hnd-objs += $(RELATED)$(SHARED)/linux_osl.o
endif

ifeq ($(wildcard $(src)/$(RELATED)$(SHARED)/sbutils.c),)
hnd-objs += $(RELATED)$(SHARED)/sbutils.o
endif

ifeq ($(wildcard $(src)/$(RELATED)$(SHARED)/hndpmu.c),)
hnd-objs += $(RELATED)$(SHARED)/hndpmu.o
endif

ifeq ($(wildcard $(src)/$(RELATED)$(SHARED)/bcmsrom.c),)
hnd-objs += $(RELATED)$(SHARED)/bcmsrom.o
endif

#ifeq ($(wildcard $(src)/$(RELATED)$(SHARED)/bcmwpa.c),)
#hnd-objs += $(RELATED)$(SHARED)/bcmwpa.o
#endif

ifeq ($(wildcard $(src)/$(RELATED)$(SHARED)/bcmotp.c),)
hnd-objs += $(RELATED)$(SHARED)/bcmotp.o
endif

ifeq ($(wildcard $(src)/$(RELATED)$(SHARED)/nicpci.c),)
hnd-objs += $(RELATED)$(SHARED)/nicpci.o
endif

$(src)/shared_ksyms.c: $(src)/shared_ksyms.sh $(HND_OBJS)
	sh -e $< $(HND_OBJS) > $@

clean-files += $(RELATED)$(SHARED)/*.o $(RELATED)$(SHARED)/.*.o.cmd
