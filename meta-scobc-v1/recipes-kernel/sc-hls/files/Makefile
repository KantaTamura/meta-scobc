SRC := $(CURDIR)
obj-m := sc-hls.o

KDIR ?= $(KERNEL_SRC)
KDIR ?= $(KERNEL_PATH)
KDIR ?= /lib/modules/$(shell uname -r)/build

KOUT ?= $(O)

ifeq ($(DEBUG),1)
	ccflags-y += -DSC_ROVER_HLS_DEBUG
endif
ccflags-y += -DSC_ROVER_HLS_DEBUG

export KBUILD_EXTRA_SYMBOLS

.PHONY: all modules modules_install install clean

all: modules

modules:
	$(MAKE) -C $(KDIR) \
		M=$(SRC) \
		$(if $(KOUT),O=$(KOUT),) \
		modules

modules_install install:
	$(MAKE) -C $(KDIR) \
		M=$(SRC) \
		$(if $(KOUT),O=$(KOUT),) \
		modules_install

clean:
	$(MAKE) -C $(KDIR) \
		M=$(SRC) \
		$(if $(KOUT),O=$(KOUT),) \
		clean
	@rm -f Module.symvers
