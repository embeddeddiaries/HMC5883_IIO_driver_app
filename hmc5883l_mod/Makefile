obj-m := iio_hmc5883.o
MY_CFLAGS += -g -DDEBUG
ccflags-y += ${MY_CFLAGS}
CC += ${MY_CFLAGS}

KERNEL_SRC := /home/jaggu/workspace/udooSumo/kernel_4.17.x+fslc/linux-fslc/
SRC := $(shell pwd)
all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

debug:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) EXTRA_CFLAGS="$(MY_CFLAGS)"

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.makers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers

