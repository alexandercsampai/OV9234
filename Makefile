obj-m += ov9234.o #[cite: 2]

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(CURDIR)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
