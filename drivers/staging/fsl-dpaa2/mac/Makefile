
obj-$(CONFIG_FSL_DPAA2_MAC) += dpaa2-mac.o

dpaa2-mac-objs := mac.o dpmac.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
