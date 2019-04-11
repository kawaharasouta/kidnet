obj-m := kidnet.o
kidnet-objs := 
#PWD := $(shell pwd)

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(CURDIR) modules
	cp kidnet.ko ../
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(CURDIR) clean
