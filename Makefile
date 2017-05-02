obj-m+=mcba_usb.o

KERNEL_SRC := /lib/modules/$(shell uname -r)/build/
SRC := $(shell pwd)
DEPMOD := depmod -a

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) clean

modules_install: all
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install
	$(DEPMOD)	

test:
	g++ -lgtest_main -lgtest -lpthread ./tests/mcba_tests.cpp -o ./tests/mcba_tests
	./tests/mcba_tests --gtest_break_on_failure

