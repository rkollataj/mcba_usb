obj-m+=mcba_usb.o
 
all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(shell pwd) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(shell pwd) clean

install: all
	cp mcba_usb.ko /lib/modules/$(shell uname -r)/kernel/drivers/net/can/usb
	depmod -a

test:
	g++ -lgtest_main -lgtest -lpthread ./tests/mcba_tests.cpp -o ./tests/mcba_tests
	./tests/mcba_tests --gtest_break_on_failure

