
obj-m+=qcpu.o

all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules -Wall

clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean
