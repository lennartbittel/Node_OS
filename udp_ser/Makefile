obj-m+=mod.o
#obj-m+=serverchar.o

all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
	#$(CC) -c connection.c -o connection.o
clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean
