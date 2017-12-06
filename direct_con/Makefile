
obj-m+=usr_cqc.o

all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
	$(CC) testebbchar.c -o test
	$(CC) -c cqc.c -o cqc.o
	#$(CC) -c udp_net/mod.c -o udp_net/mod.o
	#$(CC) -c connection.c -o connection.o
clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean
	rm test
