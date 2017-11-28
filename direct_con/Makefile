obj-m+=usr_cqc.o
#obj-m+=serverchar.o

all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
	$(CC) testebbchar.c -o test
	$(CC) -c cqc.c -o cqc.o
	#$(CC) -c connection.c -o connection.o
clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean
	rm test
