make
sudo rmmod clientchar.ko
sudo rmmod serverchar.ko
sudo insmod serverchar.ko
sudo insmod clientchar.ko
sudo ./test
