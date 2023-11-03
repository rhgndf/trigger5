make clean
make all -j
sudo rmmod trigger5 2>/dev/null
sudo insmod trigger5.ko
