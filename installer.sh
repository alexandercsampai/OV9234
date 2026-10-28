
#!/bin/bash

make clean || true 
make && \
sudo rmmod ov9234 || true && \
sudo dmesg -c && \
sudo insmod ov9234.ko && \
sudo dmesg | grep -i ov9234
