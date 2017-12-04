#!/bin/bash

for ip in `seq 1 255`; 
do
    ping -c 1 -W 1 192.168.1.$ip |grep "64 bytes" &
done
