#!/bin/sh

while [ true ]
do
    sudo mount -o remount,rw /
    make
    sudo mount -o remount,ro /
    ./Vision2020
    sleep 5
done
