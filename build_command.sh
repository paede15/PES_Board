#!/bin/env/bash

## Build
pio run

## Flash 

cp .pio/build/nucleo_f446re/firmware.bin /media/kaeptn-egli/NOD_F446RE

## Serial Terminal

cu -l /dev/ttyACM0 -s 115200
