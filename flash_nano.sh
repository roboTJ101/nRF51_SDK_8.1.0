#!/bin/bash

NRF51_SDK=~/nRF51_SDK_8.1.0

printf "\nWhich softdevice would you like to use? (m for master, s for Slave)\n"
read soft

if [ $soft = m ]
	then printf "\nUsing s120 (master) softdevice\n"
	SOFTDEVICE=$NRF51_SDK/components/softdevice/s120/hex/s120_softdevice.hex
	INPUT=nrf51422_xxac_s120.hex
elif [ $soft = s ]
	then printf "\nUsing s110 (slave) softdevice\n"
	SOFTDEVICE=$NRF51_SDK/components/softdevice/s110/hex/s110_softdevice.hex
	INPUT=nrf51422_xxac_s110.hex
else printf "\nInvalid selection. Terminating Script.\n"
	exit
fi

cd _build
OUTPUT=output.hex
srec_cat $SOFTDEVICE -intel $INPUT -intel -o $OUTPUT -intel --line-length=44
printf "\nFlashing BLE Nano...\n"
cp $OUTPUT /media/$USER/MBED
printf "Flash Complete\n"


