#!/bin/bash

NRF51_SDK = ~/nRF51_SDK_8.1.0

echo "Which softdevice would you like to use? (M for master, S for Slave)"
read soft

if [ $soft = M ]
	then SOFTDEVICE = $NRF51_SDK/components/softdevice/s120/hex/s120_softdevice.hex
elif [ $soft = S ]
	then SOFTDEVICE = $NRF51_SDK/components/softdevice/s110/hex/s110_softdevice.hex
else echo "Invalid selection. Terminating Script."
fi



