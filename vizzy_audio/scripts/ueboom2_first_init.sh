#!/bin/bash

#Pair the UEBOOM 2
bluez-simple-agent hci0 88:C6:26:EB:65:31

#Set as trusted
bluez-test-device trusted 88:C6:26:EB:65:31 yes

#Restart bluetooth
sudo /etc/init.d/bluetooth restart
