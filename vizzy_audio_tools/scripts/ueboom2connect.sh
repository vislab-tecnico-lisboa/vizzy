#!/bin/bash

#launch the pulseaudio daemon if it's not running

if ! pgrep -x "pulseaudio" > /dev/null
then
    echo "Pulseaudio server daemon is not running. Launching..."
    pulseaudio -D
fi



#Connect to the device
echo "Connecting to our UE BOOM 2 speaker"
bluez-test-audio connect 88:C6:26:EB:65:31

#Set the audio profile
echo "Setting card profile to a2dp"
pacmd set-card-profile bluez_card.88_C6_26_EB_65_31 a2dp

#Set as default speaker
echo "Setting as default system speaker"
pacmd set-default-sink bluez_sink.88_C6_26_EB_65_31

#Set as default microphone NOT WORKING
echo "Setting as default system speaker"
pacmd set-default-source bluez_source.88_C6_26_EB_65_31
