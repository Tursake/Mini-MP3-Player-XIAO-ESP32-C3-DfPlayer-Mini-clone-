The code is a mess and needs optimization, but it functions. The DFPlayer Mini clone is a MP3-TF-16P V3.0 that didn't work with any DFPlayer libraries I tried, so I went straight to serial using some guides I found.

-The XIAO ESP32 goes into sleep mode as soon as audio isn't playing
-Pressing the button on GPIO3 plays a random audio clip from the device, while the BUSY pin on the DFPlayer is being listened on through GPIO4 to determine when to go back to sleep
-Currently runs off USB-C, but will be modified to use a 402020 battery
