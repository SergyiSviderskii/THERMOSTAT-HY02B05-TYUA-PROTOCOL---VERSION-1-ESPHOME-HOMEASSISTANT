THERMOSTAT HY02B05 TYUA PROTOCOL - VERSION 1
ESPHOME, HOMEASSISTANT.
ESP - tywe3s
The code "esphome - tyua" is taken as a basis.
The reason for not using the standard "esphome - thuya" is that there are, 
albeit minor differences between version 1 and version 3 that did not allow to achieve the result I wanted achievable,
 but I won’t write it separately. Maybe this is not necessary was to do, but I did not see another.
Because no programming experience sorry, in the code description. The code is working, 
works on 6 devices. Code improvements are inevitable and will be.

1. Connect the ESP to the programmer.
https://user-images.githubusercontent.com/78846075/110201355-ee02fc80-7e62-11eb-8dd9-dc11fa137f9d.jpg
https://user-images.githubusercontent.com/38895319/110212277-a1222480-7e68-11eb-8bc7-c662ae2fbc5b.jpeg
https://github.com/arendst/Tasmota/discussions/10513#discussioncomment-5826315
ESP can not be unsoldered. Throw a jumper between the "reset" and "GND" connector "JP 1", 
connect the ESP and you can write.
Copy the original firmware, it is advisable to use esptool.

2. Copy folders thermostats-tuya.h, thermostat-hall.yaml, 
device-thermostat-HY02B05.yaml to homeassistant /config/esphome/

3. Run the ESPhome add-on and create a new file
on the ESP 8266, transfer the encryption key, passport, name instead of "hall" from the newly created file to the 
thermostat-hall.yaml file (rename "hall" to your name).

4. Compile and write to ESP. Turn off the programmer.. Turn on the device. In the router, we find the device, 
IP address and write it down in thermostat-hall.yaml (for updates).In the thermostat-lovlace.yaml 
file for the visualization panel, rename "hall" to your name.

5 etc. everything is standard.


![IMG_1628](https://github.com/SergyiSviderskii/THERMOSTAT-HY02B05-TYUA-PROTOCOL---VERSION-1-ESPHOME-HOMEASSISTANT/assets/70329544/fecb2059-b28f-438c-98f5-b0c3504e4fc8)
