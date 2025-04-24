# List of Components and Tools:

#### 1. 5V USB Type-C power adaptor

### Optional:

#### 2. Personal computer
#### 3. Self-powered USB hub
#### 4. USB Type-A to USB Type-C cable
#### 5. WiFi connection

# Very important: 

#### 1. the device should be firmly installed and facing down straight, loosing or shaking will seriously impact the accuracy of detection
#### 2. Power off at least 10 seconds and power on again to reboot the device after every time adjusted installation
#### 3. make sure there is no person or any non-fixed objects in the FOV area, ie ladders, stools, boxes etc, before powering on the device
#### 4. If there are doors possibley visible by the sensor, close all of the doors before powering on the device

#### 5. USB cable to be used in this instruction, needs to be high-quality Type-A to Type-C cable. Some low quality Type-C cables ie for phone charger, will not work for high speed data transition.

# Installing Device

Ideally, the device should be installed 2.5 to 3.5 meters above the ground, directly above the entrance, and facing down. Refer to below image for device field of view.

![Web UI](install.png)

# Quick Start

#### 1. Connect the people counting device to 5V USB Type-C power adaptor
#### 2. Leds both Green and Yellow both on and off
#### 3. Led green on and off
#### 4. Led green blinks 10 times
#### 5. Led yellow light blinks about every 3 seconds
#### 6. Device is ready

# Relay

#### Relay is low-effective

##### Pin 1:  GND

##### Pin 2 (relay channel 1):  zero person or wakeup signal
##### Pin 3 (relay channel 2):  one person
##### Pin 4 (relay channel 3):  two persons
##### Pin 5 (relay channel 4):  suspicious

##### Pin 6:  +5V

# View depth image and review boundary

#### When the device is working on view mode, response will be slower than regular mode

#### 1. Power on PC
#### 2. Power on the Self-powered USB hub
#### 3. Connect Self-powered USB hub to PC
#### 3. Connect the people counting device to self-powered USB hub
#### 4. Wait for the 10 fast blinks of the green light and the yellow light start to blink about every 3 seconds
#### 5. Launch web browser and enter http://10.42.0.1:8800 in the browser address bar

![Web UI](view.png)

# Software Update

## Connect to device console

#### 1. Power on PC
#### 2. Power on the Self-powered USB hub
#### 3. Connect Self-powered USB hub to PC
#### 3. Connect the people counting device to Self-powered USB
#### 4. Wait for the 10 fast blinks of the green light and the yellow light start to blink about every 3 seconds
#### 5. Connect to device IP address cat@10.42.0.1 (user name: cat, password: temppwd)
##### Linux: 
```
ssh cat@10.42.0.1
```

enter password: temppwd

##### Windows: 

download and install Putty from here: [Download Putty](https://www.putty.org/) 

connect to the IP address (10.42.0.1) 

enter user name: cat, password: temppwd

![Web UI](console.png)

## Connect to wifi

#### Run command: 
```
sudo nmcli d wifi list
```
#### Find the WIFI name that is desired, replace your wifi name and password and run following command:
```
sudo nmcli d wifi connect <WIFI_NAME> password <WIFI_PASSWORD> 
```

![Web UI](wifi.png)

After wifi connected, the device should have an IP address besides the 127.0.0.1 and 10.42.0.1:

```
ifconfig 
```

![Web UI](ip_address.png)

In the screenshot above, additional IP address (10.0.0.223) obtained. 

#### you may then use this wifi IP address (instead of 10.42.0.1) to connect to the device console for software update or data recording when the device installed on the ceiling, so that you do not need connect the device to your PC via an USB cable, for your convenience. 
#### Please be aware of that, the wifi IP address is dynamic, possibly changed without you knowing about it next time the device restarted, when that happened and you can't connect to the device console using wifi IP address, you may still need to connect the device to your PC to figure out the new wifi IP address.

## Update Software

### 1. Connect to device console

### 2. Run command: 

```
sudo systemctl stop peoplecount
cd ~/anti_tailgating
git pull
cd build
make
exit
```

![Web UI](update.png)

#### Power down the device for at least 10 seconds before plug the device back to 5V USB Type-C power adaptor

## Record Data

### 1. Connect to device console

### 2. Modify startup script

#### Run command: 

```
sudo systemctl stop peoplecount

cd ~/

sudo vi peoplecount_start.sh
```
File peoplecount_start.sh openned for edit

Modify the line:
```
sudo /home/cat/anti_tailgating/build/peoplecount train-detect
```
change to:
```
sudo /home/cat/anti_tailgating/build/peoplecount train-record
```

Unplug the USB cable which connected to PC, and plug in 5V power adptor.

Now the device is in Data Recording mode

### Very important: 

#### 1. after data recorded, wait for at least 5 minutes before power down the device or unpluging the power adptor, otherwise the data might not be sync properly to the SD card and the recorded data might be corrupted!

#### 2. after data recorded, follow step above of "Modify startup script" to change the startup script back to:

```
sudo /home/cat/anti_tailgating/build/peoplecount train-detect
```

## Retrieve recorded data

### 1. Login into the device console and run command:

```
sudo systemctl stop peoplecount

cd ~/

tar -czvf data.tar.gz data
```
### 2. Download data (Linux):

#### Linux:
```
scp cat@10.42.0.1:~/data.tar.gz ./
```
#### Windows:
```
pscp cat@10.42.0.1:~/data.tar.gz C:\
```
