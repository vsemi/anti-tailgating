# List of Components and Tools:

#### 5V USB Type-C power adaptor

### Optional:

#### Personal computer
#### Self-powered USB hub
#### USB Type-A to USB Type-C cable
#### WiFi connection

# Installing Device

Ideally, the device should be installed 2.5 to 3.5 meters above the ground, directly above the entrance. Refer to below image for device field of view.

### Very important: the device should be firmed installed, loosing or shaking will seriously impact the accuracy of detection. Power off at least 10 seconds and on power to reboot the device after every time after installation adjustment, and make sure there is no person or any non-fixed objects in the FOV area, ie ladders, stools, boxes etc, before powering on the device.

![Web UI](install.png)

# Quick Start

### Very important: make sure there is no person or any non-fixed objects in the FOV area, ie ladders, stools, boxes etc, before powering on the device.

#### 1. Connect the people counting device to 5V USB Type-C power adaptor, and power on
#### 2. Wait for the 10 blinks of the green light on the device
#### 3. Device is ready

# Relay

#### Relay is low-effective

#####   (Black):  GND

##### 1 (Blue):   zero person
##### 2 (Green):  one person
##### 3 (Yellow): two persons
##### 4 (White):  suspicious

#####   (Red):    +5V

# View depth image and review boundary

### Very important: USB cable to be used in following steps, needs to be a high-quality Type-A to USB Type-C cable. Some Type-C cables for phone charger, will not work.

#### 1. Power on PC
#### 2. Power on the Self-powered USB hub
#### 3. Connect Self-powered USB hub to PC
#### 3. Connect the people counting device to Self-powered USB
#### 4. Wait for the 10 blinks of the green light on the device
#### 5. Launch web browser and enter http://10.42.0.1:8800 in the browser address bar

![Web UI](view.png)

# Software Update

## Connect to device console

### Very important: USB cable to be used in following steps, needs to be a high-quality Type-A to USB Type-C cable. Some Type-C cables for phone charger, will not work.

#### 1. Power on PC
#### 2. Power on the Self-powered USB hub
#### 3. Connect Self-powered USB hub to PC
#### 3. Connect the people counting device to Self-powered USB
#### 4. Wait for the 10 blinks of the green light on the device
#### 5. Connect to device IP address cat@10.42.0.1 (user name: cat, password: temppwd)
##### Linux: 
```
ssh cat@10.42.0.1
```
##### Windows: tool such as Putty or MobaXterm can be used

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

## Update Software

#### Run command: 

```
cd ~/anti-tailgating
git pull
cd build
make
exit
```

![Web UI](update.png)

#### Power down and up to reboot the device


