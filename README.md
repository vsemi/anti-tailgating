# List of Components and Tools:

#### 5V USB Type-C power adaptor

### Optional:

#### Personal computer
#### Self-powered USB hub
#### USB Type-A to USB Type-C cable
#### WiFi connection

# Installing Device

Ideally, the device should be installed 2.5 to 3.5 meters above the ground, directly above the entrance. Refer to below image for device field of view.

![Web UI](install.png)

# Quick Start

### Very important: make sure there is no person or any non-fixed objects in the FOV area, ie ladders, stools, boxes etc, before powering on the device.

#### 1. Connect the people counting device to 5V USB Type-C power adaptor, and power on
#### 2. Wait for the 10 blinks of the green light on the device
#### 3. Device is ready

# Relay

#### Relay is low-effective

#####   ($${\color{red}Black}$$):<pre>  GND</pre>

##### 1 ($${\color{blue}Blue}$$):<pre>   zero person</pre>
##### 2 ($${\color{green}Green}$$):<pre>  one person</pre>
##### 3 ($${\color{yellow}Yellow}$$):<pre> two persons</pre>
##### 4 ($${\color{white}White}$$):<pre>  suspicious</pre>

#####   ($${\color{red}Red}$$):<pre>    +5V</pre>

# View depth image and review boundary

#### 1. Power on PC
#### 2. Power on the Self-powered USB hub
#### 3. Connect Self-powered USB hub to PC
#### 3. Connect the people counting device to Self-powered USB
#### 4. Wait for the 10 blinks of the green light on the device
#### 5. Launch web browser and enter http://10.42.0.1:8800 in the browser address bar

![Web UI](view.png)
