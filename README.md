# Linux kernel driver for Microchip CAN BUS Analyzer Tool

The CAN BUS Analyzer Tool is a simple to use low cost CAN bus monitor which can be used to develop and debug a high speed CAN network. The tool supports CAN 2.0b and ISO11898-2 and a broad range of functions which allow it to be used across various market segments including automotive, industrial, medical and marine. The toolkit comes with all the hardware and software required to connect a CAN network to a PC. The Graphical User Interface makes it easy to quickly observe and interpret bus traffic.

[Product site](http://www.microchip.com/Developmenttools/ProductDetails.aspx?PartNO=APGDT002)

Originally the tool is supported on Windows environment only. This project adds support for the tool to Linux Kernel (SocketCAN). 

## Current status
The work on the driver is ongoing. Basic functionality like configuration, sending and receiving CAN frames. Shall work already.

## Usage
### Building and installing
```
git clone https://github.com/rkollataj/mcba_usb.git
cd mcba_usb
make
sudo make install
sudo modprobe mcba_usb
```
### Basic SocketCAN usage
To start SocketCAN interface:
```
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```
To send CAN frame:
```
cansend can0 001#DEADBEEF
cansend can0 1000001#DEADBEEF
```
To dump CAN frames:
```
candump can0
```

### Supported bus settings
The tool works internally with 40Mhz clock. Following bus speed are supported by default:
* 20 Kbps
* 33.3 Kbps
* 50 Kbps
* 80 Kbps
* 83.3 Kbps
* 100 Kbps
* 125 Kbps
* 150 Kbps
* 175 Kbps
* 200 Kbps
* 225 Kbps
* 250 Kbps
* 275 Kbps
* 300 Kbps
* 500 Kbps
* 625 Kbps
* 800 Kbps
* 1000 Kbps

Note: Bittiming parameters are hardcoded inside device. Only speed can be configured using iproute2 utils.

### Termination
The tool supports build in termination. It can be controlled by sysfs. To read current termination status:
```
cat /sys/class/net/can0/termination
```
To enable termination:
```
echo 1 > /sys/class/net/can0/termination
```
To disable termination:
```
echo 0 > /sys/class/net/can0/termination
```
Termination values are stored in device's EEPROM (no need to set it again after device reconnection).
