# Linux kernel driver for Microchip CAN BUS Analyzer Tool

The CAN BUS Analyzer Tool is a simple to use low cost CAN bus monitor which can be used to develop and debug a high speed CAN network. The tool supports CAN 2.0b and ISO11898-2 and a broad range of functions which allow it to be used across various market segments including automotive, industrial, medical and marine. The toolkit comes with all the hardware and software required to connect a CAN network to a PC. The Graphical User Interface makes it easy to quickly observe and interpret bus traffic.

[Product site](http://www.microchip.com/Developmenttools/ProductDetails.aspx?PartNO=APGDT002)

The tool is supported on Windows environment only. This project adds support for the tool to Linux Kernel (SocketCAN). 

## Usage
### Building and installing
```
git clone https://github.com/rkollataj/mcba_usb.git
cd mcba_usb
make
make install
modprobe mcba_usb
```
### Supported bus settings
The tool works internally with 40Mhz clock. Following setting are supported by default:
* 20 KBps
** BRP: 100
** SJW: 1
** PROP: 5
** SEG1: 8
** SEG2: 6
* 33.3 KBps
** BRP: 48
** SJW: 1
** PROP: 8
** SEG1: 8
** SEG2: 8
* 50 KBps
** BRP: 40
** SJW: 1
** PROP: 8
** SEG1: 7
** SEG2: 4
* 80 KBps
** BRP: 20
** SJW: 1
** PROP: 8
** SEG1: 8
** SEG2: 8
* 83.3 KBps
** BRP: 20
** SJW: 1
** PROP: 8
** SEG1: 8
** SEG2: 7
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
