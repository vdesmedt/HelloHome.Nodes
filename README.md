# HelloHome.Rfm2Pi [![Build Status](https://travis-ci.org/vdesmedt/HelloHome.Rfm2Pi.svg?branch=master)](https://travis-ci.org/vdesmedt/HelloHome.Rfm2Pi)

Used as gateway between RFM69 and any serial enabled devices (i.e. Raspberry Pi).

- All messages received from RFM69 are prefixed with rfAddress (1 byte) and RSSI (2 bytes) and send over serial port
- All messages received from serial port should be prefixed un rfAddress (1 byte)

Support a SSD1306 based LCD screen to display :
- RFM69 familly (H / HW)
- Network Id
- Version of the firmware running
- Log of the 6 last message transfered
