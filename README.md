# SavvyCANServer

Just the section of ESP32RET that's responsible for sending frames to the [SavvyCAN client](https://github.com/collin80/SavvyCAN) over WiFi

## Usage
```cpp
#include "SavvyCANServer.h"

SavvyCANServer  savvyCAN(serial, wifiServer);
...
```
