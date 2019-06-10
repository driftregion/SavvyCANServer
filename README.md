# SavvyCANServer

A SavvyCAN protocol server

This isolates the section of [ESP32RET](https://github.com/collin80/ESP32RET) that's responsible for sending frames to the [SavvyCAN client](https://github.com/collin80/SavvyCAN) over WiFi.

## Status: Experimental

## Usage
```cpp
#include "SavvyCANServer.h"

WiFiServer wifiServer(23); // Register as a telnet server
SavvyCANServer savvycan(Serial, wifiServer);

void setup()
{
  savvycan.setup();
  savvycan.registerBus(MOTOR_BUS);		// type: CAN_COMMON
  savvycan.registerBus(DIAG_BUS);

  // Use a FreeRTOS task
  xTaskCreatePinnedToCore(&SavvyCANServerTask, "SavvyCANServerTask", 10800, &savvycan, 2, NULL, 0);
}

...
```
