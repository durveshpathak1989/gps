# Test Quad GPS Library

`GPSSensor` parses NMEA sentences from a GY-GPS6MV2/u-blox NEO-6M style module over UART1.

## Pin Map

| GPS signal | ESP32 pin | Notes |
| --- | ---: | --- |
| TXD | GPIO 13 | GPS TXD into ESP32 UART1 RX |
| RXD | GPIO 17 | Optional ESP32 UART1 TX to GPS RXD |
| VCC | 3.3V | Many modules accept 3.3V-5V; match your board |
| GND | GND | Common ground |

## Main INO Integration Example

```cpp
#include "GPSSensor.h"

#define PIN_GPS_RX 13
#define PIN_GPS_TX 17

void setup() {
    gps.begin(PIN_GPS_RX, PIN_GPS_TX, 9600);
}

void loop() {
    gps.update();
    GPSData d = gps.get();
    if (d.fix) {
        // d.lat, d.lon, d.altitude_m, d.sats, d.hdop
    }
}
```


## Why These Data Types

Latitude and longitude are stored as `double` because small decimal-degree differences matter for position. Altitude, speed, and course use `float` because their sensor precision is lower. Small counters such as satellites and time fields use `uint8_t` to make their range explicit.
