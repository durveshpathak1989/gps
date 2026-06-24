/**
 * ============================================================
 *  GPSSensor.h — GY-GPS6MV2 (u-blox NEO-6M) NMEA Parser
 *  ESP32 / FreeRTOS compatible
 * ============================================================
 *
 *  Wiring (4-pin module):
 *    GY-GPS6MV2 VCC  →  3.3 V  (module has onboard 3.3V reg, accepts 3.3–5V)
 *    GY-GPS6MV2 GND  →  GND
 *    GY-GPS6MV2 TXD  →  GPIO 13  (ESP32 UART1 RX)
 *    GY-GPS6MV2 RXD  →  GPIO 17  (ESP32 UART1 TX — optional, not needed for read-only)
 *
 *  Protocol: NMEA 0183, 9600 baud, 8N1
 *  Sentences parsed: $GPRMC, $GPGGA
 *
 *  Data available:
 *    - Latitude / Longitude (decimal degrees)
 *    - Altitude (metres MSL)
 *    - Speed (km/h and knots)
 *    - Course (degrees true)
 *    - Fix quality (no fix / GPS / DGPS)
 *    - Satellites in view
 *    - HDOP (horizontal dilution of precision)
 *    - UTC time (HH:MM:SS)
 *    - UTC date (DD/MM/YY)
 *    - Valid fix flag
 * ============================================================
 */

#pragma once
#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ─────────────────────────────────────────────────────────────
//  Pin / UART config
// ─────────────────────────────────────────────────────────────
#define GPS_UART_NUM        1          // UART1
#define GPS_RX_PIN          13         // GPS TXD → ESP32 GPIO13
#define GPS_TX_PIN          17         // GPS RXD ← ESP32 GPIO17 (optional)
#define GPS_BAUD            9600
#define GPS_TIMEOUT_MS      2000       // ms before fix considered stale

// ─────────────────────────────────────────────────────────────
//  NMEA sentence buffer
// ─────────────────────────────────────────────────────────────
#define GPS_NMEA_MAX_LEN    100

// ─────────────────────────────────────────────────────────────
//  GPS data struct — all public fields
// ─────────────────────────────────────────────────────────────
struct GPSData {
    // Position
    double  latitude;       // decimal degrees, positive=N, negative=S
    double  longitude;      // decimal degrees, positive=E, negative=W
    float   altitude_m;     // metres above mean sea level (from GPGGA)
    float   geoid_sep_m;    // geoid separation (from GPGGA)

    // Motion
    float   speed_kmh;      // ground speed in km/h
    float   speed_knots;    // ground speed in knots
    float   course_deg;     // true course, degrees

    // Fix quality
    uint8_t fix_quality;    // 0=no fix, 1=GPS, 2=DGPS
    uint8_t satellites;     // satellites used in fix
    float   hdop;           // horizontal dilution of precision

    // Time / date (UTC)
    uint8_t  hour, minute, second;
    uint8_t  day, month;
    uint16_t year;

    // Status
    bool    valid;          // true when fix is active and data is fresh
    bool    hasFix;         // true once first fix ever acquired
    uint32_t lastFixMs;     // millis() of last valid sentence
    uint32_t sentenceCount; // total valid sentences parsed
};

// ─────────────────────────────────────────────────────────────
//  GPSSensor class
// ─────────────────────────────────────────────────────────────
class GPSSensor {
public:
    explicit GPSSensor(HardwareSerial& serial = Serial1);

    /**
     * begin() — initialise UART1 on the GPS pins.
     * Call once in setup().
     */
    void begin(int rxPin = GPS_RX_PIN,
               int txPin = GPS_TX_PIN,
               uint32_t baud = GPS_BAUD);

    /**
     * update() — drain UART buffer and parse NMEA.
     * Call frequently from a dedicated FreeRTOS task.
     */
    void update();

    /**
     * get() — thread-safe copy of the latest GPS data.
     */
    GPSData get() const;

    /**
     * isValid() — true if fix is active and fresh (< GPS_TIMEOUT_MS old).
     */
    bool isValid() const;

    /**
     * hasFix() — true once the module has ever acquired a fix.
     */
    bool hasFix() const;

    /**
     * latStr() / lonStr() — formatted strings for display / logging.
     * e.g. "43.6532°N"  "-79.3832°E"
     */
    String latStr() const;
    String lonStr() const;

    /** Debug: print last raw NMEA sentence to Serial */
    void printRaw() const;

    /** Debug: print parsed GPSData to Serial */
    void printData() const;

private:
    HardwareSerial& _serial;

    // NMEA parse buffer
    char     _buf[GPS_NMEA_MAX_LEN];
    uint8_t  _bufIdx;
    char     _lastRaw[GPS_NMEA_MAX_LEN];

    // Live data (protected by mutex)
    GPSData  _data;
    mutable SemaphoreHandle_t _mutex;

    // ── NMEA parsing ────────────────────────────────────────
    bool _parseSentence(const char* sentence);
    bool _parseGPRMC(const char* sentence);
    bool _parseGPGGA(const char* sentence);

    bool _validateChecksum(const char* sentence) const;
    const char* _nextField(const char* p) const;

    double  _parseLatLon(const char* val, const char* dir) const;
    float   _parseFloat(const char* p) const;
    int     _parseInt(const char* p) const;
    void    _parseTime(const char* p, uint8_t& h, uint8_t& m, uint8_t& s) const;
    void    _parseDate(const char* p, uint8_t& d, uint8_t& mo, uint16_t& yr) const;
};

extern GPSSensor gps;

#endif // GPS_SENSOR_H
