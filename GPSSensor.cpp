/**
 * ============================================================
 *  GPSSensor.cpp — GY-GPS6MV2 (u-blox NEO-6M) NMEA Parser
 * ============================================================
 */

#include "GPSSensor.h"
#include "DebugConfig.h"

// ─────────────────────────────────────────────────────────────
//  Singleton
// ─────────────────────────────────────────────────────────────
GPSSensor gps(Serial1);

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────
GPSSensor::GPSSensor(HardwareSerial& serial)
    : _serial(serial), _bufIdx(0)
{
    memset(&_data, 0, sizeof(_data));
    memset(_buf,     0, sizeof(_buf));
    memset(_lastRaw, 0, sizeof(_lastRaw));
    _mutex = xSemaphoreCreateMutex();
}

// ─────────────────────────────────────────────────────────────
//  begin()
// ─────────────────────────────────────────────────────────────
void GPSSensor::begin(int rxPin, int txPin, uint32_t baud)
{
    _serial.begin(baud, SERIAL_8N1, rxPin, txPin);
    DBG_PRINTF("[GPS] UART1 init — RX=GPIO%d  TX=GPIO%d  %lu baud\n",
                  rxPin, txPin, (unsigned long)baud);
    DBG_PRINTLN(F("[GPS] Waiting for NMEA sentences..."));
    DBG_PRINTLN(F("[GPS] Cold fix typically takes 30–90 s outdoors."));
}

// ─────────────────────────────────────────────────────────────
//  update() — drain UART and parse complete sentences
// ─────────────────────────────────────────────────────────────
void GPSSensor::update()
{
    while (_serial.available()) {
        char c = (char)_serial.read();

        if (c == '$') {
            // Start of new sentence — reset buffer
            _bufIdx = 0;
            _buf[_bufIdx++] = c;
        } else if (c == '\n' || c == '\r') {
            if (_bufIdx > 6) {
                _buf[_bufIdx] = '\0';
                strncpy(_lastRaw, _buf, GPS_NMEA_MAX_LEN - 1);
                _parseSentence(_buf);
            }
            _bufIdx = 0;
        } else if (_bufIdx < GPS_NMEA_MAX_LEN - 1) {
            _buf[_bufIdx++] = c;
        } else {
            // Overflow — discard and reset
            _bufIdx = 0;
        }
    }

    // Mark stale if no fix for GPS_TIMEOUT_MS
    if (_data.valid) {
        if ((millis() - _data.lastFixMs) > GPS_TIMEOUT_MS) {
            if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                _data.valid = false;
                xSemaphoreGive(_mutex);
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  _parseSentence() — dispatch to correct parser
// ─────────────────────────────────────────────────────────────
bool GPSSensor::_parseSentence(const char* sentence)
{
    if (!_validateChecksum(sentence)) return false;

    if (strncmp(sentence, "$GPRMC", 6) == 0 ||
        strncmp(sentence, "$GNRMC", 6) == 0) {
        return _parseGPRMC(sentence);
    }
    if (strncmp(sentence, "$GPGGA", 6) == 0 ||
        strncmp(sentence, "$GNGGA", 6) == 0) {
        return _parseGPGGA(sentence);
    }
    return false;
}

// ─────────────────────────────────────────────────────────────
//  _parseGPRMC() — Recommended Minimum Specific GPS/Transit Data
//
//  $GPRMC,HHMMSS.ss,A,LLLL.LL,a,YYYYY.YY,a,x.x,x.x,DDMMYY,,,*hh
//  Fields:
//   0  $GPRMC
//   1  UTC time HHMMSS.ss
//   2  Status  A=active V=void
//   3  Latitude DDMM.MMMMM
//   4  N or S
//   5  Longitude DDDMM.MMMMM
//   6  E or W
//   7  Speed over ground (knots)
//   8  Course over ground (degrees true)
//   9  Date DDMMYY
//  10  Magnetic variation
//  11  E or W
// ─────────────────────────────────────────────────────────────
bool GPSSensor::_parseGPRMC(const char* sentence)
{
    char buf[GPS_NMEA_MAX_LEN];
    strncpy(buf, sentence, GPS_NMEA_MAX_LEN - 1);
    buf[GPS_NMEA_MAX_LEN - 1] = '\0';

    char* fields[14];
    int   n = 0;
    char* p = buf;

    // Tokenise by comma — careful: strtok modifies buf
    char* tok = strtok(p, ",");
    while (tok && n < 14) {
        fields[n++] = tok;
        tok = strtok(nullptr, ",*");
    }
    if (n < 10) return false;

    // Field 2: status
    bool active = (fields[2][0] == 'A');

    if (!active) {
        // Void — no fix, but still update time if present
        if (n > 1 && strlen(fields[1]) >= 6) {
            uint8_t h, m, s;
            _parseTime(fields[1], h, m, s);
            if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                _data.hour = h; _data.minute = m; _data.second = s;
                _data.valid = false;
                xSemaphoreGive(_mutex);
            }
        }
        return false;
    }

    // Parse fields
    uint8_t h = 0, mi = 0, sec = 0;
    uint8_t day = 0, mon = 0;
    uint16_t yr = 0;

    if (strlen(fields[1]) >= 6) _parseTime(fields[1], h, mi, sec);
    if (strlen(fields[9]) >= 6) _parseDate(fields[9], day, mon, yr);

    double lat = _parseLatLon(fields[3], fields[4]);
    double lon = _parseLatLon(fields[5], fields[6]);
    float  spd = _parseFloat(fields[7]);  // knots
    float  crs = _parseFloat(fields[8]);

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        _data.latitude    = lat;
        _data.longitude   = lon;
        _data.speed_knots = spd;
        _data.speed_kmh   = spd * 1.852f;
        _data.course_deg  = crs;
        _data.hour    = h;  _data.minute = mi; _data.second = sec;
        _data.day     = day; _data.month = mon; _data.year   = yr;
        _data.valid       = true;
        _data.hasFix      = true;
        _data.lastFixMs   = millis();
        _data.sentenceCount++;
        xSemaphoreGive(_mutex);
    }
    return true;
}

// ─────────────────────────────────────────────────────────────
//  _parseGPGGA() — Global Positioning System Fix Data
//
//  $GPGGA,HHMMSS.ss,LLLL.LL,a,YYYYY.YY,a,x,xx,x.x,x.x,M,x.x,M,,*hh
//  Fields:
//   0  $GPGGA
//   1  UTC time
//   2  Latitude
//   3  N or S
//   4  Longitude
//   5  E or W
//   6  Fix quality: 0=no fix,1=GPS,2=DGPS
//   7  Satellites in use
//   8  HDOP
//   9  Altitude (MSL)
//  10  M (metres)
//  11  Geoid separation
//  12  M
// ─────────────────────────────────────────────────────────────
bool GPSSensor::_parseGPGGA(const char* sentence)
{
    char buf[GPS_NMEA_MAX_LEN];
    strncpy(buf, sentence, GPS_NMEA_MAX_LEN - 1);
    buf[GPS_NMEA_MAX_LEN - 1] = '\0';

    char* fields[16];
    int   n = 0;
    char* tok = strtok(buf, ",");
    while (tok && n < 16) {
        fields[n++] = tok;
        tok = strtok(nullptr, ",*");
    }
    if (n < 10) return false;

    uint8_t quality = (uint8_t)_parseInt(fields[6]);
    if (quality == 0) return false;  // no fix

    uint8_t  sats = (uint8_t)_parseInt(fields[7]);
    float    hdop = _parseFloat(fields[8]);
    float    alt  = _parseFloat(fields[9]);
    float    geoid = (n > 11) ? _parseFloat(fields[11]) : 0.0f;

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        _data.fix_quality = quality;
        _data.satellites  = sats;
        _data.hdop        = hdop;
        _data.altitude_m  = alt;
        _data.geoid_sep_m = geoid;
        xSemaphoreGive(_mutex);
    }
    return true;
}

// ─────────────────────────────────────────────────────────────
//  _validateChecksum()
//  NMEA checksum: XOR of all bytes between '$' and '*'
// ─────────────────────────────────────────────────────────────
bool GPSSensor::_validateChecksum(const char* sentence) const
{
    const char* p = sentence + 1;  // skip '$'
    uint8_t calc = 0;
    while (*p && *p != '*') calc ^= (uint8_t)*p++;
    if (*p != '*') return false;
    p++;  // skip '*'

    // Parse two hex digits
    char hexStr[3] = {p[0], p[1], '\0'};
    uint8_t rxCsum = (uint8_t)strtol(hexStr, nullptr, 16);
    return calc == rxCsum;
}

// ─────────────────────────────────────────────────────────────
//  _parseLatLon()
//  NMEA format: DDMM.MMMMM or DDDMM.MMMMM
//  Returns decimal degrees.
// ─────────────────────────────────────────────────────────────
double GPSSensor::_parseLatLon(const char* val, const char* dir) const
{
    if (!val || strlen(val) < 4) return 0.0;

    double raw = atof(val);
    int    deg = (int)(raw / 100.0);
    double min = raw - (deg * 100.0);
    double dd  = deg + min / 60.0;

    if (dir[0] == 'S' || dir[0] == 'W') dd = -dd;
    return dd;
}

float GPSSensor::_parseFloat(const char* p) const
{
    if (!p || strlen(p) == 0) return 0.0f;
    return (float)atof(p);
}

int GPSSensor::_parseInt(const char* p) const
{
    if (!p || strlen(p) == 0) return 0;
    return atoi(p);
}

void GPSSensor::_parseTime(const char* p, uint8_t& h, uint8_t& m, uint8_t& s) const
{
    if (!p || strlen(p) < 6) return;
    char tmp[3] = {0};
    tmp[0]=p[0]; tmp[1]=p[1]; h = (uint8_t)atoi(tmp);
    tmp[0]=p[2]; tmp[1]=p[3]; m = (uint8_t)atoi(tmp);
    tmp[0]=p[4]; tmp[1]=p[5]; s = (uint8_t)atoi(tmp);
}

void GPSSensor::_parseDate(const char* p, uint8_t& d, uint8_t& mo, uint16_t& yr) const
{
    if (!p || strlen(p) < 6) return;
    char tmp[3] = {0};
    tmp[0]=p[0]; tmp[1]=p[1]; d  = (uint8_t)atoi(tmp);
    tmp[0]=p[2]; tmp[1]=p[3]; mo = (uint8_t)atoi(tmp);
    tmp[0]=p[4]; tmp[1]=p[5]; yr = 2000 + (uint16_t)atoi(tmp);
}

// ─────────────────────────────────────────────────────────────
//  Public accessors
// ─────────────────────────────────────────────────────────────
GPSData GPSSensor::get() const
{
    GPSData d;
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        d = _data;
        xSemaphoreGive(_mutex);
    } else {
        memset(&d, 0, sizeof(d));
    }
    return d;
}

bool GPSSensor::isValid() const
{
    bool v = false;
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        v = _data.valid;
        xSemaphoreGive(_mutex);
    }
    return v;
}

bool GPSSensor::hasFix() const
{
    bool v = false;
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        v = _data.hasFix;
        xSemaphoreGive(_mutex);
    }
    return v;
}

String GPSSensor::latStr() const
{
    GPSData d = get();
    char buf[24];
    snprintf(buf, sizeof(buf), "%.6f°%c", fabs(d.latitude), d.latitude >= 0 ? 'N' : 'S');
    return String(buf);
}

String GPSSensor::lonStr() const
{
    GPSData d = get();
    char buf[24];
    snprintf(buf, sizeof(buf), "%.6f°%c", fabs(d.longitude), d.longitude >= 0 ? 'E' : 'W');
    return String(buf);
}

void GPSSensor::printRaw() const
{
    DBG_PRINTF("[GPS RAW] %s\n", _lastRaw);
}

void GPSSensor::printData() const
{
    GPSData d = get();
    DBG_PRINTF("[GPS] Fix=%s  Sats=%d  HDOP=%.1f  Quality=%d\n",
                  d.valid ? "YES" : "NO", d.satellites, d.hdop, d.fix_quality);
    if (d.valid) {
        DBG_PRINTF("[GPS] Lat=%.6f  Lon=%.6f  Alt=%.1fm\n",
                      d.latitude, d.longitude, d.altitude_m);
        DBG_PRINTF("[GPS] Speed=%.1f km/h  Course=%.1f°\n",
                      d.speed_kmh, d.course_deg);
        DBG_PRINTF("[GPS] UTC %02d:%02d:%02d  %02d/%02d/%04d\n",
                      d.hour, d.minute, d.second, d.day, d.month, d.year);
    }
}
