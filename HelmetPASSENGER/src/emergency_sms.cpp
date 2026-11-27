#include "emergency_sms.h"
#include "pins.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// =============================================================================
// Fixed hardware wiring for the LilyGO T-SIM7600G-H R2 v1.2 modem.
// Confirmed against LilyGO's own LILYGO_SIM7600X board definition (utilities.h,
// LilyGo-Modem-Series / T-SIM7600X repos).
//
// GPS is NO LONGER sourced from the SIM7600's integrated GNSS. It comes from
// an external NEO-M8N module wired to a SoftwareSerial port (see pins.h) and
// parsed with TinyGPSPlus. GPS is therefore fully decoupled from the modem's
// AT command link - the two no longer share s_at_mutex or contend for the
// same UART at all. The SIM7600's own GNSS engine is explicitly disabled
// once at boot (see disable_internal_gnss()) since it's unused and would
// otherwise just burn power in the background.
// =============================================================================

#define SerialAT   Serial1

// Destination number. "09392187430" converted to international format for
// reliability - edit back to local format if your SIM/carrier prefers that.
#define EMERGENCY_SMS_NUMBER   "+639603721899"

#define EMERGENCY_CLICK_COUNT           5      // number of fast clicks required to "arm"
#define EMERGENCY_CLICK_WINDOW_MS       3000   // all N clicks must land inside this window
#define EMERGENCY_CLICK_MAX_GAP_MS       600   // gap between consecutive clicks longer than this resets the count
#define EMERGENCY_CONFIRM_WINDOW_MS    10000   // time allowed for the confirm click after arming
#define EMERGENCY_CONFIRM_LOCKOUT_MS     400   // ignore clicks for this long right after arming, so a trailing
                                                // arming click can't double as an accidental instant confirm
#define EMERGENCY_COOLDOWN_MS          30000   // min gap between two full send sequences
#define MODEM_BOOT_TIMEOUT_MS          20000   // waiting for first "AT" -> "OK" after power-on
#define NETWORK_REG_TIMEOUT_MS         60000   // waiting for AT+CREG? to show registered

#define EMERGENCY_UNIT_LABEL_ALREADY_DEFINED_CHECK 1 // (EMERGENCY_UNIT_LABEL comes from pins.h - "DRIVER")

// Gap enforced between the first ("help needed") and second ("coordinates")
// SMS in a single emergency sequence. Some carriers/SMSCs throttle or
// silently drop a second message sent to the same recipient within ~1s of
// the first. Cheap insurance against that - the first message has already
// gone out, so a few seconds of extra latency on the second doesn't cost
// anything that matters.
#define EMERGENCY_INTER_SMS_DELAY_MS    3000

// How long the emergency sequence will wait for a FRESH fix (one that lands
// after the sequence starts) before giving up and falling back to whatever's
// in the cache. The NEO-M8N runs continuously from boot (see gps_read_task),
// so by the time an emergency actually happens it's typically already
// locked - this timeout mostly covers the case where the module is still
// doing a genuine cold TTFF (up to ~30s typical) right as the trigger fires.
#define GPS_FIX_TIMEOUT_MS         40000

// How often the Serial heartbeat below prints GPS status, so you can watch
// fix acquisition happen in real time without needing to trigger an
// emergency. Just reports whatever the cache currently holds, on its own
// timer - independent of the GPS read task itself.
#define GPS_STATUS_LOG_MS          10000

// How often the detailed GNSS debug (satellite count + HDOP + parser health)
// is logged.
#define GPS_DETAIL_LOG_MS         10000

// How often the GSM/GPS "connection health" debug check runs. This is
// separate from the passive status logs above - it actively pokes the
// modem with a plain "AT" and "AT+CSQ" to confirm the UART link is alive
// and see signal strength, and reports whether the GPS module is producing
// NMEA data at all. Meant purely for bring-up / field debugging so you can
// tell "modem UART dead" apart from "modem alive but no signal" apart from
// "GPS never wired up" at a glance in the Serial log.
#define DEBUG_STATUS_LOG_MS        15000

// -----------------------------------------------------------------------------
// GNSS staleness - purely informational now. There's no "session" to cycle
// on a plain NMEA module the way there was with the SIM7600's AT+CGPS - if
// the NEO-M8N stops producing fixes it's a wiring/power/antenna/sky-view
// issue, not something a software restart can fix. This just logs a loud
// warning so it's obvious from the Serial log rather than silently going
// stale.
// -----------------------------------------------------------------------------
#define GPS_STALE_WARN_MS          300000  // 5m with no fix -> log a warning

static SemaphoreHandle_t s_trigger_sem = nullptr;
static volatile bool     s_busy        = false;
static volatile bool     s_gps_running = false; // true once the NEO-M8N has produced any NMEA data at all
static volatile uint8_t  s_gps_status  = 0;

static HardwareSerial* s_uart = nullptr;
static volatile uint8_t  s_network_status = 0;

// -----------------------------------------------------------------------------
// Modem UART serialization.
// -----------------------------------------------------------------------------
static SemaphoreHandle_t s_at_mutex = nullptr;

// -----------------------------------------------------------------------------
// NEO-M8N GPS module
// -----------------------------------------------------------------------------
static SoftwareSerial GPSSerial(GPS_RX_PIN, GPS_TX_PIN);
static TinyGPSPlus     gps;

// UTC timestamp of a GPS fix, taken directly from the module's own NMEA
// time/date fields (not computed from millis() - the module already knows
// real UTC time from the satellites, so use that instead of guessing age).
// Mirrors the struct used on the passenger board so both units' SMS wording
// stays consistent.
struct GpsTimestamp
{
    uint16_t year   = 0;
    uint8_t  month  = 0;
    uint8_t  day    = 0;
    uint8_t  hour   = 0;
    uint8_t  minute = 0;
    uint8_t  second = 0;
    bool     valid  = false;
};

// -----------------------------------------------------------------------------
// Last-known-good GPS fix cache. Populated continuously by gps_read_task as
// new NMEA sentences with a valid fix come in. Read by run_emergency_sequence()
// (via wait_for_fresh_fix()) and by log_gps_status() purely for visibility.
// -----------------------------------------------------------------------------
static SemaphoreHandle_t s_gps_cache_mutex = nullptr;
static double            s_last_fix_lat    = NAN;
static double            s_last_fix_lon    = NAN;
static GpsTimestamp      s_last_fix_ts;              // UTC time/date of that fix, from the module itself
static uint32_t          s_last_fix_ms     = 0;       // millis() timestamp, used only for cache "age" and staleness checks
static bool              s_last_fix_valid  = false;

static void cache_gps_fix(double lat, double lon, const GpsTimestamp& ts)
{
    if (!s_gps_cache_mutex) return;
    if (xSemaphoreTake(s_gps_cache_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bool was_valid   = s_last_fix_valid;
        s_last_fix_lat   = lat;
        s_last_fix_lon   = lon;
        s_last_fix_ts    = ts;
        s_last_fix_ms    = millis();
        s_last_fix_valid = true;
        xSemaphoreGive(s_gps_cache_mutex);

        // Only shout on the false->true transition (first fix ever, or
        // regaining one after a gap) - otherwise this would print on every
        // NMEA sentence and drown out everything else in Serial. Ongoing
        // status is covered by log_gps_status() instead.
        if (!was_valid) {
            Serial.printf("[SOS][GPS] Fix ACQUIRED: %.6f, %.6f\n", lat, lon);
        }
    }
}

// Returns true and fills lat/lon/fix_ts/age_ms if a cached fix exists.
static bool get_cached_gps_fix(double& lat, double& lon, GpsTimestamp& fix_ts, uint32_t& age_ms)
{
    if (!s_gps_cache_mutex) return false;
    bool valid = false;
    if (xSemaphoreTake(s_gps_cache_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        valid = s_last_fix_valid;
        if (valid) {
            lat    = s_last_fix_lat;
            lon    = s_last_fix_lon;
            fix_ts = s_last_fix_ts;
            age_ms = millis() - s_last_fix_ms;
        }
        xSemaphoreGive(s_gps_cache_mutex);
    }
    return valid;
}

bool emergency_sms_get_gps_location(double& lat, double& lon, uint32_t& age_ms)
{
    GpsTimestamp fix_ts;
    return get_cached_gps_fix(lat, lon, fix_ts, age_ms);
}

// Age of the last successful fix in ms, or UINT32_MAX if there's never been
// one at all. Used by the staleness warning in log_gnss_detail().
static uint32_t last_fix_age_ms()
{
    if (!s_gps_cache_mutex) return UINT32_MAX;
    uint32_t age = UINT32_MAX;
    if (xSemaphoreTake(s_gps_cache_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (s_last_fix_valid) age = millis() - s_last_fix_ms;
        xSemaphoreGive(s_gps_cache_mutex);
    }
    return age;
}

// Raw millis() timestamp (not age) of the last cached fix, or 0 if none yet.
// Used by wait_for_fresh_fix() to detect "a NEW fix landed since I started
// waiting" as opposed to just re-reading whatever was already in the cache.
static uint32_t last_fix_timestamp()
{
    if (!s_gps_cache_mutex) return 0;
    uint32_t ts = 0;
    if (xSemaphoreTake(s_gps_cache_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (s_last_fix_valid) ts = s_last_fix_ms;
        xSemaphoreGive(s_gps_cache_mutex);
    }
    return ts;
}

// Formats a GpsTimestamp as "HH:MM:SS UTC DD/MM/YYYY", or "time unknown" if
// the module never got a valid date/time alongside the location fix.
static void format_fix_time(const GpsTimestamp& ts, char* out, size_t out_size)
{
    if (ts.valid) {
        snprintf(out, out_size, "%02u:%02u:%02u UTC %02u/%02u/%04u",
                 ts.hour, ts.minute, ts.second, ts.day, ts.month, ts.year);
    } else {
        snprintf(out, out_size, "time unknown");
    }
}

// Periodic Serial heartbeat so GPS fix status is visible continuously,
// without needing to trigger an emergency to find out. Reports whatever the
// cache currently holds; doesn't itself touch the GPS UART.
static void log_gps_status()
{
    static uint32_t last_ms = 0;
    uint32_t now = millis();
    if (now - last_ms < GPS_STATUS_LOG_MS) return;
    last_ms = now;

    double       lat = NAN, lon = NAN;
    GpsTimestamp fix_ts;
    uint32_t     age_ms = 0;
    bool         have_fix = get_cached_gps_fix(lat, lon, fix_ts, age_ms);

    if (have_fix) {
        s_gps_status = 2;
        char time_label[24];
        format_fix_time(fix_ts, time_label, sizeof(time_label));
        Serial.printf("[SOS][GPS] status: VALID fix, age=%lus, lat=%.6f, lon=%.6f, acquired=%s, module_talking=%d\n",
                      (unsigned long)(age_ms / 1000), lat, lon, time_label, (int)s_gps_running);
    } else {
        s_gps_status = s_gps_running ? 1 : 0;
        Serial.printf("[SOS][GPS] status: NO FIX YET, module_talking=%d\n", (int)s_gps_running);
    }
}

// Timed the same way as log_gps_status() - reports on its own cadence.
// Gives satellite count + HDOP + parser health (chars processed / failed
// checksum), which is what tells you "module's alive and tracking, just not
// converged yet" apart from "wiring/power problem, nothing's coming through
// at all."
static void log_gnss_detail()
{
    static uint32_t last_ms = 0;
    uint32_t now = millis();
    if (now - last_ms < GPS_DETAIL_LOG_MS) return;
    last_ms = now;

    if (gps.charsProcessed() < 10) {
        Serial.println("[SOS][GNSS] no NMEA data received yet - check NEO-M8N wiring/power/baud (GPS_RX_PIN/GPS_TX_PIN in pins.h)");
        return;
    }

    int    sats = gps.satellites.isValid() ? (int)gps.satellites.value() : -1;
    double hdop = gps.hdop.isValid() ? gps.hdop.hdop() : NAN;

    Serial.printf("[SOS][GNSS] sats=%d HDOP=%.1f fix=%s | chars=%lu failedChecksum=%lu\n",
                  sats, hdop, gps.location.isValid() ? "YES" : "NO",
                  (unsigned long)gps.charsProcessed(), (unsigned long)gps.failedChecksum());

    uint32_t age = last_fix_age_ms();
    if (age != UINT32_MAX && age >= GPS_STALE_WARN_MS) {
        Serial.printf("[SOS][GNSS] WARNING: no fix in %lus - check antenna/sky view if this persists\n",
                      (unsigned long)(age / 1000));
    }
}

// Background task: continuously drains the NEO-M8N's SoftwareSerial output
// into TinyGPSPlus and caches any valid fix as soon as it arrives, along
// with the module's own UTC date/time for that fix (falls back to "time
// unknown" if TinyGPS++ doesn't have a valid date/time alongside the
// location yet - same handling as the passenger board).
static void gps_read_task(void* /*param*/)
{
    for (;;) {
        while (GPSSerial.available()) {
            char c = (char)GPSSerial.read();
            if (!s_gps_running && gps.charsProcessed() > 10) {
                s_gps_running = true;
                Serial.println("[SOS][GPS] NEO-M8N is talking (NMEA data received)");
            }
            if (gps.encode(c)) {
                if (gps.location.isValid() && gps.location.isUpdated()) {
                    GpsTimestamp ts;
                    if (gps.date.isValid() && gps.time.isValid()) {
                        ts.year   = gps.date.year();
                        ts.month  = gps.date.month();
                        ts.day    = gps.date.day();
                        ts.hour   = gps.time.hour();
                        ts.minute = gps.time.minute();
                        ts.second = gps.time.second();
                        ts.valid  = true;
                    }
                    cache_gps_fix(gps.location.lat(), gps.location.lng(), ts);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Waits up to timeout_ms for a NEW fix to land - i.e. one whose cache
// timestamp differs from whatever was already cached when this was called -
// rather than immediately returning a fix that might be stale from well
// before the emergency was triggered. If nothing new lands in time, the
// caller falls back to get_cached_gps_fix() directly for whatever's on
// hand, however old.
static bool wait_for_fresh_fix(double& lat, double& lon, GpsTimestamp& fix_ts, uint32_t timeout_ms)
{
    uint32_t baseline_ts = last_fix_timestamp();
    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        uint32_t ts = last_fix_timestamp();
        if (ts != 0 && ts != baseline_ts) {
            uint32_t age_ms;
            if (get_cached_gps_fix(lat, lon, fix_ts, age_ms)) return true;
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    return false;
}

// -----------------------------------------------------------------------------
// Low-level AT helpers (SMS / network only - GPS no longer touches these)
// -----------------------------------------------------------------------------

static String modem_read_response(uint32_t timeout_ms)
{
    String resp;
    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        while (SerialAT.available()) {
            resp += (char)SerialAT.read();
        }
        if (resp.indexOf("OK") >= 0 || resp.indexOf("ERROR") >= 0) {
            delay(20);
            while (SerialAT.available()) resp += (char)SerialAT.read();
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return resp;
}

static String modem_send_at(const char* cmd, uint32_t timeout_ms = 2000)
{
    if (s_at_mutex) xSemaphoreTakeRecursive(s_at_mutex, portMAX_DELAY);

    while (SerialAT.available()) SerialAT.read();
    SerialAT.println(cmd);
    String resp = modem_read_response(timeout_ms);

    if (s_at_mutex) xSemaphoreGiveRecursive(s_at_mutex);
    return resp;
}

static bool modem_at_ok(const char* cmd, uint32_t timeout_ms = 2000)
{
    String r = modem_send_at(cmd, timeout_ms);
    bool ok = (r.indexOf("OK") >= 0) && (r.indexOf("ERROR") < 0);
    if (!ok) {
        Serial.printf("[SOS] AT command failed: %s -> [%s]\n", cmd, r.c_str());
    }
    return ok;
}

// -----------------------------------------------------------------------------
// Internal SIM7600 GNSS shutdown (one-shot, at boot)
//
// The SIM7600 has its own onboard GNSS engine, completely separate from and
// unused by this codebase now that GPS comes from the external NEO-M8N (see
// header comment at the top of this file). Left enabled it just burns extra
// power in the background, and on some SIM7600 firmware builds an active
// internal GNSS session has been reported to interact oddly with AT+CFUN /
// network behavior. AT+CGPS=0 stops that session explicitly.
//
// This is fire-and-forget: it doesn't gate anything else in the boot
// sequence (network registration proceeds regardless of whether this
// succeeds), and it's only ever sent once per boot - see the gnss_off_sent
// latch in modem_boot_task().
// -----------------------------------------------------------------------------
static void disable_internal_gnss()
{
    // Some SIM7600 firmware takes a moment longer than a typical AT command
    // if a GNSS session happened to already be running (e.g. left on from a
    // prior firmware/config), so give this a slightly longer timeout than
    // the 2000ms default.
    if (modem_at_ok("AT+CGPS=0", 5000)) {
        Serial.println("[SOS] Internal SIM7600 GNSS disabled (AT+CGPS=0) - GPS comes from external NEO-M8N only");
    } else {
        Serial.println("[SOS] WARNING: AT+CGPS=0 failed - internal SIM7600 GNSS may still be running "
                        "(harmless functionally, since GPS reads always come from the NEO-M8N, but wastes power)");
    }
}

// -----------------------------------------------------------------------------
// SMS delivery reports (+CDS URCs)
// -----------------------------------------------------------------------------

static void configure_delivery_reports()
{
    if (!modem_at_ok("AT+CNMI=2,1,0,2,0", 1000)) {
        Serial.println("[SOS] Failed to enable delivery-report URCs (AT+CNMI) - "
                        "delivery confirmation won't be available, only send-time ACK");
    }
    if (!modem_at_ok("AT+CSMP=17,167,0,0", 1000)) {
        Serial.println("[SOS] Failed to set AT+CSMP status-report flag - "
                        "delivery confirmation won't be available, only send-time ACK");
    }
}

static void poll_delivery_reports()
{
    if (!s_at_mutex) return;
    if (xSemaphoreTakeRecursive(s_at_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;

    static String buf;
    while (SerialAT.available()) {
        buf += (char)SerialAT.read();
    }
    if (buf.length() > 512) {
        buf.remove(0, buf.length() - 512);
    }

    int idx = buf.indexOf("+CDS:");
    if (idx >= 0) {
        int end = buf.indexOf('\n', idx);
        String line = (end >= 0) ? buf.substring(idx, end) : buf.substring(idx);
        line.trim();
        Serial.printf("[SOS][SMS] Delivery report: %s\n", line.c_str());
        buf = (end >= 0) ? buf.substring(end + 1) : "";
    }

    xSemaphoreGiveRecursive(s_at_mutex);
}

// -----------------------------------------------------------------------------
// Power-on / network
// -----------------------------------------------------------------------------

static void modem_pwrkey_pulse()
{
    digitalWrite(MODEM_PWRKEY_PIN, HIGH);
    delay(300);
    digitalWrite(MODEM_PWRKEY_PIN, LOW);
}

static bool ensure_modem_ready()
{
    if (modem_send_at("AT", 1000).indexOf("OK") >= 0) return true;

    Serial.println("[SOS] Modem not responding - power-on sequence...");
    modem_pwrkey_pulse();

    uint32_t start = millis();
    while (millis() - start < MODEM_BOOT_TIMEOUT_MS) {
        if (modem_send_at("AT", 1000).indexOf("OK") >= 0) {
            modem_send_at("ATE0", 500);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    return false;
}

static bool wait_for_network(uint32_t timeout_ms)
{
    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        String r = modem_send_at("AT+CREG?", 2000);
        if (r.indexOf(",1") >= 0 || r.indexOf(",5") >= 0) return true;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    return false;
}

// -----------------------------------------------------------------------------
// DTR-based power-save sleep/wake.
//
// The modem stays REGISTERED on the network while asleep - this only idles
// its UART interface (AT+CSCLK=1 lets the module drop into its low-power
// state whenever DTR is held HIGH; pulling DTR LOW again wakes the UART
// without needing to re-register or re-authenticate with the SIM).
//
// s_modem_sleeping is purely a local bookkeeping flag so callers (namely
// log_debug_status()) know not to poke the UART with AT commands while the
// modem is expected to be asleep - it does not reflect actual hardware
// state if something external changed DTR out from under us.
// -----------------------------------------------------------------------------
static volatile bool s_modem_sleeping = false;

// Enables DTR-controlled sleep on the module (one-time, after boot/registration)
// and immediately puts it to sleep via DTR HIGH.
static void modem_enter_sleep()
{
    if (s_modem_sleeping) return;

    if (!modem_at_ok("AT+CSCLK=1", 2000)) {
        Serial.println("[SOS] WARNING: AT+CSCLK=1 failed - modem may not support/accept "
                        "DTR sleep, leaving it awake");
        return;
    }

    delay(50); // let the command settle before dropping DTR
    digitalWrite(MODEM_DTR_PIN, HIGH);
    s_modem_sleeping = true;
    Serial.println("[SOS] Modem asleep (DTR HIGH, AT+CSCLK=1) - still registered on network");
}

// Wakes the modem's UART back up (DTR LOW) and confirms it's responding to
// AT again before returning. Safe to call even if the modem wasn't
// actually asleep (e.g. AT+CSCLK=1 was never accepted).
static bool modem_wake_and_confirm(uint32_t timeout_ms)
{
    digitalWrite(MODEM_DTR_PIN, LOW);
    s_modem_sleeping = false;

    // Give the UART a moment to come back, then confirm with AT - some
    // SIM7600 firmware needs a throwaway character/command to fully wake
    // before it starts replying normally.
    delay(50);
    modem_send_at("AT", 300); // throwaway, ignore result

    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        if (modem_send_at("AT", 1000).indexOf("OK") >= 0) return true;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    return false;
}

// -----------------------------------------------------------------------------
// SMS
// -----------------------------------------------------------------------------

static bool send_sms(const char* number, const String& text)
{
    if (s_at_mutex) xSemaphoreTakeRecursive(s_at_mutex, portMAX_DELAY);

    bool ok = false;
    modem_send_at("AT+CMGF=1", 1000);

    while (SerialAT.available()) SerialAT.read();
    SerialAT.print("AT+CMGS=\"");
    SerialAT.print(number);
    SerialAT.println("\"");

    String r;
    uint32_t start = millis();
    while (millis() - start < 3000) {
        while (SerialAT.available()) r += (char)SerialAT.read();
        if (r.indexOf('>') >= 0) break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (r.indexOf('>') < 0) {
        Serial.println("[SOS] Modem never gave the SMS prompt");
        ok = false;
    } else {
        SerialAT.print(text);
        SerialAT.write(0x1A);

        String resp = modem_read_response(15000);

        ok = (resp.indexOf("+CMGS:") >= 0);
        if (!ok) {
            Serial.printf("[SOS] SMS send did not get +CMGS: confirmation -> [%s]\n", resp.c_str());
        }
    }

    if (s_at_mutex) xSemaphoreGiveRecursive(s_at_mutex);
    return ok;
}

static bool send_sms_with_retry(const char* number, const String& text, const char* label)
{
    bool sent = false;
    for (int attempt = 0; attempt < 2 && !sent; attempt++) {
        sent = send_sms(number, text);
        if (!sent) {
            Serial.printf("[SOS] %s SMS send failed, retrying...\n", label);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    Serial.printf("[SOS] %s SMS %s\n", label, sent ? "sent" : "FAILED after retries");
    return sent;
}

// -----------------------------------------------------------------------------
// Diagnostics
// -----------------------------------------------------------------------------

static void log_modem_diagnostics()
{
    Serial.println("[SOS] --- modem diagnostics ---");
    Serial.print("[SOS] AT+CPIN?  -> "); Serial.println(modem_send_at("AT+CPIN?", 2000));
    Serial.print("[SOS] AT+CFUN?  -> "); Serial.println(modem_send_at("AT+CFUN?", 2000));
    Serial.print("[SOS] AT+CSQ    -> "); Serial.println(modem_send_at("AT+CSQ", 2000));
    Serial.print("[SOS] AT+CREG?  -> "); Serial.println(modem_send_at("AT+CREG?", 2000));
    Serial.print("[SOS] AT+COPS?  -> "); Serial.println(modem_send_at("AT+COPS?", 2000));
    Serial.println("[SOS] --------------------------");
}

// -----------------------------------------------------------------------------
// Lightweight periodic connection-health debug check for GSM + GPS.
//
// This is intentionally cheap and non-blocking-by-design:
//  - Skips entirely while an emergency sequence is actively using the
//    modem (s_busy), so it never delays a real SMS send.
//  - Grabs s_at_mutex with a short timeout instead of portMAX_DELAY - if
//    something else holds it briefly, this cycle is just skipped rather
//    than stalling emergency_sms_process()'s caller.
//  - Only sends short, fast AT commands (AT, AT+CSQ, AT+CREG?) - no CPIN/
//    CFUN/COPS interrogation like log_modem_diagnostics(), which is
//    reserved for actual failure cases.
//
// Meant to answer, at a glance in the Serial log: "is the modem UART
// alive right now, does it have signal/network, and is the GPS module
// actually producing NMEA data" - the three most common bring-up
// failure points (modem wiring/baud, no SIM signal, GPS wiring/baud).
// -----------------------------------------------------------------------------
static void log_debug_status()
{
    static uint32_t last_ms = 0;
    uint32_t now = millis();
    if (now - last_ms < DEBUG_STATUS_LOG_MS) return;
    last_ms = now;

    Serial.println("[DEBUG] --- GSM / GPS connection check ---");

    // ---- GSM: quick AT ping + signal + registration ----
    if (s_busy) {
        Serial.println("[DEBUG][GSM] skipped - emergency sequence in progress, modem busy");
    } else if (s_modem_sleeping) {
        // Modem is intentionally in DTR sleep to save power. Waking it just
        // to answer a debug ping would defeat the point of sleeping in the
        // first place, so skip the active AT poke entirely - last known
        // network_status (set at boot / after the last emergency) still
        // holds. If you need live signal/registration visibility, wake the
        // modem deliberately (e.g. via a dedicated debug trigger) rather
        // than lowering DEBUG_STATUS_LOG_MS.
        Serial.println("[DEBUG][GSM] skipped - modem asleep (DTR power-save), not waking for a debug poll");
    } else if (!s_at_mutex || xSemaphoreTakeRecursive(s_at_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        Serial.println("[DEBUG][GSM] skipped - could not get modem UART lock this cycle");
    } else {
        while (SerialAT.available()) SerialAT.read();
        SerialAT.println("AT");
        String at_resp = modem_read_response(1000);
        bool at_ok = at_resp.indexOf("OK") >= 0;
        Serial.printf("[DEBUG][GSM] AT       -> %s\n", at_ok ? "OK (modem UART alive)" : "NO RESPONSE (check MODEM_RX_PIN/MODEM_TX_PIN wiring/baud, or modem power)");

        if (at_ok) {
            SerialAT.println("AT+CSQ");
            String csq_resp = modem_read_response(1000);
            csq_resp.trim();
            Serial.printf("[DEBUG][GSM] AT+CSQ   -> %s\n", csq_resp.c_str());

            SerialAT.println("AT+CREG?");
            String creg_resp = modem_read_response(1000);
            bool registered = (creg_resp.indexOf(",1") >= 0) || (creg_resp.indexOf(",5") >= 0);
            Serial.printf("[DEBUG][GSM] AT+CREG? -> %s (%s)\n", creg_resp.c_str(),
                          registered ? "REGISTERED on network" : "NOT registered yet");
        }

        xSemaphoreGiveRecursive(s_at_mutex);
    }

    // ---- GPS: is the NEO-M8N producing any data, and does it have a fix ----
    unsigned long chars = gps.charsProcessed();
    if (chars < 10) {
        Serial.println("[DEBUG][GPS] NO DATA - nothing received from NEO-M8N (check GPS_RX_PIN/GPS_TX_PIN wiring, baud, and module power)");
    } else {
        int    sats = gps.satellites.isValid() ? (int)gps.satellites.value() : -1;
        double hdop = gps.hdop.isValid() ? gps.hdop.hdop() : NAN;
        Serial.printf("[DEBUG][GPS] module_talking=1, chars=%lu, failedChecksum=%lu, sats=%d, HDOP=%.1f, fix=%s\n",
                      chars, (unsigned long)gps.failedChecksum(), sats, hdop,
                      gps.location.isValid() ? "YES" : "NO");
    }

    Serial.println("[DEBUG] -----------------------------------");
}

// -----------------------------------------------------------------------------
// Sequence orchestration
//
// SMS wording now matches the passenger board's style:
//   1st: "EMERGENCY - <UNIT> needs help. Getting GPS location..."
//   2nd (fresh or cached fix): "EMERGENCY - <UNIT> location: lat,lon (acquired HH:MM:SS UTC DD/MM/YYYY)"
//   2nd (no fix at all): "EMERGENCY - <UNIT> GPS unavailable"
// A cached (non-fresh) fix is called out as "last known" with its age, since
// unlike the passenger board's single wait, the driver board keeps a running
// background cache and can fall back to an older fix rather than reporting
// total unavailability.
// -----------------------------------------------------------------------------

static void run_emergency_sequence()
{
    s_busy = true;
    Serial.println("[SOS] Emergency confirmed - starting send sequence");

    Serial.println("[SOS] Waking modem from DTR sleep...");
    if (!modem_wake_and_confirm(MODEM_BOOT_TIMEOUT_MS)) {
        Serial.println("[SOS] Modem did not wake/respond via DTR - trying full power-on as fallback");
    }

    if (!ensure_modem_ready()) {
        Serial.println("[SOS] Modem did not respond at all - aborting");
        s_busy = false;
        return;
    }

    if (!wait_for_network(NETWORK_REG_TIMEOUT_MS)) {
        Serial.println("[SOS] Network registration not confirmed - trying SMS anyway");
        log_modem_diagnostics();
    }

    String msg1 = "EMERGENCY - " EMERGENCY_UNIT_LABEL " needs help. Getting GPS location...";
    Serial.printf("[SOS] First message: %s\n", msg1.c_str());
    send_sms_with_retry(EMERGENCY_SMS_NUMBER, msg1, "First (immediate)");

    vTaskDelay(pdMS_TO_TICKS(EMERGENCY_INTER_SMS_DELAY_MS));

    double       lat = NAN, lon = NAN;
    GpsTimestamp fix_ts;
    bool has_fix = wait_for_fresh_fix(lat, lon, fix_ts, GPS_FIX_TIMEOUT_MS);

    double       fallback_lat = NAN, fallback_lon = NAN;
    GpsTimestamp fallback_ts;
    uint32_t     fallback_age_ms = 0;
    bool         has_fallback = !has_fix && get_cached_gps_fix(fallback_lat, fallback_lon, fallback_ts, fallback_age_ms);

    if (has_fix) {
        char location[64];
        snprintf(location, sizeof(location), "%.6f,%.6f", lat, lon);

        char time_label[24];
        format_fix_time(fix_ts, time_label, sizeof(time_label));

        String msg2 = "EMERGENCY - " EMERGENCY_UNIT_LABEL " location: ";
        msg2 += location;
        msg2 += " (acquired ";
        msg2 += time_label;
        msg2 += ")";

        Serial.printf("[SOS] Second message: %s\n", msg2.c_str());
        send_sms_with_retry(EMERGENCY_SMS_NUMBER, msg2, "Second (location)");
    } else if (has_fallback) {
        char location[64];
        snprintf(location, sizeof(location), "%.6f,%.6f", fallback_lat, fallback_lon);

        char time_label[24];
        format_fix_time(fallback_ts, time_label, sizeof(time_label));

        String msg2 = "EMERGENCY - " EMERGENCY_UNIT_LABEL " last known location: ";
        msg2 += location;
        msg2 += " (acquired ";
        msg2 += time_label;
        msg2 += ", ";
        msg2 += String(fallback_age_ms / 1000);
        msg2 += "s ago, no fresh fix)";

        Serial.printf("[SOS] No fresh GPS fix - falling back to cached fix, age=%lus\n",
                      (unsigned long)(fallback_age_ms / 1000));
        Serial.printf("[SOS] Second message: %s\n", msg2.c_str());
        send_sms_with_retry(EMERGENCY_SMS_NUMBER, msg2, "Second (last-known location)");
    } else {
        String msg2 = "EMERGENCY - " EMERGENCY_UNIT_LABEL " GPS unavailable";
        Serial.println("[SOS] No GPS fix within timeout, and no cached fix available either");
        Serial.printf("[SOS] Second message: %s\n", msg2.c_str());
        send_sms_with_retry(EMERGENCY_SMS_NUMBER, msg2, "Second (GPS unavailable)");
        log_modem_diagnostics();
        log_gnss_detail();
    }

    Serial.println("[SOS] Sequence finished - putting modem back to sleep");
    modem_enter_sleep();

    s_busy = false;
}

static void emergency_task(void* /*param*/)
{
    while (true) {
        if (xSemaphoreTake(s_trigger_sem, portMAX_DELAY) == pdTRUE) {
            run_emergency_sequence();
        }
    }
}

// Retries modem power-on and network registration indefinitely (with a
// backoff between rounds) until both succeed. Only once actually
// registered does the modem get put to sleep - so the "awake and trying"
// state persists across as many rounds as it takes, rather than falling
// back to "will retry on trigger" and leaving the modem in a half-ready
// state until an emergency happens to wake it.
//
// disable_internal_gnss() is fired exactly once, right after the modem
// FIRST responds to AT - gated by the local gnss_off_sent latch so it
// doesn't get re-sent on every registration retry round. It deliberately
// isn't gated on network registration succeeding: GNSS on/off is a modem-
// local setting with nothing to do with the cellular side, so there's no
// reason to wait on that before turning it off.
#define MODEM_BOOT_RETRY_DELAY_MS   10000

static void modem_boot_task(void* /*param*/)
{
    bool gnss_off_sent = false;

    for (;;) {
        if (!ensure_modem_ready()) {
            Serial.println("[SOS] Modem failed to respond at boot - retrying...");
            s_network_status = 0;
            vTaskDelay(pdMS_TO_TICKS(MODEM_BOOT_RETRY_DELAY_MS));
            continue;
        }

        if (!gnss_off_sent) {
            disable_internal_gnss();
            gnss_off_sent = true;
        }

        if (!wait_for_network(NETWORK_REG_TIMEOUT_MS)) {
            s_network_status = 1;
            Serial.println("[SOS] Modem ready but NOT registered yet - retrying registration...");
            log_modem_diagnostics();
            vTaskDelay(pdMS_TO_TICKS(MODEM_BOOT_RETRY_DELAY_MS));
            continue;
        }

        s_network_status = 2;
        Serial.println("[SOS] Modem ready and REGISTERED on network - configuring, then sleeping");

        configure_delivery_reports();
        modem_enter_sleep();
        break;
    }
    vTaskDelete(nullptr);
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

uint8_t emergency_sms_get_gps_status() { return s_gps_status; }
uint8_t emergency_sms_get_network_status() { return s_network_status; }
static void uart_control_task(void* param);

void emergency_sms_init()
{
    pinMode(MODEM_FLIGHT_PIN, OUTPUT);
    digitalWrite(MODEM_FLIGHT_PIN, HIGH);

    pinMode(MODEM_DTR_PIN, OUTPUT);
    digitalWrite(MODEM_DTR_PIN, LOW);

    pinMode(MODEM_PWRKEY_PIN, OUTPUT);
    digitalWrite(MODEM_PWRKEY_PIN, LOW);

    SerialAT.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

    GPSSerial.begin(GPS_BAUD, SWSERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    // Initialize Serial2 (UART2) to accept control lines from the Rider/ESP32
    s_uart = &Serial2;
    s_uart->begin(UART_BAUD_RATE, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
    Serial.println("[SOS] Serial2 control link ready");

    s_trigger_sem     = xSemaphoreCreateBinary();
    s_at_mutex        = xSemaphoreCreateRecursiveMutex();
    s_gps_cache_mutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(emergency_task,  "sos_task",      4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(modem_boot_task, "sos_boot",      4096, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(gps_read_task,   "sos_gps_read",  4096, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(uart_control_task, "sos_uart", 4096, nullptr, 3, nullptr, 1);

    Serial.println("[SOS] Emergency SMS module initialized (GPS: external NEO-M8N)");
}

// Listens on Serial2 for simple line-based commands coming from the
// passenger ESP32 (e.g. "SOS\n"). When an "SOS" line is received,
// this triggers the remote emergency SMS sequence.
static void uart_control_task(void* /*param*/)
{
    // Driver GPS frames contain two signed coordinates plus status fields;
    // leave enough room for the complete line as well as the short SOS command.
    char line[64] = {};
    size_t length = 0;

    for (;;) {
        if (!s_uart || !s_uart->available()) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        char c = (char)s_uart->read();

        if (c == '\n' || c == '\r') {
            line[length] = '\0';
            if (strcmp(line, "SOS") == 0) {
                Serial.println("[SOS] Remote emergency trigger received (via UART)");
                emergency_sms_trigger_remote();
            } else {
                // The driver forwards its continuously updated NEO-M8N fix as
                // GPS,<lat_e6>,<lon_e6>,<age_ms>,<valid>. The passenger used to
                // ignore these frames (and its 16-byte command buffer could not
                // hold them), leaving its emergency SMS cache empty even when
                // the driver already had a satellite fix.
                long lat_e6 = 0;
                long lon_e6 = 0;
                unsigned int valid = 0;
                if (sscanf(line, "GPS,%ld,%ld,%*lu,%u", &lat_e6, &lon_e6, &valid) == 3 &&
                    valid == 1 && lat_e6 >= -90000000L && lat_e6 <= 90000000L &&
                    lon_e6 >= -180000000L && lon_e6 <= 180000000L) {
                    GpsTimestamp ts; // Driver frame has no UTC timestamp.
                    cache_gps_fix((double)lat_e6 / 1000000.0,
                                  (double)lon_e6 / 1000000.0, ts);
                }
            }
            length = 0;
        } else if (length < sizeof(line) - 1) {
            line[length++] = c;
        } else {
            length = 0;
        }
    }
}

void emergency_sms_process()
{
    poll_delivery_reports();
    log_gps_status();
    log_gnss_detail();
    log_debug_status();
}

void emergency_sms_trigger_remote()
{
    if (!s_trigger_sem) return;
    xSemaphoreGive(s_trigger_sem);
}

bool emergency_sms_busy() { return s_busy; }