#include <Arduino.h>
#include <Wire.h>
#include <esp_sleep.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

#include "pins.h"


// =====================================================
// MAX17048
// =====================================================

SFE_MAX1704X lipo(MAX1704X_MAX17048);

float batteryVoltage = 0;
float batteryPercent = 0;
float batteryChangeRate = 0;   // %/hr from MAX17048 CRATE register


// =====================================================
// LOCAL LED (single red LED, plain on/off — no RGB here anymore)
//
// This board's local LED ONLY shows charging/full status:
//   - not charging, not full : OFF
//   - charging (not yet full): BLINKING
//   - full (charger + topped off): SOLID
//
// The full 5-tier battery-level display (blue/red/critical-blink/
// charging-chase/full-both) that used to live on this board's RGB LED has
// moved to the HelmetRIDER ESP32. This board sends a 1-byte state code
// over UART (see sendBatteryStateUart() below) so that board can drive its
// own RGB LED with that same behavior.
// =====================================================

#define CHG_LED_COMMON_ANODE 0

#define BATT_LOW_PCT        30
#define BATT_CRITICAL_PCT   20

#define BLINK_ON_MS  500
#define BLINK_OFF_MS 500

#define SAMPLE_INTERVAL_MS 2000


// =====================================================
// CHARGE DETECTION (CRATE - FULL cutoff only)
// =====================================================
#define CHARGE_RATE_FULL_MAX       0.5f   // %/hr - below this near BATT_FULL_PCT = full/topped off
#define BATT_FULL_PCT              97.0f
#define CHARGE_CONFIRM_SAMPLES  3


// =====================================================
// CHARGE DETECTION (via GPIO39 VBUS voltage divider, read as ADC)
// =====================================================
//
// GPIO39 is ADC1_CH3. Previously read as a plain digital level via
// digitalRead(), matching the ext1 deep-sleep wakeup's fixed hardware
// VIH threshold exactly. Switched to analogRead() here so software can
// see the actual divider voltage rather than just a HIGH/LOW verdict -
// useful if the divider output sits in an ambiguous middle zone rather
// than swinging cleanly rail-to-rail.
//
// IMPORTANT: esp_sleep_enable_ext1_wakeup() in buckle_switch.cpp still
// wakes off the pin's raw DIGITAL level (fixed hardware VIH, ~2/3 VDD)
// - deep-sleep wake hardware has no concept of analogRead(). That
// threshold is INDEPENDENT of the thresholds below. If the two disagree,
// software's "charger present" verdict and the actual ext1 wake trigger
// can disagree too.
//
// Measured on real hardware: charger absent -> raw ~0. Charger present
// -> ~2.5V. That's a wide gap, so thresholds below have generous margin
// on both sides rather than sitting close to a single midpoint value.
// Default ADC1 attenuation is 11dB (~0-3.3V range), 12-bit resolution
// (0-4095).
#define CHARGE_SENSE_ADC_HIGH   1500   // must exceed this to be considered "present"
#define CHARGE_SENSE_ADC_LOW     500   // must drop below this to be considered "absent"

bool isChargerPresent()
{
    // Oversample to smooth out single-shot ADC noise / transient spikes
    uint32_t sum = 0;
    const int N = 8;
    for (int i = 0; i < N; i++) sum += analogRead(PIN_CHARGE_SENSE);
    int raw = sum / N;

    static bool present = false;  // sticky state with hysteresis
    if (raw >= CHARGE_SENSE_ADC_HIGH) present = true;
    else if (raw <= CHARGE_SENSE_ADC_LOW) present = false;
    // else: stays whatever it last was — dead zone between thresholds
    return present;
}


enum ChargeState
{
    CHG_NONE,
    CHG_CHARGING,
    CHG_FULL
};

// =====================================================
// UART STATE CODE (sent to HelmetRIDER ESP32)
//
// Sent every SAMPLE_INTERVAL_MS over Serial2 (PIN_UART_TX), framed as
// [UART_CTRL_MAGIC][UART_CTRL_BATTERY_STATE][state_code] rather than a
// bare byte. This link also carries the E5/5E-framed telemetry bundle
// (see link_manager.cpp / another file on this board), so a bare byte
// risks being silently swallowed by the receiver's sync-hunting parser.
// Both UART_CTRL_MAGIC and UART_CTRL_BATTERY_STATE MUST match the values
// used in HelmetRIDER's bt_audio.cpp receive parser.
//
// This board only sends the CURRENT STATE, not blink phase - HelmetRIDER
// must reproduce the blink/chase timing itself (500ms on/off, matching
// BLINK_ON_MS/BLINK_OFF_MS above) when it receives CRITICAL or CHARGING.
//
// Priority order when driving the remote RGB LED from this code should
// match the old local logic exactly: FULL > CHARGING > CRITICAL > normal
// RED/BLUE by percent. See determineUartState() below.
// =====================================================
#define UART_CTRL_MAGIC          0xAB
#define UART_CTRL_BATTERY_STATE  0x03   // must match HelmetRIDER's bt_audio.cpp UART_CTRL_BATTERY_STATE

enum BattUartState : uint8_t
{
    BATT_UART_BLUE       = 0,  // normal, >30%              -> solid BLUE
    BATT_UART_RED        = 1,  // normal, <=30%              -> solid RED
    BATT_UART_CRITICAL   = 2,  // critical, <=20%            -> blinking RED
    BATT_UART_CHARGING   = 3,  // charging, not yet full     -> alternating RED/BLUE chase
    BATT_UART_FULL       = 4,  // charger + topped off       -> BOTH LEDs solid
};


static uint32_t sampleTimer = 0;
static uint32_t flashTimer = 0;
static bool flashState = false;
static bool s_critical = false;

static ChargeState chargeState = CHG_NONE;
static ChargeState pendingChargeState = CHG_NONE;
static uint8_t chargeConfirmCount = 0;


// =====================================================
// LOCAL LED CONTROL
// =====================================================

void writeChgLed(bool on)
{
#if CHG_LED_COMMON_ANODE
    digitalWrite(PIN_CHG_LED, on ? LOW : HIGH);
#else
    digitalWrite(PIN_CHG_LED, on ? HIGH : LOW);
#endif
}


// =====================================================
// UART TX to HelmetRIDER
// =====================================================

uint8_t determineUartState()
{
    if (chargeState == CHG_FULL)     return BATT_UART_FULL;
    if (chargeState == CHG_CHARGING) return BATT_UART_CHARGING;
    if (s_critical)                  return BATT_UART_CRITICAL;
    return (batteryPercent <= BATT_LOW_PCT) ? BATT_UART_RED : BATT_UART_BLUE;
}

void sendBatteryStateUart(uint8_t code)
{
    // Framed as [MAGIC][CMD][state_code] rather than a bare byte, so it
    // can safely interleave with the E5/5E-framed telemetry bundle already
    // going out on this same Serial2 link without ever being mistaken for
    // telemetry sync bytes.
    uint8_t frame[3] = { UART_CTRL_MAGIC, UART_CTRL_BATTERY_STATE, code };
    Serial2.write(frame, sizeof(frame));
}


// =====================================================
// START
// =====================================================

void batteryLedBegin()
{
    Serial.println("MAX17048 Battery Monitor");

    Wire.begin(
        PIN_I2C_SDA,
        PIN_I2C_SCL
    );

    if(!lipo.begin())
    {
        Serial.println("MAX17048 NOT FOUND");
    }
    else
    {
        Serial.println("MAX17048 READY");

        lipo.disableHibernate();

        Serial.printf("MAX17048 HIBRT thresholds: Act=0x%02X Hib=0x%02X, currently hibernating=%s\n",
                      lipo.getHIBRTActThr(),
                      lipo.getHIBRTHibThr(),
                      lipo.isHibernating() ? "YES" : "no");

        // Preserve the gauge's learned SOC across MCU resets. Quick-start is
        // only appropriate when the gauge itself reports a power-on reset;
        // using it on every boot can replace a good learned estimate with a
        // less accurate first guess.
        if (lipo.isReset()) {
            Serial.println("MAX17048 - gauge power-on reset, running quickStart");
            lipo.quickStart();
            delay(200);
        }
    }

    // pinMode no longer required for analogRead() on PIN_CHARGE_SENSE,
    // but left here harmlessly in case anything else ever needs a
    // plain digital read of this pin too.
    pinMode(
        PIN_CHARGE_SENSE,
        INPUT
    );

    pinMode(
        PIN_CHG_LED,
        OUTPUT
    );

    // NOTE: UART2 initialization is centralized in bt_audio.cpp so
    // audio/telemetry and battery state share the same Serial2 instance.
    // sendBatteryStateUart() only needs Serial2 to be initialized once.

    writeChgLed(false);
    flashTimer = 0;
    flashState = false;
}


// =====================================================
// UPDATE
// =====================================================

void batteryLedUpdate()
{
    uint32_t now=millis();

    if(now-sampleTimer >= SAMPLE_INTERVAL_MS)
    {
        sampleTimer=now;

        batteryVoltage =
            lipo.getVoltage();

        // Clamp right at the source: MAX17048's ModelGauge SOC output is an
        // estimate, not hard-bounded -- it can read slightly above 100% near
        // full and slightly negative near empty. Clamping here means every
        // consumer of the module-level batteryPercent -- critical/low
        // comparisons, UART state, the PBAT send below, batteryGetPercent()
        // -- works from the same bounded value instead of raw sensor noise.
        float rawSoc = lipo.getSOC();
        if (rawSoc < 0.0f)   rawSoc = 0.0f;
        if (rawSoc > 100.0f) rawSoc = 100.0f;
        batteryPercent = rawSoc;

        batteryChangeRate =
            lipo.getChangeRate();

        bool chargerPresent = isChargerPresent();

        // Edge-triggered: only act when presence actually changes from
        // the previous sample, not on every tick while it stays the same.
        static bool lastChargerPresent = false;

        if (chargerPresent != lastChargerPresent)
        {
            if (!chargerPresent)
            {
                chargeState = CHG_NONE;
                pendingChargeState = CHG_NONE;
                chargeConfirmCount = 0;
            }
            lastChargerPresent = chargerPresent;
        }

        if (chargerPresent)
        {
            ChargeState sampleFullState =
                (batteryPercent >= BATT_FULL_PCT &&
                 batteryChangeRate <  CHARGE_RATE_FULL_MAX &&
                 batteryChangeRate > -CHARGE_RATE_FULL_MAX)
                    ? CHG_FULL
                    : CHG_CHARGING;

            if(sampleFullState == pendingChargeState)
            {
                if(chargeConfirmCount < 255)
                    chargeConfirmCount++;
            }
            else
            {
                pendingChargeState = sampleFullState;
                chargeConfirmCount = 1;
            }

            if(chargeConfirmCount >= CHARGE_CONFIRM_SAMPLES)
            {
                chargeState = pendingChargeState;
            }
            else if(chargeState == CHG_NONE)
            {
                chargeState = CHG_CHARGING;
            }
        }

        s_critical = (batteryPercent <= BATT_CRITICAL_PCT);

        uint8_t uartState = determineUartState();
            sendBatteryStateUart(uartState);
            // Also send raw percentage for the attached ESP32 to consume
            // format: "PBAT,<percent>\n"
            // batteryPercent is already clamped to [0,100] above, so this
            // cast is safe and can no longer send garbage near empty/full.
            uint8_t pct = (uint8_t)batteryPercent;
            Serial2.printf("PBAT,%u\n", (unsigned)pct);

        Serial.printf(
            "Battery %.2fV %.1f%% rate=%.2f%%/hr hib=%s vbus=%s%s%s uart_state=%u\n",
            batteryVoltage,
            batteryPercent,
            batteryChangeRate,
            lipo.isHibernating() ? "YES" : "no",
            chargerPresent ? "YES" : "no",
            (batteryPercent <= BATT_CRITICAL_PCT) ? " CRITICAL" : "",
            (chargeState == CHG_CHARGING) ? " CHARGING" :
            (chargeState == CHG_FULL)     ? " FULL"     : "",
            uartState
        );
    }

    // Local LED: charging = blink, full = solid, otherwise off.
    if(chargeState == CHG_FULL)
    {
        flashTimer = 0;
        flashState = false;
        writeChgLed(true);
        return;
    }

    if(chargeState == CHG_CHARGING)
    {
        if(now-flashTimer > (flashState ? BLINK_ON_MS : BLINK_OFF_MS))
        {
            flashTimer = now;
            flashState = !flashState;
        }

        writeChgLed(flashState);
        return;
    }

    flashTimer = 0;
    flashState = false;
    writeChgLed(false);
}


uint8_t batteryGetPercent()
{
    // batteryPercent is already clamped to [0,100] at the point it's read
    // from the sensor (see batteryLedUpdate()), so a plain cast is safe here.
    return (uint8_t)batteryPercent;
}

bool batteryIsCritical()
{
    return s_critical;
}

float batteryGetChangeRate()
{
    return batteryChangeRate;
}

bool batteryIsCharging()
{
    return chargeState == CHG_CHARGING;
}

bool batteryIsFull()
{
    return chargeState == CHG_FULL;
}

// Debounced charge-state check (updated once per SAMPLE_INTERVAL_MS),
// used for LED display logic in this file. buckle_switch.cpp uses the
// raw isChargerPresent() instead for its sleep gate, since sleep
// decisions need the instant, undebounced reading rather than this
// filtered/confirmed state.
bool batteryIsChargerConnected()
{
    return chargeState != CHG_NONE;
}