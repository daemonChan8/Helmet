# HelmetRIDER Driver Unit - User Manual

> Scope: This manual describes the complete firmware behavior implemented in the
> driver project, including the LilyGO main board and the attached HelmetRIDER
> ESP32 module. It covers features visible to the rider and service-level
> behavior such as Bluetooth, BLE telemetry, charging, GPS, modem, and sleep.

## 1. Purpose

The driver unit provides Bluetooth audio, rider-to-passenger intercom, volume and mode controls, battery indication, helmet-wear monitoring, buckle protection, and emergency assistance messaging.

## 1.1 System Components

- **LilyGO main board:** Battery sensing and charging detection, local battery LED, Bluetooth A2DP phone audio, BLE telemetry, cellular modem, GPS, SMS, and buckle/charge wake or sleep coordination.
- **HelmetRIDER ESP32 audio board:** Green and red controls, speaker, microphone, battery-state RGB LED, rider FSR wear sensor, and ESP-NOW link to the passenger ESP32.
- **Passenger unit:** Receives and forwards music and intercom audio and reports passenger battery and wear telemetry.

The phone pairs over Classic Bluetooth A2DP with the main board under the name
**Helmet-RIDER**. Inter-unit audio uses ESP-NOW and the UART bus, not a direct
Bluetooth connection to the passenger helmet.

## 2. Before a Ride

1. Charge the unit until the indicator shows the fully charged state.
2. Fasten the helmet buckle.
3. Put on the helmet and confirm that the green control button is usable.
4. On the phone, open Bluetooth settings and pair with **Helmet-RIDER**.
5. Start playback from the phone. Music is played only when the unit is in Music mode.
6. Confirm that the passenger unit is powered and within radio range.

Keep the buckle fastened while riding. Opening the buckle or, on the main driver board, disconnecting charging while buckled open can put the unit into deep sleep.

## 3. Controls

### Green Multifunction Button

| Action | Result |
|---|---|
| Press and release | Increase speaker volume |
| Press and hold for at least 0.6 seconds | Switch between Music and Intercom |
| Continue holding after the mode change | Nothing extra; release normally |

The speaker has four volume levels:

1. 25%
2. 50%
3. 75%
4. 100%

The level wraps back to 25% after 100%. A level confirmation tone is heard after volume changes. The volume setting does not change emergency-alert output.

### Red Emergency Button

The red button uses a deliberate five-press sequence to reduce accidental activation:

1. Press the red button five times.
2. Keep presses within 7 seconds.
3. Keep each press no more than 1.2 seconds apart.
4. The unit then enters armed mode and gives a confirmation tone.
5. Press the red button once more within 10 seconds to confirm.

After confirmation:

- An emergency 700 Hz tone announces sending.
- The speaker switches to an emergency heartbeat tone.
- The system sends the emergency state to the passenger unit.
- The driver modem attempts to send SMS assistance messages.
- The intercom and normal music background are suspended.

To stop or cancel emergency mode, press the red emergency button once while emergency mode is active.

If no confirmation press occurs within 10 seconds, the emergency request is cancelled automatically.

## 4. Audio Modes

### Music Mode

Music mode plays audio received from the paired phone over Bluetooth. This is the default startup mode.

Use the green button to switch to Intercom when communication with the passenger is required.

### Intercom Mode

Intercom mode sends the rider microphone to the passenger unit and receives the passenger microphone. It transmits only when microphone input exceeds the noise gate.

Music and other background audio are not used as the intercom source in this mode.

The green mode change is synchronized to the passenger unit.

## 5. Battery Indicator

The battery indicator is a red/blue LED on the rider helmet display:

| Indicator | Meaning |
|---|---|
| Solid blue | Battery above 30% |
| Solid red | Battery 30% or less |
| Red blinking | Critical battery at 20% or less |
| Alternating red and blue | Charging; battery not full |
| Red and blue solid | Charging and full |

The indicator updates approximately every two seconds.

If charging is detected, battery charging is shown first. A critical battery is treated before normal low-battery indication.

The main LilyGO board has one separate local red charge LED:

| Local main-board LED | Meaning |
|---|---|
| Off | Not charging and not full |
| 500 ms on / 500 ms off | Charging, battery not full |
| Solid | Charger present and battery fully topped off |

The helmet-mounted ESP32 RGB indicator mirrors the main-board five-state battery
result over UART. The local main-board LED does not show normal battery percentage.

The battery estimate is read every 2 seconds and is clamped to 0-100%. Low battery
is at 30% or less and critical battery is at 20% or less. Charging is confirmed
from charger voltage and charge change-rate measurements; full state requires
about 97% reported charge, a small change rate, and several confirmed samples.

## 6. Helmet and Buckle Safety

The buckle switch detects whether the chin strap buckle is open. After the buckle remains open for about 50 milliseconds:

- The unit coordinates the connected passenger unit to sleep.
- The driver unit enters deep sleep when it is not charging.
- Charging may prevent the open-buckle sleep path.
- A critical battery can enter coordinated deep sleep after a short continuous critical period.

Close the buckle to wake the driver hardware. Charging state can also wake an ext1-supported deep-sleep unit.

The wear sensor distinguishes installed-worn status with a filtered pressure reading. The reading must remain below the wear threshold for 3 seconds before wear is reported as removed.

## 7. Emergency SMS

Emergency SMS is handled by the main driver board.

The system normally attempts to send:

1. An immediate emergency message.
2. A second message after GPS location is obtained or a location attempt expires.

If no fresh fix is available, it may send a cached last-known location or report that GPS location is unavailable.

The SMS sends are formatted as:

- `EMERGENCY - DRIVER needs help. Getting GPS location...`
- Fresh GPS: `EMERGENCY - DRIVER location: <latitude>,<longitude> (acquired <UTC time>)`
- Cached GPS: `EMERGENCY - DRIVER last known location: <latitude>,<longitude> (acquired <UTC time>, <age>s ago, no fresh fix)`
- No location: `EMERGENCY - DRIVER GPS unavailable`

The modem wakes, tries network registration, and attempts each SMS twice before
reporting a final failure. The modem wakes for an emergency and returns to sleep
after the sequence. At boot, it retries modem power and network registration in
10-second backoff cycles until registered. It disables the modem's internal
GNSS because GPS is supplied by the external NEO-M8N module.

The first SMS is sent immediately. The system then waits up to 40 seconds for a
new valid GPS fix. A previously cached valid GPS fix may be used if no fresh fix
arrives. SMS delivery reports and cellular/GPS health diagnostics are supported
through the modem and serial debug output.

For the SMS feature to operate:

- The SIM must be installed and active.
- Cellular coverage must be available.
- The modem must be powered and able to register.
- The external GPS antenna/module must have a clear view of the sky.

SMS attempts are not guaranteed to succeed. Never rely on this device as a substitute for calling emergency services directly.

## 8. Pairing Bluetooth

1. Set the phone's Bluetooth mode on.
2. Select **Helmet-RIDER**.
3. Accept the pairing request or enter PIN **0000** if requested.
4. Select/enable media audio if prompted.
5. Start a song or audio stream.

To repair after a connection failure:

- Forget **Helmet-RIDER** on the phone.
- Restart Bluetooth on the phone.
- Reconnect to **Helmet-RIDER**.

The Bluetooth sink is an A2DP audio receiver. It is discoverable and accepts the
PIN **0000** if the phone requests one. Bluetooth pairing is for phone audio only;
the phone is not used to control intercom or emergency behavior.

The main board also provides BLE telemetry while Classic Bluetooth is connected.
The BLE device uses the same **Helmet-RIDER** name and contains two telemetry
characteristics:

| Characteristic | Values |
|---|---|
| Rider telemetry | Battery percentage, wear status (0/1), GPS-valid status (0/1) |
| Passenger telemetry | Passenger battery percentage, passenger wear status (0/1) |

The passenger and rider values use `N/A` before a value has ever been received.
Actual 0 values are valid data and are not converted to `N/A`. Rider telemetry
notifies on change and approximately once per second; passenger telemetry
notifies when passenger data changes. Notifications are queued until a BLE client
connects and enables notifications.

## 9. Troubleshooting

| Problem | Action |
|---|---|
| Phone does not find Helmet-RIDER | Restart the main unit, enable phone Bluetooth, and check that the unit is powered. |
| Audio is silent | Check volume, confirm Music mode, reconnect the phone, and verify that media audio is enabled. |
| Passenger cannot hear rider | Hold or talk into the microphone and confirm Intercom mode. |
| Passenger cannot be heard | Confirm that the passenger is in Intercom mode and the helmet microphone is wired correctly. |
| Emergency does not confirm | Complete five presses within 7 seconds, then press once within 10 seconds. |
| SMS was not received | Check cellular coverage, SIM status, modem power, and GPS; an external SMS recipient is normally used. |
| No location in SMS | Move the helmet to an open area and allow the GPS module to obtain a fix. |
| Battery LED is blue while unplugged | Battery is above 30%, or the remote LED is no longer receiving a battery update. |
| Unit does not start | Charge the battery and check that the buckle is in a valid powered position. |
| Unit sleeps unexpectedly | Confirm the buckle is not open and charging state is stable. |
