# HelmetRIDER Passenger Unit - User Manual

> Scope: This manual describes the complete firmware behavior implemented in the
> passenger project, including the LilyGO main board and the attached
> HelmetRIDER passenger ESP32 module.

## 1. Purpose

The passenger unit provides speaker output, passenger-to-driver intercom, volume and mode controls, battery indication, helmet-wear monitoring, buckle protection, and emergency activation.

## 1.1 System Components

- **LilyGO passenger main board:** Battery sensing and charging detection, local battery LED, own cellular modem, own external GPS receiver, SMS sequence, and buckle/charge sleep coordination.
- **Passenger ESP32 audio board:** Green and red controls, speaker, microphone, remote battery-state RGB LED, passenger FSR wear sensor, ESP-NOW link to the driver, and UART link to the passenger main board.

The passenger ESP32 does not pair directly with a phone. It receives phone music
from the driver unit by radio and UART, while the two ESP32 boards handle
passenger intercom with ESP-NOW.

## 2. Before a Ride

1. Charge the passenger unit.
2. Fasten the helmet buckle.
3. Put on the helmet.
4. Confirm that the speaker and microphone are fitted securely inside the helmet.
5. Start the driver unit and confirm that the passenger LED and audio link respond.
6. Keep the units within reliable radio range.

Opening the buckle puts the passenger unit into deep sleep after a 50 ms stable buckle-open reading. Closing the buckle wakes the helmet system.

## 3. Controls

### Green Multifunction Button

| Action | Result |
|---|---|
| Press and release | Increase speaker volume |
| Press and hold for at least 0.6 seconds | Switch between Music and Intercom |
| Continue holding after the mode change | Nothing extra; release normally |

The passenger speaker has four volume levels:

1. 25%
2. 50%
3. 75%
4. 100%

The level loops back to 25% after 100%. A level-specific beep sequence confirms each volume level. The mode and volume controls work locally and synchronize audio behavior with the driver unit.

### Red Emergency Button

The passenger red button uses a five-press emergency sequence:

1. Press the red button five times.
2. Keep the presses within 7 seconds.
3. Keep consecutive presses no more than 1.2 seconds apart.
4. Wait for the armed confirmation tone.
5. Press once more within 10 seconds to confirm the emergency.

After confirmation:

- A sending tone is heard.
- The passenger tells the driver unit to activate emergency mode.
- The driver unit becomes responsible for sending SMS messages.
- The passenger and driver stop normal music/intercom processing.
- Emergency speaker output uses a fixed alert tone that is not controlled by the normal volume button.

Press the red emergency button once while emergency mode is active to stop it.

If confirmation is not received within 10 seconds, the emergency request times out.

## 4. Audio Modes

### Music Mode

Music received from the driver's paired phone is played by the passenger speaker.

Use a long press on the green button to switch from Music to Intercom.

### Intercom Mode

In Intercom mode:

- The passenger microphone sends speech to the driver.
- The passenger speaker receives the driver's microphone.
- Microphone noise below the input gate is not transmitted.
- Background audio is paused/ducked while intercom control tones are played.

If the passenger changes mode, the passenger unit informs the driver unit. The mode change is synchronized between both helmets.

## 5. Battery Indicator

The passenger battery LED mirrors the driver unit's battery state through the UART connection:

| Indicator | Meaning |
|---|---|
| Solid blue | Battery above 30% |
| Solid red | Battery 30% or less |
| Red blinking | Critical battery at 20% or less |
| Alternating red and blue | Charging; battery not full |
| Red and blue solid | Charging and full |

Blink and alternating LED patterns change every 500 milliseconds.

If no recent battery state is received, the passenger LED may remain at its previous state. Restart the complete system if the indicator remains incorrect.

## 6. Helmet and Buckle Safety

The buckle switch is checked continuously. If it remains open beyond the debounce period, the unit enters ESP32 deep sleep.

The pressure/wear sensor monitors whether the helmet is worn:

- A filtered reading must reach the wear threshold.
- Wear remains indicated for 3 seconds after the reading falls below the threshold.
- This hold period helps avoid flicker during small movements.

The wear state is sent to the driver unit as helmet telemetry.

## 7. Emergency SMS

Passenger emergency confirmation sends `SOS` over UART from the passenger ESP32
to the passenger main board. The passenger main board is therefore responsible
for sending the passenger's own emergency SMS sequence. It may receive GPS
coordinates previously forwarded by the driver, but normally uses its own
NEO-M8N GPS module and cache.

The passenger main board attempts to send:

1. An immediate emergency message.
2. A location message after obtaining or failing to obtain GPS.

The SMS text uses the passenger unit label:

- `EMERGENCY - PASSENGER needs help. Getting GPS location...`
- Fresh GPS: `EMERGENCY - PASSENGER location: <latitude>,<longitude> (acquired <UTC time>)`
- Cached GPS: `EMERGENCY - PASSENGER last known location: <latitude>,<longitude> (acquired <UTC time>, <age>s ago, no fresh fix)`
- No location: `EMERGENCY - PASSENGER GPS unavailable`

Each SMS is attempted twice if delivery is not confirmed. The modem wakes and
tries network registration, and waits up to 40 seconds for a fresh GPS fix.
The system can fall back to a previously cached location or report GPS unavailable.
After the sequence, the modem returns to sleep.

The passenger main board continuously reads GPS independently and caches valid
fixes. It disables the modem's internal GNSS and relies on the external NEO-M8N
for emergency location.

Emergency SMS depends on:

- An active SIM.
- Cellular coverage.
- Working driver modem power and registration.
- A working NEO-M8N GPS module and visible sky.

SMS success is not guaranteed. Call emergency services directly when immediate help is required.

## 8. Starting a Ride

1. Ensure both units are charged.
2. Fasten both buckles.
3. Power both units and wait for the ESP-NOW link to initialize.
4. Pair the phone only with the **Helmet-RIDER** device on the driver unit.
5. Start in Music mode or switch to Intercom with a long green-button press.
6. Set volume with short green-button presses.
7. Recheck battery indicators before long rides.

## 8.1 Local Passenger Battery LED

In addition to the passenger helmet ESP32's mirrored RGB battery indicator, the
passenger LilyGO main board has a local red charge LED:

| Main-board local LED | Meaning |
|---|---|
| Off | Not charging and not full |
| 500 ms on / 500 ms off | Charging, battery not full |
| Solid | Charging and battery fully topped off |

Main-board battery readings update every 2 seconds. The displayed estimate is
clamped to 0-100%. Low battery is 30% or less, critical battery is 20% or less,
and full charge is determined from approximately 97% charge plus a nearly stable
charge change rate.

## 9. Troubleshooting

| Problem | Action |
|---|---|
| No passenger speaker audio | Check passenger volume, mode, battery, and communication link. |
| Passenger cannot talk to driver | Long-press the green button to select Intercom and speak close to the passenger microphone. |
| Passenger microphone is too quiet | Check microphone seating and ensure there is sufficient input. |
| No audio from phone | Pair the phone with the **Helmet-RIDER** name on the driver unit, enable media audio, and start playback. |
| Emergency does not activate | Complete five red-button presses within 7 seconds, then confirm once within 10 seconds. |
| Emergency LED/speaker remains active | Press the red button once to clear active emergency mode. |
| Passenger indicator is incorrect | Check the UART link to the driver board and restart both units. |
| Passenger unit sleeps | Close and refasten the buckle, then release the deep-sleep wake condition. |
| Wear status is wrong | Fit the helmet correctly and check the FSR wiring and threshold behavior. |
| SMS not received | Check the driver's SIM, network registration, modem, and GPS; SMS delivery cannot be guaranteed. |
