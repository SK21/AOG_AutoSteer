# AOG AutoSteer — Implementation Plan
## Dual GPS Source + Dual Steering Mode Refactor

**Date:** 2026-05-09  
**Hardware target:** AS15-3 PCB (Teensy 4.1)  
**Reference projects:** AOG_AutoSteer (existing), ToolDual (reference)

---

## 1. Goals

1. Support two GPS/heading sources interchangeably on the same PCB:
   - **UM982 / ByNav C2-M20D-U** — single unit, dual antenna, KSXT sentence
   - **F9P + IMU** — ArduSimple/SimpleRTK2B with BNO080 or TM171 IMU

2. Support two steering modes:
   - **Wheel Angle (autosteer)** — WAS-based, existing PGN set
   - **Tool XTE (implement steering)** — cross-track error, PGN 230/233

3. Deliver all four combinations in a **single binary** configured at runtime via UDP/EEPROM.

---

## 2. Architecture Decision: Single Repo, Runtime Configuration

### Recommendation: ONE repo, ONE binary, TWO runtime fields in `ModuleConfig`

The GPS source and steering mode are stored in `ModuleConfig` (EEPROM offset 110), sent and received via the existing PGN 32300 config mechanism — the same way serial port assignments, IMU type, and ADS1115 presence are already stored.

| Factor | Compile-time `#define` | Runtime `ModuleConfig` |
|---|---|---|
| Flashing wrong binary | Risk — wrong #define → wrong mode | Eliminated — one binary, all modes |
| Field reconfiguration | Requires recompile + reflash | Change in AOG settings, instant |
| Flash usage | ~15 KB savings per excluded mode | ~15 KB extra (<1% of 2 MB) — negligible |
| Branching overhead | None | ~60 ns per 25 ms loop = 0.00024% — negligible |
| Follows existing pattern | No (new mechanism) | Yes — matches MDL.ReceiverSerialPort etc. |
| Library install requirement | Must install only needed libs | Both IMU libs must be installed |

**Verdict:** On a Teensy 4.1 (2 MB flash, 1 MB RAM, 600 MHz Cortex-M7), code size and branching overhead are not constraints. Runtime config eliminates the wrong-binary-flashed failure mode and matches how the existing codebase already manages variation. The four build variants become four EEPROM configurations of one firmware image.

### Constants (add to a shared header, e.g. top of `AutoSteerTeensy.ino`)

```cpp
// GPS Source values (stored in MDL.GPSSource)
#define GPS_F9P_IMU   0    // ArduSimple F9P + BNO080/TM171
#define GPS_KSXT      1    // UM982 / ByNav C2-M20D-U

// Steering Mode values (stored in MDL.SteeringMode)
#define STEER_WHEEL_ANGLE  0    // WAS-based autosteer — PGN 253/252/251/254
#define STEER_TOOL_XTE     1    // XTE implement steering — PGN 230/233
```

### `ModuleConfig` additions (EEPROM offset 110)

The two new fields are appended to the end of the existing struct. All other fields are unchanged.

```cpp
struct ModuleConfig {
    // --- existing fields (unchanged) ---
    uint8_t  ReceiverSerialPort;    // serial port for GPS receiver
    uint8_t  IMUSerialPort;         // serial port for IMU
    uint8_t  PassThruInSerialPort;
    uint8_t  PassThrOutSerialPort;
    uint8_t  IMUType;               // 0=BNO080 RVC, 1=TM171 EasyProfile
    uint8_t  ADS1115_Found;
    // ... other existing fields ...

    // --- NEW fields ---
    uint8_t  GPSSource;             // GPS_F9P_IMU(0) or GPS_KSXT(1)
    uint8_t  SteeringMode;          // STEER_WHEEL_ANGLE(0) or STEER_TOOL_XTE(1)
};
```

Both fields default to 0 (F9P + IMU, wheel angle autosteer) — preserving existing behaviour with no EEPROM migration needed for existing boards.

### Resulting repo structure (single repo, no compile guards on .ino files)

```
AOG_AutoSteer/                 ← single repo, all four runtime configs
  AutoSteerTeensy.ino          ← MODIFIED: mode constants + runtime loop branching
  GPS_KSXT.ino                 ← NEW: UM982/ByNav KSXT handler (always compiled)
  IMU.ino                      ← UNCHANGED: called only when MDL.GPSSource == GPS_F9P_IMU
  Begin.ino                    ← MODIFIED: runtime serial init
  Panda.ino                    ← MODIFIED: runtime GPS data source
  SteerComm.ino                ← MINOR: runtime heading validity
  Steering.ino                 ← called only when MDL.SteeringMode == STEER_WHEEL_ANGLE
  SteerSwitches.ino            ← called only when MDL.SteeringMode == STEER_WHEEL_ANGLE
  Toolsteer.ino                ← NEW (from ToolDual): called when STEER_TOOL_XTE
  ToolsteerPID.ino             ← NEW (from ToolDual): called when STEER_TOOL_XTE
  ToolComm.ino                 ← NEW: PGN 230/233 handler
  zADS1115.h/.cpp              ← NEW (from ToolDual): actuator position ADC
  [all other files unchanged]
```

---

## 3. Part A — GPS Source Abstraction

### 3.1 KSXT sentence field map

From ArduPilot implementation and Unicore UG017 documentation:

```
$KSXT,YYYYMMDDHHMMSS.ss,lon,lat,alt,heading,pitch,roll,speed,,fix,hdgStatus,sats,satsUsed,,,, velN,velE,velD,,*XX

Field  [0]  UTC time
Field  [1]  Longitude       decimal degrees
Field  [2]  Latitude        decimal degrees
Field  [3]  Altitude        metres
Field  [4]  Heading/Yaw     0–360°
Field  [5]  Pitch           degrees
Field  [6]  Roll            degrees
Field  [7]  Speed           m/s
Field  [8]  (empty)
Field  [9]  Fix status      ≥1 = valid position
Field  [10] Heading status  3 = RTK dual antenna fixed
Field  [11] Satellites tracked
Field  [12] Satellites used
Field [13-15] (empty)
Field [16]  Velocity North  km/h
Field [17]  Velocity East   km/h
Field [18]  Velocity Down   km/h
Field [19]  (empty)
```

---

### 3.2 New file: `GPS_KSXT.ino`

Always compiled. Functions are only called when `MDL.GPSSource == GPS_KSXT`.

```cpp
// Published to the rest of the firmware via these globals
// (declared extern in AutoSteerTeensy.ino)
float GPS_Heading  = 0;    // degrees 0-360
float GPS_Roll     = 0;    // degrees, positive = right side up
float GPS_Pitch    = 0;    // degrees
float GPS_Speed    = 0;    // km/h
float GPS_Lat      = 0;    // decimal degrees
float GPS_Lon      = 0;    // decimal degrees
float GPS_Alt      = 0;    // metres
uint8_t GPS_Fix    = 0;    // 0=none, 1=GNSS, 2=DGNSS, etc.
uint8_t GPS_HdgStatus = 0; // 0=none, 1=float, 3=RTK dual fixed
uint8_t GPS_Sats   = 0;
bool GPS_DataValid = false;
uint32_t GPS_LastUpdate = 0;

// KSXT parser state
static char  _ksxtBuf[120];
static uint8_t _ksxtIdx = 0;
static bool  _ksxtActive = false;

// Called from DoPanda() when MDL.GPSSource == GPS_KSXT
void ReadKSXT()
{
    HardwareSerial* port = GetSerialPort(MDL.ReceiverSerialPort);
    if (!port) return;

    while (port->available())
    {
        char c = port->read();

        if (c == '$') {
            _ksxtIdx = 0;
            _ksxtActive = true;
        }

        if (_ksxtActive) {
            if (_ksxtIdx < sizeof(_ksxtBuf) - 1)
                _ksxtBuf[_ksxtIdx++] = c;

            if (c == '\n') {
                _ksxtBuf[_ksxtIdx] = '\0';
                if (strncmp(_ksxtBuf, "$KSXT,", 6) == 0)
                    ParseKSXT(_ksxtBuf);
                _ksxtActive = false;
                _ksxtIdx = 0;
            }
        }
    }
}

void ParseKSXT(const char* sentence)
{
    const char* star = strrchr(sentence, '*');
    if (!star) return;

    char buf[120];
    strncpy(buf, sentence + 6, sizeof(buf)); // skip "$KSXT,"
    buf[sizeof(buf)-1] = '\0';

    char* fields[21];
    uint8_t nFields = 0;
    char* p = buf;
    fields[nFields++] = p;
    while (*p && nFields < 21) {
        if (*p == ',' || *p == '*') {
            *p = '\0';
            fields[nFields++] = p + 1;
        }
        p++;
    }
    if (nFields < 13) return;

    uint8_t fix = (uint8_t)atoi(fields[9]);
    uint8_t hdgSt = (uint8_t)atoi(fields[10]);
    if (fix < 1) {
        GPS_DataValid = false;
        return;
    }

    GPS_Fix       = fix;
    GPS_HdgStatus = hdgSt;
    GPS_Lon       = atof(fields[1]);
    GPS_Lat       = atof(fields[2]);
    GPS_Alt       = atof(fields[3]);
    GPS_Heading   = atof(fields[4]);
    GPS_Pitch     = atof(fields[5]);
    GPS_Roll      = atof(fields[6]);
    GPS_Speed     = atof(fields[7]) * 3.6f; // m/s → km/h
    GPS_Sats      = (uint8_t)atoi(fields[11]);
    GPS_DataValid = true;
    GPS_LastUpdate = millis();
}

bool GPS_Connected()
{
    return (millis() - GPS_LastUpdate) < 4000;
}

// Send ByNav startup configuration on first boot.
// Saved inside the ByNav (SAVECONFIG) so safe to send every boot.
void SendKSXTConfig(HardwareSerial* port)
{
    delay(200);
    port->println("UNLOGALL");                delay(100);
    port->println("RTKTYPE ROVER");           delay(100);
    port->println("WORKFREQS ALL ALL");       delay(100);
    port->println("SET OBSFREQ 10");          delay(100);
    port->println("HEADINGOFFSET 90");        delay(100);
    port->println("DUALANTENNAPOWER ON");     delay(100);
    port->println("RTKTIMEOUT 500");          delay(100);
    port->println("INTERFACEMODE COM1 AUTO AUTO"); delay(100);
    port->println("SNRCUTOFF 15");            delay(100);
    port->println("LOG KSXT ONTIME 0.1");     delay(100);
    port->println("SAVECONFIG");              delay(500);
}
```

---

### 3.3 Modifications to `Begin.ino`

Replace the existing serial port and IMU init block with a runtime conditional:

```cpp
// In DoSetup(), after Serial0 init:

if (MDL.GPSSource == GPS_KSXT)
{
    // ByNav C2-M20D-U on Serial4 (RX4/TX4 via adapter)
    // MDL.ReceiverSerialPort must be set to 4 via AOG config
    SerialReceiver = SetSerialPort(MDL.ReceiverSerialPort);
    if (SerialReceiver) {
        SerialReceiver->begin(115200);
        SerialReceiverEnabled = true;
        SendKSXTConfig(SerialReceiver);
    }
    SerialIMUEnabled = false;  // ByNav has integrated INS
    SerialPassThruEnabled = false;
}
else // GPS_F9P_IMU
{
    // Existing F9P + IMU init — unchanged from current Begin.ino
    SerialReceiver = SetSerialPort(MDL.ReceiverSerialPort);
    if (SerialReceiver) {
        SerialReceiver->begin(ReceiverBaud);  // 460800
        SerialReceiverEnabled = true;
    }
    SerialIMU = SetSerialPort(MDL.IMUSerialPort);
    if (SerialIMU) {
        SerialIMU->begin(IMUBaud);            // 115200
        SerialIMUEnabled = true;
    }
    SerialPassIn  = SetSerialPort(MDL.PassThruInSerialPort);
    SerialPassOut = SetSerialPort(MDL.PassThrOutSerialPort);
    if (SerialPassIn && SerialPassOut) {
        SerialPassIn->begin(PassThruBaud);
        SerialPassOut->begin(PassThruBaud);
        SerialPassThruEnabled = true;
    }
}
```

---

### 3.4 Modifications to `Panda.ino` — GPS abstraction

The `DoPanda()` function currently reads NMEA from the F9P and queries the IMU. Under KSXT the data arrives pre-computed.

**Replace the top of `DoPanda()`:**

```cpp
void DoPanda()
{
    if (MDL.GPSSource == GPS_KSXT)
    {
        ReadKSXT();   // fills GPS_* globals

        if (GPS_DataValid)
        {
            // Map KSXT data into the existing IMU_* globals so
            // BuildPanda() and SendSteerData() work without changes
            IMU_Heading = GPS_Heading * 10.0f;   // tenths of degrees
            IMU_Roll    = GPS_Roll    * 10.0f;
            IMU_Pitch   = GPS_Pitch   * 10.0f;
            IMU_YawRate = 0;                     // not in KSXT

            BuildPandaFromKSXT();
        }
        // Route NTRIP corrections to ByNav COM1 RX (TX4)
        ReceiveNtripAndForward();
    }
    else // GPS_F9P_IMU
    {
        // Existing DoPanda() code — unchanged
        // (NMEA parser reads GGA+VTG, IMU fills IMU_* globals)
        ...existing code...
    }
}

void BuildPandaFromKSXT()
{
    char latBuf[12], lonBuf[13];

    auto toNMEA = [](double deg, char* buf, int bufLen) {
        bool neg = deg < 0; deg = fabs(deg);
        int d = (int)deg;
        double m = (deg - d) * 60.0;
        snprintf(buf, bufLen, "%02d%08.5f", d, m);
        (void)neg;
    };

    toNMEA(GPS_Lat, latBuf, sizeof(latBuf));
    toNMEA(GPS_Lon, lonBuf, sizeof(lonBuf));

    char hdgBuf[8], rollBuf[8], pitchBuf[8], spdBuf[8], altBuf[8];
    snprintf(hdgBuf,   sizeof(hdgBuf),  "%.1f", GPS_Heading);
    snprintf(rollBuf,  sizeof(rollBuf), "%.2f", GPS_Roll);
    snprintf(pitchBuf, sizeof(pitchBuf),"%.2f", GPS_Pitch);
    snprintf(spdBuf,   sizeof(spdBuf),  "%.2f", GPS_Speed * 0.539957f); // km/h → knots
    snprintf(altBuf,   sizeof(altBuf),  "%.1f", GPS_Alt);

    char fixBuf[3], satBuf[4];
    snprintf(fixBuf, sizeof(fixBuf), "%d", GPS_Fix);
    snprintf(satBuf, sizeof(satBuf), "%02d", GPS_Sats);

    snprintf(nme, sizeof(nme),
        "$PANDA,0,%.7f,%s,%.7f,%s,%s,%s,1.0,%s,0,%s,%s,%s,0,0",
        fabs(GPS_Lat), (GPS_Lat >= 0 ? "N" : "S"),
        fabs(GPS_Lon), (GPS_Lon >= 0 ? "E" : "W"),
        fixBuf, satBuf, altBuf, spdBuf,
        hdgBuf, rollBuf, pitchBuf);

    CalculateChecksum(nme);

    UDPsteering.beginPacket(DestinationIP, 9999);
    UDPsteering.write((uint8_t*)nme, strlen(nme));
    UDPsteering.endPacket();

    isGGA_Updated = true;
}
```

---

### 3.5 `IMU.ino` — no structural changes

No compile guard is added. The IMU read functions simply are not called when `MDL.GPSSource == GPS_KSXT`. The call sites in the main loop already go through `DoPanda()` which branches on `MDL.GPSSource`.

Both IMU libraries (Adafruit BNO08x RVC and EasyProfile/TM171) must be installed in the Arduino library manager regardless of which GPS source is selected at runtime — they are always compiled into the binary.

---

### 3.6 Modifications to `SteerComm.ino`

Replace the IMU connection check for PGN 253 fields 7–10:

```cpp
void SendSteerData()
{
    bool headingValid;
    int16_t hVal, rVal;

    if (MDL.GPSSource == GPS_KSXT)
    {
        headingValid = GPS_Connected() && (GPS_HdgStatus >= 1);
        hVal = headingValid ? (int16_t)(GPS_Heading * 10) : 9999;
        rVal = headingValid ? (int16_t)(GPS_Roll    * 10) : 8888;
    }
    else
    {
        headingValid = IMU_Connected();
        hVal = headingValid ? (int16_t)IMU_Heading : 9999;
        rVal = headingValid ? (int16_t)IMU_Roll    : 8888;
    }

    PGN_253[5] = (uint8_t)((int16_t)(steerAngleActual * 100));
    PGN_253[6] = (uint8_t)((int16_t)(steerAngleActual * 100) >> 8);
    PGN_253[7] = (uint8_t)hVal;
    PGN_253[8] = (uint8_t)(hVal >> 8);
    PGN_253[9] = (uint8_t)rVal;
    PGN_253[10]= (uint8_t)(rVal >> 8);
    // ... rest unchanged
}
```

---

### 3.7 Serial port assignment summary (KSXT runtime config)

| Teensy Port | Baud | Direction | Signal |
|---|---|---|---|
| Serial4 (RX4) | 115200 | IN | ByNav COM1 TX → KSXT sentences |
| Serial4 (TX4) | 115200 | OUT | ByNav COM1 RX ← NTRIP RTCM + startup config |
| Serial8 (RX8) | 115200 | IN | ByNav COM2 TX → optional secondary |
| Serial8 (TX8) | 115200 | OUT | ByNav COM2 RX ← (optional) |
| Serial3 | — | — | **Not used** (no external IMU) |
| Serial2 | — | — | **Not used** (no RS232 passthrough needed) |

`MDL.ReceiverSerialPort = 4` must be set via AOG module config for KSXT mode.

### 3.8 Serial port assignment summary (F9P + IMU runtime config)

| Teensy Port | Baud | Direction | Signal |
|---|---|---|---|
| Serial4 (RX4) | 460800 | IN | F9P UART1 TX → GGA + VTG |
| Serial4 (TX4) | 460800 | OUT | F9P UART1 RX ← NTRIP RTCM |
| Serial8 (RX8) | 57600 | IN | F9P UART2 TX → passthrough |
| Serial2 (TX2) | 57600 | OUT | F9P UART2 RX → RS232 DB9 |
| Serial3 (RX3) | 115200 | IN | BNO080 / TM171 IMU |

---

## 4. Part B — AOG_AutoSteer Changes (wheel angle steering)

Changes to the autosteer codebase are **GPS abstraction only**. The entire steering pipeline (WAS → PID → motor PWM → PGN 253) is unchanged and runs whenever `MDL.SteeringMode == STEER_WHEEL_ANGLE`.

### Files changed

| File | Change | Size |
|---|---|---|
| `AutoSteerTeensy.ino` | Add GPS_SOURCE / STEER_MODE constants; main loop runtime branch | ~20 lines added |
| `GPS_KSXT.ino` | New file — KSXT parser + ByNav config (always compiled) | ~150 lines |
| `Begin.ino` | Runtime conditional serial init (`if MDL.GPSSource`) | ~25 lines changed |
| `Panda.ino` | Runtime GPS data source + `BuildPandaFromKSXT()` | ~80 lines changed |
| `SteerComm.ino` | Runtime heading validity check | ~10 lines changed |
| `Config.ino` | Receive and store new `GPSSource` + `SteeringMode` fields via PGN 32300 | ~10 lines changed |

### Files NOT changed

`Steering.ino`, `SteerSwitches.ino`, `Analog.ino`, `IMU.ino`, `EthernetUpdate.ino`, `zNMEAParser.h`, `Adafruit_BNO08x_RVC.*`, `EasyProfile.*`

### EEPROM layout (Part B only)

Two bytes appended to `ModuleConfig` at EEPROM offset 110. All existing offsets unchanged. Both default to 0 (existing behaviour).

---

## 5. Part C — Tool Steering Mode (same repo, runtime `MDL.SteeringMode`)

### 5.1 What it does

The firmware's role depends on which tool steering mode is selected in Twol:

**In all modes** the firmware sends PANDA sentences (position, heading, roll, speed) to AgOpenGPS. Twol reads that data to track the implement's position relative to the AB line. GPS accuracy matters here — dual-antenna heading from the ByNav is accurate at 0 km/h, where single-antenna derived heading fails. The firmware never uses GPS directly in its control algorithm.

**Active / Follow Pivot:** Two-level control loop. Twol computes tool XTE from the PANDA position data and sends it via PGN 233. The firmware runs a PID or bang-bang algorithm, drives the hydraulic actuator to zero the XTE, and reports actuator position back via PGN 230. A physical actuator is required.

**Passive:** Twol uses the PANDA position data to shift the vehicle's Pure Pursuit goal point, steering the tractor to drag the implement onto the AB line. PGN 233 is not sent. The firmware receives no guidance commands and outputs no PWM — it is a GPS/PANDA relay only. No actuator required; one fitted to the implement sits idle.

| Mode | PGN 233 sent? | Firmware actuator output | Actuator required? |
|---|---|---|---|
| Active | Yes | Drives to zero XTE | Yes |
| Passive | No | None | No |
| Follow Pivot | Yes | Drives to zero XTE | Yes |

No WAS. No wheel angle. Completely separate from vehicle autosteer.

### 5.2 Shared files — no changes needed

| File | Notes |
|---|---|
| `zNMEAParser.h` | NMEA parser — used by both modes |
| `EthernetUpdate.ino` | Firmware update — used by both modes |
| `Config.ino` | Config UDP handler — used by both modes |
| `Panda.ino` | GPS abstraction already applied in Part A |
| `GPS_KSXT.ino` | GPS abstraction already applied in Part A |

### 5.3 Files to add from ToolDual

All files are always compiled. Functions are called only when `MDL.SteeringMode == STEER_TOOL_XTE`.

| ToolDual File | Destination | Changes |
|---|---|---|
| `Toolsteer.ino` | `Toolsteer.ino` | Remove compile guards; adapt to MDL.SteeringMode runtime check |
| `ToolsteerPID.ino` | `ToolsteerPID.ino` | No logic changes |
| `zADS1115.h/.cpp` | `zADS1115.h/.cpp` | No changes — actuator position ADC |
| `zRelPos.ino` | **Not ported** | ByNav/UM982 handles dual antenna internally |
| `ToolDual.ino` RELPOSNED section | **Not ported** | Replaced by `GPS_KSXT.ino` |

### 5.4 Main loop runtime branching (`AutoSteerTeensy.ino`)

```cpp
void loop()
{
    if (millis() - LoopLast >= LOOP_TIME)  // 25 ms
    {
        LoopLast = millis();

        if (MDL.SteeringMode == STEER_WHEEL_ANGLE)
        {
            // Existing autosteer pipeline — unchanged
            ReadWAS();
            calcSteeringPID();
            motorDrive();
            SendSteerData();          // PGN 253
            ReceiveSteerData();       // PGN 254/252
        }
        else // STEER_TOOL_XTE
        {
            ReadActuatorPosition();   // ADS1115
            UpdateWatchdog();
            if (ToolSteerEnabled()) {
                if (toolSettings.isDirectionalValve)
                    BangBangDrive();
                else {
                    calcSteeringPID();
                    motorDrive(pwmDrive);
                }
            } else {
                motorDrive(0);
            }
            SendToolFeedback();       // PGN 230
        }

        ReceiveConfig();              // PGN 32300 — always
        Blink();
    }

    // Continuous (outside timed loop)
    DoPanda();                        // branches on MDL.GPSSource internally
    ReceiveToolSteerData();           // PGN 233 — runs always, acts only if STEER_TOOL_XTE
    ReceiveUpdate();
}
```

### 5.5 New `Tool_Settings` EEPROM struct

```cpp
// EEPROM offset 60 — loaded/saved unconditionally alongside SteerSettings
struct Tool_Settings {
    uint8_t  kP               = 40;    // Proportional gain
    uint8_t  kD               = 0;     // Derivative gain
    uint8_t  minPWM           = 20;    // Minimum PWM to overcome friction
    uint8_t  lowPWM           = 25;    // Low end of dynamic PWM range
    uint8_t  highPWM          = 100;   // Maximum PWM
    int16_t  zeroOffset_APOS  = 0;     // Actuator ADC zero point
    float    lowHighDistance  = 10.0f; // XTE threshold (cm) for dynamic PWM
    uint8_t  CytronDriver     = 1;     // 1=Cytron MD30C, 0=IBT2
    uint8_t  invertAPOS       = 0;     // Invert actuator position sense
    uint8_t  invertActuator   = 0;     // Invert motor direction
    uint8_t  maxActuatorLimit = 60;    // Hard stop at this % of travel
    uint8_t  isDirectionalValve = 0;   // 0=proportional, 1=bang-bang
    uint8_t  valveOnTime      = 5;     // Bang-bang: loop ticks energised per pulse (5 × 25ms = 125ms)
    uint8_t  valveOffTime     = 15;    // Bang-bang: loop ticks de-energised per pulse (15 × 25ms = 375ms)
};
```

### 5.6 PGN 233 — ToolSteerData (receive from AgIO)

```
Byte  0-1:  Header 0x80, 0x81
Byte  2:    0x7F
Byte  3:    233 (PGN)
Byte  4:    Length
Bytes 5-6:  Tool XTE (mm, int16_le) → divide by 10 → cm
Byte  7:    Guidance status (bit 0 = guidance enabled)
Bytes 8-9:  Vehicle XTE (mm, int16_le) — decoded in firmware, not used in control logic
Byte  10:   GPS speed (x10 knots)
Bytes 11-12 Manual PWM override (int16_le)
Byte  13:   CRC
```

### 5.7 PGN 230 — Feedback to AgIO (transmit)

```
Byte  0-1:  Header 0x80, 0x81
Bytes 2-3:  226, 230
Byte  4:    Length (8)
Bytes 5-6:  Actuator position percent (int16_le)
Bytes 7-8:  PWM drive display (int16_le)
Byte  9:    Switch byte
Bytes 10-12 Padding (0x00)
Byte  13:   CRC
```

### 5.8 Tool steering PID algorithm

```cpp
// In ToolsteerPID.ino::calcSteeringPID()

void calcSteeringPID()
{
    float errorAbs = fabsf(toolXTE_cm);

    // Proportional
    float pValue = toolSettings.kP * toolXTE_cm * 0.2f;

    // Dynamic PWM ceiling
    int16_t newMax;
    if (errorAbs < toolSettings.lowHighDistance)
        newMax = (int16_t)((errorAbs * lowHighPerCM) + toolSettings.lowPWM);
    else
        newMax = toolSettings.highPWM;

    // Derivative (every 11 loops to reduce noise)
    float dValue = 0;
    if (++derivCount >= 11) {
        derivCount = 0;
        dValue = (toolXTE_cm - lastXTE_Error) * 30.0f * (toolSettings.kD * 0.1f);
        lastXTE_Error = toolXTE_cm;
    }

    pwmDrive = (int16_t)(pValue + dValue);

    if (pwmDrive >  newMax) pwmDrive =  newMax;
    if (pwmDrive < -newMax) pwmDrive = -newMax;

    if      (pwmDrive > 0) pwmDrive += toolSettings.minPWM;
    else if (pwmDrive < 0) pwmDrive -= toolSettings.minPWM;
}
```

### 5.9 Bang-bang directional valve

```cpp
void BangBangDrive()
{
    static uint32_t valveTimer = 0;
    static bool valveState = false;

    if (fabsf(toolXTE_cm) < 0.5f) {
        motorDrive(0);
        return;
    }

    int8_t dir = (toolXTE_cm > 0) ? 1 : -1;

    if (fabsf(toolXTE_cm) >= toolSettings.lowHighDistance) {
        motorDrive(255 * dir);
    } else {
        uint32_t now = millis();
        uint32_t period = valveState ? toolSettings.valveOnTime
                                     : toolSettings.valveOffTime;
        if (now - valveTimer >= period) {
            valveTimer = now;
            valveState = !valveState;
        }
        motorDrive(valveState ? (255 * dir) : 0);
    }
}
```

### 5.10 Actuator position feedback (ADS1115)

```cpp
// In Analog.ino — always compiled, called only when STEER_TOOL_XTE
void ReadActuatorPosition()
{
    if (!ADSfound) return;

    Wire.beginTransmission(ADS1115_Address);
    Wire.write(0x01);           // config register
    Wire.write(0xC1);           // AIN0 single-ended, start conversion
    Wire.write(0x80);
    Wire.endTransmission();

    delay(8);

    Wire.beginTransmission(ADS1115_Address);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom((int)ADS1115_Address, 2);

    int16_t raw = (Wire.read() << 8) | Wire.read();

    // Mid-scale at 6805 counts (~2.5V); ±6800 counts = ±100% travel
    actuatorPosition = (raw >> 1) - 6805 + toolSettings.zeroOffset_APOS;
    actuatorPositionPercent = (float)actuatorPosition / 68.0f;

    if (toolSettings.invertAPOS)
        actuatorPositionPercent = -actuatorPositionPercent;

    actuatorAtLimit = (fabsf(actuatorPositionPercent) > toolSettings.maxActuatorLimit);
}
```

### 5.11 Watchdog

```cpp
// 250-loop timeout at 40 Hz = 6.25 seconds without a PGN 233

#define WATCHDOG_THRESHOLD    200
#define WATCHDOG_FORCE_VALUE  250

static uint16_t watchdogTimer = WATCHDOG_FORCE_VALUE;

void UpdateWatchdog()
{
    if (++watchdogTimer > 250)
        watchdogTimer = WATCHDOG_FORCE_VALUE;
}

bool ToolSteerEnabled()
{
    return (watchdogTimer < WATCHDOG_THRESHOLD) && (guidanceStatus == 1);
}

// In ReceiveToolSteerData() — reset when valid PGN 233 arrives:
// watchdogTimer = 0;
```

---

## 5.12 Twol ↔ Firmware Protocol: Full Analysis

### Passive / Active / Follow — Architecture

**The mode distinction is entirely in Twol. The firmware is completely mode-agnostic.**

Twol has three mutually exclusive tool steering modes (UI checkboxes in `FormToolSteer.cs`):

| Mode | Flag | Correction mechanism | Actuator needed? |
|---|---|---|---|
| **Active** | `isFollowCurrent` | Tool XTE → PGN 233 → implement firmware → actuator | Yes |
| **Passive** | `isPassiveSteering` | Tool XTE → shifts vehicle Pure Pursuit goal point → tractor steers | No |
| **Follow Pivot** | `isFollowPivot` | Tool position vs recorded vehicle pivot history → PGN 233 → actuator | Yes |

**Active and Follow Pivot** send XTE via PGN 233 to the implement firmware, which drives the actuator to zero it. The firmware is mode-agnostic — it acts on whatever signed XTE value arrives.

**Passive steering** works differently. In `CGuidance.cs` (Pure Pursuit branch only), the tool antenna distance from the AB line (`distanceFromCurrentLineTool`) is fed into a proportional-integral accumulator (`passiveDistance`). That accumulated offset is then applied directly to the vehicle's Pure Pursuit goal point:

```csharp
goalPoint.easting  += (Math.Sin(curList[B].heading + 1.57) * passiveDistance);
goalPoint.northing += (Math.Cos(curList[B].heading + 1.57) * passiveDistance);
```

The `+1.57` rotates the AB heading 90° to get the perpendicular direction. The vehicle's `steerAngle` is computed from this shifted goal point — so the **tractor** is steered to drag the implement back onto the AB line. The implement follows passively with no actuator of its own.

Passive steering is Pure Pursuit only — Stanley has no equivalent goal-point shift mechanism.

Passive mode activates after a UTurn: `isPassiveTriggered` is set on UTurn entry, and correction only begins (`isPassiveSteeringFlag = true`) once the vehicle re-acquires the line (heading error < 1.5°, vehicle XTE < 10 cm, tool XTE < 20 cm).

### PGN 233 is not sent in passive mode

From `Position.designer.cs`:

```csharp
if (Settings.Tool.setToolSteer.isFollowCurrent || Settings.Tool.setToolSteer.isFollowPivot)
{
    // pack and send PGN 233 ...
    SendUDPMessageTool(PGN_233.pgn, epModuleTool);
}
```

`isPassiveSteering` is absent from this condition. PGN 233 is never sent in passive mode — the implement firmware receives no guidance commands regardless of whether an actuator is fitted. The firmware's own watchdog reinforces this: without incoming PGN 233 packets the watchdog fires after ~6 seconds and forces `motorDrive(0)`.

### Summary: what the firmware does in each mode

| Mode | PGN 233 sent? | Firmware actuator output | Firmware GPS role |
|---|---|---|---|
| **Active** | Yes | Drives to zero XTE | PANDA relay → Twol computes tool XTE |
| **Passive** | No | None — watchdog forces 0 | PANDA relay → Twol shifts vehicle goal point |
| **Follow Pivot** | Yes | Drives to zero XTE | PANDA relay → Twol computes tool XTE |

In passive mode the firmware is a GPS/PANDA relay only. An actuator fitted to the implement in passive mode sits completely idle.

**Implement GPS is required in all three modes** — Twol reads PANDA to compute `distanceFromCurrentLineTool` regardless of whether the correction goes to an actuator or to the vehicle goal point.

**Firmware impact: none.** No firmware changes are needed to support any of the three modes. The mode distinction is entirely in Twol.

---

### PGN 233 → PGN 230: Request / Response Cycle

PGN 230 is **not sent on a timer**. It is sent immediately inside the PGN 233 handler — every incoming guidance packet triggers one feedback packet:

```
Twol (10 Hz)          Firmware
  │──── PGN 233 ───────▶│  toolXTE, vehicleXTE, guidanceStatus, manualPWM
  │                      │  → calcSteeringPID() → motorDrive()
  │◀─── PGN 230 ─────────│  actuatorPositionPercent, pwmDisplay, switchByte
```

At 10 Hz this gives Twol a fresh actuator position read every 100 ms, which is sufficient for the `isZeroToolSteer` centering loop.

---

### PGN 230 Field Usage in Twol

From `UDPComm.Designer.cs`:

```csharp
case 230:
    if (length != 14) break;
    mc.actualActuatorPositionPercent = (Int16)((data[6] << 8) + data[5]);
    mc.pwmToolDisplay                = (Int16)((data[8] << 8) + data[7]);
    // data[9] (switchByte) — parsed but not acted on
    break;
```

| PGN 230 bytes | Firmware source | Twol usage |
|---|---|---|
| 5–6 `actuatorPositionPercent` | ADS1115 scaled to ±100% | `isZeroToolSteer` stop condition; OpenGL % display |
| 7–8 `pwmDisplay` | Current `pwmDrive` value | OpenGL PWM bar graph |
| 9 `switchByte` | Work/steer/remote switches | Received, not acted on |

---

### `actualActuatorPositionPercent` Closes the Centering Loop

This is the only PGN 230 field that feeds back into Twol control logic:

```csharp
// Position.designer.cs — isZeroToolSteer mode
distX1000 = (Int16)(mc.actualActuatorPositionPercent < 0
    ? manualSteerPWM : -manualSteerPWM);

// Stop when within 5% of centre — read from firmware PGN 230
if (Math.Abs(mc.actualActuatorPositionPercent) < 5)
    gydTool.isZeroToolSteer = false;
```

---

### `vehicleXTE_cm` — Received in Firmware, Never Used

Twol always computes and sends vehicle XTE (vehicle pivot distance to AB line) in PGN 233 bytes 8–9. The firmware decodes it:

```cpp
vehicleXTE_cm = (float)(udpPacket.udpData[xteVehLo] |
                ((int8_t)udpPacket.udpData[xteVehHi]) << 8) * 0.1f;
```

It is never referenced again — not in `calcSteeringPID()`, not in `motorDrive()`. Reserved for potential future use; no control logic needed.

---

### Hello / Connection Heartbeat

```
Twol timer tick         Firmware
  │  traffic.helloFromAutoSteer++  (each tick)
  │──── Hello PGN 200 ────▶│
  │◀─── Hello PGN 226 ──────│  actuatorPositionPercent, raw ADC, switchByte
  │  traffic.helloFromAutoSteer = 0
  │  if counter < 3 → green indicator
  │  if counter ≥ 3 → red indicator
```

---

### Known Bug: Switch Byte Always Zero in PGN 230

In `ToolDual/Toolsteer.ino` the switch byte is set correctly then immediately overwritten:

```cpp
PGN_230[9]  = switchByte;   // ← set correctly (line 388)
PGN_230[10] = 0;
PGN_230[9]  = 0;            // ← overwrites switchByte! (line 391)
PGN_230[12] = 0;
```

**Fix when porting to this repo:**

```cpp
PGN_230[9]  = switchByte;   // correct — do not touch
PGN_230[10] = 0;
PGN_230[11] = 0;
PGN_230[12] = 0;
```

---

### Implication: Passive mode already works without an actuator

Actuator-free implement correction is already implemented in Twol's passive mode — no firmware changes required and no future Twol changes needed. The tractor steers to compensate for implement drift entirely within the existing codebase. Active and Follow Pivot modes require a physical actuator; passive mode does not.

---

## 6. Shared Code — No Special Strategy Needed

Because all modes live in one repo and one binary, shared infrastructure files (`zNMEAParser.h`, `EthernetUpdate.ino`, `Panda.ino`, `GPS_KSXT.ino`, `Config.ino`, `Begin.ino`) are compiled once and always present. No symlinks, no subtrees, no copy discipline required.

---

## 7. EEPROM Layout

| Address | Field | Notes |
|---|---|---|
| 0 | InoID (uint16) | Identifies firmware |
| 4 | InoType | Single value for unified binary |
| 10 | `SteerSettings` (11 bytes) | Wheel angle tuning — always loaded/saved |
| 60 | `Tool_Settings` (17 bytes) | Actuator tuning — always loaded/saved |
| 40 | `SteerConfig` (9 bytes) | Autosteer tuning |
| 110 | `ModuleConfig` (28 + 2 bytes) | Shared — includes new `GPSSource` + `SteeringMode` bytes at end |
| 168 | `ModuleNetwork` (6 bytes) | Shared — IP config |

`InoType` is a single value for this unified binary — all four runtime configurations share the same value. The firmware update handler validates the binary matches the board hardware, not the runtime mode.

---

## 8. Runtime Configuration Matrix

All four combinations are configurations of the **same binary**. Select via AOG module config panel (PGN 32300).

| Config | `MDL.GPSSource` | `MDL.SteeringMode` | `MDL.ReceiverSerialPort` | Notes |
|---|---|---|---|---|
| **AutoSteer + F9P** | 0 (GPS_F9P_IMU) | 0 (WHEEL_ANGLE) | 4, 460800 baud | Existing behaviour — default |
| **AutoSteer + UM982** | 1 (GPS_KSXT) | 0 (WHEEL_ANGLE) | 4, 115200 baud | KSXT heading replaces IMU |
| **ToolSteer + F9P** | 0 (GPS_F9P_IMU) | 1 (TOOL_XTE) | 4, 460800 baud | Actuator required |
| **ToolSteer + UM982** | 1 (GPS_KSXT) | 1 (TOOL_XTE) | 4, 115200 baud | Actuator required |

To switch configuration: change `GPSSource` and/or `SteeringMode` in AOG module settings — no reflashing needed.

---

## 9. Implementation Order

### Phase 1 — GPS Abstraction in AOG_AutoSteer (lowest risk)
1. Add `GPS_F9P_IMU`/`GPS_KSXT` constants to `AutoSteerTeensy.ino`
2. Add `GPSSource` field to `ModuleConfig`; default 0
3. Update `Config.ino` to receive/save new field via PGN 32300
4. Wrap `Begin.ino` serial init in `if (MDL.GPSSource == GPS_KSXT) ... else ...`
5. Verify F9P + IMU build (GPSSource=0) compiles and functions identically
6. Add `GPS_KSXT.ino` file
7. Modify `Panda.ino` and `SteerComm.ino` for runtime GPS data source
8. Test on bench with ByNav module outputting KSXT sentences

### Phase 2 — UM982/ByNav hardware validation
1. Confirm 242mA @ 3.3V does not stress AZ1117 (or power via USB-C separately)
2. Verify adapter pin mapping against ByNav datasheet pin 15/16/18/19
3. Send `LOG KSXT ONTIME 0.1` and verify sentence arrives on Serial4
4. Check heading vs known bearing, roll on flat ground ≈ 0°

### Phase 3 — Tool steering mode (same repo)
1. Add `STEER_WHEEL_ANGLE`/`STEER_TOOL_XTE` constants and `SteeringMode` to `ModuleConfig`
2. Update `Config.ino` to receive/save `SteeringMode` via PGN 32300
3. Split main loop into runtime `if (MDL.SteeringMode == ...)` branches
4. Port `Toolsteer.ino`, `ToolsteerPID.ino` from ToolDual into repo (no compile guards)
5. Add `zADS1115.h/.cpp` from ToolDual
6. Create `ToolComm.ino` (PGN 230/233 handler)
7. Apply switch-byte fix (Section 5.12)

### Phase 4 — Integration testing
1. AutoSteer + UM982: drive test, verify heading in AOG matches compass
2. AutoSteer + F9P: regression — confirm existing behaviour unchanged
3. ToolSteer + UM982: bench test with Twol sending PGN 233, verify actuator moves
4. ToolSteer: verify PGN 230 actuator position feedback in Twol UI

---

## 10. Risk Register

| Risk | Likelihood | Mitigation |
|---|---|---|
| KSXT field order differs between ByNav and UM982 firmware versions | Medium | Log raw sentence on startup and verify field count |
| AZ1117 thermal overload at 242mA | Medium | Power ByNav via USB-C; use 3.3V on adapter for signal reference only |
| ByNav startup config sent every boot causes re-init delay | Low | Gate `SendKSXTConfig()` behind a first-boot EEPROM flag |
| PANDA sentence format mismatch (KSXT has no DGPS age) | Low | Hardcode `ageDGPS = "0"` in `BuildPandaFromKSXT()` |
| Adapter pin 15/16 mismatch with ByNav actual TX/RX | Medium | Verify with oscilloscope or By_Connect before first firmware flash |
| Both IMU libraries must be installed even for KSXT-only use | Low | Document in README; libraries are small and free |
| `ModuleConfig` struct size change breaks EEPROM read on old firmware | Low | New fields at end of struct; old firmware ignores extra bytes; new firmware defaults to 0 |
