// ByNav C2-M20D-U / UM982 KSXT sentence parser                                                                     
// ReadKSXT() and BuildPandaFromKSXT() are called from Panda.ino
// SendKSXTConfig() is called from Begin.ino
// All functions are only called when MDL.GPSSource == GPS_KSXT

float    GPS_Heading = 0;    // degrees 0-360
float    GPS_Roll = 0;    // degrees
float    GPS_Pitch = 0;    // degrees
float    GPS_Speed = 0;    // km/h
float    GPS_Lat = 0;    // decimal degrees
float    GPS_Lon = 0;    // decimal degrees
float    GPS_Alt = 0;    // metres
uint8_t  GPS_Fix = 0;    // 0=none, ≥1=valid
uint8_t  GPS_HdgStatus = 0;    // 3=RTK dual fixed
uint8_t  GPS_Sats = 0;
float    GPS_AgeDGPS = 0;    // seconds since last DGPS correction
bool     GPS_DataValid = false;
uint32_t GPS_LastUpdate = 0;
char GPS_Time[12] = "000000.00";

static char    _ksxtBuf[160];
static uint8_t _ksxtIdx = 0;
static bool    _ksxtActive = false;

void ReadKSXT()
{
	if (!SerialReceiverEnabled) return;

	while (SerialReceiver->available())
	{
		char c = SerialReceiver->read();

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
	// verify NMEA checksum
	const char* star = strrchr(sentence, '*');
	if (!star || strlen(star) < 3) return;

	uint8_t calcCK = 0;
	for (const char* p = sentence + 1; p < star; p++)
		calcCK ^= (uint8_t)*p;

	uint8_t rxCK = (uint8_t)strtol(star + 1, NULL, 16);
	if (calcCK != rxCK) return;

	// copy and split into fields
	char buf[160];
	strncpy(buf, sentence + 6, sizeof(buf) - 1);  // skip "$KSXT,"
	buf[sizeof(buf) - 1] = '\0';

	char* fields[21];
	uint8_t nFields = 0;
	char* p = buf;
	fields[nFields++] = p;
	while (*p && nFields < 21)
	{
		if (*p == ',' || *p == '*') 
		{
			*p = '\0';
			fields[nFields++] = p + 1;
		}
		p++;
	}
	if (nFields < 13) return;

	// field[9] = fix status — reject if no position fix
	uint8_t fix = (uint8_t)atoi(fields[9]);
	if (fix < 1)
	{
		GPS_DataValid = false;
		return;
	}

	// extract HHMMSS.ss from YYYYMMDDHHMMSS.ss
	if (strlen(fields[0]) >= 15) strncpy(GPS_Time, fields[0] + 8, sizeof(GPS_Time) - 1);

	// KSXT field order: [1]=lon [2]=lat [3]=alt [4]=heading [5]=pitch [6]=speed_angle [7]=speed(km/h) [8]=roll
	GPS_Fix = fix;
	GPS_HdgStatus = (uint8_t)atoi(fields[10]);
	GPS_Lon = atof(fields[1]);
	GPS_Lat = atof(fields[2]);
	GPS_Alt = atof(fields[3]);
	GPS_Heading = atof(fields[4]);
	GPS_Pitch = atof(fields[5]);
	GPS_Speed = atof(fields[7]);             // already km/h per spec
	GPS_Roll = atof(fields[8]);
	GPS_Sats = (uint8_t)atoi(fields[12]);
	if (nFields > 19) GPS_AgeDGPS = atof(fields[19]);

	GPS_DataValid = true;
	GPS_LastUpdate = millis();
}

bool GPS_Connected()
{
	return (millis() - GPS_LastUpdate) < 4000;
}

void SendKSXTConfig(HardwareSerialIMXRT* port)
{
	delay(200);
	port->println("UNLOGALL");                     delay(100);
	port->println("RTKTYPE ROVER");                delay(100);
	port->println("WORKFREQS ALL ALL");            delay(100);
	port->println("SET OBSFREQ 10");               delay(100);
	port->println("HEADINGOFFSET 90");             delay(100);
	port->println("DUALANTENNAPOWER ON");          delay(100);
	port->println("RTKTIMEOUT 500");               delay(100);
	port->println("INTERFACEMODE COM1 AUTO AUTO"); delay(100);
	port->println("SNRCUTOFF 15");                 delay(100);
	port->println("LOG KSXT ONTIME 0.1");          delay(100);
	port->println("SAVECONFIG");                   delay(500);
}

  // SendKSXTConfig — one-time startup configuration for the ByNav/UM982 receiver.
  // Sent every boot; SAVECONFIG persists settings to receiver flash so re-sending
  // is safe and ensures a known state even after a factory reset.
  //
  // Command sequence:
  //   UNLOGALL            — silence any previously active log streams before
  //                         reconfiguring, preventing message collisions during setup
  //   RTKTYPE ROVER       — receiver acts as RTK rover (consumes corrections);
  //                         not a base station
  //   WORKFREQS ALL ALL   — enable all constellations and all frequency bands
  //                         (GPS L1/L2/L5, GLONASS, Galileo, BeiDou, etc.)
  //   SET OBSFREQ 10      — internal measurement engine runs at 10 Hz, matching
  //                         the KSXT output rate below
  //   HEADINGOFFSET 90    — the dual-antenna baseline is mounted perpendicular to
  //                         the direction of travel (antennas side-by-side across
  //                         the cab), so the raw baseline azimuth is 90° off from
  //                         forward; this offset rotates the output to true heading
  //   DUALANTENNAPOWER ON — enables power to the second antenna port
  //   RTKTIMEOUT 500      — receiver holds its last RTK solution for 500 s after
  //                         RTCM corrections stop arriving before downgrading fix
  //   INTERFACEMODE COM1 AUTO AUTO
  //                       — COM1 accepts any input protocol (RTCM3, NMEA, etc.)
  //                         and outputs any protocol; needed so NTRIP RTCM bytes
  //                         forwarded on TX4 are accepted without a fixed mode
  //   SNRCUTOFF 15        — exclude satellites with SNR below 15 dB-Hz; removes
  //                         weak signals that degrade RTK ambiguity resolution
  //   LOG KSXT ONTIME 0.1 — stream one KSXT sentence every 100 ms (10 Hz)
  //   SAVECONFIG          — write all settings to non-volatile flash; 500 ms delay
  //                         after this command allows the flash write to complete
  //
  // The 100 ms inter-command delays give the receiver time to parse and apply each
  // command before the next one arrives.


//$KSXT, YYYYMMDDHHMMSS.ss, lon, lat, alt, heading, pitch, speed_angle, speed, roll, fix, hdgStatus, sats, satsUsed, , , , velN, velE, velD, , * XX
//
//Field[0]  UTC time
//Field[1]  Longitude       decimal degrees
//Field[2]  Latitude        decimal degrees
//Field[3]  Altitude        metres
//Field[4]  Heading / Yaw     0–360°
//Field[5]  Pitch           degrees
//Field[6]  Speed angle     direction of travel vs True North, 0–360° (not used)
//Field[7]  Speed           km/h
//Field[8]  Roll            degrees
//Field[9]  Fix status      ≥1 = valid position
//Field[10] Heading status  3 = RTK dual antenna fixed
//Field[11] Satellites tracked
//Field[12] Satellites used
//Field[13]  East position   m (relative to base)
//Field[14]  North position  m (relative to base)
//Field[15]  Up position     m (relative to base)
//Field[16]  Velocity East   km / h
//Field[17]  Velocity North  km / h
//Field[18]  Velocity Down   km / h
//Field[19]  Age of differential  seconds → GPS_AgeDGPS
//Field[20]  Satellites at base station

//$KSXT, 20260510141923.00, -114.0000000, 51.0000000, 541.171, 124.191, -1.481, 0.241, 0.024, , 4, 3, 27, 24, , , , -0.010, 0.022, -0.00  1, , * XX
//
//Field breakdown matching GPS_KSXT.ino:94–104 :
//
//	┌─────────┬───────────────────┬─────────────────────────────────────────────────────────────┐
//	│  Index  │       Value       │                           Meaning                           │
//	├─────────┼───────────────────┼─────────────────────────────────────────────────────────────┤
//	│[0]      │ 20260510141923.00 │ UTC datetime — code extracts[8:] → 141923.00 as GPS_Time   │
//	├─────────┼───────────────────┼─────────────────────────────────────────────────────────────┤
//	│[1]      │ - 114.0000000     │ Longitude, decimal degrees → GPS_Lon                        │
//	├─────────┼───────────────────┼─────────────────────────────────────────────────────────────┤
//	│[2]      │ 51.0000000        │ Latitude, decimal degrees → GPS_Lat                         │
//	├─────────┼───────────────────┼─────────────────────────────────────────────────────────────┤
//	│[3]      │ 541.171           │ Altitude, metres → GPS_Alt                                  │
//	├─────────┼───────────────────┼─────────────────────────────────────────────────────────────┤
//	│[4]      │ 124.191           │ Heading / Yaw, 0–360° → GPS_Heading                           │
//	├─────────┼───────────────────┼─────────────────────────────────────────────────────────────┤
//	│[5]      │ - 1.481           │ Pitch, degrees → GPS_Pitch                                  │
//	├─────────┼───────────────────┼─────────────────────────────────────────────────────────────┤
//	│[6]      │ 0.241             │ Roll, degrees → GPS_Roll                                    │
//	├─────────┼───────────────────┼─────────────────────────────────────────────────────────────┤
//	│[7]      │ 0.024             │ Speed, m / s(multiplied ×3.6 → GPS_Speed in km / h)            │
//	├─────────┼───────────────────┼─────────────────────────────────────────────────────────────┤
//	│[8]      │(empty)            │ Reserved                                                    │
//	├─────────┼───────────────────┼─────────────────────────────────────────────────────────────┤
//	│[9]      │ 4                 │ Fix status — ≥1 sets GPS_Fix; 0 sets GPS_DataValid = false  │
//	├─────────┼───────────────────┼─────────────────────────────────────────────────────────────┤
//	│[10]     │ 3                 │ Heading status — 3 = RTK dual - antenna fixed → GPS_HdgStatus │
//	├─────────┼───────────────────┼─────────────────────────────────────────────────────────────┤
//	│[11]     │ 27                │ Satellites tracked → GPS_Sats                               │
//	├─────────┼───────────────────┼─────────────────────────────────────────────────────────────┤
//	│[12]     │ 24                │ Satellites used(parsed but not stored)                     │
//	├─────────┼───────────────────┼─────────────────────────────────────────────────────────────┤
//	│[13–15]  │(empty)            │ Reserved                                                    │
//	├─────────┼───────────────────┼─────────────────────────────────────────────────────────────┤
//	│[16]     │ - 0.010           │ Velocity North, km / h(parsed but not stored)                │
//	├─────────┼───────────────────┼─────────────────────────────────────────────────────────────┤
//	│[17]     │ 0.022             │ Velocity East, km / h(parsed but not stored)                 │
//	├─────────┼───────────────────┼─────────────────────────────────────────────────────────────┤
//	│[18]     │ - 0.001           │ Velocity Down, km / h(parsed but not stored)                 │
//	├─────────┼───────────────────┼─────────────────────────────────────────────────────────────┤
//	│[19]     │(empty)            │ Reserved                                                    │
//	└─────────┴───────────────────┴─────────────────────────────────────────────────────────────┘
