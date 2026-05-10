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
	while (*p && nFields < 21) {
		if (*p == ',' || *p == '*') {
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

	// KSXT field order: [1]=lon [2]=lat [3]=alt [4]=heading [5]=pitch [6]=roll [7]=speed(m/s)
	GPS_Fix = fix;
	GPS_HdgStatus = (uint8_t)atoi(fields[10]);
	GPS_Lon = atof(fields[1]);
	GPS_Lat = atof(fields[2]);
	GPS_Alt = atof(fields[3]);
	GPS_Heading = atof(fields[4]);
	GPS_Pitch = atof(fields[5]);
	GPS_Roll = atof(fields[6]);
	GPS_Speed = atof(fields[7]) * 3.6f;   // m/s → km/h
	GPS_Sats = (uint8_t)atoi(fields[11]);

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


//$KSXT, YYYYMMDDHHMMSS.ss, lon, lat, alt, heading, pitch, roll, speed, , fix, hdgStatus, sats, satsUsed, , , , velN, velE, velD, , * XX
//
//Field[0]  UTC time
//Field[1]  Longitude       decimal degrees
//Field[2]  Latitude        decimal degrees
//Field[3]  Altitude        metres
//Field[4]  Heading / Yaw     0–360°
//Field[5]  Pitch           degrees
//Field[6]  Roll            degrees
//Field[7]  Speed           m / s
//Field[8](empty)
//Field[9]  Fix status      ≥1 = valid position
//Field[10] Heading status  3 = RTK dual antenna fixed
//Field[11] Satellites tracked
//Field[12] Satellites used
//Field[13 - 15](empty)
//Field[16]  Velocity North  km / h
//Field[17]  Velocity East   km / h
//Field[18]  Velocity Down   km / h
//Field[19](empty)
