// Bynav C2 PASHR sentence handler
// Active when MDL.GPSSource == GPS_ByNav

// $PASHR,024224.00,37.186,T,0.000,-76.837,0.000,0.000,0.500,0.200,2*10
// 1    header      -   $PASHR
// 2    UTC Time    0   024224.00
// 3    heading     1   37.186
// 4    North       2   T
// 5    Roll        3   0.000
// 6    Pitch       4   0.000
// 7    Heave       5   0.000
// 8    Roll dev    6   0.000
// 9    Pitch dev   7   0.500
// 10   Yaw dev     8   0.200
// 11   status      9   2
// 12   check sum   10  10

// decimal degrees -> NMEA ddmm.mmmmm (+ hemisphere)
static void degToNMEA(double deg, char* out, char* hemi, bool isLat)
{
	hemi[0] = isLat ? (deg < 0 ? 'S' : 'N') : (deg < 0 ? 'W' : 'E');
	hemi[1] = '\0';
	deg = fabs(deg);
	int    d = (int)deg;
	double m = (deg - d) * 60.0;
	if (isLat) sprintf(out, "%02d%08.5f", d, m);   // e.g. 5325.45456
	else       sprintf(out, "%03d%08.5f", d, m);   // e.g. 10340.65736
}

static char    _ksxtBuf[160];
static uint8_t _ksxtIdx = 0;

void KSXT_Feed(char c)
{
	if (c == '$') _ksxtIdx = 0;
	if (_ksxtIdx < sizeof(_ksxtBuf) - 1) _ksxtBuf[_ksxtIdx++] = c;
	if (c == '\n')
	{
		_ksxtBuf[_ksxtIdx] = '\0';
		if (strncmp(_ksxtBuf, "$KSXT,", 6) == 0) ParseKSXT(_ksxtBuf);
		_ksxtIdx = 0;
	}
}

// $KSXT,YYYYMMDDHHMMSS.ss,lon,lat,alt,head,pitch,roll,spd(m/s),,posQual,hdgQual,sats,...*<32-bit CRC>
void ParseKSXT(char* s)
{
	char* f[22]; uint8_t n = 0; char* p = s + 6;   // skip "$KSXT,"
	f[n++] = p;
	while (*p && n < 22) { if (*p == ',' || *p == '*') { *p = '\0'; f[n++] = p + 1; } p++; }
	if (n < 12) return;                            // need through sats
	if (atoi(f[9]) < 1) return;                    // field[9] pos fix status: 0 = no fix

	// time: extract hhmmss.ss from YYYYMMDDHHMMSS.ss
	if (strlen(f[0]) >= 14) { strncpy(fixTime, f[0] + 8, 11); fixTime[11] = '\0'; }

	// position (KSXT is decimal degrees → NMEA)
	degToNMEA(atof(f[2]), latitude, latNS, true);
	degToNMEA(atof(f[1]), longitude, lonEW, false);

	strncpy(altitude, f[3], sizeof(altitude) - 1); altitude[sizeof(altitude) - 1] = '\0';
	fixQuality[0] = f[9][0]; fixQuality[1] = '\0';           // may need remap for RTK display
	strncpy(numSats, f[11], sizeof(numSats) - 1); numSats[sizeof(numSats) - 1] = '\0';
	strcpy(HDOP, "1.0");                                     // not in KSXT
	strcpy(ageDGPS, "0");                                    // not in KSXT

	// speed m/s → knots
	sprintf(speedKnots, "%.3f", atof(f[7]) * 1.94384f);

	// attitude → shared ATT_ globals (heading used by BuildPanda + SteerComm)
	ATT_Time = millis();
	ATT_Heading = atof(f[4]) * 10.0f;   // field[4] heading
	ATT_Pitch = atof(f[5]) * 10.0f;   // field[5] pitch
	ATT_Roll = atof(f[6]) * 10.0f;   // field[6] roll (0.00 flat = correct)
	itoa((int16_t)ATT_Roll, attRoll, 10);
	itoa((int16_t)ATT_Pitch, attPitch, 10);
	itoa(0, attYawRate, 10);

	BuildPanda();
}

void ByNavConfig()
{
	Serial.println("ByNav config ...");

	if (ByNavValueMatches())
	{
		Serial.println("ByNav saved config found.");
		// log config
		delay(200);
		SerialReceiver->println("LOG COM1 KSXT ONTIME 0.1");   delay(100);
	}
	else
	{
		Serial.println("ByNav full config required.");
		SendFullConfig();
	}


	Serial.println("ByNav config finished.");
}

bool ByNavValueMatches()
{
	while (SerialReceiver->available()) SerialReceiver->read();

	SerialReceiver->println("DUALANTENNAPOWER");

	uint32_t start = millis();
	char buf[96];
	uint8_t idx = 0;

	while (millis() - start < 1500)
	{
		while (SerialReceiver->available())
		{
			char c = SerialReceiver->read();

			if (c == '\n')
			{
				buf[idx] = 0;

				if (strstr(buf, "DUALANTENNAPOWER") && strstr(buf, " ON"))
				{
					return true;
				}

				idx = 0;
			}
			else if (c != '\r')
			{
				if (idx < sizeof(buf) - 1)
				{
					buf[idx++] = c;
				}
				else
				{
					idx = 0; // overflow protection
				}
			}
		}
	}

	return false;
}

void SendFullConfig()
{
	delay(200);
	SerialReceiver->println("UNLOGALL");                     delay(100);
	SerialReceiver->println("RTKTYPE ROVER");                delay(100);
	SerialReceiver->println("SET OBSFREQ 10");               delay(100);
	SerialReceiver->println("HEADINGOFFSET 90");             delay(100);
	SerialReceiver->println("DUALANTENNAPOWER ON");          delay(100);
	SerialReceiver->println("RTKTIMEOUT 500");               delay(100);
	SerialReceiver->println("INTERFACEMODE COM1 AUTO AUTO"); delay(100);
	SerialReceiver->println("SNRCUTOFF 15");                 delay(100);
	SerialReceiver->println("LOG COM1 KSXT ONTIME 0.1");     delay(100);
	SerialReceiver->println("SAVECONFIG");                   delay(1000);
	SerialReceiver->println("REBOOT");                       delay(8000);
}

