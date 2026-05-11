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

void PASHR_Handler()
{
	char buf[12];

	// solution status field[9]: 0=invalid, 1=single point, 2=RTK
	parser.getArg(9, buf);
	if (atoi(buf) < 1) return;

	ATT_Time = millis();

	// heading: field[1], degrees True North 0-360
	parser.getArg(1, buf);
	ATT_Heading = atof(buf) * 10.0f;

	// roll: field[3], degrees
	parser.getArg(3, buf);
	ATT_Roll = atof(buf) * 10.0f;
	itoa((int16_t)ATT_Roll, attRoll, 10);

	// pitch: field[4], degrees
	parser.getArg(4, buf);
	ATT_Pitch = atof(buf) * 10.0f;
	itoa((int16_t)ATT_Pitch, attPitch, 10);

	itoa(0, attYawRate, 10);
}

void ByNavConfig()
{
	Serial.println("ByNav config ...");

	if (ByNavValueMatches())
	{
		Serial.println("ByNav saved config found.");
	}
	else
	{
		Serial.println("ByNav full config required.");
		SendFullConfig();
	}

	// log config
	delay(200);
	SerialReceiver->println("UNLOGALL");              delay(100);
	SerialReceiver->println("LOG GNGGA ONTIME 0.1");  delay(100);
	SerialReceiver->println("LOG GNVTG ONTIME 0.1");  delay(100);
	SerialReceiver->println("LOG PASHR ONTIME 0.1");  delay(100);

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
	SerialReceiver->println("WORKFREQS ALL ALL");            delay(100);
	SerialReceiver->println("SET OBSFREQ 10");               delay(100);
	SerialReceiver->println("HEADINGOFFSET 90");             delay(100);
	SerialReceiver->println("DUALANTENNAPOWER ON");          delay(100);
	SerialReceiver->println("RTKTIMEOUT 500");               delay(100);
	SerialReceiver->println("INTERFACEMODE COM1 AUTO AUTO"); delay(100);
	SerialReceiver->println("SNRCUTOFF 15");                 delay(100);
	SerialReceiver->println("SAVECONFIG");                   delay(1000);
	SerialReceiver->println("REBOOT");                       delay(8000);
}

