// PGN 230 buffer — from module to Twol                                                                             
// [0x80, 0x81, IP3, 230, 8, actualLo, actualHi, pwmLo, pwmHi, 0, 0, 0, 0, CRC]
static uint8_t PGN_230_buf[] = { 0x80, 0x81, 0, 230, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };

void ReceiveToolSteerData()
{
	static const int MaxReadBuffer = 100;

	if (Ethernet.linkStatus() == LinkON)
	{
		uint16_t len = UDPsteering.parsePacket();
		if (len)
		{
			byte Data[MaxReadBuffer];
			UDPsteering.read(Data, MaxReadBuffer);
			if (Data[0] == 0x80 && Data[1] == 0x81 && Data[2] == 0x7F)
			{
				switch (Data[3])
				{
				case 200:
					SendHelloReply();
					break;

				case 201:
					if (Data[4] == 5 && Data[5] == 201 && Data[6] == 201)
					{
						MDLnetwork.IP0 = Data[7];
						MDLnetwork.IP1 = Data[8];
						MDLnetwork.IP2 = Data[9];
						SaveNetworks();
						SCB_AIRCR = 0x05FA0004;
					}
					break;

				case 202:
					if (Data[4] == 3 && Data[5] == 202 && Data[6] == 202)
						SendScanIDreply();
					break;

				case 232:
					// tool settings from Twol
					if (len >= 20)
					{
						toolSettings.kP = Data[5];
						toolSettings.kD = Data[6];
						toolSettings.minPWM = Data[7];
						toolSettings.highPWM = Data[8];
						toolSettings.lowPWM = (uint8_t)((float)toolSettings.minPWM * 1.2f);
						toolSettings.zeroOffset_APOS = (int16_t)(Data[10] << 8 | Data[9]);
						toolSettings.lowHighDistance = (float)Data[11];
						toolSettings.CytronDriver = Data[12];
						toolSettings.invertAPOS = Data[13];
						toolSettings.invertActuator = Data[14];
						toolSettings.maxActuatorLimit = Data[15];
						toolSettings.isDirectionalValve = Data[16];
						toolSettings.valveOffTime = Data[17];
						toolSettings.valveOnTime = Data[18];
						SaveData();
					}
					break;

				case 233:
					// tool steer command from Twol
					// XTE and vehicle XTE sent as mm (×1000 in Twol), stored as cm here (÷10)
					if (len >= 14)
					{
						toolXTE_cm = (float)(int16_t)(Data[6] << 8 | Data[5]) / 10.0f;
						guidanceStatus = Data[7];
						vehicleXTE_cm = (float)(int16_t)(Data[9] << 8 | Data[8]) / 10.0f;
						Speed_KMH = Data[10] * 0.1f;
						manualPWM = (int16_t)(Data[12] << 8 | Data[11]);
						watchdogTimer = 0;
						SendToolFeedback();
					}
					break;
				}
			}
		}
	}
}

void SendToolFeedback()
{
	PGN_230_buf[2] = MDLnetwork.IP3;

	int16_t pos = (int16_t)actuatorPositionPercent;
	PGN_230_buf[5] = (uint8_t)pos;
	PGN_230_buf[6] = (uint8_t)(pos >> 8);

	PGN_230_buf[7] = (uint8_t)pwmDisplay;
	PGN_230_buf[8] = (uint8_t)(pwmDisplay >> 8);

	// bytes 9–12 remain 0

	int16_t CK_A = 0;
	for (uint8_t i = 2; i < sizeof(PGN_230_buf) - 1; i++)
		CK_A += PGN_230_buf[i];
	PGN_230_buf[sizeof(PGN_230_buf) - 1] = (uint8_t)CK_A;

	if (Ethernet.linkStatus() == LinkON)
	{
		UDPsteering.beginPacket(DestinationIP, DestinationPort);
		UDPsteering.write(PGN_230_buf, sizeof(PGN_230_buf));
		UDPsteering.endPacket();
	}
}
