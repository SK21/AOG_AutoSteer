
byte SerialMSB;
byte SerialLSB;
unsigned int SerialPGN;
byte SerialPGNlength;
byte SerialReceive[35];
bool PGNfound;

void ReceiveSerialConfig()
{
	if (Serial.available())
	{
		if (Serial.available() > 50)
		{
			// clear buffer and reset pgn
			while (Serial.available())
			{
				Serial.read();
			}
			SerialPGN = 0;
			PGNfound = false;
		}

		if (PGNfound)
		{
			if (Serial.available() > SerialPGNlength - 3)
			{
				for (int i = 2; i < SerialPGNlength; i++)
				{
					SerialReceive[i] = Serial.read();
				}
				ReadPGNs(SerialReceive, SerialPGNlength);

				// reset pgn
				SerialPGN = 0;
				PGNfound = false;
			}
		}
		else
		{
			switch (SerialPGN)
			{
			case 32300:
				SerialPGNlength = 12;
				PGNfound = true;
				break;

			case 32301:
				SerialPGNlength = 26;
				PGNfound = true;
				break;

			case 32303:
				SerialPGNlength = 3;
				PGNfound = true;
				break;

			case 32503:
				SerialPGNlength = 6;
				PGNfound = true;
				break;

			default:
				// find pgn
				SerialMSB = Serial.read();
				SerialPGN = SerialMSB << 8 | SerialLSB;

				SerialReceive[0] = SerialLSB;
				SerialReceive[1] = SerialMSB;

				SerialLSB = SerialMSB;
				break;
			}
		}
	}
}

void ReceiveEthernetConfig()
{
	byte Data[35];
	if (Ethernet.linkStatus() == LinkON)
	{
		uint16_t len = UDPconfig.parsePacket();
		if (len)
		{
			if (len > 35) len = 35;
			UDPconfig.read(Data, len);
			ReadPGNs(Data, len);
		}
	}
}

void ReadPGNs(byte Data[], uint16_t len)
{
	byte PGNlength;
	uint16_t PGN = Data[1] << 8 | Data[0];

	switch (PGN)
	{
	case 32300:
		//      teensy steering config
		//0     HeaderLo    44
		//1     HeaderHi    126
		//2     Receiver    0 none, 1 simpleRTK2B
		//3     Receiver serial port
		//4     IMU serial port
		//5     Min. speed
		//6     Max. speed
		//7     Pulse Cal X 10, Lo
		//8     Pulse Cal X 10, Hi
		//9     relay control type  0 - no relays, 1 - PCA9685, 2 - PCA9555 8 relays, 3 - PCA9555 16 relays, 4 - MCP23017, 5 - Teensy GPIO
		//10    Commands
		//          - bit 0, swap pitch for roll
		//          - bit 1, invert roll
		//          - bit 2, use 4-20 pressure sensor
		//          - bit 3, relay on signal
		//          - bit 4, zero WAS
		//11    CRC

		PGNlength = 12;
		if (len > PGNlength - 1)
		{
			if (GoodCRC(Data, PGNlength))
			{
				MDL.Receiver = Data[2];
				MDL.ReceiverSerialPort = Data[3];
				MDL.IMUSerialPort = Data[4];
				MDL.MinSpeed = Data[5];
				MDL.MaxSpeed = Data[6];
				MDL.PulseCal = Data[7] | Data[8] << 8;
				MDL.RelayControl = Data[9];

				uint8_t Commands = Data[10];
				if (bitRead(Commands, 0)) MDL.SwapRollPitch = 1; else MDL.SwapRollPitch = 0;
				if (bitRead(Commands, 1)) MDL.InvertRoll = 1; else MDL.InvertRoll = 0;
				if (bitRead(Commands, 2)) MDL.Use4_20 = 1; else MDL.Use4_20 = 0;
				if (bitRead(Commands, 3)) MDL.RelayOnSignal = 1; else MDL.RelayOnSignal = 0;
				if (bitRead(Commands, 4)) MDL.ZeroOffset = AINs.AIN0;

				EEPROM.put(110, MDL);
			}
		}
		break;

	case 32301:
		//      Steering pins
		//0     HeaderLo    45
		//1     HeaderHi    126
		//2     motor dir
		//3     motor pwm
		//4     power relay
		//5     steering relay
		//6     steer switch
		//7     work switch
		//8     speed pulse
		//9-24  relay pins
		//25    CRC

		PGNlength = 26;
		if (len > PGNlength - 1)
		{
			if (GoodCRC(Data, PGNlength))
			{
				MDL.Dir1 = Data[2];
				MDL.PWM1 = Data[3];
				MDL.PowerRelay = Data[4];
				MDL.SteeringRelay = Data[5];
				MDL.SteerSw = Data[6];
				MDL.WorkSw = Data[7];
				MDL.SpeedPulse = Data[8];

				for (int i = 0; i < 16; i++)
				{
					MDL.RelayPins[i] = Data[i + 9];
				}

				EEPROM.put(110, MDL);

				SCB_AIRCR = 0x05FA0004; //reset the Teensy   
			}
		}
		break;

	case 32303:
		//      Info request from PCBsetup
		//0     HeaderLo    47
		//1     HeaderHi    126
		//2     CRC

		PGNlength = 3;
		if (len > PGNlength - 1)
		{
			if (GoodCRC(Data, PGNlength))
			{
				SendStatus();
			}
		}
		break;

	case 32503:
		//      New IP
		//0     HeaderLo    247
		//1     HeaderHI    126
		//2     IP 0
		//3     IP 1
		//4     IP 2
		//5     CRC

		PGNlength = 6;
		if (len > PGNlength - 1)
		{
			if (GoodCRC(Data, PGNlength))
			{
				MDL.IP0 = Data[2];
				MDL.IP1 = Data[3];
				MDL.IP2 = Data[4];

				EEPROM.put(110, MDL);

				SCB_AIRCR = 0x05FA0004; //reset the Teensy   
			}
		}
		break;

	default:
		break;
	}
}

void SendStatus()
{
	//		PGN32302
	//0		HeaderLo	46
	//1		HeaderHi	126
	//2		InoID Lo
	//3		InoID Hi
	//4		Status byte
	//5		-
	//6		CRC

	byte data[7];
	data[0] = 46;
	data[1] = 126;
	data[2] = (byte)InoID;
	data[3] = InoID >> 8;

	byte Status = IMUstarted;
	if (ADSfound) Status = Status | 0b10;
	if (PCA9555PW_found || MCP23017_found) Status = Status | 0b100;
	if (digitalRead(MDL.SteerSw)) Status = Status | 0b1000;
	if (millis() - AOGTime < 4000) Status = Status | 0b10000;
	if (millis() - ReceiverTime < 4000) Status = Status | 0b100000;
	if (millis() - NtripTime < 4000) Status = Status | 0b1000000;
	if (Ethernet.linkStatus() == LinkON) Status = Status | 0b10000000;

	data[4] = Status;
	data[5] = 0;
	data[6] = CRC(data, 6, 0);

	// to serial
	Serial.print(data[0]);
	for (int i = 1; i < 7; i++)
	{
		Serial.print(",");
		Serial.print(data[i]);
	}
	Serial.println("");

	// to ethernet
	if (Ethernet.linkStatus() == LinkON)
	{
		static uint8_t ipDest[] = { 255,255,255,255 };
		UDPsteering.beginPacket(ipDest, ConfigDestinationPort);
		UDPsteering.write(data, sizeof(data));
		UDPsteering.endPacket();
	}
}
