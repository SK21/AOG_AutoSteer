
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
		//5     -
		//6     -
		//7     Pulse Cal X 10, Lo
		//8     Pulse Cal X 10, Hi
		//9     relay control type  0 - no relays, 1 - GPIOs, 2 - PCA9555 8 relays, 3 - PCA9555 16 relays, 4 - MCP23017
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
	}
}
