
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
		//9     -
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

				uint8_t Commands = Data[10];
				if (bitRead(Commands, 0)) MDL.SwapRollPitch = 1; else MDL.SwapRollPitch = 0;
				if (bitRead(Commands, 1)) MDL.InvertRoll = 1; else MDL.InvertRoll = 0;
				if (bitRead(Commands, 4)) MDL.ZeroOffset = AINs.AIN0;

				SaveData();
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
		//8     -
		//9-24  -
		//25    CRC

		PGNlength = 26;
		if (len > PGNlength - 1)
		{
			if (GoodCRC(Data, PGNlength))
			{
				MDL.Dir1 = Data[2];
				MDL.PWM1 = Data[3];
				MDL.SteeringRelay = Data[5];
				MDL.SteerSw = Data[6];
				MDL.WorkSw = Data[7];

				SaveData();

				SCB_AIRCR = 0x05FA0004; //reset the Teensy   
			}
		}
		break;

	case 32503:
		//PGN32503, Subnet change
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

				SaveData();

				// restart the Teensy
				SCB_AIRCR = 0x05FA0004;
			}
		}
		break;
	}
}

void SendStatus()
{
	//PGN32401, module info 
	   //0     145
	   //1     126
	   //2     250	Autosteer Module ID
	   //3     BuildDay
	   //4     BuildMonth
	   //5     BuildYear
	   //6     BuildType        0 - Teensy AutoSteer, 1 - Teensy Rate, 2 - Nano Rate, 3 - Nano SwitchBox, 4 - ESP Rate
	   //7     -
	   //8     -
	   //9     -
	   //10    -
	   //11    CRC

	PGNlength = 12;
	byte Data[PGNlength];

	Data[0] = 145;
	Data[1] = 126;
	Data[2] = 250;
	Data[3] = BuildDay;
	Data[4] = BuildMonth;
	Data[5] = BuildYear;
	Data[6] = BuildType;
	Data[7] = 0;
	Data[8] = 0;
	Data[9] = 0;
	Data[10] = 0;

	if (Ethernet.linkStatus() == LinkON)
	{
		// send ethernet
		UDPconfig.beginPacket(DestinationIP, ConfigDestinationPort);
		UDPconfig.write(Data, PGNlength);
		UDPconfig.endPacket();
	}
	else 
	{
		// send serial
		Serial.print(Data[0]);
		for (int i = 1; i < PGNlength; i++)
		{
			Serial.print(",");
			Serial.print(Data[i]);
		}
		Serial.println("");
	}
}
