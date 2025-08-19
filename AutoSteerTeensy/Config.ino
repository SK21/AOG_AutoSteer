
void ReceiveConfig()
{
	if (Ethernet.linkStatus() == LinkON)
	{
		uint16_t len = UDPconfig.parsePacket();
		if (len)
		{
			uint8_t PGNlength;
			byte Data[35];
			UDPconfig.read(Data, 35);

			uint16_t PGN = Data[1] << 8 | Data[0];
			switch (PGN)
			{
			case 32300:
				//      teensy steering config
				//0     HeaderLo    44
				//1     HeaderHi    126
				//2		power relay pin
				//3		steer relay pin
				//4     WAS pin
				//5     Current pin
				//6		Steer switch pin
				//7		Work switch pin
				//8		Dir pin
				//9		PWM pin
				//10    Receiver serial port
				//11	PassThr Out port
				//12	PassThru In port
				//13    IMU serial port
				//14    Commands
				//          - bit 0, zero WAS
				//          - bit 1, invert roll
				//          - bit 2, use ADS1115
				//			- bit 3, Auto zero WAS
				//15	IMU type	// 0 BNO080, 1 TM171
				//16	CRC

				PGNlength = 17;
				if (len > PGNlength - 1)
				{
					if (GoodCRC(Data, PGNlength))
					{
						MDL.PowerRelayPin = Data[2];
						MDL.SteeringRelayPin = Data[3];
						MDL.WasPin = Data[4];
						MDL.AnalogPin = Data[5];
						MDL.SteerSwitchPin = Data[6];
						MDL.WorkSwitchPin = Data[7];
						MDL.DirPin = Data[8];
						MDL.PWMpin = Data[9];
						MDL.ReceiverSerialPort = Data[10];
						MDL.PassThrOutSerialPort = Data[11];
						MDL.PassThruInSerialPort = Data[12];
						MDL.IMUSerialPort = Data[13];

						uint8_t Commands = Data[14];
						if (bitRead(Commands, 0)) MDL.ZeroOffset = WasReading - ADSoffset;
						if (bitRead(Commands, 1)) MDL.InvertRoll = true; else MDL.InvertRoll = false;
						if (bitRead(Commands, 2)) MDL.ADS1115Enabled = true; else MDL.ADS1115Enabled = false;
						if (bitRead(Commands, 3)) MDL.AutoZero = true; else MDL.AutoZero = false;

						MDL.IMUtype = Data[15];

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

			case 32504:
				// PGN32504, commands
				// 0	HeaderLo	248
				// 1	HeaderHi	126
				// 2	Command
				// 3	CRC
				PGNlength = 4;
				
				if (len > PGNlength - 1)
				{
					if (GoodCRC(Data, PGNlength))
					{
						if (bitRead(Data[2], 0)) SendStatus();
					}
				}
				break;
			}
		}
	}
}

void SendStatus()
{
	// PGN32505
	// 0	HeaderLo	249
	// 1	HeaderHi	126
	// 2	InoID Lo
	// 3	InoID Hi
	// 4	IMU heading Lo
	// 5	IMU heading Hi
	// 6	WAS reading Lo
	// 7	WAS reading Hi
	// 8	Analog reading Lo
	// 9	Analog reading Hi
	// 10	Status
	//			- bit 0, IMU enabled
	//			- bit 1, Receiver enabled
	//			- bit 2, Pass Thru enabled
	//			- bit 3, ADS1115 found
	//			- bit 4, AOG connected
	//			- bit 5, Steering On
	//			- bit 6, Steer switch On
	// 11	MaxLoopTime Lo
	// 12	MaxLoopTime Hi
	// 13	ZeroOffset Lo
	// 14	ZeroOffset Hi
	// 15	Current WAS Lo
	// 16	Current WAS Hi
	// 17	CRC

	const uint8_t PGNlength = 18;

	byte data[PGNlength];
	data[0] = 249;
	data[1] = 126;

	data[2] = (byte)InoID;
	data[3] = InoID >> 8;

	uint16_t heading = (int)IMU_Heading;
	data[4] = (byte)heading;
	data[5] = heading >> 8;

	data[6] = (byte)WasReading;
	data[7] = WasReading >> 8;

	uint16_t reading = (int)AnalogReadingAverage;
	data[8] = (byte)reading;
	data[9] = reading >> 8;

	byte status = 0;
	if (SerialIMUEnabled) status |= 0b00000001;
	if (SerialReceiverEnabled) status |= 0b00000010;
	if (SerialPassThruEnabled) status |= 0b00000100;
	if (ADSfound) status |= 0b00001000;
	if (millis() - AOGTime < 4000) status |= 0b00010000;
	if (AOGsteeringReady) status |= 0b00100000;
	if (SteerSwitch == LOW) status |= 0b01000000;

	data[10] = status;
	data[11] = (byte)MaxLoopTime;
	data[12] = MaxLoopTime >> 8;
	data[13] = (byte)MDL.ZeroOffset;
	data[14] = MDL.ZeroOffset >> 8;

	int16_t CurrentWas = WasReading - MDL.ZeroOffset - ADSoffset;
	data[15] = (byte)CurrentWas;
	data[16] = CurrentWas >> 8;

	data[17] = CRC(data, 17, 0);

	if (Ethernet.linkStatus() == LinkON)
	{
		UDPconfig.beginPacket(DestinationIP, ConfigDestinationPort);
		UDPconfig.write(data, PGNlength);
		UDPconfig.endPacket();
	}
}

