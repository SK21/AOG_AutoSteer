
void ReceiveConfig()
{
	if (Ethernet.linkStatus() == LinkON)
	{
		uint16_t len = UDPconfig.parsePacket();
		if (len)
		{
			byte PGNlength;
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
				//15	-
				//16	CRC

				PGNlength = 17;
				if (len > PGNlength - 1)
				{
					if (GoodCRC(Data, PGNlength))
					{
						MDL.PowerRelayPin = Data[2];
						MDL.SteeringRelayPin = Data[3];
						MDL.WasPin = Data[4];
						MDL.CurrentPin = Data[5];
						MDL.SteerSwitchPin = Data[6];
						MDL.WorkSwitchPin = Data[7];
						MDL.DirPin = Data[8];
						MDL.PWMpin = Data[9];
						MDL.ReceiverSerialPort = Data[10];
						MDL.PassThrOutSerialPort = Data[11];
						MDL.PassThruInSerialPort = Data[12];
						MDL.IMUSerialPort = Data[13];

						uint8_t Commands = Data[14];
						if (bitRead(Commands, 0)) MDL.ZeroOffset = WasReading;
						if (bitRead(Commands, 1)) MDL.InvertRoll = true; else MDL.InvertRoll = false;
						if (bitRead(Commands, 2)) MDL.ADS1115Enabled = true; else MDL.ADS1115Enabled = false;

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
	}
}

