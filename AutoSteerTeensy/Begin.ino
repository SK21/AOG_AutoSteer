
void DoSetup()
{
	uint8_t ErrorCount;

	pinMode(LED_BUILTIN, OUTPUT);

	Serial.begin(38400);
	delay(3000);
	Serial.println("");
	Serial.println("");
	Serial.println("");

	// eeprom
	LoadData();
	LoadNetworks();

	Serial.println("");
	Serial.println(InoDescription);

	// version
	uint16_t yr = InoID % 10 + 2020;
	uint16_t rest = InoID / 10;
	uint8_t mn = rest % 100;
	uint16_t dy = rest / 100;

	String fwVer;
	if (mn <= 12 && dy <= 31)
	{
		fwVer = "Firmware Version: v";
		fwVer += String(yr);
		fwVer += ".";
		if (mn < 10) fwVer += "0";
		fwVer += String(mn);
		fwVer += ".";
		if (dy < 10) fwVer += "0";
		fwVer += String(dy);
	}
	else
	{
		fwVer = "Firmware Version: invalid";
	}
	Serial.println(fwVer);

	// receiver
	uint32_t rxBaud = (MDL.GPSSource == GPS_KSXT) ? 115200 : ReceiverBaud;
	SerialReceiver = SetSerialPort(MDL.ReceiverSerialPort, rxBaud);
	SerialReceiverEnabled = (SerialReceiver != nullptr);
	if (SerialReceiverEnabled)
	{
		static char ReceiverBufferIn[512];
		static char ReceiverBufferOut[512];
		SerialReceiver->addMemoryForRead(ReceiverBufferIn, 512);
		SerialReceiver->addMemoryForWrite(ReceiverBufferOut, 512);

		Serial.println("");
		Serial.print("Connecting to receiver on serial port ");
		Serial.println(MDL.ReceiverSerialPort);
		Serial.println("");

		if (MDL.GPSSource == GPS_KSXT) SendKSXTConfig(SerialReceiver);
	}
	else
	{
		Serial.println("");
		Serial.print("Invalid Receiver Port: ");
		Serial.println(MDL.ReceiverSerialPort);
		Serial.println("");
	}

	parser.setErrorHandler(errorHandler);
	parser.addHandler("G-GGA", GGA_Handler);
	parser.addHandler("G-VTG", VTG_Handler);

	// pins
	if (MDL.WorkSwitchPin < NC) pinMode(MDL.WorkSwitchPin, INPUT_PULLUP);
	if (MDL.SteerSwitchPin < NC) pinMode(MDL.SteerSwitchPin, INPUT_PULLUP);
	if (MDL.SteeringRelayPin < NC) pinMode(MDL.SteeringRelayPin, OUTPUT);
	if (MDL.DirPin < NC) pinMode(MDL.DirPin, OUTPUT);

	if (MDL.PWMpin < NC)
	{
		pinMode(MDL.PWMpin, OUTPUT);
		analogWriteFrequency(MDL.PWMpin, MDL.PWMFrequency);
	}

	if (MDL.PowerRelayPin < NC) pinMode(MDL.PowerRelayPin, OUTPUT);
	if (MDL.EncoderPin < NC && SteerConfig.ShaftEncoder) pinMode(MDL.EncoderPin, INPUT_PULLUP);

	if (MDL.SpeedPulsePin < NC)
	{
		pinMode(MDL.SpeedPulsePin, OUTPUT);
		noTone(MDL.SpeedPulsePin);
	}

	Wire.begin();			// I2C on pins SCL 19, SDA 18
	Wire.setClock(400000);	//Increase I2C data rate to 400kHz

	// ADS1115
	ADSfound = false;
	if (MDL.ADS1115Enabled)
	{
		for (int i = 0; i < 2; i++)
		{
			if (i == 0) ADS1115_Address = 72; else ADS1115_Address = 73;

			ErrorCount = 0;
			Serial.print("Starting ADS1115 at address ");
			Serial.println(ADS1115_Address);
			while (!ADSfound)
			{
				Wire.beginTransmission(ADS1115_Address);
				Wire.write(0b00000000);	//Point to Conversion register
				Wire.endTransmission();
				ADSfound = (Wire.requestFrom(ADS1115_Address, 2) == 2);
				Serial.print(".");
				delay(500);
				if (ErrorCount++ > 5) break;
			}
			Serial.println("");
			if (ADSfound)
			{
				Serial.println("ADS1115 found.");
				Serial.println("");
				break;
			}
		}
		if (!ADSfound)
		{
			Serial.println("ADS1115 not found.");
			Serial.println("ADS1115 disabled.");
			Serial.println("");
		}
	}

	// analog pins
	analogReadResolution(12);

	// ethernet 
	Serial.println("Starting Ethernet ...");
	MDLnetwork.IP3 = 126;
	IPAddress LocalIP(MDLnetwork.IP0, MDLnetwork.IP1, MDLnetwork.IP2, MDLnetwork.IP3);
	static uint8_t LocalMac[] = { 0x00,0x0B,0x42,0x11,0x22,MDLnetwork.IP3 };

	Ethernet.begin(LocalMac, 0);
	Ethernet.setLocalIP(LocalIP);

	delay(1000);
	if (Ethernet.linkStatus() == LinkON)
	{
		Serial.println("Ethernet Connected.");
	}
	else
	{
		Serial.println("Ethernet Not Connected.");
	}
	Serial.print("IP Address: ");
	Serial.println(Ethernet.localIP());
	DestinationIP = IPAddress(MDLnetwork.IP0, MDLnetwork.IP1, MDLnetwork.IP2, 255);	// update from saved data

	// UDP
	UDPsteering.begin(ListeningPort);
	UDPconfig.begin(ConfigListeningPort);
	UDPntrip.begin(NtripPort);
	UpdateComm.begin(UpdateReceivePort);

	// GPS pass-through
	SerialPassIn = SetSerialPort(MDL.PassThruInSerialPort, PassThruBaud);
	SerialPassOut = SetSerialPort(MDL.PassThrOutSerialPort, PassThruBaud);
	SerialPassThruEnabled = (SerialPassIn != nullptr && SerialPassOut != nullptr);
	if (SerialPassThruEnabled)
	{
		Serial.println("");
		Serial.println("GPS pass-through enabled.");
	}
	else
	{
		Serial.println("");
		Serial.println("GPS pass-through disabled.");
	}

	// IMU
	if (MDL.GPSSource == GPS_F9P_IMU)
	{
		SerialIMU = SetSerialPort(MDL.IMUSerialPort, IMUBaud);
		SerialIMUEnabled = (SerialIMU != nullptr);
		if (SerialIMUEnabled)
		{
			switch (MDL.IMUtype)
			{
			case 0:
				// BNO080
				Serial.println("");
				Serial.println("Using BNO080 IMU.");
				rvc.begin(SerialIMU);
				break;

			case 1:
				// TM171
				Serial.println("");
				Serial.println("Using TM171 IMU.");
				break;
			}
		}
		else
		{
			Serial.println("");
			Serial.println("IMU serial port invalid. IMU disabled.");
		}
	}
	else
	{
		SerialIMUEnabled = false;
		Serial.println("");
		Serial.println("IMU disabled (KSXT mode).");
	}

	LoopLast = millis();

	Serial.println("");
	Serial.println("Finished setup.");
	Serial.println("");
}

// eeprom map:
// ID				0
// module type		4
// steer settings	10
// steer config		40
// module settings	110
// network			168

void LoadData()
{
	bool IsValid = false;
	int16_t StoredID;
	int8_t StoredType;
	EEPROM.get(0, StoredID);
	EEPROM.get(4, StoredType);
	if (StoredID == InoID && StoredType == InoType)
	{
		// load stored data
		Serial.println("Loading stored settings.");
		EEPROM.get(10, SteerSettings);
		EEPROM.get(40, SteerConfig);
		EEPROM.get(60, toolSettings);
		EEPROM.get(110, MDL);

		IsValid = ValidData();
	}

	if (!IsValid)
	{
		Serial.println("Stored settings not valid.");
		LoadDefaults();
		SaveData();
	}
}

void SaveData()
{
	// update stored data
	Serial.println("Updating stored data.");
	EEPROM.put(0, InoID);
	EEPROM.put(4, InoType);
	EEPROM.put(10, SteerSettings);
	EEPROM.put(40, SteerConfig);
	EEPROM.put(60, toolSettings);
	EEPROM.put(110, MDL);
}

bool ValidData()
{
	bool Result = true;
	if (MDL.SteeringRelayPin > 41 && MDL.SteeringRelayPin != NC)
	{
		Result = false;
	}
	else if (MDL.WasPin > 41 && MDL.WasPin != NC)
	{
		Result = false;
	}
	else if (MDL.DirPin > 41 && MDL.DirPin != NC)
	{
		Result = false;
	}
	else if (MDL.PWMpin > 41 && MDL.PWMpin != NC)
	{
		Result = false;
	}
	else if (MDL.EncoderPin > 41 && MDL.EncoderPin != NC)
	{
		Result = false;
	}
	else if (MDL.SpeedPulsePin > 41 && MDL.SpeedPulsePin != NC)
	{
		Result = false;
	}
	else if (MDL.PowerRelayPin > 41 && MDL.PowerRelayPin != NC)
	{
		Result = false;
	}
	else if (MDL.ReceiverSerialPort > 8)
	{
		Result = false;
	}
	else if (MDL.IMUSerialPort > 8)
	{
		Result = false;
	}

	return Result;
}

void LoadDefaults()
{
	Serial.println("Loading default settings.");

	// AS15-3
	MDL.ID = 0;
	MDL.ReceiverSerialPort = 4;
	MDL.IMUSerialPort = 3;
	MDL.PassThruInSerialPort = 8;
	MDL.PassThrOutSerialPort = 2;
	MDL.PowerRelayPin = 0;
	MDL.SteeringRelayPin = 1;
	MDL.SteerSwitchPin = 30;
	MDL.WorkSwitchPin = 31;
	MDL.WasPin = 25;
	MDL.AnalogPin = 26;
	MDL.DirPin = 23;
	MDL.PWMpin = 22;
	MDL.PWMFrequency = 490;
	MDL.EncoderPin = NC;
	MDL.SpeedPulsePin = NC;
	MDL.SpeedPulseCal = 255;
	MDL.ZeroOffset = 0;
	MDL.IMUtype = 0;
	MDL.ADS1115Enabled = false;
	MDL.AutoZero = false;
	MDL.GPSSource = 0;   // GPS_F9P_IMU
	MDL.SteeringMode = 0;   // STEER_WHEEL_ANGLE
}

void LoadNetworks()
{
	ModuleNetwork tmp;
	EEPROM.get(168, tmp);
	if (tmp.Identifier == 9876)
	{
		MDLnetwork = tmp;
	}
	else
	{
		// load network defaults
		MDLnetwork.Identifier = 9876;
		MDLnetwork.IP0 = 192;
		MDLnetwork.IP1 = 168;
		MDLnetwork.IP2 = 1;
		MDLnetwork.IP3 = 126;

		SaveNetworks();
	}
}

void SaveNetworks()
{
	EEPROM.put(168, MDLnetwork);
}

HardwareSerialIMXRT* SetSerialPort(uint8_t port, uint32_t baud)
{
	HardwareSerialIMXRT* NewPort = nullptr;

	switch (port)
	{
	case 1: NewPort = &Serial1; break;
	case 2: NewPort = &Serial2; break;
	case 3: NewPort = &Serial3; break;
	case 4: NewPort = &Serial4; break;
	case 5: NewPort = &Serial5; break;
	case 6: NewPort = &Serial6; break;
	case 7: NewPort = &Serial7; break;
	case 8: NewPort = &Serial8; break;
	default: NewPort = nullptr; break;
	}

	if (NewPort != nullptr) NewPort->begin(baud);
	return NewPort;
}
