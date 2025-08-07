
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

	Serial.println("");
	Serial.println(InoDescription);

	// version
	uint16_t yr = InoID % 10 + 2020;
	uint16_t rest = InoID / 10;
	uint8_t mn = rest % 100;
	uint16_t dy = rest / 100;

	Serial.print("Module Version: v");
	Serial.print(yr);
	Serial.print(".");
	Serial.print(mn);
	Serial.print(".");
	Serial.println(dy);

	// receive data from gps receiver
	switch (MDL.ReceiverSerialPort)
	{
	case 1:
		SerialReceiver = &Serial1;
		break;
	case 2:
		SerialReceiver = &Serial2;
		break;
	case 3:
		SerialReceiver = &Serial3;
		break;
	case 4:
		SerialReceiver = &Serial4;
		break;
	case 5:
		SerialReceiver = &Serial5;
		break;
	case 6:
		SerialReceiver = &Serial6;
		break;
	case 7:
		SerialReceiver = &Serial7;
		break;
	default:
		SerialReceiver = &Serial8;
		break;
	}

	// Receiver
	SerialReceiver->begin(ReceiverBaud);
	static char ReceiverBufferIn[512];
	static char ReceiverBufferOut[512];
	SerialReceiver->addMemoryForRead(ReceiverBufferIn, 512);
	SerialReceiver->addMemoryForWrite(ReceiverBufferOut, 512);

	parser.setErrorHandler(errorHandler);
	parser.addHandler("G-GGA", GGA_Handler);
	parser.addHandler("G-VTG", VTG_Handler);

	Serial.print("Connecting to receiver on serial port ");
	Serial.println(MDL.ReceiverSerialPort);
	Serial.println("");

	// pins
	pinMode(MDL.WorkSwitchPin, INPUT_PULLUP);
	pinMode(MDL.SteerSwitchPin, INPUT_PULLUP);
	pinMode(MDL.SteeringRelayPin, OUTPUT);
	pinMode(MDL.DirPin, OUTPUT);
	pinMode(MDL.PWMpin, OUTPUT);

	SteerSwitch = HIGH;

	Wire.begin();			// I2C on pins SCL 19, SDA 18
	Wire.setClock(400000);	//Increase I2C data rate to 400kHz

	// ADS1115
	if (MDL.ADS1115Enabled)
	{
		ADSfound = false;
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
				Wire.requestFrom(ADS1115_Address, 2);
				ADSfound = Wire.available();
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
	IPAddress LocalIP(MDL.IP0, MDL.IP1, MDL.IP2, MDL.IP3);
	static uint8_t LocalMac[] = { 0x00,0x00,0x56,0x00,0x00,MDL.IP3 };

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
	DestinationIP = IPAddress(MDL.IP0, MDL.IP1, MDL.IP2, 255);	// update from saved data
	Serial.println("");

	UDPsteering.begin(ListeningPort);
	UDPconfig.begin(ConfigListeningPort);
	UDPntrip.begin(NtripPort);

	// update
	UpdateComm.begin(UpdateReceivePort);

	// GPS pass-through
	switch (MDL.PassThruInSerialPort)
	{
	case 1:
		SerialPassIn = &Serial1;
		break;
	case 2:
		SerialPassIn = &Serial2;
		break;
	case 3:
		SerialPassIn = &Serial3;
		break;
	case 4:
		SerialPassIn = &Serial4;
		break;
	case 5:
		SerialPassIn = &Serial5;
		break;
	case 6:
		SerialPassIn = &Serial6;
		break;
	case 7:
		SerialPassIn = &Serial7;
		break;
	default:
		SerialPassIn = &Serial8;
		break;
	}
	SerialPassIn->begin(PassThruBaud);

	switch (MDL.PassThrOutSerialPort)
	{
	case 1:
		SerialPassOut = &Serial1;
		break;
	case 2:
		SerialPassOut = &Serial2;
		break;
	case 3:
		SerialPassOut = &Serial3;
		break;
	case 4:
		SerialPassOut = &Serial4;
		break;
	case 5:
		SerialPassOut = &Serial5;
		break;
	case 6:
		SerialPassOut = &Serial6;
		break;
	case 7:
		SerialPassOut = &Serial7;
		break;
	default:
		SerialPassOut = &Serial8;
		break;
	}
	SerialPassOut->begin(PassThruBaud);

	// IMU
	switch (MDL.IMUSerialPort)
	{
	case 1:
		SerialIMU = &Serial1;
		break;
	case 2:
		SerialIMU = &Serial2;
		break;
	case 3:
		SerialIMU = &Serial3;
		break;
	case 4:
		SerialIMU = &Serial4;
		break;
	case 5:
		SerialIMU = &Serial5;
		break;
	case 6:
		SerialIMU = &Serial6;
		break;
	case 7:
		SerialIMU = &Serial7;
		break;
	default:
		SerialIMU = &Serial8;
		break;
	}

	SerialIMU->begin(IMUBaud);

	switch (MDL.IMUtype)
	{
	case 0:
		// BNO080
		Serial.println("Using BNO080 IMU.");
		rvc.begin(SerialIMU);
		break;

	case 1:
		// TM171
		Serial.println("Using TM171 IMU.");
		break;
	}

	Serial.println("");
	Serial.println("Finished setup.");
	Serial.println("");
}

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
		EEPROM.get(110, MDL);

		IsValid = ValidData();
		if (!IsValid)
		{
			Serial.println("Stored settings not valid.");
		}
	}

	if (!IsValid)
	{
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
	EEPROM.put(110, MDL);
}

bool ValidData()
{
	bool Result = true;
	if (MDL.SteeringRelayPin > 41)
	{
		Result = false;
	}
	else if (MDL.WasPin > 41)
	{
		Result = false;
	}
	else if (MDL.DirPin > 41)
	{
		Result = false;
	}
	else if (MDL.PWMpin > 41)
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
	MDL.PowerRelayPin = 0;
	MDL.SteeringRelayPin = 1;
	MDL.WasPin = 25;
	MDL.CurrentPin = 26;
	MDL.SteerSwitchPin = 30;
	MDL.WorkSwitchPin = 31;
	MDL.DirPin = 23;
	MDL.PWMpin = 22;
	MDL.ReceiverSerialPort = 8;
	MDL.PassThrOutSerialPort = 2;
	MDL.PassThruInSerialPort = 4;
	MDL.IMUSerialPort = 3;
	MDL.ZeroOffset = 0;
	MDL.InvertRoll = false;
	MDL.ADS1115Enabled = false;
	MDL.IP0 = 192;
	MDL.IP1 = 168;
	MDL.IP2 = 1;
	MDL.IP3 = 126;
	MDL.IMUtype = 0;
}


