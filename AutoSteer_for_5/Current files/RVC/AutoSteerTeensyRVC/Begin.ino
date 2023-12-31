
void DoSetup()
{
	int ADS[] = { 0x48,0x49,0x4A,0x4B };	// ADS1115 addresses
	uint8_t ErrorCount;
	int16_t StoredID = 0;
	int8_t StoredType = 0;

	// watchdog timer
	WDT_timings_t config;
	config.timeout = 60;	// seconds
	wdt.begin(config);

	pinMode(LED_BUILTIN, OUTPUT);

	Serial.begin(38400);
	delay(5000);
	Serial.println();
	Serial.println(InoDescription);
	Serial.println();

	// eeprom data
	EEPROM.get(0, StoredID);              // read identifier
	EEPROM.get(4, StoredType);
	if (StoredID == InoID && StoredType == InoType)
	{
		// load stored data
		Serial.println("Loading stored settings.");
		EEPROM.get(10, steerSettings);
		EEPROM.get(40, steerConfig);
		EEPROM.get(110, MDL);
	}
	else
	{
		// update stored data
		Serial.println("Updating stored data.");
		EEPROM.put(0, InoID);
		EEPROM.put(4, InoType);
		EEPROM.put(10, steerSettings);
		EEPROM.put(40, steerConfig);
		EEPROM.put(110, MDL);
	}

	Serial.println("");
	Serial.print("Module Version: ");
	Serial.println(InoID);
	Serial.println("");

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

	// serial bno
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

	if (MDL.Receiver != 0)
	{
		SerialReceiver->begin(ReceiverBaud);
		static char ReceiverBufferIn[512];
		static char ReceiverBufferOut[512];
		SerialReceiver->addMemoryForRead(ReceiverBufferIn, 512);
		SerialReceiver->addMemoryForWrite(ReceiverBufferOut, 512);

		parser.setErrorHandler(errorHandler);
		parser.addHandler("G-GGA", GGA_Handler);
		parser.addHandler("G-VTG", VTG_Handler);
	}

	// pins
	pinMode(MDL.Encoder, INPUT_PULLUP);
	pinMode(MDL.WorkSw, INPUT_PULLUP);
	pinMode(MDL.SteerSw, INPUT_PULLUP);
	pinMode(MDL.SteeringRelay, OUTPUT);
	pinMode(MDL.Dir1, OUTPUT);
	pinMode(MDL.PWM1, OUTPUT);
	pinMode(MDL.SpeedPulse, OUTPUT);
	pinMode(MDL.PowerRelay, OUTPUT);

	noTone(MDL.SpeedPulse);
	SteerSwitch = HIGH;

	Wire.begin();			// I2C on pins SCL 19, SDA 18
	Wire.setClock(400000);	//Increase I2C data rate to 400kHz

	// ADS1115
	for (int i = 0; i < 4; i++)
	{
		ADS1115_Address = ADS[i];
		Serial.print("Starting ADS1115 at address ");
		Serial.print(ADS1115_Address);
		ErrorCount = 0;
		while (!ADSfound)
		{
			Wire.beginTransmission(ADS1115_Address);
			Wire.write(0b00000000);	//Point to Conversion register
			Wire.endTransmission();
			Wire.requestFrom(ADS1115_Address, 2);
			ADSfound = Wire.available();
			Serial.print(".");
			delay(500);
			if (ErrorCount++ > 10) break;
		}
		Serial.println("");
		if (ADSfound)
		{
			Serial.print("ADS1115 connected at address ");
			Serial.println(ADS1115_Address);
			Serial.println("");
			break;
		}
		else
		{
			Serial.print("ADS1115 not found.");
			Serial.println("");
		}
	}
	if (!ADSfound)
	{
		Serial.println("ADS1115 disabled.");
		Serial.println("");
	}

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
	UDPntrip.begin(MDL.NtripPort);
	UDPconfig.begin(ConfigListeningPort);

	// IMU
	SerialIMU->begin(IMUBaud);
	static char IMUBufferIn[512];
	static char IMUBufferOut[512];
	SerialIMU->addMemoryForRead(IMUBufferIn, 512);
	SerialIMU->addMemoryForWrite(IMUBufferOut, 512);

	Serial.println("Starting  BNO RVC  ...");
	rvc.begin(SerialIMU);
	ErrorCount = 0;
	while (!IMUstarted)
	{
		IMUstarted = rvc.read(&heading);
		Serial.print(".");
		delay(500);
		if (ErrorCount++ > 10) break;
	}
	Serial.println("");
	if (IMUstarted)
	{
		Serial.println("BNO RVC IMU started.");
	}
	else
	{
		Serial.println("BNO RVC IMU failed to start.");
	}

	// Relays
	switch (MDL.RelayControl)
	{
	case 2:
	case 3:
		// PCA9555 I/O expander on default address 0x20
		Serial.println("");
		Serial.println("Starting PCA9555 I/O Expander ...");
		ErrorCount = 0;
		while (!PCA9555PW_found)
		{
			Serial.print(".");
			Wire.beginTransmission(0x20);
			PCA9555PW_found = (Wire.endTransmission() == 0);
			ErrorCount++;
			delay(500);
			if (ErrorCount > 5) break;
		}

		Serial.println("");
		if (PCA9555PW_found)
		{
			Serial.println("PCA9555 expander found.");

			PCA.attach(Wire);
			PCA.polarity(PCA95x5::Polarity::ORIGINAL_ALL);
			PCA.direction(PCA95x5::Direction::OUT_ALL);
			PCA.write(PCA95x5::Level::H_ALL);
		}
		else
		{
			Serial.println("PCA9555 expander not found.");
		}
		Serial.println("");
		break;

	case 4:
		// MCP23017 I/O expander on default address 0x20
		Serial.println("");
		Serial.println("Starting MCP23017 ...");
		ErrorCount = 0;
		while (!MCP23017_found)
		{
			Serial.print(".");
			Wire.beginTransmission(0x20);
			MCP23017_found = (Wire.endTransmission() == 0);
			ErrorCount++;
			delay(500);
			if (ErrorCount > 5) break;
		}

		Serial.println("");
		if (MCP23017_found)
		{
			Serial.println("MCP23017 found.");
			MCP.begin_I2C();

			for (int i = 0; i < 16; i++)
			{
				MCP.pinMode(MDL.RelayPins[i], OUTPUT);
			}
		}
		else
		{
			Serial.println("MCP23017 not found.");
		}
		break;

	case 5:
		// Relay GPIO Pins
		for (int i = 0; i < 16; i++)
		{
			if (MDL.RelayPins[i] > 0)
			{
				pinMode(MDL.RelayPins[i], OUTPUT);
			}
		}
		break;
	}

	Serial.println("");
	Serial.println("Finished setup.");
	Serial.println("");
}

