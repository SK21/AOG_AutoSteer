int16_t StoredID = 0;
uint8_t IMUaddress;
uint8_t ErrorCount;

void DoSetup()
{
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
	if (StoredID == InoID)
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
		EEPROM.put(10, steerSettings);
		EEPROM.put(40, steerConfig);
		EEPROM.put(110, MDL);
	}


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

	// send gps corrections to receiver
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

	// Wemos D1 Mini serial port
	switch (MDL.WemosSerialPort)
	{
	case 1:
		SerialWemos = &Serial1;
		break;
	case 2:
		SerialWemos = &Serial2;
		break;
	case 3:
		SerialWemos = &Serial3;
		break;
	case 4:
		SerialWemos = &Serial4;
		break;
	case 5:
		SerialWemos = &Serial5;
		break;
	case 6:
		SerialWemos = &Serial6;
		break;
	case 7:
		SerialWemos = &Serial7;
		break;
	default:
		SerialWemos = &Serial8;
		break;
	}

	SerialWemos->begin(115200);

	static char SerialWemosSendBuffer[512];
	static char SerialWemosReadBuffer[512];
	SerialWemos->addMemoryForWrite(SerialWemosSendBuffer, 512);
	SerialWemos->addMemoryForRead(SerialWemosReadBuffer, 512);

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
	pinMode(MDL.SteerSw_Relay, OUTPUT);
	pinMode(MDL.Dir1, OUTPUT);
	pinMode(MDL.PWM1, OUTPUT);
	pinMode(MDL.SpeedPulse, OUTPUT);

	Wire.begin();			// I2C on pins SCL 19, SDA 18
	Wire.setClock(400000);	//Increase I2C data rate to 400kHz

	// ADS1115
	Serial.println("Starting ADS1115 ...");
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
		Serial.println("ADS1115 connected.");
		Serial.println("");
		MDL.AnalogMethod = 0;
	}
	else
	{
		Serial.println("ADS1115 not found.");
		Serial.println("");

		switch (MDL.AnalogMethod)
		{

		case 1:
			Serial.println("Using Teensy pins for analog data.");
			Serial.println("");
			break;

		case 2:
			Serial.println("Using Wemos D1 Mini for analog data.");
			Serial.println("");
			break;
		}
	}

	// ethernet 
	Serial.println("Starting Ethernet ...");
	IPAddress LocalIP(MDL.IP0, MDL.IP1, MDL.IP2, MDL.IP3);
	static uint8_t LocalMac[] = { 0x00,0x00,0x56,0x00,0x00,MDL.IP3 };

	Ethernet.begin(LocalMac, 0);
	Ethernet.setLocalIP(LocalIP);
	DestinationIP = IPAddress(MDL.IP0, MDL.IP1, MDL.IP2, 255);	// update from saved data

	Serial.print("IP Address: ");
	Serial.println(Ethernet.localIP());
	delay(1000);
	if (Ethernet.linkStatus() == LinkON)
	{
		Serial.println("Ethernet Connected.");
	}
	else
	{
		Serial.println("Ethernet Not Connected.");
	}
	Serial.println("");

	UDPsteering.begin(ListeningPort);
	UDPntrip.begin(MDL.NtripPort);
	UDPswitches.begin(ListeningPortSwitches);

	noTone(MDL.SpeedPulse);
	SteerSwitch = HIGH;

	// usb host
	myusb.begin();

	// IMU
	SerialIMU->begin(IMUBaud);
	static char IMUBufferIn[512];
	static char IMUBufferOut[512];
	SerialIMU->addMemoryForRead(IMUBufferIn, 512);
	SerialIMU->addMemoryForWrite(IMUBufferOut, 512);

	if (!StartIMU(MDL.IMU))	// try saved IMU first
	{
		for (int i = 1; i < 5; i++)
		{
			if (i != MDL.IMU)
			{
				if (StartIMU(i))
				{
					MDL.IMU = i;
					EEPROM.put(110, MDL);
					break;
				}
			}
		}
	}

	if (MDL.IMU == 0)
	{
		Serial.println("");
		Serial.println("IMU not found.");
	}


	Serial.println("");
	Serial.println("Finished setup.");
	Serial.println("");
}

bool StartIMU(byte ID)
{
	bool IMUstarted = false;
	ErrorCount = 0;
	switch (ID)
	{
	case 3:
		Serial.println("Starting  CMPS14 IMU  ...");
		while (!IMUstarted)
		{
			Wire1.beginTransmission(CMPS14_ADDRESS);
			IMUstarted = !Wire1.endTransmission();
			Serial.print(".");
			delay(500);
			if (ErrorCount++ > 10) break;
		}
		Serial.println("");
		if (IMUstarted)
		{
			Serial.println("CMPS14 IMU started.");
		}
		else
		{
			Serial.println("CMPS14 IMU failed to start.");
		}
		break;

	case 4:
		Serial.println("Starting  BNO RVC  ...");
		IMUstarted = rvc.begin(SerialIMU);
		Serial.println("");
		if (IMUstarted)
		{
			Serial.println("BNO RVC IMU started.");
		}
		else
		{
			Serial.println("BNO RVC IMU failed to start.");
		}
		break;
	}
	return IMUstarted;
}
