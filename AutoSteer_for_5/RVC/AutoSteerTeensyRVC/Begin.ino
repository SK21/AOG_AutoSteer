int16_t StoredID = 0;
uint8_t ErrorCount;
bool IMUstarted = false;

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
	}
	else
	{
		Serial.println("ADS1115 not found.");
		Serial.println("");

		Serial.println("Using Teensy pins for analog data.");
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

	Serial.println("");
	Serial.println("Finished setup.");
	Serial.println("");
}

//	AS14 config
//	uint8_t Receiver = 1;			// 0 none, 1 SimpleRTK2B, 2 Sparkfun F9p
//	uint8_t ReceiverSerialPort = 4;	// GPS receiver
//	uint8_t	IMUSerialPort = 7;		// IMU
//	uint16_t NtripPort = 2233;		// local port to listen on for NTRIP data
//	uint16_t ZeroOffset = 6500;
//	uint8_t MinSpeed = 1;
//	uint8_t MaxSpeed = 15;
//	uint16_t PulseCal = 255;		// Hz/KMH X 10
//	uint8_t SwapRollPitch = 0;		// 0 use roll value for roll, 1 use pitch value for roll
//	uint8_t InvertRoll = 0;
//	uint8_t Dir1 = 26;
//	uint8_t PWM1 = 25;
//	uint8_t SteeringRelay = 36;		// pin for steering disconnect relay
//	uint8_t SteerSw = 39;
//	uint8_t WorkSw = 27;
//	uint8_t CurrentSensor = 10;
//	uint8_t PressureSensor = 26;
//	uint8_t Encoder = 38;
//	uint8_t SpeedPulse = 37;
//	uint8_t IP0 = 192;
//	uint8_t IP1 = 168;
//	uint8_t IP2 = 1;
//	uint8_t IP3 = 126;
//	uint8_t PowerRelay = 0;			// pin for 12V out relay


//	AS15 config
//	uint8_t Receiver = 1;			// 0 none, 1 SimpleRTK2B, 2 Sparkfun F9p
//	uint8_t ReceiverSerialPort = 8;	// gps receiver
//	uint8_t	IMUSerialPort = 5;		// Adafruit 5, Sparkfun 4
//	uint16_t NtripPort = 2233;		// local port to listen on for NTRIP data
//	uint16_t ZeroOffset = 6500;
//	uint8_t MinSpeed = 1;
//	uint8_t MaxSpeed = 15;
//	uint16_t PulseCal = 255;		// Hz/KMH X 10
//	uint8_t SwapRollPitch = 0;		// 0 use roll value for roll, 1 use pitch value for roll
//	uint8_t InvertRoll = 0;
//	uint8_t Dir1 = 23;
//	uint8_t PWM1 = 22;
//	uint8_t SteeringRelay = 7;		// pin for steering disconnect relay
//	uint8_t SteerSw = 26;
//	uint8_t WorkSw = 27;
//	uint8_t CurrentSensor = 0;		// Ads1115
//	uint8_t PressureSensor = 0;	// Ads1115
//	uint8_t Encoder = 0;			// none
//	uint8_t SpeedPulse = 28;
//	uint8_t IP0 = 192;
//	uint8_t IP1 = 168;
//	uint8_t IP2 = 1;
//	uint8_t IP3 = 126;
//	uint8_t PowerRelay = 0;			// pin for 12V out relay


