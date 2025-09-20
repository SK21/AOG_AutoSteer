
void ReadSwitches()
{
	// steer switch		- low, steering on 
	//					- high, steering off

	// AOGsteeringReady (from AOG)	- false, steering off
	//								- true, steering on

	static bool LatchedOff = false;		// keeps steering off after sensor shut-off until reset
	static bool ModuleSteeringReady = false;
	static uint8_t ReadingLast = HIGH;
	static const uint8_t DebounceTime = 50;	// ms
	static uint32_t LastTime;

	uint8_t SWreading = digitalRead(MDL.SteerSwitchPin);

	if (SteerConfig.SteerSwitch == 1)
	{
		// pushed on, released off
		// holds the GPIO pin either low(steering on) or high(steering off)
		if (SWreading)
		{
			// pin high, turn off
			ModuleSteeringReady = false;
			LatchedOff = false;	// reset LatchedOff
		}
		else
		{
			// pin low, turn on
			if (!LatchedOff) ModuleSteeringReady = true;
		}
	}
	else if (SteerConfig.SteerButton == 1)
	{
		// push and release - on, push again and release - off
		// momentary, grounds the GPIO pin

		if (SWreading != ReadingLast && (millis() - LastTime > DebounceTime))
		{
			if (SWreading == HIGH) ModuleSteeringReady = !ModuleSteeringReady;
			ReadingLast = SWreading;
			LastTime = millis();
		}
	}
	else
	{
		// no switch, match AOG status
		if (AOGsteeringReady)
		{
			if (!LatchedOff) ModuleSteeringReady = true;
		}
		else
		{
			ModuleSteeringReady = false;
			LatchedOff = false;	// reset LatchedOff
		}
	}

	if (LatchedOff || !SensorsSteeringReady(ModuleSteeringReady))
	{
		ModuleSteeringReady = false;
		LatchedOff = true;
	}

	switchByte = digitalRead(MDL.WorkSwitchPin);  // read work switch, Low on, High off

	if (ModuleSteeringReady)
	{
		SteerSwitch = LOW;
		//switchByte |= 0b00000000;
	}
	else
	{
		SteerSwitch = HIGH;
		switchByte |= 0b00000010;
	}
}

bool SensorsSteeringReady(bool MDLready)
{
	static uint16_t EncoderCounts = 0;
	static uint8_t EncoderRead;
	static const float MaxSteeringError = 5;	// degrees
	bool Result = true;

	if (SteerConfig.CurrentSensor)
	{
		float SensorSample = (float)AnalogReadingValue;
		SensorSample = (512.0 - SensorSample) * 0.5;
		if (SensorSample < 0) SensorSample = 0;
		AnalogReadingAverage = AnalogReadingAverage * 0.7 + SensorSample * 0.3;
		if (AnalogReadingAverage > SteerConfig.PulseCountMax)
		{
			AnalogReadingAverage = 0;
			Result = false;
		}
	}
	else if (SteerConfig.PressureSensor)
	{
		float SensorSample = (float)AnalogReadingValue * 0.25;
		AnalogReadingAverage = AnalogReadingAverage * 0.7 + SensorSample * 0.3;
		if (AnalogReadingAverage > SteerConfig.PulseCountMax)
		{
			AnalogReadingAverage = 0;
			Result = false;
		}
	}
	else if (SteerConfig.ShaftEncoder)
	{

		// virtual encoder based on steering error
		if (MDLready)
		{
			EncoderRead = abs(steerAngleError);
			EncoderCounts += (EncoderRead > MaxSteeringError);

			if (EncoderCounts >= SteerConfig.PulseCountMax)
			{
				EncoderCounts = 0;
				Result = false;
			}
		}
		else
		{
			EncoderCounts = 0;
		}
	}
	return Result;
}

