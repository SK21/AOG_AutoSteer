
void ReadSwitches()
{
	static bool LatchedOff = false;		// keeps steering off after sensor shut-off until reset
	static int ReadingLast = HIGH;		
	static const int DebounceTime = 50;	// ms
	static uint32_t LastTime;
	static bool ModuleSteeringReady = false;

	int SWreading = digitalRead(MDL.SteerSwitchPin);

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
			if(!LatchedOff) ModuleSteeringReady = true;
		}
		else
		{
			ModuleSteeringReady = false; 
			LatchedOff = false;	// reset LatchedOff
		}
	}

	if (!SensorsSteeringReady())
	{
		ModuleSteeringReady = false;
		LatchedOff = true;
	}
	
	switchByte = digitalRead(MDL.WorkSwitchPin);  // read work switch, Low on, High off

	if (ModuleSteeringReady)
	{
		SteerSwitch = LOW;
	}
	else
	{
		SteerSwitch = HIGH;
		switchByte |= 0b00000010;
	}
}

bool SensorsSteeringReady()
{
	bool Result = true;

	if (SteerConfig.CurrentSensor)
	{
		float SensorSample = (float)AnalogReadingValue;
		SensorSample = (512.0 - SensorSample) * 0.5;
		if (SensorSample < 0) SensorSample = 0;
		AnalogReadingAverage = AnalogReadingAverage * 0.7f + SensorSample * 0.3f;
		Result = (AnalogReadingAverage <= SteerConfig.PulseCountMax);
	}
	else if (SteerConfig.PressureSensor)
	{
		float SensorSample = (float)AnalogReadingValue * 0.25;
		AnalogReadingAverage = AnalogReadingAverage * 0.7 + SensorSample * 0.3;
		Result = (AnalogReadingAverage <= SteerConfig.PulseCountMax);
	}
	return Result;
}
