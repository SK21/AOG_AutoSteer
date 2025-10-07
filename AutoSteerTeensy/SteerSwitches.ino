
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
	static uint16_t EncoderCounts = 0;     // virtual pulses counted
	static float ve_accumDeg = 0.0f;       // accumulated degrees since last virtual pulse
	static float ve_prevError = 0.0f;      // previous steerAngleError sample
	static bool ve_initialized = false;    // first-sample flag
	static const float degreesPerCount = 5.0f;  // adjust sensitivity
	const float VE_MIN_DELTA = 0.05f;      // ignore jitter smaller than this (degrees)

	bool Result = true; // single exit point: will be returned at function end

	// --- CurrentSensor branch ---
	if (SteerConfig.CurrentSensor)
	{
		float SensorSample = (float)AnalogReadingValue;
		SensorSample = (512.0f - SensorSample) * 0.5f;
		if (SensorSample < 0.0f) SensorSample = 0.0f;
		AnalogReadingAverage = AnalogReadingAverage * 0.7f + SensorSample * 0.3f;
		if (AnalogReadingAverage > SteerConfig.PulseCountMax)
		{
			AnalogReadingAverage = 0.0f;
			Result = false;
		}
	}
	// --- PressureSensor branch ---
	else if (SteerConfig.PressureSensor)
	{
		float SensorSample = (float)AnalogReadingValue * 0.25f;
		AnalogReadingAverage = AnalogReadingAverage * 0.7f + SensorSample * 0.3f;
		if (AnalogReadingAverage > SteerConfig.PulseCountMax)
		{
			AnalogReadingAverage = 0.0f;
			Result = false;
		}
	}
	// --- Virtual encoder branch ---
	else if (SteerConfig.ShaftEncoder)
	{
		if (MDLready)
		{
			// process virtual encoder
			float currError = steerAngleError;     // signed degrees
			float absCurr = fabsf(currError);
			float absPrev = fabsf(ve_prevError);

			if (!ve_initialized)
			{
				// initialize previous sample to avoid spurious big delta
				ve_prevError = currError;
				ve_initialized = true;
			}
			else
			{
				// compute magnitude increase only
				float magDelta = absCurr - absPrev;
				if (magDelta > VE_MIN_DELTA)
				{
					ve_accumDeg += magDelta;
				}

				// update previous sample
				ve_prevError = currError;

				// convert accumulated degrees into virtual pulses
				while (ve_accumDeg >= degreesPerCount)
				{
					ve_accumDeg -= degreesPerCount;
					EncoderCounts++;
				}

				// if exceeded pulse threshold, disengage
				if (EncoderCounts >= SteerConfig.PulseCountMax)
				{
					EncoderCounts = 0;
					ve_accumDeg = 0.0f;
					Result = false;
				}
			}
		}
		else
		{
			// reset when module not ready
			EncoderCounts = 0;
			ve_accumDeg = 0.0f;
			ve_prevError = 0.0f;
			ve_initialized = false;
		}
	}
	return Result;
}
