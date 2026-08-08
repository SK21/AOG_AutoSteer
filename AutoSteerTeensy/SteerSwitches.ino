
// Counts edges on the work switch pin when it is wired to a steering column pulse sensor.
// 2 ms debounce caps counting at 500 edges/s - far above the ~32 edges/s a 16 pulse/rev
// sensor produces at one wheel turn per second, and low enough to reject electrical noise.
void WorkSwitchISR()
{
	uint32_t Now = micros();
	if (Now - WorkSwitchLastEdge < 2000) return;
	WorkSwitchLastEdge = Now;
	WorkSwitchEdges++;
}

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

	// With a kickout sensor on the pin the level is a pulse train, so reporting it would flick
	// AOG's work switch on and off at the pulse rate. Report the work switch permanently off.
	if (SteerConfig.WorkSwitchKickout) switchByte = 1;
	else switchByte = digitalRead(MDL.WorkSwitchPin);  // read work switch, Low on, High off

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
	// --- Steering wheel encoder branch ---
	// AOG's Shaft Encoder checkbox turns the kickout on; the WorkSwitchKickout wiring flag
	// picks the source - real pulses on the work switch pin, or the virtual encoder.
	else if (SteerConfig.ShaftEncoder)
	{
		if (SteerConfig.WorkSwitchKickout)
		{
			// Real pulses from the steering column sensor. WorkSwitchEdges free runs and is
			// never written here, so the WebUI can show it as a lifetime edge count for
			// checking the tap on the machine - turning the wheel moves it whether or not
			// steering is engaged. The kickout works from a moving baseline instead.
			// 32 bit reads are atomic on the Teensy, so no interrupt guard is needed.
			static uint32_t ws_base = 0;	// edge count at the start of the current window
			static uint32_t ws_last = 0;	// edge count at the previous sample
			static uint32_t ws_quiet = 0;	// millis() of the last new edge

			uint32_t Edges = WorkSwitchEdges;

			uint8_t Threshold = SteerConfig.PulseCountMax;
			if (Threshold < 1) Threshold = 1;	// 0 would disengage on every pass

			if (MDLready)
			{
				if (Edges != ws_last)
				{
					// still turning, keep accumulating
					ws_last = Edges;
					ws_quiet = millis();
				}
				else if (millis() - ws_quiet > 250)
				{
					// wheel stopped - drop the window, so stray edges cannot add up to
					// the threshold over minutes and kick out with nobody at the wheel
					ws_base = Edges;
				}

				// unsigned subtraction, stays correct across a counter wrap
				if (Edges - ws_base >= Threshold)
				{
					ws_base = Edges;
					ws_quiet = millis();
					Result = false;
				}
			}
			else
			{
				// hold the window at the current count while the module is not ready
				ws_base = Edges;
				ws_last = Edges;
				ws_quiet = millis();
			}
		}
		else if (MDLready)
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
