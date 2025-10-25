void DoSteering()
{
	static const float LOW_HIGH_DEGREES = 5.0;		// How many degrees before decreasing Max PWM

	//************** Steering Angle ******************
	int16_t CurrentWas = WasReading - MDL.ZeroOffset;
	helloSteerPosition = CurrentWas;

	//  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
	// For InvertWAS the negative offset allows the zero hbar in AOG to show correctly. The zero offset has to be
	// redone when changing to InvertWAS.
	if (SteerConfig.InvertWAS)
	{
		CurrentWas = (CurrentWas - SteerSettings.wasOffset);   
		steerAngleActual = (float)(CurrentWas) / -SteerSettings.steerSensorCounts;
	}
	else
	{
		CurrentWas = (CurrentWas + SteerSettings.wasOffset);
		steerAngleActual = (float)(CurrentWas) / SteerSettings.steerSensorCounts;
	}

	if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * SteerSettings.AckermanFix);
	steerAngleError = steerAngleActual - steerAngleSetPoint;

	if ((millis() - AOGTime > 4000) || !AOGsteeringReady || SteerSwitch == HIGH || Speed_KMH < SteerConfig.MinSpeed)
	{
		// steering disabled

		pwmDrive = 0;

		// release relays
		digitalWrite(MDL.SteeringRelayPin, LOW);
		digitalWrite(MDL.PowerRelayPin, LOW);

	}
	else
	{
		// steering enabled

		// limit PWM when steer angle error is low
		MaxPWMvalue = SteerSettings.highPWM;
		if (abs(steerAngleError) < LOW_HIGH_DEGREES)
		{
			MaxPWMvalue = (abs(steerAngleError) * ((SteerSettings.highPWM - SteerSettings.lowPWM) / LOW_HIGH_DEGREES)) + SteerSettings.lowPWM;
		}

		// PID
		pwmDrive = SteerSettings.Kp * steerAngleError;

		//add min throttle factor so no delay from motor resistance.
		if (pwmDrive < 0) pwmDrive -= SteerSettings.minPWM;
		else if (pwmDrive > 0) pwmDrive += SteerSettings.minPWM;
		if (pwmDrive > MaxPWMvalue) pwmDrive = MaxPWMvalue;
		if (pwmDrive < -MaxPWMvalue) pwmDrive = -MaxPWMvalue;

		if (SteerConfig.InvertSteer) pwmDrive *= -1;

		if (SteerConfig.IsDanfoss)
		{
			// Danfoss: PWM 25% On = Left Position max  (below Valve=Center)
			// Danfoss: PWM 50% On = Center Position
			// Danfoss: PWM 75% On = Right Position max (above Valve=Center)
			pwmDrive = (constrain(pwmDrive, -250, 250));

			// Calculations below make sure pwmDrive values are between 65 and 190
			// This means they are always positive, so in motorDrive, no need to check for
			// SteerConfig.isDanfoss anymore
			pwmDrive = pwmDrive >> 2; // Devide by 4
			pwmDrive += 128;          // add Center Pos.
		}

		// engage relays
		digitalWrite(MDL.SteeringRelayPin, HIGH);
		digitalWrite(MDL.PowerRelayPin, HIGH);

		AutoZeroWAS();
	}

	// pwm value out to motor
	digitalWrite(MDL.DirPin, (pwmDrive >= 0));
	analogWrite(MDL.PWMpin, abs(pwmDrive));
}

void AutoZeroWAS()
{
	static uint32_t lastAdjustMs = 0;
	static float fracAccumulator = 0.0f;
	static int16_t lastSavedOffset = 0;
	static uint32_t lastSaveMs = 0;
	static bool initialized = false;

	// New: track angle rate and require stability
	static float lastAngle = 0.0f;
	static uint32_t lastAngleMs = 0;

	if (MDL.AutoZero && SteerSettings.steerSensorCounts > 0)
	{
		// Compute angle rate (deg/s), sampled opportunistically
		uint32_t now = millis();
		if (lastAngleMs == 0) { lastAngleMs = now; lastAngle = steerAngleActual; }
		float angleRateDegPerSec = 0.0f;
		uint32_t dt = now - lastAngleMs;
		if (dt >= 50) // update every >=50 ms
		{
			angleRateDegPerSec = (steerAngleActual - lastAngle) * (1000.0f / (float)dt);
			lastAngle = steerAngleActual;
			lastAngleMs = now;
		}

		// Check if we're mostly centered and stable
		bool smallSetpoint = fabsf(steerAngleSetPoint) <= 1.0f;
		bool smallEffort   = abs(pwmDrive) <= (SteerSettings.minPWM + 5);
		bool modestError   = fabsf(steerAngleError) <= 2.0f;
		bool stableAngle   = fabsf(angleRateDegPerSec) <= 3.0f; // new: don’t zero while turning

		if (smallSetpoint && smallEffort && modestError && stableAngle)
		{
			if (now - lastAdjustMs > 200)
			{
				lastAdjustMs = now;

				// Determine polarity based on InvertWAS
				int8_t polarity = SteerConfig.InvertWAS ? 1 : -1;

				// Counts to shift (clamped to ±2 counts per adjustment)
				float deltaCounts = -steerAngleError * (float)SteerSettings.steerSensorCounts * (float)polarity * 0.02f;
				if (deltaCounts >  2.0f) deltaCounts =  2.0f;
				if (deltaCounts < -2.0f) deltaCounts = -2.0f;

				fracAccumulator += deltaCounts;

				int16_t step = 0;
				if (fabsf(fracAccumulator) >= 1.0f)
				{
					// Truncate toward zero (matches prior floor/ceil logic)
					step = (int16_t)(fracAccumulator > 0.0f ? floorf(fracAccumulator) : ceilf(fracAccumulator));
					fracAccumulator -= step;
				}

				if (step != 0)
				{
					MDL.ZeroOffset = constrain((int32_t)MDL.ZeroOffset + step, -10000, 10000);
				}

				if (!initialized)
				{
					lastSavedOffset = MDL.ZeroOffset;
					lastSaveMs = now;
					initialized = true;
				}

				// Note: SaveData() can block; consider deferring to a safe context.
				if ((now - lastSaveMs > 300000) && abs(MDL.ZeroOffset - lastSavedOffset) >= 10)
				{
					SaveData();
					lastSavedOffset = MDL.ZeroOffset;
					lastSaveMs = now;
				}
			}
		}
	}
}