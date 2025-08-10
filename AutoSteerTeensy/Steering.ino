
void DoSteering()
{
	//************** Steering Angle ******************
	helloSteerPosition = WasReading - MDL.ZeroOffset;

	//  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
	if (SteerConfig.InvertWAS)
	{
		WasReading = (WasReading - MDL.ZeroOffset - SteerSettings.wasOffset);   // 1/2 of full scale
		steerAngleActual = (float)(WasReading) / -SteerSettings.steerSensorCounts;
	}
	else
	{
		WasReading = (WasReading - MDL.ZeroOffset + SteerSettings.wasOffset);   // 1/2 of full scale
		steerAngleActual = (float)(WasReading) / SteerSettings.steerSensorCounts;
	}

	if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * SteerSettings.AckermanFix);
	steerAngleError = steerAngleActual - steerAngleSetPoint;

	if ((millis() - AOGTime > 4000) || (bitRead(guidanceStatus, 0) == LOW) || SteerSwitch == HIGH || Speed_KMH < SteerConfig.MinSpeed)
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

		// Gently auto - trim WAS zero based on steady - state error
		//AutoZeroWAS();

		// engage relays
		digitalWrite(MDL.SteeringRelayPin, HIGH);
		digitalWrite(MDL.PowerRelayPin, HIGH);
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

	// Sanity check for sensor scaling
	if (SteerSettings.steerSensorCounts < 1.0f) return;

	// Check if steering is active
	bool autosteerEnabled =
		(millis() - AOGTime <= 4000) &&
		(bitRead(guidanceStatus, 0) == HIGH) &&
		(SteerSwitch == LOW) &&
		(Speed_KMH >= SteerConfig.MinSpeed);

	if (!autosteerEnabled) return;

	// Check if we're mostly centered and stable
	bool smallSetpoint = fabs(steerAngleSetPoint) <= 0.5f;
	bool smallEffort = abs(pwmDrive) <= (SteerSettings.minPWM + 5);
	bool modestError = fabs(steerAngleError) <= 1.5f;

	if (!(smallSetpoint && smallEffort && modestError)) return;

	uint32_t now = millis();
	if (now - lastAdjustMs < 200) return;  // adjust every 200ms
	lastAdjustMs = now;

	// Determine polarity based on InvertWAS
	int8_t polarity = SteerConfig.InvertWAS ? 1 : -1;

	// Calculate how much to shift zero offset based on error
	float deltaCounts = -steerAngleError * SteerSettings.steerSensorCounts * polarity * 0.02f;
	if (deltaCounts > 2.0f) deltaCounts = 2.0f;
	if (deltaCounts < -2.0f) deltaCounts = -2.0f;
	fracAccumulator += deltaCounts;

	int16_t step = 0;
	if (fracAccumulator >= 1.0f) {
		step = (int16_t)floor(fracAccumulator);
		fracAccumulator -= step;
	}
	else if (fracAccumulator <= -1.0f) {
		step = (int16_t)ceil(fracAccumulator);
		fracAccumulator -= step;
	}

	if (step != 0) {
		MDL.ZeroOffset = constrain((int32_t)MDL.ZeroOffset + step, -1000, 1000);
	}

	if (!initialized) {
		lastSavedOffset = MDL.ZeroOffset;
		lastSaveMs = now;
		initialized = true;
	}

	if ((now - lastSaveMs > 15000) && abs(MDL.ZeroOffset - lastSavedOffset) >= 5) {
		SaveData();
		lastSavedOffset = MDL.ZeroOffset;
		lastSaveMs = now;
	}
}