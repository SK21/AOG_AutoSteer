
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

		// engage relays
		digitalWrite(MDL.SteeringRelayPin, HIGH);
		digitalWrite(MDL.PowerRelayPin, HIGH);
	}

	// pwm value out to motor
	digitalWrite(MDL.DirPin, (pwmDrive >= 0));
	analogWrite(MDL.PWMpin, abs(pwmDrive));
}