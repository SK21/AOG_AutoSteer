
void DoSteering()
{
	//************** Steering Angle ******************
	steeringPosition = AINs.AIN0;
	helloSteerPosition = steeringPosition - MDL.ZeroOffset;

	//  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
	if (steerConfig.InvertWAS)
	{
		steeringPosition = (steeringPosition - MDL.ZeroOffset - steerSettings.wasOffset);   // 1/2 of full scale
		steerAngleActual = (float)(steeringPosition) / -steerSettings.steerSensorCounts;
	}
	else
	{
		steeringPosition = (steeringPosition - MDL.ZeroOffset + steerSettings.wasOffset);   // 1/2 of full scale
		steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
	}

	if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);
	steerAngleError = steerAngleActual - steerAngleSetPoint;

	if ((millis() - CommTime > 4000) || (bitRead(guidanceStatus, 0) == 0) || SteerSwitch == HIGH || (Speed_KMH < MDL.MinSpeed) || Speed_KMH > MDL.MaxSpeed)
	{
		// steering disabled

		pwmDrive = 0;

		// release steer relay
		digitalWrite(MDL.SteerSw_Relay, LOW);
	}
	else
	{
		// steering enabled

		// limit PWM when steer angle error is low
		MaxPWMvalue = steerSettings.highPWM;
		if (abs(steerAngleError) < LOW_HIGH_DEGREES)
		{
			MaxPWMvalue = (abs(steerAngleError) * ((steerSettings.highPWM - steerSettings.lowPWM) / LOW_HIGH_DEGREES)) + steerSettings.lowPWM;
		}

		// PID
		pwmDrive = steerSettings.Kp * steerAngleError;

		//add min throttle factor so no delay from motor resistance.
		if (pwmDrive < 0) pwmDrive -= steerSettings.minPWM;
		else if (pwmDrive > 0) pwmDrive += steerSettings.minPWM;
		if (pwmDrive > MaxPWMvalue) pwmDrive = MaxPWMvalue;
		if (pwmDrive < -MaxPWMvalue) pwmDrive = -MaxPWMvalue;

		if (steerConfig.MotorDriveDirection) pwmDrive *= -1;

		if (steerConfig.IsDanfoss)
		{
			// Danfoss: PWM 25% On = Left Position max  (below Valve=Center)
			// Danfoss: PWM 50% On = Center Position
			// Danfoss: PWM 75% On = Right Position max (above Valve=Center)
			pwmDrive = (constrain(pwmDrive, -250, 250));

			// Calculations below make sure pwmDrive values are between 65 and 190
			// This means they are always positive, so in motorDrive, no need to check for
			// steerConfig.isDanfoss anymore
			pwmDrive = pwmDrive >> 2; // Devide by 4
			pwmDrive += 128;          // add Center Pos.
		}

		// engage steer relay
		digitalWrite(MDL.SteerSw_Relay, HIGH);
	}

	// pwm value out to motor
	digitalWrite(MDL.Dir1, (pwmDrive >= 0));
	analogWrite(MDL.PWM1, abs(pwmDrive));
}

float tmpIMU;

void ReadIMU()
{
	switch (MDL.IMU)
	{

	case 3:	// CMPS14
		//the heading x10
		Wire.beginTransmission(CMPS14_ADDRESS);
		Wire.write(0x02);
		Wire.endTransmission();

		Wire.requestFrom(CMPS14_ADDRESS, 3);
		while (Wire.available() < 3);

		IMU_Heading = Wire.read() << 8 | Wire.read();

		//3rd byte pitch
		IMU_Pitch = Wire.read();

		//roll
		Wire.beginTransmission(CMPS14_ADDRESS);
		Wire.write(0x1C);
		Wire.endTransmission();

		Wire.requestFrom(CMPS14_ADDRESS, 2);
		while (Wire.available() < 2);

		tmpIMU = int16_t(Wire.read() << 8 | Wire.read());

		//Complementary filter
		IMU_Roll = 0.9 * IMU_Roll + 0.1 * tmpIMU;

		//Get the Z gyro
		Wire.beginTransmission(CMPS14_ADDRESS);
		Wire.write(0x16);
		Wire.endTransmission();

		Wire.requestFrom(CMPS14_ADDRESS, 2);
		while (Wire.available() < 2);

		tmpIMU = int16_t(Wire.read() << 8 | Wire.read());

		//Complementary filter
		IMU_YawRate = 0.93 * IMU_YawRate + 0.07 * tmpIMU;
		break;

	case 4:
		BNO08x_RVC_Data heading;
		if (rvc.read(&heading))
		{
			IMU_Heading = heading.yaw;
			if (IMU_Heading < 0 && IMU_Heading >= -180) //Scale BNO085 yaw from [-180?;180?] to [0;360?]
			{
				IMU_Heading = IMU_Heading + 360;
			}
			IMU_Heading *= 10.0;

			if (MDL.SwapRollPitch)
			{
				IMU_Roll = heading.pitch * 10;
				if (MDL.InvertRoll) IMU_Roll *= -1.0;

				IMU_Pitch = heading.roll * 10;
			}
			else
			{
				IMU_Roll = heading.roll * 10;
				if (MDL.InvertRoll) IMU_Roll *= -1.0;

				IMU_Pitch = heading.pitch * 10;
			}

			if (MDL.GyroOn)
			{
				IMU_YawRate = heading.z_accel;
			}
		}
		break;
	}
}

