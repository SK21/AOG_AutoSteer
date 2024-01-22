uint32_t PulseStart;
int SwitchPulseCount;
bool PulseRead;
bool PulseLast;
int8_t SWreading = HIGH;
int8_t SWprevious = LOW;
uint32_t SWtime = 0;
uint8_t SWdebounce = 50;

void ReadSwitches()
{
	// steer switch		- low, steering on 
	//					- high, steering off

	// guidanceStatus	- low, steering off
	//					- high, steering on

	if (steerConfig.SteerSwitch == 1)
	{
		// on off switch
		SWreading = digitalRead(MDL.SteerSw);

		if (SWreading)
		{
			// pin high, turn off
			SteerSwitch = SWreading;
		}
		else
		{
			// pin low and previously off, turn on
			if (SWprevious) SteerSwitch = SWreading;
		}
		SWprevious = SWreading;
	}
	else if (steerConfig.SteerButton == 1)
	{
		// push button
		// pin is pulled high and goes low when button is pushed

		SWreading = digitalRead(MDL.SteerSw);
		if (SWreading == LOW && SWprevious == HIGH && millis() - SWtime > SWdebounce)
		{
			if (SteerSwitch == HIGH)
			{
				SteerSwitch = LOW;
			}
			else
			{
				SteerSwitch = HIGH;
			}
			SWtime = millis();
		}
		SWprevious = SWreading;
	}
	else
	{
		// no switch, match status
		if (guidanceStatus)
		{
			// steering on
			// previously off, turn on
			if (SWprevious) SteerSwitch = LOW;
		}
		else
		{
			// steering off
			SteerSwitch = HIGH;
		}
		SWprevious = !guidanceStatus;
	}

	// sensors
	if (steerConfig.CurrentSensor)
	{
		float SensorSample = (float)(AINs.AIN3 );	
		SensorSample = (abs(512 - SensorSample)) * 0.5;
		SensorReading = SensorReading * 0.7 + SensorSample * 0.3;
		if (SensorReading >= steerConfig.PulseCountMax)
		{
			SteerSwitch = HIGH;
			SWprevious = LOW;
		}
	}
	else if (steerConfig.PressureSensor)
	{
		float SensorSample = 0;
		if (MDL.Use4_20)
		{
			// analog 4-20
			SensorSample = (float)(AINs.AIN2 );
		}
		else
		{
			// analog 12V
			SensorSample = (float)(AINs.AIN1);
		}

		SensorReading = SensorReading * 0.6 + SensorSample * 0.4;
		if (SensorReading >= steerConfig.PulseCountMax)
		{
			SteerSwitch = HIGH;
			SWprevious = LOW;
		}
	}
	else if (steerConfig.ShaftEncoder)
	{
		PulseRead = digitalRead(MDL.Encoder);
		if ((PulseRead != PulseLast) && (millis() - PulseStart > SWdebounce))
		{
			PulseStart = millis();
			PulseLast = PulseRead;
			SwitchPulseCount++;

			if (SwitchPulseCount >= steerConfig.PulseCountMax)
			{
				SteerSwitch = HIGH;
				SWprevious = LOW;
				SwitchPulseCount = 0;
			}
		}
	}

	switchByte = SteerSwitch << 1;
	switchByte |= digitalRead(MDL.WorkSw);  // read work switch, Low on, High off
}