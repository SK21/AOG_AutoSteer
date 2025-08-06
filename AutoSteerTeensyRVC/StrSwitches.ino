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

	if (SteerConfig.SteerSwitch == 1)
	{
		// on off switch
		SWreading = digitalRead(MDL.SteerSwitchPin);

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
	else if (SteerConfig.SteerButton == 1)
	{
		// push button
		// pin is pulled high and goes low when button is pushed

		SWreading = digitalRead(MDL.SteerSwitchPin);
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
	if (SteerConfig.CurrentSensor)
	{
		float SensorSample = (float)(CurrentReading);	
		SensorSample = (abs(512 - SensorSample)) * 0.5;
		SensorReading = SensorReading * 0.7 + SensorSample * 0.3;
		if (SensorReading >= SteerConfig.PulseCountMax)
		{
			SteerSwitch = HIGH;
			SWprevious = LOW;
		}
	}

	switchByte = SteerSwitch << 1;
	switchByte |= digitalRead(MDL.WorkSwitchPin);  // read work switch, Low on, High off
}