
void ReadAnalog()
{
	static uint8_t AdsPin = 0;
	static int16_t Aread;
	static bool ConversionPending = false;

	if (ADSfound)
	{
		// use ADS1115
		//	AS15 config
		//	AIN0	WAS 5V
		//	AIN1	
		//	AIN2	
		//	AIN3	Current/pressure
		// Only do one of either a read or a request per loop. Saves loop time and
		// doesn't affect ADC read time that much.
		// Shifting by 2 gives an effective range of 0-6666 for a 5V sensor. About 12.7 bits of resolution.

		if (ConversionPending)
		{
			// read value if available
			Wire.beginTransmission(ADS1115_Address);
			Wire.write(0b00000000); //Point to Conversion register
			Wire.endTransmission();
			if (Wire.requestFrom(ADS1115_Address, 2) == 2)
			{
				Aread = (int16_t)(Wire.read() << 8 | Wire.read());
				if (Aread < 0) Aread = 0;
				uint16_t ScaledReading = (uint16_t)((uint16_t)Aread >> 2);

				switch (AdsPin)
				{
				case 0:
					WasReading = ScaledReading;
					break;

				default:
					AnalogReadingValue = ScaledReading;
					break;
				}
				ConversionPending = false;
			}
		}
		else
		{
			// start new read
			Wire.beginTransmission(ADS1115_Address);
			Wire.write(0b00000001); // Point to Config Register

			// Write the MSB + LSB of Config Register
			// MSB: Bits 15:8
			// Bit  15    0=No effect, 1=Begin Single Conversion (in power down mode)
			// Bits 14:12   How to configure A0 to A3 (comparator or single ended)
			// Bits 11:9  Programmable Gain 000=6.144v 001=4.096v 010=2.048v .... 111=0.256v
			// Bits 8     0=Continuous conversion mode, 1=Power down single shot

			if (AdsPin == 0)
			{
				AdsPin = 3;
			}
			else
			{
				AdsPin = 0;
			}

			switch (AdsPin)
			{
				// single ended
			case 0:
				Wire.write(0b11000001);	// AIN0
				break;
			case 1:
				Wire.write(0b11010001);	// AIN1
				break;
			case 2:
				Wire.write(0b11100001);	// AIN2
				break;
			case 3:
				Wire.write(0b11110001);	// AIN3
				break;
			}

			// LSB: Bits 7:0
			// Bits 7:5 Data Rate (Samples per second) 000=8, 001=16, 010=32, 011=64,
			//      100=128, 101=250, 110=475, 111=860
			// Bit  4   Comparator Mode 0=Traditional, 1=Window
			// Bit  3   Comparator Polarity 0=low, 1=high
			// Bit  2   Latching 0=No, 1=Yes
			// Bits 1:0 Comparator # before Alert pin goes high
			//      00=1, 01=2, 10=4, 11=Disable this feature
			Wire.write(0b11100011);	//860 samples/sec
			Wire.endTransmission();

			ConversionPending = true;
		}
	}
	else
	{
		// use Teensy analog pins
		if (MDL.WasPin < NC) WasReading = (uint16_t)analogRead(MDL.WasPin);
		if (MDL.AnalogPin < NC) AnalogReadingValue = (uint16_t)analogRead(MDL.AnalogPin);
	}
}


