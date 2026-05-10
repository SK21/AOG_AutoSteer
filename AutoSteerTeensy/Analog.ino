
void ReadAnalog()
{
	static uint16_t Aread;
	static const uint8_t pattern[4] = { 0, 3, 0, 0 };	// prioritize AIN0 (WAS)
	static uint8_t patIdx = 0;							// index into pattern

	// use ADS1115 through Teensy
	if (ADSfound)
	{
		// based on https://github.com/RalphBacon/ADS1115-ADC/blob/master/ADS1115_ADC_16_bit_SingleEnded.ino

		// read current value
		Wire.beginTransmission(ADS1115_Address);
		Wire.write(0b00000000); //Point to Conversion register
		Wire.endTransmission();
		Wire.requestFrom(ADS1115_Address, 2);
		Aread = (Wire.read() << 8 | Wire.read());

		// decode which channel this sample belongs to
		uint8_t channel = pattern[patIdx];

		switch (channel)
		{
		case 0:	// AIN0: WAS (effective 14-bit, 0-16383)
			WasReading = Aread >> 1;
			break;

		case 3:	// AIN3: analog current (0-127)
			AnalogReadingValue = Aread >> 8;
			break;

		default:
			// AIN1/2 not used
			break;
		}

		// advance schedule and configure next conversion
		patIdx = (uint8_t)((patIdx + 1) % 4);
		uint8_t nextChannel = pattern[patIdx];

		Wire.beginTransmission(ADS1115_Address);
		Wire.write(0b00000001); // Point to Config Register

		// Write the MSB + LSB of Config Register
		// MSB: Bits 15:8
		// Bit  15    0=No effect, 1=Begin Single Conversion (in power down mode)
		// Bits 14:12   How to configure A0 to A3 (comparator or single ended)
		// Bits 11:9  Programmable Gain 000=6.144v 001=4.096v 010=2.048v .... 111=0.256v
		// Bits 8     0=Continuous conversion mode, 1=Power down single shot

		// MSB: set MUX for next channel (continuous mode)
		switch (nextChannel)
		{
		case 0:
			Wire.write(0b01000000);	// AIN0
			break;
		case 3:
			Wire.write(0b01110000);	// AIN3
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
	}
	else
	{
		// use Teensy analog pins
		if (MDL.WasPin < NC) WasReading = (uint16_t)analogRead(MDL.WasPin);	// 12 bit, 0-4095
		if (MDL.AnalogPin < NC) AnalogReadingValue = (uint16_t)analogRead(MDL.AnalogPin);
	}
}

// written by ReadActuatorPosition(), read by Toolsteer.ino
int16_t actuatorPosition = 0;
float   actuatorPositionPercent = 0;

void ReadActuatorPosition()
{
	static bool initialized = false;

	if (ADSfound)
	{
		if (!initialized)
		{
			// configure ADS1115: AIN0 vs GND, ±6.144V, continuous, 860 SPS
			Wire.beginTransmission(ADS1115_Address);
			Wire.write(0b00000001);  // config register
			Wire.write(0b01000000);  // AIN0, ±6.144V, continuous
			Wire.write(0b11100011);  // 860 SPS, comparator disabled
			Wire.endTransmission();
			initialized = true;
			return;
		}

		// read last completed conversion
		Wire.beginTransmission(ADS1115_Address);
		Wire.write(0b00000000);  // conversion register
		Wire.endTransmission();
		Wire.requestFrom(ADS1115_Address, 2);
		uint16_t raw = (Wire.read() << 8 | Wire.read());

		// 14-bit result, mid-scale ~6805 counts at 2.5V
		actuatorPosition = (int16_t)(raw >> 1) - 6805 + toolSettings.zeroOffset_APOS;
		actuatorPositionPercent = (float)actuatorPosition / 68.0f;

		if (toolSettings.invertAPOS)
			actuatorPositionPercent = -actuatorPositionPercent;
	}
	else
	{
		// no ADS1115 — read actuator pot directly from WasPin (12-bit, 0-4095)
		if (MDL.WasPin < NC)
		{
			int16_t raw = (int16_t)analogRead(MDL.WasPin);
			actuatorPosition = raw - 2048 + toolSettings.zeroOffset_APOS;
			actuatorPositionPercent = (float)actuatorPosition / 20.48f;
			if (toolSettings.invertAPOS)
				actuatorPositionPercent = -actuatorPositionPercent;
		}
	}
}
