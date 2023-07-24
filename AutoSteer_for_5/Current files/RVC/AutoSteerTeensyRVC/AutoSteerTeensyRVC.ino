// autosteer for Teensy 4.1
// uses BNO in RVC mode over serial

#define InoDescription "AutoSteerTeensyRVC   23-Jul-2023"
const uint16_t InoID = 23073;	// change to send defaults to eeprom, ddmmy, no leading 0

#include <Wire.h>
#include <EEPROM.h> 
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <Watchdog_t4.h>	// https://github.com/tonton81/WDT_T4
#include "zNMEAParser.h"	
#include "Adafruit_BNO08x_RVC.h"

#include "PCA95x5_RC.h"		// modified from https://github.com/hideakitai/PCA95x5

#include <Adafruit_MCP23008.h>
#include <Adafruit_MCP23X08.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_MCP23XXX.h>

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>

#define ReceiverBaud 460800
#define IMUBaud 115200
#define MaxReadBuffer 100	// bytes
#define LOW_HIGH_DEGREES 5.0	//How many degrees before decreasing Max PWM

struct ModuleConfig
{
	//	AS15 config
	uint8_t Receiver = 1;			// 0 none, 1 SimpleRTK2B, 2 Sparkfun F9p
	uint8_t ReceiverSerialPort = 8;	// gps receiver
	uint8_t	IMUSerialPort = 5;		// Adafruit 5, Sparkfun 4
	uint16_t NtripPort = 2233;		// local port to listen on for NTRIP data
	uint16_t ZeroOffset = 6500;
	uint8_t MinSpeed = 1;
	uint8_t MaxSpeed = 15;
	uint16_t PulseCal = 255;		// Hz/KMH X 10
	uint8_t SwapRollPitch = 0;		// 0 use roll value for roll, 1 use pitch value for roll
	uint8_t InvertRoll = 0;
	uint8_t Dir1 = 23;
	uint8_t PWM1 = 22;
	uint8_t SteeringRelay = 7;		// pin for steering disconnect relay
	uint8_t SteerSw = 26;
	uint8_t WorkSw = 27;
	uint8_t CurrentSensor = 0;		// Ads1115
	uint8_t PressureSensor = 0;		// Ads1115
	uint8_t Encoder = 0;			// none
	uint8_t SpeedPulse = 28;
	uint8_t IP0 = 192;
	uint8_t IP1 = 168;
	uint8_t IP2 = 1;
	uint8_t IP3 = 126;
	uint8_t PowerRelay = 0;			// pin for 12V out relay
	uint8_t	Use4_20 = 0;			// use 4-20 pressure sensor instead of 0-5V
	uint8_t RelayControl = 0;		// 0 - no relays, 1 - RS485, 2 - PCA9555 8 relays, 3 - PCA9555 16 relays, 4 - MCP23017, 5 - Teensy GPIO
	uint8_t RelayPins[16] = { 8,9,10,11,12,25,26,27,0,0,0,0,0,0,0,0 };		// pin numbers when GPIOs are used for relay control (5), default RC11
	uint8_t MCP20317Pins[16] = { 8,9,10,11,12,13,14,15,7,6,5,4,3,2,1,0 };   // 0 to 7 are on Port A, ex: GPA0 = 0, 8 to 15 are on Port B, ex: GPB0 = 8, default RC5 and RC8
};

ModuleConfig MDL;

struct PCBanalog
{
	int16_t AIN0;	// WAS
	int16_t AIN1;	// 0-5V pressure sensor
	int16_t AIN2;	// current sensor
	int16_t AIN3;	// 4-20 pressure sensor
};

PCBanalog AINs;

struct Storage 
{
	uint8_t Kp = 40;  //proportional gain
	uint8_t lowPWM = 10;  //band of no action
	int16_t wasOffset = 0;
	uint8_t minPWM = 9;
	uint8_t highPWM = 60;//max PWM value
	float steerSensorCounts = 30;
	float AckermanFix = 1;     //sent as percent
};  

Storage steerSettings;  //11 bytes

struct Setup
{
	uint8_t InvertWAS = 0;
	uint8_t IsRelayActiveHigh = 0; //if zero, active low (default)
	uint8_t MotorDriveDirection = 0;
	uint8_t SingleInputWAS = 1;
	uint8_t CytronDriver = 1;
	uint8_t SteerSwitch = 0;  //1 if switch selected
	uint8_t SteerButton = 0;  //1 if button selected
	uint8_t ShaftEncoder = 0;
	uint8_t PressureSensor = 0;
	uint8_t CurrentSensor = 0;
	uint8_t PulseCountMax = 5;
	uint8_t IsDanfoss = 0;
	uint8_t IsUseY_Axis = 0;     //Set to 0 to use X Axis, 1 to use Y avis
};

Setup steerConfig;          //9 bytes

extern float tempmonGetTemp(void);

// Ethernet steering
EthernetUDP UDPsteering;	// UDP Steering traffic, to and from AGIO
uint16_t ListeningPort = 8888;
uint16_t DestinationPort = 9999;	// port that AGIO listens on
IPAddress DestinationIP(MDL.IP0, MDL.IP1, MDL.IP2, 255);

EthernetUDP UDPntrip;	// from AGIO to receiver
char NtripBuffer[512];	// buffer for ntrip data

//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0; //the desired angle from AgOpen
int16_t steeringPosition = 0;
float steerAngleError = 0; //setpoint - actual
float Speed_KMH = 0.0;
int8_t guidanceStatus;

float IMU_Heading = 0;
float IMU_Roll = 0;
float IMU_Pitch = 0;
float IMU_YawRate = 0;

// switches
int8_t SteerSwitch = LOW;	// Low on, High off
int8_t SWreading = HIGH;
int8_t SWprevious = LOW;
uint32_t  SWtime = 0;
uint8_t  SWdebounce = 50;
int8_t switchByte = 0;
int8_t workSwitch = 0;
float SensorReading;

//pwm variables
int16_t pwmDrive = 0;
int16_t MaxPWMvalue = 255;

//Heart beat hello AgIO
uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
uint8_t helloFromAutoSteer[] = { 128, 129, 126, 126, 5, 0, 0, 0, 0, 0, 71 };
int16_t helloSteerPosition = 0;

//fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
uint8_t PGN_253[] = { 128, 129, 123, 253, 8, 0, 0, 0, 0, 0,0,0,0, 12 };

//fromAutoSteerData FD 250 - sensor values etc
uint8_t PGN_250[] = { 128, 129, 123, 250, 8, 0, 0, 0, 0, 0,0,0,0, 12 };

WDT_T4<WDT1> wdt;
const uint16_t  LOOP_TIME = 25;	// 40 hz, main loop
uint32_t  LoopLast = LOOP_TIME;

NMEAParser<2> parser;
Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();
BNO08x_RVC_Data heading;

uint32_t CommTime;
bool ADSfound = false;
byte PGNlength;

HardwareSerial* SerialIMU;
HardwareSerial* SerialReceiver;

elapsedMillis imuDelayTimer;
bool isGGA_Updated = false;
int ADS1115_Address;

byte RelayLo = 0;	// sections 0-7
byte RelayHi = 0;	// sections 8-15

PCA9555 PCA;
bool PCA9555PW_found = false;
Adafruit_MCP23X17 MCP;
bool MCP23017_found = false;

void setup()
{
	DoSetup();
}

void loop()
{
	if (millis() - LoopLast >= LOOP_TIME)
	{
		LoopLast = millis();
		ReadAnalog();
		ReadSwitches();
		DoSteering();
		SendSpeedPulse();
		ReceiveConfigData();
		CheckRelays();
	}
	ReadIMU();
	DoPanda();
	ReceiveSteerData();
	Blink();
	wdt.feed();
}

bool State = false;
elapsedMillis BlinkTmr;
byte ResetRead;
elapsedMicros LoopTmr;
uint32_t MaxLoopTime;
void Blink()
{
	if (BlinkTmr > 1000)
	{
		BlinkTmr = 0;
		State = !State;
		digitalWrite(LED_BUILTIN, State);
		Serial.println(".");	// needed to allow PCBsetup to connect

		Serial.print(" Loop Time (Micros): ");
		Serial.print(MaxLoopTime);

		Serial.print(", WAS: ");
		Serial.print(AINs.AIN0);

		Serial.print(", Heading: ");
		Serial.print(IMU_Heading / 10.0);

		//Serial.print(", Temp: ");
		//Serial.print(tempmonGetTemp());

		Serial.println("");

		if (ResetRead++ > 5)
		{
			MaxLoopTime = 0;
			ResetRead = 0;
		}
	}
	if (LoopTmr > MaxLoopTime) MaxLoopTime = LoopTmr;
	LoopTmr = 0;
}

uint32_t SpeedPulseTime;
void SendSpeedPulse()
{
	// https://discourse.agopengps.com/t/get-feed-rate-from-ago-and-transform-it-into-weedkiller-sprayer-computer-compatible-pulses/2958/39
	// PulseCal: hz/mph - 41.0, hz/kmh - 25.5

	if (millis() - SpeedPulseTime > 400) //This section runs every 400 millis.  It gets speed and changes the frequency of the tone generator.
	{
		SpeedPulseTime = millis();
		if (Speed_KMH < 1.22) // If the speed is lower than (0.76 MPH, 1.22 KMH) or (31.1 Hz) it forces the output to 0. Tone will not work under 31 Hz 
		{
			noTone(MDL.SpeedPulse);
		}
		else
		{
			tone(MDL.SpeedPulse, (Speed_KMH * MDL.PulseCal / 10));
		}
	}
}

bool GoodCRC(byte Data[], byte Length)
{
	byte ck = CRC(Data, Length - 1, 0);
	bool Result = (ck == Data[Length - 1]);
	return Result;
}

byte CRC(byte Chk[], byte Length, byte Start)
{
	byte Result = 0;
	int CK = 0;
	for (int i = Start; i < Length; i++)
	{
		CK += Chk[i];
	}
	Result = (byte)CK;
	return Result;
}





