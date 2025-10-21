
// autosteer for Teensy 4.1

#include <Wire.h>
#include <EEPROM.h> 
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

#include "zNMEAParser.h"	
#include "Adafruit_BNO08x_RVC.h"	// https://github.com/VikingVoltage/Adafruit_BNO08x_RVC/tree/Get-most-recent-BNO08x-Data-from-Serial-Buffer
#include <Adafruit_Sensor.h>

#include "FXUtil.h"		// read_ascii_line(), hex file support
extern "C" {
#include "FlashTxx.h"		// TLC/T3x/T4x/TMM flash primitives
}

// Motion Module Interface:				https://www.syd-dynamics.com/download-center/
#include <EasyObjectDictionary.h>
#include <EasyProfile.h>
EasyObjectDictionary eOD;
EasyProfile          eP(&eOD);

#define InoDescription "AutoSteerTeensy"
const uint16_t InoID = 10105;	// change to send defaults to eeprom, ddmmy, no leading 0
const uint8_t InoType = 0;		// 0 - Teensy AutoSteer, 1 - Teensy Rate, 2 - Nano Rate, 3 - Nano SwitchBox, 4 - ESP Rate

#define ReceiverBaud 460800
#define IMUBaud 115200
#define PassThruBaud 57600		// for RS232
#define NC 0xFF					// Pin not connected

struct ModuleConfig		// about 28 bytes
{
	//	AS15-3 config
	uint8_t ID = 0;
	uint8_t ReceiverSerialPort = 4;	
	uint8_t	IMUSerialPort = 3;	
	uint8_t PassThruInSerialPort = 8;		// from F9P Uart2
	uint8_t PassThrOutSerialPort = 2;		// to Max232 for DB9 connector
	uint8_t PowerRelayPin = 0;
	uint8_t SteeringRelayPin = 1;	// pin for steering disconnect relay
	uint8_t SteerSwitchPin = 30;
	uint8_t WorkSwitchPin = 31;
	uint8_t WasPin = 25;
	uint8_t AnalogPin = 26;
	uint8_t DirPin = 23;
	uint8_t PWMpin = 22;
	uint8_t EncoderPin = NC;
	uint8_t SpeedPulsePin = NC;
	uint16_t SpeedPulseCal = 255;	// Hz/KMH X 10
	int16_t ZeroOffset = 6500;
	uint8_t IMUtype = 0;	// 0 BNO080, 1 TM171
	bool InvertRoll = false;
	bool ADS1115Enabled = false;
	bool AutoZero = false;
};

ModuleConfig MDL;

struct ModuleNetwork
{
	uint16_t Identifier = 9876;
	uint8_t IP0 = 192;
	uint8_t IP1 = 168;
	uint8_t IP2 = 1;
	uint8_t IP3 = 126;
};

ModuleNetwork MDLnetwork;

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

Storage SteerSettings;  //11 bytes

struct Setup
{
	uint8_t InvertWAS = 0;
	uint8_t InvertRelays = 0;	//if zero, active low (default)
	uint8_t InvertSteer = 0;
	uint8_t SingleInputWAS = 1;
	uint8_t CytronDriver = 1;
	uint8_t SteerSwitch = 0;	//1 if switch selected
	uint8_t SteerButton = 0;	//1 if button selected
	uint8_t ShaftEncoder = 0;
	uint8_t PressureSensor = 0;
	uint8_t CurrentSensor = 0;
	uint8_t PulseCountMax = 5;
	uint8_t IsDanfoss = 0;
	uint8_t UseIMU_Y_Axis = 0;	//Set to 0 to use X Axis, 1 to use Y avis
	float MinSpeed = 0;		// minimum kmh X 10
};

Setup SteerConfig;          //9 bytes

// Ethernet steering
EthernetUDP UDPsteering;	// UDP Steering traffic, to and from AGIO
const uint16_t ListeningPort = 8888;
const uint16_t DestinationPort = 9999;	// port that AGIO listens on
IPAddress DestinationIP(MDLnetwork.IP0, MDLnetwork.IP1, MDLnetwork.IP2, 255);
uint32_t AOGTime;

EthernetUDP UDPntrip;				// from AGIO to receiver
const uint16_t NtripPort = 2233;	// local port to listen on for NTRIP data
char NtripBuffer[512];				// buffer for ntrip data

// Ethernet config
EthernetUDP UDPconfig;
const uint16_t ConfigListeningPort = 28888;
const uint16_t ConfigDestinationPort = 29500;

// firmware update
EthernetUDP UpdateComm;
const uint16_t UpdateReceivePort = 29100;
const uint16_t UpdateSendPort = 29000;
uint32_t buffer_addr, buffer_size;
bool FirmwareUpdateMode = false;

//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0; //the desired angle from AgOpen
float Speed_KMH = 0.0;
bool AOGsteeringReady = false;	// AOG is ready to steer pending steer switch 
uint8_t SteerSwitch = HIGH;	// Low on, High off
uint8_t switchByte = 0;
float AnalogReadingAverage;

// IMU
float IMU_Heading = 0;
float IMU_Roll = 0;
float IMU_Pitch = 0;
float IMU_YawRate = 0;
uint32_t IMUtime;

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

NMEAParser<2> parser;
Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();
BNO08x_RVC_Data BNOdata;

uint16_t WasReading;
uint16_t AnalogReadingValue;
bool ADSfound = false;
int16_t ADS1115_Address = 72;

HardwareSerial* SerialIMU;
HardwareSerial* SerialReceiver;
HardwareSerial* SerialPassIn;
HardwareSerial* SerialPassOut;

bool SerialIMUEnabled = false;
bool SerialReceiverEnabled = false;
bool SerialPassThruEnabled = false;

const uint16_t  LOOP_TIME = 25;	// 40 hz, main loop
uint32_t  LoopLast;
uint16_t MaxLoopTime = 0;	// micros
float steerAngleError = 0;

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
		ReceiveConfig();
		SendSpeedPulse();
	}
	ReadIMU();
	DoPanda();
	ReceiveSteerData();
	ReceiveUpdate();
	Blink();
	if (SerialPassThruEnabled && SerialPassIn->available()) SerialPassOut->write(SerialPassIn->read());
}
void Blink()
{
	static bool State = false;
	static elapsedMillis BlinkTmr;
	static elapsedMicros LoopTmr;
	static byte Count = 0;
	static bool Initialized = false;
	
	if (BlinkTmr > 1000)
	{
		BlinkTmr = 0;
		State = !State;
		digitalWrite(LED_BUILTIN, State);
		if (!FirmwareUpdateMode)
		{
			Serial.print(" Micros: ");
			Serial.print(MaxLoopTime);

			Serial.print(", ");
			Serial.print(IMU_Heading / 10.0);

			Serial.println("");

			if (Count++ > 10)
			{
				Count = 0;
				MaxLoopTime = 0;
			}
		}
	}

	if (!Initialized)
	{
		Initialized = true;
		MaxLoopTime = LoopTmr;
	}

	if (LoopTmr > MaxLoopTime) MaxLoopTime = LoopTmr;
	LoopTmr = 0;
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
	for (int i = Start; i < Length; i++)
	{
		Result += Chk[i];
	}
	return Result;
}

byte ParseModID(byte ID)
{
	// top 4 bits
	return ID >> 4;
}

void SendSpeedPulse()
{
	// https://discourse.agopengps.com/t/get-feed-rate-from-ago-and-transform-it-into-weedkiller-sprayer-computer-compatible-pulses/2958/39
	// PulseCal: hz/mph - 41.0, hz/kmh - 25.5

	const uint16_t UpdateMS = 200;
	const uint16_t MinHz = 31;	// Tone will not work under 31 Hz 
	const uint16_t MaxHz = 1200;

	static float LastSpeed = 0;
	static uint32_t LastTime = 0;
	static bool ToneIsOn = false;

	if (MDL.SpeedPulsePin < NC && (millis() - LastTime > UpdateMS))
	{
		LastTime = millis();
		float PulseCal = MDL.SpeedPulseCal / 10.0;
		if (PulseCal < 1) PulseCal = 25.5;

		float CutOffSpeed = MinHz / PulseCal;

		if (ToneIsOn)
		{
			if (Speed_KMH < CutOffSpeed)
			{
				noTone(MDL.SpeedPulsePin);
				ToneIsOn = false;
			}
			else if (abs(Speed_KMH - LastSpeed) > 0.2)
			{
				uint16_t hz = Speed_KMH * PulseCal + 0.5;	// round up
				hz = constrain(hz, MinHz, MaxHz);
				tone(MDL.SpeedPulsePin, hz);
				LastSpeed = Speed_KMH;
			}
		}
		else
		{
			if (Speed_KMH >= (CutOffSpeed * 1.05))	// Add 5% to stop chatter
			{
				ToneIsOn = true;

				uint16_t hz = Speed_KMH * PulseCal + 0.5;
				hz = constrain(hz, MinHz, MaxHz);
				tone(MDL.SpeedPulsePin, hz);
				LastSpeed = Speed_KMH;
			}
		}
	}
}


