#include <Wire.h>
#include <EEPROM.h> 
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

#include "zNMEAParser.h"	
#include "Adafruit_BNO08x_RVC.h"

#include <Adafruit_MCP23008.h>
#include <Adafruit_MCP23X08.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_MCP23XXX.h>

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>

#include "FXUtil.h"		// read_ascii_line(), hex file support
extern "C" {
#include "FlashTxx.h"		// TLC/T3x/T4x/TMM flash primitives
}

// autosteer for Teensy 4.1
// uses BNO in RVC mode over serial

#include <Adafruit_Sensor.h>
#define InoDescription "AutoSteerTeensyRVC   07-Mar-2025"
const uint16_t InoID = 7035;	// change to send defaults to eeprom, ddmmy, no leading 0
const uint8_t InoType = 0;		// 0 - Teensy AutoSteer, 1 - Teensy Rate, 2 - Nano Rate, 3 - Nano SwitchBox, 4 - ESP Rate

#define ReceiverBaud 460800
#define IMUBaud 115200
#define MaxReadBuffer 100	// bytes
#define LOW_HIGH_DEGREES 5.0	//How many degrees before decreasing Max PWM
#define NC 0xFF		// Pin not connected

struct ModuleConfig
{
	//	AS15-2 config
	uint8_t Receiver = 1;			// 0 none, 1 SimpleRTK2B, 2 Sparkfun F9p
	uint8_t ReceiverSerialPort = 8;	// 8 for both micro and SimpleRTK2B
	uint8_t	IMUSerialPort = 5;		// Adafruit 5, Sparkfun 4
	uint16_t NtripPort = 2233;		// local port to listen on for NTRIP data
	int16_t ZeroOffset = 0;
	uint8_t SwapRollPitch = 0;		// 0 use roll value for roll, 1 use pitch value for roll
	uint8_t InvertRoll = 0;
	uint8_t Dir1 = 23;
	uint8_t PWM1 = 22;
	uint8_t SteeringRelay = 7;		// pin for steering disconnect relay
	uint8_t SteerSw = 26;
	uint8_t WorkSw = 27;
	uint8_t IP0 = 192;
	uint8_t IP1 = 168;
	uint8_t IP2 = 1;
	uint8_t IP3 = 126;
	uint8_t AdsAddress = 0x49;		
};

ModuleConfig MDL;

struct PCBanalog
{
	int16_t AIN0;	// WAS
	int16_t AIN1;	// -
	int16_t AIN2;	// -
	int16_t AIN3;	// current
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

// Ethernet steering
EthernetUDP UDPsteering;	// UDP Steering traffic, to and from AGIO
uint16_t ListeningPort = 8888;
uint16_t DestinationPort = 9999;	// port that AGIO listens on
IPAddress DestinationIP(MDL.IP0, MDL.IP1, MDL.IP2, 255);

EthernetUDP UDPntrip;	// from AGIO to receiver
char NtripBuffer[512];	// buffer for ntrip data

// GPS pass-through from Serial 3 to Serial 1, 57600 baud
HardwareSerial* GPSserial;
HardwareSerial* SerialOut;

// Ethernet config
EthernetUDP UDPconfig;
uint16_t ConfigListeningPort = 28888;
uint16_t ConfigDestinationPort = 29999;

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
int8_t switchByte = 0;
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

const uint16_t  LOOP_TIME = 25;	// 40 hz, main loop
uint32_t  LoopLast = LOOP_TIME;

NMEAParser<2> parser;
Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();
BNO08x_RVC_Data heading;

uint32_t AOGTime;
uint32_t NtripTime;

bool ADSfound = false;
byte PGNlength;

HardwareSerial* SerialIMU;
HardwareSerial* SerialReceiver;

elapsedMillis imuDelayTimer;
bool isGGA_Updated = false;
int ADS1115_Address;

byte DataConfig[MaxReadBuffer];
uint16_t PGNconfig;

bool IMUstarted = false;

// firmware update
EthernetUDP UpdateComm;
uint16_t UpdateReceivePort = 29100;
uint16_t UpdateSendPort = 29000;
uint32_t buffer_addr, buffer_size;
bool UpdateMode = false;

//******************************************************************************
// hex_info_t struct for hex record and hex file info
//******************************************************************************
typedef struct {  //
	char* data;   // pointer to array allocated elsewhere
	unsigned int addr;  // address in intel hex record
	unsigned int code;  // intel hex record type (0=data, etc.)
	unsigned int num; // number of data bytes in intel hex record

	uint32_t base;  // base address to be added to intel hex 16-bit addr
	uint32_t min;   // min address in hex file
	uint32_t max;   // max address in hex file

	int eof;    // set true on intel hex EOF (code = 1)
	int lines;    // number of hex records received
} hex_info_t;

static char data[16];// buffer for hex data

hex_info_t hex =
{ // intel hex info struct
  data, 0, 0, 0,        //   data,addr,num,code
  0, 0xFFFFFFFF, 0,     //   base,min,max,
  0, 0					//   eof,lines
};

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
		ReceiveEthernetConfig();
	}
	ReadIMU();
	DoPanda();
	ReceiveSteerData();
	ReceiveUpdate();
	Blink();
	if (GPSserial->available()) SerialOut->write(GPSserial->read());
}

bool State = false;
elapsedMillis BlinkTmr;
byte ResetRead;
elapsedMicros LoopTmr;
uint32_t MaxLoopTime;
uint16_t debug1;

void Blink()
{
	if (BlinkTmr > 1000)
	{
		BlinkTmr = 0;
		State = !State;
		digitalWrite(LED_BUILTIN, State);
		if (!UpdateMode)
		{
			Serial.print(" Micros: ");
			Serial.print(MaxLoopTime);

			Serial.print(", WAS: ");
			Serial.print(helloSteerPosition);

			Serial.print(", Heading: ");
			Serial.print(IMU_Heading / 10.0);

			//Serial.print(", ");
			//Serial.print(debug1);

			Serial.println("");

			if (ResetRead++ > 5)
			{
				MaxLoopTime = 0;
				ResetRead = 0;
			}
		}
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
	int CK = 0;
	for (int i = Start; i < Length; i++)
	{
		CK += Chk[i];
	}
	Result = (byte)CK;
	return Result;
}

