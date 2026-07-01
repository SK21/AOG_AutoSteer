// CAN.ino - CAN bus support for machine steering hardware (Teensy 4.1 / FlexCAN_T4)
//
// SCOPE: these buses talk to the STEERING HARDWARE (e.g. a CAN steering valve /
// ECU), NOT to AgOpenGPS. AOG communication stays on Ethernet/UDP with PGNs. AOG
// does not speak CAN, so there are deliberately NO AOG PGNs on these buses.
//
// This is the transport / plumbing layer only - no message protocol is defined
// yet. It provides: bring-up of both controllers, a receive pump, a send helper,
// and simple diagnostics. When the steering hardware's protocol is known, decode
// incoming frames in CAN_Dispatch() and build outgoing frames with CAN_Send().
// (Structure mirrors the RCteensy CANBus.ino, minus the ISOBUS/PGN specifics.)
//
// PINS: FlexCAN controller pins are fixed in silicon on the Teensy 4.1 and cannot
// be remapped, so each bus MUST use its controller's dedicated pin pair:
//
//   CanA = FlexCAN CAN1 : CTX1 = pin 22, CRX1 = pin 23
//   CanB = FlexCAN CAN3 : CTX3 = pin 31, CRX3 = pin 30
//
// (The Teensy 4.1's third controller, CAN2, lives on pins 0/1 and is unused here.)
//
// NOTE - pin reuse: on the classic (non-CAN) board these same pins drive the motor
// output (PWM = 22, DIR = 23) and the switch inputs (Steer = 30, Work = 31). A CAN
// steering board therefore cannot also use those as GPIO. Keep STEER_CAN_ENABLED
// OFF on classic PWM/analog boards - it is off by default for exactly this reason.

// ---- build switch -----------------------------------------------------------
// Leave this commented out on classic PWM/analog steering boards: CAN then compiles
// to nothing (no flash, no RAM, no pins touched). Define it only on a PCB actually
// wired for CAN, where pins 22/23 and 30/31 are routed to CAN transceivers.
// #define STEER_CAN_ENABLED

#ifdef STEER_CAN_ENABLED

#include <FlexCAN_T4.h>

// Uncomment for verbose per-frame RX logging on Serial (development only).
// #define STEER_CAN_DEBUG

// Bus identifiers used by CAN_Send() / CAN_Dispatch().
#define CAN_BUS_A 0    // FlexCAN CAN1, pins 22/23
#define CAN_BUS_B 1    // FlexCAN CAN3, pins 30/31

// Bit rate per bus. Most CAN steering valves (Danfoss PVED-CL, etc.) and ISOBUS run
// at 250 kbps; some hardware uses 500 kbps. Set per bus once the hardware is known.
static const uint32_t CanA_BaudRate = 250000;
static const uint32_t CanB_BaudRate = 250000;

// Per-bus runtime enables. Both transceivers are populated on the PCB, but a given
// machine typically uses only one. These are plain globals for now (no EEPROM / PGN
// plumbing yet) so the protocol layer can be built out first.
bool CanA_Enabled = true;
bool CanB_Enabled = false;

// Optional transceiver standby/enable pin per bus (e.g. STBY on an MCP2562).
// 255 = not driven by the MCU (standby tied inactive in hardware).
static const uint8_t CanA_StandbyPin = 255;
static const uint8_t CanB_StandbyPin = 255;

// FlexCAN controller instances. Template args are the RX/TX FIFO depths.
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CanA;    // pins 22/23
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CanB;    // pins 30/31

// Lightweight diagnostics - useful on Serial now, and for a future status report.
struct CanStats
{
	uint32_t rxCount = 0;
	uint32_t txCount = 0;
	uint32_t lastRxMillis = 0;
};
CanStats CanA_Stats;
CanStats CanB_Stats;

#ifdef STEER_CAN_DEBUG
static void CAN_LogFrame(uint8_t busId, const CAN_message_t& msg)
{
	Serial.print("CAN");
	Serial.print(busId == CAN_BUS_A ? "A" : "B");
	Serial.print(msg.flags.extended ? " ext 0x" : " std 0x");
	Serial.print(msg.id, HEX);
	Serial.print(" [");
	Serial.print(msg.len);
	Serial.print("]");
	for (uint8_t i = 0; i < msg.len; i++)
	{
		Serial.print(' ');
		Serial.print(msg.buf[i], HEX);
	}
	Serial.println();
}
#endif

//-----------------------------------------------------------------------------
// Frame dispatch - PROTOCOL HOOK.
// Machine-steering frame decoding goes here once the hardware's protocol is
// defined. Do NOT add AOG PGNs. For now it only (optionally) logs.
//-----------------------------------------------------------------------------
static void CAN_Dispatch(uint8_t busId, const CAN_message_t& msg)
{
#ifdef STEER_CAN_DEBUG
	CAN_LogFrame(busId, msg);
#endif
	// No message handlers yet - add steering-hardware decoding here.
	(void)busId;
	(void)msg;
}

//-----------------------------------------------------------------------------
// Bring up the CAN controllers. Called once from DoSetup().
//-----------------------------------------------------------------------------
void CAN_Begin()
{
	if (CanA_Enabled)
	{
		if (CanA_StandbyPin != 255)
		{
			pinMode(CanA_StandbyPin, OUTPUT);
			digitalWrite(CanA_StandbyPin, LOW);   // LOW = transceiver active
		}
		CanA.begin();
		CanA.setBaudRate(CanA_BaudRate);
		CanA.setMaxMB(16);
		CanA.setMBFilter(ACCEPT_ALL);             // accept all until protocol filters exist
		Serial.print("CAN bus A (CAN1, pins 22/23) started at ");
		Serial.print(CanA_BaudRate);
		Serial.println(" bps.");
	}

	if (CanB_Enabled)
	{
		if (CanB_StandbyPin != 255)
		{
			pinMode(CanB_StandbyPin, OUTPUT);
			digitalWrite(CanB_StandbyPin, LOW);
		}
		CanB.begin();
		CanB.setBaudRate(CanB_BaudRate);
		CanB.setMaxMB(16);
		CanB.setMBFilter(ACCEPT_ALL);
		Serial.print("CAN bus B (CAN3, pins 30/31) started at ");
		Serial.print(CanB_BaudRate);
		Serial.println(" bps.");
	}
}

//-----------------------------------------------------------------------------
// Drain both receive queues. Call every loop() (not gated by LOOP_TIME) so RX
// keeps up with bus traffic.
//-----------------------------------------------------------------------------
void CAN_Process()
{
	CAN_message_t msg;

	if (CanA_Enabled)
	{
		while (CanA.read(msg))
		{
			CanA_Stats.rxCount++;
			CanA_Stats.lastRxMillis = millis();
			CAN_Dispatch(CAN_BUS_A, msg);
		}
	}

	if (CanB_Enabled)
	{
		while (CanB.read(msg))
		{
			CanB_Stats.rxCount++;
			CanB_Stats.lastRxMillis = millis();
			CAN_Dispatch(CAN_BUS_B, msg);
		}
	}
}

//-----------------------------------------------------------------------------
// Send a frame on the given bus. id is 11-bit (standard) or 29-bit (extended).
// Returns true if the controller queued the frame.
//-----------------------------------------------------------------------------
bool CAN_Send(uint8_t busId, uint32_t id, const uint8_t* data, uint8_t len, bool extended)
{
	if (len > 8) len = 8;

	CAN_message_t msg;
	msg.id = id;
	msg.flags.extended = extended ? 1 : 0;
	msg.len = len;
	for (uint8_t i = 0; i < len; i++) msg.buf[i] = data[i];

	bool ok = false;
	switch (busId)
	{
	case CAN_BUS_A:
		if (CanA_Enabled && CanA.write(msg)) { CanA_Stats.txCount++; ok = true; }
		break;
	case CAN_BUS_B:
		if (CanB_Enabled && CanB.write(msg)) { CanB_Stats.txCount++; ok = true; }
		break;
	}
	return ok;
}

#else   // STEER_CAN_ENABLED not defined -> empty stubs so call sites still compile

void CAN_Begin() {}
void CAN_Process() {}

#endif  // STEER_CAN_ENABLED
