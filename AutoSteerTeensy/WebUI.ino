// Hand-rolled settings web page served over QNEthernet (EthernetServer on port 80).
// Reachable at http://<module IP> (e.g. http://192.168.10.126).
// Pages: /  /pins  /modes  /label
//
// POST handlers write the same structs and call the same Save*() functions as the
// PGN config path (ReceiveConfig), then defer a reset via webRebootPending so the
// HTTP reply flushes first. The PGN/PCBsetup path is unaffected.

// ---------- small helpers ----------

static uint8_t webClampByte(const String& s)
{
	long v = s.toInt();
	if (v < 0)   v = 0;
	if (v > 255) v = 255;
	return (uint8_t)v;
}

static String webUrlDecode(const String& s)
{
	String out;
	out.reserve(s.length());
	for (size_t i = 0; i < s.length(); i++)
	{
		char c = s[i];
		if (c == '+')
		{
			out += ' ';
		}
		else if (c == '%' && i + 2 < s.length())
		{
			char h[3] = { s[i + 1], s[i + 2], 0 };
			out += (char)strtol(h, nullptr, 16);
			i += 2;
		}
		else
		{
			out += c;
		}
	}
	return out;
}

// pull one field out of an x-www-form-urlencoded body (returns "" if absent)
static String webField(const String& body, const char* name)
{
	String hay = "&" + body;
	String key = "&" + String(name) + "=";
	int i = hay.indexOf(key);
	if (i < 0) return "";
	i += key.length();
	int end = hay.indexOf('&', i);
	if (end < 0) end = hay.length();
	return webUrlDecode(hay.substring(i, end));
}

static bool webChecked(const String& body, const char* name)
{
	// an unchecked checkbox is not submitted at all
	return webField(body, name).length() > 0;
}

static String webEscape(const String& s)
{
	String out;
	out.reserve(s.length() + 8);
	for (uint16_t i = 0; i < s.length(); i++)
	{
		char c = s[i];
		switch (c)
		{
		case '&': out += "&amp;";  break;
		case '<': out += "&lt;";   break;
		case '>': out += "&gt;";   break;
		case '"': out += "&quot;"; break;
		default:  out += c;        break;
		}
	}
	return out;
}

static String webIP()
{
	IPAddress ip = Ethernet.localIP();
	return String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);
}

static String webBoardLabel()
{
	char buf[17];
	memcpy(buf, MDLboard.Text, 16);
	buf[16] = 0;
	return webEscape(String(buf));
}

static void webScheduleReboot()
{
	webRebootPending = true;
	webRebootAt = millis() + 1500;	// leave time for the reply to be sent
}

// firmware version string, decoded from InoID exactly like DoBegin() in Begin.ino (ddmmy)
static String webFwVersion()
{
	uint16_t yr   = InoID % 10 + 2020;
	uint16_t rest = InoID / 10;
	uint8_t  mn   = rest % 100;
	uint16_t dy   = rest / 100;

	if (mn > 12 || dy > 31) return "invalid";

	String v = "v";
	v += String(yr);
	v += ".";
	if (mn < 10) v += "0";
	v += String(mn);
	v += ".";
	if (dy < 10) v += "0";
	v += String(dy);
	return v;
}

// green/grey indicator for a boolean flag
static String webFlag(bool on)
{
	return on ? "<span style='color:#127a12;font-weight:700;'>ON</span>"
	          : "<span style='color:#999;'>off</span>";
}

// one status row; the value cell carries an id so /status polling can update it in place
static String webRowStatus(const char* label, const char* id, const String& value)
{
	String st = "<tr><td class='label-col'>";
	st += label;
	st += "</td><td class='input-col' id='";
	st += id;
	st += "'>";
	st += value;
	st += "</td></tr>";
	return st;
}

// compact JSON snapshot of the PGN 32505 fields, polled by the main page
static String webStatusJson()
{
	int16_t currentWas = (int16_t)WasReading - (int16_t)MDL.ZeroOffset;
	String j = "{";
	j += "\"h\":"    + String(ATT_Heading / 10.0f, 1);	// ATT_Heading is decidegrees (0-3600)
	j += ",\"was\":" + String(WasReading);
	j += ",\"cwas\":"+ String(currentWas);
	j += ",\"zo\":"  + String(MDL.ZeroOffset);
	j += ",\"cur\":" + String((int)AnalogReadingAverage);
	j += ",\"loop\":"+ String(MaxLoopTime);
	j += ",\"imu\":" + String(SerialIMUEnabled ? 1 : 0);
	j += ",\"rcv\":" + String(SerialReceiverEnabled ? 1 : 0);
	j += ",\"pt\":"  + String(SerialPassThruEnabled ? 1 : 0);
	j += ",\"ads\":" + String(ADSfound ? 1 : 0);
	j += ",\"aog\":" + String((millis() - AOGTime < 4000) ? 1 : 0);
	j += ",\"steer\":" + String(AOGsteeringReady ? 1 : 0);
	j += ",\"sw\":"  + String(SteerSwitch == LOW ? 1 : 0);
	j += "}";
	return j;
}

// ---------- theme (ported verbatim from the ESP32 module's PgNetwork.ino GetPage2) ----------

static String webCss()
{
	String st = "";
	st += "html { font-family: Helvetica, Arial, sans-serif; display:inline-block; margin:0 auto; text-align:center; }";
	st += "body { margin-top:50px; background-color:wheat; font-family:Arial, Helvetica, Sans-Serif; }";
	st += "h1 { color:#444444; margin:50px auto 12px; text-decoration: underline; }";
	st += "h1.subhead { margin:20px auto 12px; }";
	st += "table.center { margin-left:auto; margin-right:auto; border-collapse:collapse; table-layout:fixed; }";
	st += "td.label-col { width:200px; text-align:left; padding:8px 12px; vertical-align:middle; }";
	st += "td.input-col { width:320px; padding:8px 12px; vertical-align:middle; }";
	st += ".control-width { width:320px; max-width:90%; margin:0 auto; box-sizing:border-box; }";
	st += ".InputCell { display:block; width:100%; height:36px; box-sizing:border-box; text-align:center; font-size:18px; font-weight:700; padding:4px 6px; }";
	st += ".button-72 {";
	st += "  align-items: center;";
	st += "  background-color: initial;";
	st += "  background-image: linear-gradient(rgba(179, 132, 201, .84), rgba(57, 31, 91, .84) 50%);";
	st += "  border-radius: 42px;";
	st += "  border-width: 0;";
	st += "  box-shadow: rgba(57, 31, 91, 0.24) 0 2px 2px, rgba(179, 132, 201, 0.4) 0 8px 12px;";
	st += "  color: #FFFFFF;";
	st += "  cursor: pointer;";
	st += "  display: inline-flex;";
	st += "  font-family: Quicksand, sans-serif;";
	st += "  font-size: 18px;";
	st += "  font-weight: 700;";
	st += "  justify-content: center;";
	st += "  letter-spacing: .04em;";
	st += "  line-height: 16px;";
	st += "  margin: 12px auto;";
	st += "  padding: 12px 18px;";
	st += "  text-align: center;";
	st += "  text-decoration: none;";
	st += "  text-shadow: rgba(255, 255, 255, 0.4) 0 0 4px, rgba(255, 255, 255, 0.2) 0 0 12px, rgba(57, 31, 91, 0.6) 1px 1px 4px, rgba(57, 31, 91, 0.32) 4px 4px 16px;";
	st += "  user-select: none;";
	st += "  -webkit-user-select: none;";
	st += "  touch-action: manipulation;";
	st += "  vertical-align: baseline;";
	st += "  width:320px;";
	st += "  max-width:90%;";
	st += "}";
	st += "#submitBtn { margin-top: 36px; }";
	st += "a:link { font-size:150%; }";
	st += ".checkbox-row { display:flex; align-items:center; height:44px; box-sizing:border-box; }";
	st += ".checkbox-left { display:flex; align-items:center; justify-content:flex-start; }";
	st += "input[type=checkbox].styled {";
	st += "  -webkit-appearance: none; appearance: none;";
	st += "  width:44px; height:44px; display:inline-block; position:relative; margin:0; padding:0; box-sizing:border-box;";
	st += "  border-radius:10px;";
	st += "  background-image: linear-gradient(rgba(179,132,201,.84), rgba(57,31,91,.84) 50%);";
	st += "  box-shadow: rgba(57,31,91,0.24) 0 2px 2px, rgba(179,132,201,0.4) 0 8px 12px;";
	st += "  cursor:pointer; vertical-align:middle; outline: none; border: 1px solid rgba(57,31,91,0.25);";
	st += "}";
	st += "input[type=checkbox].styled::after {";
	st += "  content: ''; position: absolute; left: 50%; top: 50%;";
	st += "  width: 12px; height: 22px;";
	st += "  border-right: 4px solid white; border-bottom: 4px solid white;";
	st += "  transform: translate(-50%,-60%) rotate(45deg) scale(0);";
	st += "  transform-origin: center; transition: transform 0.12s ease-in-out; border-radius:2px;";
	st += "}";
	st += "input[type=checkbox].styled:checked::after { transform: translate(-50%,-60%) rotate(45deg) scale(1); }";
	st += "input[type=checkbox].styled:focus { box-shadow: 0 0 0 3px rgba(179,132,201,0.22); }";
	st += "select.InputCell { -webkit-appearance:menulist; appearance:menulist; }";
	st += ".label-normal { font-weight:normal; }";
	st += ".hint { font-size: 12px; color: #333; margin-top: 4px; }";
	st += ".status { margin: 2px auto 16px; font-size: 16px; }";
	st += ".preset-btn { background-image: linear-gradient(rgba(179,132,201,.84), rgba(57,31,91,.84) 50%); color:#fff; border:0; border-radius:20px; padding:8px 16px; margin:4px; font-size:16px; font-weight:700; cursor:pointer; }";
	return st;
}

static String webHead(const String& title, const char* refresh)
{
	String st = "<!DOCTYPE html><html><head><meta charset='utf-8'>";
	st += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
	if (refresh) { st += "<meta http-equiv='refresh' content='"; st += refresh; st += "'>"; }
	st += "<title>" + title + "</title>";
	st += "<style>" + webCss() + "</style></head><body>";
	return st;
}

static String webFoot()
{
	return "</body></html>";
}

// ---------- row builders ----------

static String webRowNum(const char* label, const char* name, int value)
{
	String st = "<tr><td class='label-col'><span class='label-normal'>";
	st += label;
	st += "</span></td><td class='input-col'><div class='control-width'>";
	st += "<input class='InputCell' type='number' min='0' max='255' name='";
	st += name;
	st += "' value='" + String(value) + "'></div></td></tr>";
	return st;
}

static String webRowSelect2(const char* label, const char* name, int value, const char* opt0, const char* opt1)
{
	String st = "<tr><td class='label-col'><span class='label-normal'>";
	st += label;
	st += "</span></td><td class='input-col'><div class='control-width'>";
	st += "<select class='InputCell' name='";
	st += name;
	st += "'>";
	st += "<option value='0'"; if (value == 0) st += " selected"; st += ">"; st += opt0; st += "</option>";
	st += "<option value='1'"; if (value == 1) st += " selected"; st += ">"; st += opt1; st += "</option>";
	st += "</select></div></td></tr>";
	return st;
}

static String webRowCheck(const char* label, const char* name, bool checked)
{
	String st = "<tr><td class='label-col'><span class='label-normal'>";
	st += label;
	st += "</span></td><td class='input-col'><div class='control-width'><div class='checkbox-row'><div class='checkbox-left'>";
	st += "<label style='display:inline-flex;align-items:center;gap:8px;cursor:pointer;'>";
	st += "<input class='styled' type='checkbox' name='";
	st += name;
	st += "' value='1'";
	if (checked) st += " checked";
	st += "></label></div></div></div></td></tr>";
	return st;
}

// ---------- pages ----------

static String webPageMain()
{
	String st = webHead("AutoSteer Setup", nullptr);

	st += "<h1 align=center>AutoSteer</h1>";

	// --- navigation ---
	st += "<p><a class='button-72' href='/pins'>Pins &amp; Ports</a></p>";
	st += "<p><a class='button-72' href='/modes'>Modes</a></p>";
	st += "<p><a class='button-72' href='/label'>Board Label</a></p>";

	// --- identity ---
	st += "<table class='center'>";
	st += "<tr><td class='label-col'>IP Address</td><td class='input-col'>" + webIP() + "</td></tr>";
	st += "<tr><td class='label-col'>Firmware Version</td><td class='input-col'>" + webFwVersion() + "</td></tr>";
	st += "<tr><td class='label-col'>Board Label</td><td class='input-col'>" + webBoardLabel() + "</td></tr>";
	st += "</table>";

	// --- live status (mirrors PGN 32505, see SendStatus() in Config.ino) ---
	// Values are seeded server-side, then refreshed by polling /status (small JSON)
	// so the heavy HTML page is served only once and the steer loop isn't hammered.
	int16_t currentWas = (int16_t)WasReading - (int16_t)MDL.ZeroOffset;
	st += "<table class='center'>";
	st += webRowStatus("IMU heading",       "h",     String(ATT_Heading / 10.0f, 1));	// decidegrees -> degrees
	st += webRowStatus("WAS reading",       "was",   String(WasReading));
	st += webRowStatus("Current WAS",       "cwas",  String(currentWas));
	st += webRowStatus("Zero offset",       "zo",    String(MDL.ZeroOffset));
	st += webRowStatus("Current sensor",    "cur",   String((int)AnalogReadingAverage));
	st += webRowStatus("Max loop time",     "loop",  String(MaxLoopTime) + " us");
	st += webRowStatus("IMU enabled",       "imu",   webFlag(SerialIMUEnabled));
	st += webRowStatus("Receiver enabled",  "rcv",   webFlag(SerialReceiverEnabled));
	st += webRowStatus("Pass-thru enabled", "pt",    webFlag(SerialPassThruEnabled));
	st += webRowStatus("ADS1115 found",     "ads",   webFlag(ADSfound));
	st += webRowStatus("AOG connected",     "aog",   webFlag(millis() - AOGTime < 4000));
	st += webRowStatus("Steering on",       "steer", webFlag(AOGsteeringReady));
	st += webRowStatus("Steer switch on",   "sw",    webFlag(SteerSwitch == LOW));
	st += "</table>";

	// one-click WAS zero: captures the current WAS as the new zero (no reboot needed)
	st += "<p><button class='button-72' type='button' onclick='Z()'>Zero WAS now</button></p>";

	// poll the lightweight /status endpoint instead of reloading the whole page
	st += "<script>";
	st += "function T(i,v){var e=document.getElementById(i);if(e)e.textContent=v;}";
	st += "function F(i,v){var e=document.getElementById(i);if(e)e.innerHTML=v?\"<span style='color:#127a12;font-weight:700;'>ON</span>\":\"<span style='color:#999;'>off</span>\";}";
	st += "function U(j){";
	st += "T('h',j.h);T('was',j.was);T('cwas',j.cwas);T('zo',j.zo);T('cur',j.cur);T('loop',j.loop+' us');";
	st += "F('imu',j.imu);F('rcv',j.rcv);F('pt',j.pt);F('ads',j.ads);F('aog',j.aog);F('steer',j.steer);F('sw',j.sw);}";
	st += "function P(){fetch('/status').then(function(r){return r.json();}).then(U).catch(function(){});}";
	st += "function Z(){if(confirm('Set the current WAS reading as zero?'))";
	st += "fetch('/zero',{method:'POST'}).then(function(r){return r.json();}).then(U).catch(function(){});}";
	st += "setInterval(P,1000);";
	st += "</script>";

	st += webFoot();
	return st;
}

static String webPagePins()
{
	String st = webHead("Pins & Ports", nullptr);
	st += "<h1 align=center>Pins &amp; Ports</h1>";
	st += "<form method='post' action='/pins'><table class='center'>";
	st += webRowNum("Power relay pin",   "PowerRelayPin",        MDL.PowerRelayPin);
	st += webRowNum("Steer relay pin",   "SteeringRelayPin",     MDL.SteeringRelayPin);
	st += webRowNum("WAS pin",           "WasPin",               MDL.WasPin);
	st += webRowNum("Current pin",       "AnalogPin",            MDL.AnalogPin);
	st += webRowNum("Steer switch pin",  "SteerSwitchPin",       MDL.SteerSwitchPin);
	st += webRowNum("Work switch pin",   "WorkSwitchPin",        MDL.WorkSwitchPin);
	st += webRowNum("Dir pin",           "DirPin",               MDL.DirPin);
	st += webRowNum("PWM pin",           "PWMpin",               MDL.PWMpin);
	st += webRowNum("Receiver serial port", "ReceiverSerialPort",   MDL.ReceiverSerialPort);
	st += webRowNum("RS232 out port",       "PassThrOutSerialPort", MDL.PassThrOutSerialPort);
	st += webRowNum("RS232 in port",        "PassThruInSerialPort", MDL.PassThruInSerialPort);
	st += webRowNum("IMU serial port",      "IMUSerialPort",        MDL.IMUSerialPort);
	st += "</table>";
	st += "<hr style='width:80%;margin:28px auto 8px;border:0;border-top:2px solid #b38fc9;'>";
	st += "<div style='text-align:center;margin:8px auto 4px;'><span class='label-normal'>Load board preset</span><br>";
	st += "<button type='button' class='preset-btn' onclick=\"preset('AS15')\">AS15</button>";
	st += "<button type='button' class='preset-btn' onclick=\"preset('AS15-2')\">AS15-2</button>";
	st += "<button type='button' class='preset-btn' onclick=\"preset('AS15-3')\">AS15-3</button>";
	st += "<div class='hint'>Fills the fields above (blank pins = 255, not connected). Nothing is saved until you press Save / Restart.</div></div>";
	st += "<p style='margin-top:8px'><input class='button-72' type='submit' value='Save / Restart'></p>";
	st += "</form><p><a href='/'>Back</a></p>";
	// board presets (values from the PCB reference table); blanks = 255 (NC)
	st += "<script>var P={";
	st += "'AS15':{ReceiverSerialPort:8,IMUSerialPort:5,PassThruInSerialPort:255,PassThrOutSerialPort:255,PowerRelayPin:0,SteeringRelayPin:7,SteerSwitchPin:26,WorkSwitchPin:27,WasPin:255,AnalogPin:255,DirPin:23,PWMpin:22},";
	st += "'AS15-2':{ReceiverSerialPort:8,IMUSerialPort:5,PassThruInSerialPort:3,PassThrOutSerialPort:1,PowerRelayPin:255,SteeringRelayPin:7,SteerSwitchPin:26,WorkSwitchPin:27,WasPin:255,AnalogPin:255,DirPin:23,PWMpin:22},";
	st += "'AS15-3':{ReceiverSerialPort:4,IMUSerialPort:3,PassThruInSerialPort:8,PassThrOutSerialPort:2,PowerRelayPin:0,SteeringRelayPin:1,SteerSwitchPin:30,WorkSwitchPin:31,WasPin:25,AnalogPin:26,DirPin:23,PWMpin:22}";
	st += "};function preset(b){var p=P[b];for(var k in p){var e=document.getElementsByName(k);if(e.length)e[0].value=p[k];}}</script>";
	st += webFoot();
	return st;
}

static String webPageModes()
{
	String st = webHead("Modes", nullptr);
	st += "<h1 align=center>Modes</h1>";
	st += "<form method='post' action='/modes'><table class='center'>";
	st += webRowSelect2("GPS source",    "GPSSource",    MDL.GPSSource,    "F9P + IMU",   "ByNav");
	st += webRowSelect2("Steering mode", "SteeringMode", MDL.SteeringMode, "Steer tractor", "Steer tool");
	st += webRowSelect2("IMU type",      "IMUtype",      MDL.IMUtype,      "BNO080",      "TM171");
	st += webRowCheck("Use ADS1115",  "ads",      MDL.ADS1115Enabled);
	st += webRowCheck("Auto-zero WAS", "autozero", MDL.AutoZero);
	st += "</table>";
	st += "<hr style='width:80%;margin:32px auto 8px;border:0;border-top:2px solid #b38fc9;'>";
	st += "<div style='text-align:center;margin:8px auto 24px;'><span class='label-normal'>ADS1115 board preset</span><br>";
	st += "<button type='button' class='preset-btn' onclick=\"presetAds('AS15')\">AS15</button>";
	st += "<button type='button' class='preset-btn' onclick=\"presetAds('AS15-2')\">AS15-2</button>";
	st += "<button type='button' class='preset-btn' onclick=\"presetAds('AS15-3')\">AS15-3</button>";
	st += "<div class='hint'>Sets Use ADS1115 &mdash; AS15 / AS15-2: ON, AS15-3: OFF. Nothing is saved until you press Save / Restart.</div></div>";
	st += "<p><input class='button-72' id='submitBtn' type='submit' value='Save / Restart'></p>";
	st += "</form><p><a href='/'>Back</a></p>";
	st += "<script>var A={'AS15':true,'AS15-2':true,'AS15-3':false};";
	st += "function presetAds(b){document.getElementsByName('ads')[0].checked=A[b];}</script>";
	st += webFoot();
	return st;
}

static String webPageLabel()
{
	String st = webHead("Board Label", nullptr);
	st += "<h1 align=center>Board Label</h1>";
	st += "<form method='post' action='/label'><table class='center'>";
	st += "<tr><td class='label-col'><span class='label-normal'>Label</span></td>";
	st += "<td class='input-col'><div class='control-width'>";
	st += "<input class='InputCell' type='text' maxlength='16' name='label' value='" + webBoardLabel() + "'>";
	st += "</div></td></tr>";
	st += "<tr><td colspan='2'><div class='control-width'><div class='hint'>Up to 16 characters. Survives a firmware reflash.</div></div></td></tr>";
	st += "</table>";
	st += "<p><input class='button-72' id='submitBtn' type='submit' value='Save'></p>";
	st += "</form><p><a href='/'>Back</a></p>";
	st += webFoot();
	return st;
}

static String webPageSaved(bool rebooting)
{
	String st = rebooting ? webHead("Saved", "4; url=/") : webHead("Saved", nullptr);
	st += "<h1 align=center>Saved</h1>";
	if (rebooting)
		st += "<p class='status' style='text-align:center'>Settings saved. Module is restarting&hellip;</p>";
	else
		st += "<p class='status' style='text-align:center'>Board label saved.</p>";
	st += "<p style='text-align:center'><a href='/'>Home</a></p>";
	st += webFoot();
	return st;
}

// ---------- apply (POST) ----------

static void webApplyPins(const String& b)
{
	MDL.PowerRelayPin        = webClampByte(webField(b, "PowerRelayPin"));
	MDL.SteeringRelayPin     = webClampByte(webField(b, "SteeringRelayPin"));
	MDL.WasPin               = webClampByte(webField(b, "WasPin"));
	MDL.AnalogPin            = webClampByte(webField(b, "AnalogPin"));
	MDL.SteerSwitchPin       = webClampByte(webField(b, "SteerSwitchPin"));
	MDL.WorkSwitchPin        = webClampByte(webField(b, "WorkSwitchPin"));
	MDL.DirPin               = webClampByte(webField(b, "DirPin"));
	MDL.PWMpin               = webClampByte(webField(b, "PWMpin"));
	MDL.ReceiverSerialPort   = webClampByte(webField(b, "ReceiverSerialPort"));
	MDL.PassThrOutSerialPort = webClampByte(webField(b, "PassThrOutSerialPort"));
	MDL.PassThruInSerialPort = webClampByte(webField(b, "PassThruInSerialPort"));
	MDL.IMUSerialPort        = webClampByte(webField(b, "IMUSerialPort"));
	SaveData();
	webScheduleReboot();
}

static void webApplyModes(const String& b)
{
	MDL.GPSSource      = (uint8_t)webField(b, "GPSSource").toInt();
	MDL.SteeringMode   = (uint8_t)webField(b, "SteeringMode").toInt();
	MDL.IMUtype        = (uint8_t)webField(b, "IMUtype").toInt();
	MDL.ADS1115Enabled = webChecked(b, "ads");
	MDL.AutoZero       = webChecked(b, "autozero");
	SaveData();
	webScheduleReboot();
}

// One-shot WAS zero (dedicated "Zero WAS now" button on the main page). Mirrors the
// PGN Commands bit 0 path in ReceiveConfig(), but needs no reboot: DoSteering() reads
// MDL.ZeroOffset live every loop, so the new zero takes effect immediately and the
// status panel shows Current WAS drop to ~0. SaveData() persists it across restarts.
static void webApplyZero()
{
	MDL.ZeroOffset = WasReading;
	SaveData();
}

static void webApplyLabel(const String& b)
{
	String t = webField(b, "label");
	memset(MDLboard.Text, 0, sizeof(MDLboard.Text));
	for (uint8_t i = 0; i < 16 && i < t.length(); i++) MDLboard.Text[i] = t[i];
	SaveBoardID();
}

// ---------- response + request handling ----------

// build the response for a given method + path (no EthernetClient in the signature,
// so the Arduino auto-prototype generator can't trip over the QNEthernet type)
static String webBuildResponse(bool isPost, const String& path, const String& body)
{
	if (isPost)
	{
		if      (path == "/pins")  { webApplyPins(body);  return webPageSaved(true);  }
		else if (path == "/modes") { webApplyModes(body); return webPageSaved(true);  }
		else if (path == "/label") { webApplyLabel(body); return webPageSaved(false); }
		else if (path == "/zero")  { webApplyZero();      return webStatusJson();     }
		return webPageMain();
	}
	if      (path == "/status") return webStatusJson();
	else if (path == "/pins")  return webPagePins();
	else if (path == "/modes") return webPageModes();
	else if (path == "/label") return webPageLabel();
	return webPageMain();
}

// Non-blocking HTTP handler. Reads the request incrementally across loop passes
// (only whatever bytes are already buffered each pass, never busy-waiting), and
// tears the socket down with the non-blocking close() instead of stop(). The old
// version blocked the steer loop on every request: it spun waiting for the request
// bytes and then called client.stop(), which blocks until the TCP close round-trip
// completes. That injected a stall into the loop on every 1 s status poll.
void HandleWebClient()
{
	static EthernetClient client;
	static bool     busy = false;
	static String   reqLine, line, body;
	static int      contentLength;
	static bool     firstLine, headersDone;
	static uint32_t start;

	if (!busy)
	{
		client = webServer.available();
		if (!client) return;
		busy = true;
		reqLine = ""; line = ""; body = "";
		contentLength = 0; firstLine = true; headersDone = false;
		start = millis();
	}

	// give up on a client that stalls, so a half-open connection can't wedge us
	if (millis() - start > 1000) { client.close(); busy = false; return; }

	// request line + headers: consume only what's buffered right now, then return
	while (!headersDone && client.available())
	{
		char c = client.read();
		if (c == '\r') continue;
		if (c == '\n')
		{
			if (firstLine) { reqLine = line; firstLine = false; }
			else
			{
				String low = line; low.toLowerCase();
				if (low.startsWith("content-length:"))
					contentLength = line.substring(line.indexOf(':') + 1).toInt();
			}
			if (line.length() == 0) headersDone = true;
			line = "";
		}
		else line += c;
	}

	if (!headersDone)
	{
		if (!client.connected()) { client.close(); busy = false; }
		return;	// come back next pass with more bytes
	}

	// body (POST): same incremental approach
	while ((int)body.length() < contentLength && client.available())
		body += (char)client.read();

	if ((int)body.length() < contentLength)
	{
		if (!client.connected()) { client.close(); busy = false; }
		return;
	}

	bool isPost = reqLine.startsWith("POST");
	int sp1 = reqLine.indexOf(' ');
	int sp2 = reqLine.indexOf(' ', sp1 + 1);
	String path = (sp1 >= 0 && sp2 > sp1) ? reqLine.substring(sp1 + 1, sp2) : "/";

	String resp = webBuildResponse(isPost, path, body);
	bool isJson = (path == "/status") || (isPost && path == "/zero");

	client.print(F("HTTP/1.1 200 OK\r\n"));
	client.print(isJson ? F("Content-Type: application/json\r\n") : F("Content-Type: text/html\r\n"));
	client.print(F("Connection: close\r\n"));
	client.print(F("Content-Length: "));
	client.print(resp.length());
	client.print(F("\r\n\r\n"));

	// Write the body. QNEthernet's write() returns a short count when its send buffer
	// is full, so loop until every byte is queued (small /status responses go in one
	// pass; large pages like /pins take a few). close() is non-blocking: it flushes the
	// FIN through the stack without waiting for the peer, unlike stop().
	const char* p = resp.c_str();
	size_t remaining = resp.length();
	uint32_t tsend = millis();
	while (remaining > 0 && client.connected() && (millis() - tsend < 1000))
	{
		size_t n = client.write((const uint8_t*)p, remaining);
		if (n > 0) { p += n; remaining -= n; tsend = millis(); }
		client.flush();
		Ethernet.loop();	// service the stack so it pushes/drains the send buffer (non-blocking)
	}
	client.close();
	busy = false;
}

void DoWebUI()
{
	Ethernet.loop();	// service QNEthernet (link detection + lwIP) every pass
	HandleWebClient();	// serve the settings web page (non-blocking, one client per pass)

	// deferred reset after a web settings change (lets the HTTP reply go out first)
	if (webRebootPending && millis() > webRebootAt) SCB_AIRCR = 0x05FA0004;
}
