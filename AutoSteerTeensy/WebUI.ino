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
	st += "<table class='center'>";
	st += "<tr><td class='label-col'>IP Address</td><td class='input-col'>" + webIP() + "</td></tr>";
	st += "<tr><td class='label-col'>Firmware ID</td><td class='input-col'>" + String(InoID) + "</td></tr>";
	st += "<tr><td class='label-col'>Board Label</td><td class='input-col'>" + webBoardLabel() + "</td></tr>";
	st += "</table>";
	st += "<p><a class='button-72' href='/pins'>Pins &amp; Ports</a></p>";
	st += "<p><a class='button-72' href='/modes'>Modes</a></p>";
	st += "<p><a class='button-72' href='/label'>Board Label</a></p>";
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
	st += "<p><input class='button-72' id='submitBtn' type='submit' value='Save / Restart'></p>";
	st += "</form><p><a href='/'>Back</a></p>";
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
	st += webRowCheck("Zero WAS now",  "zerowas",  false);
	st += "</table>";
	st += "<p><input class='button-72' id='submitBtn' type='submit' value='Save / Restart'></p>";
	st += "</form><p><a href='/'>Back</a></p>";
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
	if (webChecked(b, "zerowas")) MDL.ZeroOffset = WasReading;	// one-shot, mirrors PGN Commands bit 0
	SaveData();
	webScheduleReboot();
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
		return webPageMain();
	}
	if      (path == "/pins")  return webPagePins();
	else if (path == "/modes") return webPageModes();
	else if (path == "/label") return webPageLabel();
	return webPageMain();
}

void HandleWebClient()
{
	EthernetClient client = webServer.available();
	if (!client) return;

	String reqLine, line, body;
	int contentLength = 0;
	bool firstLine = true, headersDone = false;
	uint32_t start = millis();

	// request line + headers (bounded so a stuck client can't stall the steer loop)
	while (client.connected() && !headersDone && (millis() - start < 300))
	{
		if (!client.available()) continue;
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

	// body (POST)
	while ((int)body.length() < contentLength && client.connected() && (millis() - start < 600))
	{
		if (client.available()) body += (char)client.read();
	}

	bool isPost = reqLine.startsWith("POST");
	int sp1 = reqLine.indexOf(' ');
	int sp2 = reqLine.indexOf(' ', sp1 + 1);
	String path = (sp1 >= 0 && sp2 > sp1) ? reqLine.substring(sp1 + 1, sp2) : "/";

	String resp = webBuildResponse(isPost, path, body);

	client.print(F("HTTP/1.1 200 OK\r\n"));
	client.print(F("Content-Type: text/html\r\n"));
	client.print(F("Connection: close\r\n"));
	client.print(F("Content-Length: "));
	client.print(resp.length());
	client.print(F("\r\n\r\n"));

	// Send the body in full. QNEthernet's write() buffers only as much as fits and returns
	// a short count when its send buffer is full, so a single print() truncates large pages
	// (e.g. /pins). Loop + flush until every byte is out.
	const char* p = resp.c_str();
	size_t remaining = resp.length();
	uint32_t tsend = millis();
	while (remaining > 0 && client.connected() && (millis() - tsend < 2000))
	{
		size_t n = client.write((const uint8_t*)p, remaining);
		client.flush();
		if (n > 0) { p += n; remaining -= n; tsend = millis(); }
		else delay(1);
	}
	client.stop();
}

void DoWebUI()
{
	Ethernet.loop();	// service QNEthernet (link detection + lwIP) every pass
	HandleWebClient();	// serve the settings web page (non-blocking, one client per pass)

	// deferred reset after a web settings change (lets the HTTP reply go out first)
	if (webRebootPending && millis() > webRebootAt) SCB_AIRCR = 0x05FA0004;
}
