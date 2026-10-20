
void ReceiveSteerData()
{
    static const int MaxReadBuffer = 100;	// bytes

    if (Ethernet.linkStatus() == LinkON)
    {
        int len = UDPsteering.parsePacket();	// QNEthernet returns -1 when empty; use signed + > 0
        if (len > 0)
        {
            uint8_t SettingsByte;
            byte Data[MaxReadBuffer];
            UDPsteering.read(Data, MaxReadBuffer);
            if ((Data[0] == 0x80) && (Data[1] == 0x81 && Data[2] == 0x7F))  // 0x7F is source, AGIO
            {
                switch (Data[3])
                {
                case 200:
                    // hello from AGIO
                    SendHelloReply();
                    break;

                case 201:
                    // update IP
                    if (Data[4] == 5 && Data[5] == 201 && Data[6] == 201)
                    {
                        MDLnetwork.IP0 = Data[7];
                        MDLnetwork.IP1 = Data[8];
                        MDLnetwork.IP2 = Data[9];

                        SaveNetworks();

                        //reset the Teensy
                        SCB_AIRCR = 0x05FA0004;
                    }
                    break;

                case 202:
                    // ID scan request
                    if (Data[4] == 3 && Data[5] == 202 && Data[6] == 202)
                    {
                        SendScanIDreply();
                    }
                    break;

                case 251:
                    // steer config
                    SettingsByte = Data[5];

                    if (bitRead(SettingsByte, 0)) SteerConfig.InvertWAS = 1; else SteerConfig.InvertWAS = 0;
                    if (bitRead(SettingsByte, 1)) SteerConfig.InvertRelays = 1; else SteerConfig.InvertRelays = 0;
                    if (bitRead(SettingsByte, 2)) SteerConfig.InvertSteer = 1; else SteerConfig.InvertSteer = 0;
                    if (bitRead(SettingsByte, 3)) SteerConfig.SingleInputWAS = 1; else SteerConfig.SingleInputWAS = 0;
                    if (bitRead(SettingsByte, 4)) SteerConfig.CytronDriver = 1; else SteerConfig.CytronDriver = 0;
                    if (bitRead(SettingsByte, 5)) SteerConfig.SteerSwitch = 1; else SteerConfig.SteerSwitch = 0;
                    if (bitRead(SettingsByte, 6)) SteerConfig.SteerButton = 1; else SteerConfig.SteerButton = 0;
                    if (bitRead(SettingsByte, 7)) SteerConfig.ShaftEncoder = 1; else SteerConfig.ShaftEncoder = 0;

                    SteerConfig.PulseCountMax = Data[6];
                    //SteerConfig.MinSpeed = (float)Data[7] * 0.1;

                    SettingsByte = Data[8]; //setting1 - Danfoss valve etc

                    if (bitRead(SettingsByte, 0)) SteerConfig.IsDanfoss = 1; else SteerConfig.IsDanfoss = 0;
                    if (bitRead(SettingsByte, 1)) SteerConfig.PressureSensor = 1; else SteerConfig.PressureSensor = 0;
                    if (bitRead(SettingsByte, 2)) SteerConfig.CurrentSensor = 1; else SteerConfig.CurrentSensor = 0;
                    if (bitRead(SettingsByte, 3)) SteerConfig.UseIMU_Y_Axis = 1; else SteerConfig.UseIMU_Y_Axis = 0;

                    SaveData();

                    //reset the Teensy
                    SCB_AIRCR = 0x05FA0004;
                    break;

                case 252:
                    // autosteer settings
                    SteerSettings.Kp = Data[5];
                    SteerSettings.highPWM = Data[6];
                    SteerSettings.minPWM = Data[8];

                    //SteerSettings.lowPWM = Data[7];
                    SteerSettings.lowPWM = (byte)((float)SteerSettings.minPWM * 1.2);

                    SteerSettings.steerSensorCounts = (float)Data[9];
                    SteerSettings.wasOffset = Data[10] | Data[11] << 8;
                    SteerSettings.AckermanFix = (float)Data[12] * 0.01;

                    SaveData();
                    break;

                case 254:
                    // autosteer data
                    Speed_KMH = (Data[6] << 8 | Data[5]) * 0.1;

                    if (bitRead(Data[7], 0) == 1) AOGsteeringReady = true; else AOGsteeringReady = false;

                    steerAngleSetPoint = (float)((int16_t)(Data[9] << 8 | Data[8])) * 0.01;
                    SendSteerData();
                    AOGTime = millis();
                    break;

                case DiagPGN:
                    // steer diagnostics request from AOG's control data log
                    DiagRequested = true;
                    DiagRequestTime = millis();
                    break;
                }
            }
        }
    }
}

void SendSteerData()
{
    // Steer Data 1
    // steer angle
    int16_t tmp = (int)(steerAngleActual * 100);
    PGN_253[5] = (byte)tmp;
    PGN_253[6] = tmp >> 8;

    // heading and roll
    if (ATT_Connected())
    {
        tmp = (int)(ATT_Heading);
        PGN_253[7] = (byte)tmp;
        PGN_253[8] = tmp >> 8;
        tmp = (int)(ATT_Roll);
        PGN_253[9] = (byte)tmp;
        PGN_253[10] = tmp >> 8;
    }
    else
    {
        tmp = 9999;
        PGN_253[7] = (byte)tmp;
        PGN_253[8] = tmp >> 8;
        tmp = 8888;
        PGN_253[9] = (byte)tmp;
        PGN_253[10] = tmp >> 8;
    }

    PGN_253[9] = (byte)tmp;
    PGN_253[10] = tmp >> 8;

    PGN_253[11] = switchByte;
    PGN_253[12] = abs(pwmDrive);    // only works for positive values

    //add the checksum
    int16_t CK_A = 0;
    for (uint8_t i = 2; i < sizeof(PGN_253) - 1; i++)
    {
        CK_A = (CK_A + PGN_253[i]);
    }
    PGN_253[sizeof(PGN_253) - 1] = CK_A;

    // to ethernet
    if (Ethernet.linkStatus() == LinkON)
    {
        UDPsteering.beginPacket(DestinationIP, DestinationPort);
        UDPsteering.write(PGN_253, sizeof(PGN_253));
        UDPsteering.endPacket();
    }

    // Steer Data 2
    if (SteerConfig.PressureSensor || SteerConfig.CurrentSensor)
    {
        PGN_250[5] = (byte)AnalogReadingAverage;

        //add the checksum for AOG2
        CK_A = 0;
        for (uint8_t i = 2; i < sizeof(PGN_250) - 1; i++)
        {
            CK_A = (CK_A + PGN_250[i]);
        }
        PGN_250[sizeof(PGN_250) - 1] = CK_A;

        // to ethernet
        if (Ethernet.linkStatus() == LinkON)
        {
            UDPsteering.beginPacket(DestinationIP, DestinationPort);
            UDPsteering.write(PGN_250, sizeof(PGN_250));
            UDPsteering.endPacket();
        }
    }
}

void SendSteerDiag()
{
    // PGN 249, steer diagnostics for AOG's control data log. Sent every loop (25 ms)
    // for DiagHoldTime after the last request from AOG.
    //0     HeaderLo        128
    //1     HeaderHi        129
    //2     source          126
    //3     PGN             249
    //4     length          29
    //5-8   millis          uint32
    //9-10  WAS reading     uint16, raw
    //11-12 ZeroOffset      int16
    //13-14 actual angle    int16, X 100
    //15-16 set point       int16, X 100
    //17-18 command PWM     int16, before InvertSteer / Danfoss
    //19-20 output PWM      int16, as driven
    //21    max PWM         low/high PWM limit in effect
    //22    flags           bit0 enabled, bit1 AOG timeout, bit2 AOG not ready, bit3 steer switch off, bit4 below min speed, bit5 IMU connected
    //23    switch byte
    //24-25 set point age   uint16, ms since last PGN 254
    //26-27 speed           uint16, km/h X 10
    //28-29 IMU heading     int16, X 10, 9999 = none
    //30-31 IMU roll        int16, X 10, 8888 = none
    //32-33 max loop time   uint16, micros
    //34    CRC

    if (DiagRequested && (millis() - DiagRequestTime < DiagHoldTime) && Ethernet.linkStatus() == LinkON)
    {
        uint32_t ms = millis();
        int16_t ZeroOffset = MDL.ZeroOffset;
        int16_t Actual = (int16_t)(steerAngleActual * 100);
        int16_t SetPoint = (int16_t)(steerAngleSetPoint * 100);
        uint8_t MaxPWM = constrain(MaxPWMvalue, 0, 255);
        uint16_t SetPointAge = min(ms - AOGTime, (uint32_t)65535);
        uint16_t Speed = (uint16_t)(Speed_KMH * 10);
        int16_t Heading = 9999;
        int16_t Roll = 8888;
        if (ATT_Connected())
        {
            Heading = (int16_t)ATT_Heading;
            Roll = (int16_t)ATT_Roll;
        }

        memcpy(&PGN_249[5], &ms, 4);
        memcpy(&PGN_249[9], &WasReading, 2);
        memcpy(&PGN_249[11], &ZeroOffset, 2);
        memcpy(&PGN_249[13], &Actual, 2);
        memcpy(&PGN_249[15], &SetPoint, 2);
        memcpy(&PGN_249[17], &DiagCmdPWM, 2);
        memcpy(&PGN_249[19], &pwmDrive, 2);
        PGN_249[21] = MaxPWM;
        PGN_249[22] = DiagFlags;
        PGN_249[23] = switchByte;
        memcpy(&PGN_249[24], &SetPointAge, 2);
        memcpy(&PGN_249[26], &Speed, 2);
        memcpy(&PGN_249[28], &Heading, 2);
        memcpy(&PGN_249[30], &Roll, 2);
        memcpy(&PGN_249[32], &MaxLoopTime, 2);

        uint8_t CK_A = 0;
        for (uint8_t i = 2; i < sizeof(PGN_249) - 1; i++)
        {
            CK_A += PGN_249[i];
        }
        PGN_249[sizeof(PGN_249) - 1] = CK_A;

        UDPsteering.beginPacket(DestinationIP, DestinationPort);
        UDPsteering.write(PGN_249, sizeof(PGN_249));
        UDPsteering.endPacket();
    }
}

void SendHelloReply()
{
    int16_t sa = (int16_t)(steerAngleActual * 100);

    helloFromAutoSteer[5] = (uint8_t)sa;
    helloFromAutoSteer[6] = sa >> 8;

    helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
    helloFromAutoSteer[8] = helloSteerPosition >> 8;
    helloFromAutoSteer[9] = switchByte;

    // to ethernet
    if (Ethernet.linkStatus() == LinkON)
    {
        UDPsteering.beginPacket(DestinationIP, DestinationPort);
        UDPsteering.write(helloFromAutoSteer, sizeof(helloFromAutoSteer));
        UDPsteering.endPacket();

        bool headingAvailable = ATT_Connected();
        if (headingAvailable)
        {
            UDPsteering.beginPacket(DestinationIP, DestinationPort);
            UDPsteering.write(helloFromIMU, sizeof(helloFromIMU));
            UDPsteering.endPacket();
        }
    }
}

void SendScanIDreply()
{
    IPAddress rem_ip = UDPsteering.remoteIP();

    uint8_t scanReply[] = { 128, 129, MDLnetwork.IP3, 203, 7, MDLnetwork.IP0, MDLnetwork.IP1, MDLnetwork.IP2, MDLnetwork.IP3,
                            rem_ip[0],rem_ip[1],rem_ip[2], 23 };

    //checksum
    int16_t CK_A = 0;
    for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
    {
        CK_A = (CK_A + scanReply[i]);
    }
    scanReply[sizeof(scanReply) - 1] = CK_A;

    static uint8_t ipDest[] = { 255,255,255,255 };
    uint16_t portDest = 9999; //AOG port that listens

    // to ethernet
    if (Ethernet.linkStatus() == LinkON)
    {
        UDPsteering.beginPacket(ipDest, portDest);
        UDPsteering.write(scanReply, sizeof(scanReply));
        UDPsteering.endPacket();
    }
}

