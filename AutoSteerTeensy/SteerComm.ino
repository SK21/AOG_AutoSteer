
void ReceiveSteerData()
{
    // ethernet
    if (Ethernet.linkStatus() == LinkON)
    {
        uint16_t len = UDPsteering.parsePacket();
        if (len)
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
                        MDL.IP0 = Data[7];
                        MDL.IP1 = Data[8];
                        MDL.IP2 = Data[9];
                        SaveData();

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
                    //SteerConfig.MinSpeed = (double)Data[7] * 0.1;

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

    // BNOdata
    if (IMU_Connected())
    {
        tmp = (int)(IMU_Heading);
        PGN_253[7] = (byte)tmp;
        PGN_253[8] = tmp >> 8;

        tmp = (int)(IMU_Roll);
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

void SendHelloReply()
{
    int16_t sa = (int16_t)(steerAngleActual * 100);

    helloFromAutoSteer[5] = (uint8_t)sa;
    helloFromAutoSteer[6] = sa >> 8;

    helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
    helloFromAutoSteer[8] = helloSteerPosition >> 8;
    helloFromAutoSteer[9] = switchByte;

    ////add the checksum
    //int16_t CK_A = 0;
    //for (uint8_t i = 2; i < sizeof(helloFromAutoSteer) - 1; i++)
    //{
    //    CK_A = (CK_A + helloFromAutoSteer[i]);
    //}
    //helloFromAutoSteer[sizeof(helloFromAutoSteer) - 1] = CK_A;

    // to ethernet
    if (Ethernet.linkStatus() == LinkON)
    {
        UDPsteering.beginPacket(DestinationIP, DestinationPort);
        UDPsteering.write(helloFromAutoSteer, sizeof(helloFromAutoSteer));
        UDPsteering.endPacket();

        if (IMU_Connected())
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

    uint8_t scanReply[] = { 128, 129, MDL.IP3, 203, 7, MDL.IP0, MDL.IP1, MDL.IP2, MDL.IP3,
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

