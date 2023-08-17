
byte DataConfig[MaxReadBuffer];
uint16_t PGNconfig;

void ReceiveUDPconfig()
{
    if (Ethernet.linkStatus() == LinkON)
    {
        uint16_t len = UDPconfig.parsePacket();
        if (len)
        {
            UDPconfig.read(DataConfig, MaxReadBuffer);
            PGNconfig = DataConfig[1] << 8 | DataConfig[0];
            switch (PGNconfig)
            {
            case 32400:
                //      teensy steering config
                //0     HeaderLo    144
                //1     HeaderHi    126
                //2     Receiver    0 none, 1 simpleRTK2B
                //3     Receiver serial port
                //4     IMU serial port
                //5     Min. speed
                //6     Max. speed
                //7     Pulse Cal X 10, Lo
                //8     Pulse Cal X 10, Hi
                //9     relay control type  0 - no relays, 1 - RS485, 2 - PCA9555 8 relays, 3 - PCA9555 16 relays, 4 - MCP23017, 5 - Teensy GPIO
                //10    Commands
                //          - bit 0, swap pitch for roll
                //          - bit 1, invert roll
                //          - bit 2, use 4-20 pressure sensor
                //          - bit 3, relay on signal
                //          - bit 4, zero WAS
                //11    CRC

                PGNlength = 12;
                if (len > PGNlength - 1)
                {
                    if (GoodCRC(DataConfig, PGNlength))
                    {
                        MDL.Receiver = DataConfig[2];
                        MDL.ReceiverSerialPort = DataConfig[3];
                        MDL.IMUSerialPort = DataConfig[4];
                        MDL.MinSpeed = DataConfig[5];
                        MDL.MaxSpeed = DataConfig[6];
                        MDL.PulseCal = DataConfig[7] | DataConfig[8] << 8;
                        MDL.RelayControl = DataConfig[9];

                        uint8_t Commands = DataConfig[10];
                        if (bitRead(Commands, 0)) MDL.SwapRollPitch = 1; else MDL.SwapRollPitch = 0;
                        if (bitRead(Commands, 1)) MDL.InvertRoll = 1; else MDL.InvertRoll = 0;
                        if (bitRead(Commands, 2)) MDL.Use4_20 = 1; else MDL.Use4_20 = 0;
                        if (bitRead(Commands, 3)) MDL.RelayOnSignal = 1; else MDL.RelayOnSignal = 0;
                        if (bitRead(Commands, 4)) MDL.ZeroOffset = AINs.AIN0;

                        EEPROM.put(110, MDL);
                    }
                }
                break;

            case 32401:
                //      Steering pins
                //0     HeaderLo    145
                //1     HeaderHi    126
                //2     motor dir
                //3     motor pwm
                //4     power relay
                //5     steering relay
                //6     steer switch
                //7     work switch
                //8     speed pulse
                //9-24  relay pins
                //25    CRC

                PGNlength = 26;
                if (len > PGNlength - 1)
                {
                    if (GoodCRC(DataConfig, PGNlength))
                    {
                        MDL.Dir1 = DataConfig[2];
                        MDL.PWM1 = DataConfig[3];
                        MDL.PowerRelay = DataConfig[4];
                        MDL.SteeringRelay = DataConfig[5];
                        MDL.SteerSw = DataConfig[6];
                        MDL.WorkSw = DataConfig[7];
                        MDL.SpeedPulse = DataConfig[8];

                        for (int i = 0; i < 16; i++)
                        {
                            MDL.RelayPins[i] = DataConfig[i + 9];
                        }

                        EEPROM.put(110, MDL);

                        SCB_AIRCR = 0x05FA0004; //reset the Teensy   
                    }
                }
                break;

            case 32402:
                //      Info request from PCBsetup
                //0     HeaderLo    146
                //1     HeaderHi    126
                //2     Info ID     0 IP address, 1 Ino ID
                //3     CRC

                PGNlength = 4;
                if (len > PGNlength - 1)
                {
                    if (GoodCRC(DataConfig, PGNlength))
                    {
                        switch (DataConfig[2])
                        {
                        case 0:
                            // subnet
                            UDPSendIP();
                            break;
                            
                        case 1:
                            // Ino ID
                            UDPSendInoID();
                            break;
                        }
                    }
                }
                break;

            case 32404:
                //      New IP
                //0     HeaderLo    148
                //1     HeaderHI    126
                //2     IP 0
                //3     IP 1
                //4     IP 2
                //5     CRC

                PGNlength = 6;
                if (len > PGNlength - 1)
                {
                    if (GoodCRC(DataConfig, PGNlength))
                    {
                        MDL.IP0 = DataConfig[2];
                        MDL.IP1 = DataConfig[3];
                        MDL.IP2 = DataConfig[4];

                        EEPROM.put(110, MDL);

                        SCB_AIRCR = 0x05FA0004; //reset the Teensy   
                    }
                }
                break;
            }
        }
    }
}

void UDPSendIP()
{
    //0     HeaderLo
    //1     HeaderHi
    //2     Info ID
    //3     module type
    //4     module ID
    //5     IP 0
    //6     IP 1
    //7     IP 2
    //8     -
    //9     CRC

    if (Ethernet.linkStatus() == LinkON)
    {
        byte Data[10];
        Data[0] = 147;
        Data[1] = 126;
        Data[2] = 0;
        Data[3] = 0;
        Data[4] = 0;
        Data[5] = MDL.IP0;
        Data[6] = MDL.IP1;
        Data[7] = MDL.IP2;
        Data[9] = CRC(Data, 9, 0);

        static uint8_t ipDest[] = { 255,255,255,255 };

        // to ethernet
        UDPsteering.beginPacket(ipDest, ConfigDestinationPort);
        UDPsteering.write(Data, sizeof(Data));
        UDPsteering.endPacket();
    }
}

void UDPSendInoID()
{
    //0     HeaderLo
    //1     HeaderHi
    //2     Info ID
    //3     Ino ID lo
    //4     Ino ID Hi
    //5     
    //6     
    //7     
    //8     
    //9     CRC

    if (Ethernet.linkStatus() == LinkON)
    {
        byte Data[10];
        Data[0] = 147;
        Data[1] = 126;
        Data[2] = 1;
        Data[3] = (byte)InoID;
        Data[4] = InoID >> 8;
        Data[9] = CRC(Data, 9, 0);

        static uint8_t ipDest[] = { 255,255,255,255 };

        // to ethernet
        UDPsteering.beginPacket(ipDest, ConfigDestinationPort);
        UDPsteering.write(Data, sizeof(Data));
        UDPsteering.endPacket();
    }
}

