byte SerialMSB;
byte SerialLSB;

void ReceiveSerialConfig()
{
    if (Serial.available())
    {
        if (Serial.available() > 50)
        {
            // clear buffer
            while (Serial.available())
            {
                Serial.read();
            }
            PGNconfig = 0;
        }

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
            if (Serial.available() > PGNlength - 3)
            {
                PGNconfig = 0;	// reset pgn
                DataConfig[0] = 144;
                DataConfig[1] = 126;
                for (int i = 2; i < PGNlength; i++)
                {
                    DataConfig[i] = Serial.read();
                }

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
            if (Serial.available() > PGNlength - 3)
            {
                PGNconfig = 0;	// reset pgn
                DataConfig[0] = 145;
                DataConfig[1] = 126;
                for (int i = 2; i < PGNlength; i++)
                {
                    DataConfig[i] = Serial.read();
                }

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
            //2     CRC

            PGNlength = 3;
            if (Serial.available() > PGNlength - 3)
            {
                PGNconfig = 0;	// reset pgn
                DataConfig[0] = 146;
                DataConfig[1] = 126;
                for (int i = 2; i < PGNlength; i++)
                {
                    DataConfig[i] = Serial.read();
                }

                if (GoodCRC(DataConfig, PGNlength))
                {
                    SerialSendInfo();
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
            if (Serial.available() > PGNlength - 3)
            {
                PGNconfig = 0;	// reset pgn
                DataConfig[0] = 148;
                DataConfig[1] = 126;
                for (int i = 2; i < PGNlength; i++)
                {
                    DataConfig[i] = Serial.read();
                }
                
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

        default:
            // find pgn
            SerialMSB = Serial.read();
            PGNconfig = SerialMSB << 8 | SerialLSB;
            SerialLSB = SerialMSB;
            break;
        }
    }
}

void SerialSendInfo()
{
    // PGN 32403
    //0     HeaderLo    147
    //1     HeaderHi    126
    //2     InoID lo
    //3     InoID Hi
    //4     IP 0
    //5     IP 1
    //6     IP 2
    //7     IMU started
    //8     ADS found
    //9     PCA9555PW found
    //10    MCP23017 found
    //11    WAS lo
    //12    WAS Hi
    //13    Steer Switch
    //14    guidance status
    //15    Zero offset Lo
    //16    Zero offset Hi
    //17    heading Lo
    //18    heading Hi
    //19    connected
    //20    Receiver time
    //21    Ntrip time
    //22    Loop time lo
    //23    Loop time Hi
    //24    Ethernet
    //25    CRC

    byte Data[26];
    Data[0] = 147;
    Data[1] = 126;
    Data[2] = (byte)InoID;
    Data[3] = InoID >> 8;
    Data[4] = MDL.IP0;
    Data[5] = MDL.IP1;
    Data[6] = MDL.IP2;
    Data[7] = IMUstarted;
    Data[8] = ADSfound;
    Data[9] = PCA9555PW_found;
    Data[10] = MCP23017_found;
    Data[11] = AINs.AIN0;
    Data[12] = AINs.AIN0 >> 8;
    Data[13] = digitalRead(MDL.SteerSw);
    Data[14] = guidanceStatus;
    Data[15] = MDL.ZeroOffset;
    Data[16] = MDL.ZeroOffset >> 8;

    uint16_t Heading = IMU_Heading;
    Data[17] = Heading;
    Data[18] = Heading >> 8;

    Data[19] = (millis() - AOGTime < 4000);
    Data[20] = (millis() - ReceiverTime < 4000);
    Data[21] = (millis() - NtripTime < 4000);
    Data[22] = MaxLoopTime;
    Data[23] = MaxLoopTime >> 8;
    Data[24] = (Ethernet.linkStatus() == LinkON);

    Data[25] = CRC(Data, 25, 0);

    Serial.print(Data[0]);
    for (int i = 1; i < 26; i++)
    {
        Serial.print(",");
        Serial.print(Data[i]);
    }
    Serial.println("");
}

