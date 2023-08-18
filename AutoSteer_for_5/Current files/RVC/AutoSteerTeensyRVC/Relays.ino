uint8_t Rlys;
uint32_t LastRelaySend;
bool RelayStatus[16];
bool BitState;
uint8_t IOpin;
uint8_t Relays8[] = { 7,5,3,1,8,10,12,14 }; // 8 relay module and a PCA9535PW
uint8_t Relays16[] = { 15,14,13,12,11,10,9,8,0,1,2,3,4,5,6,7 }; // 16 relay module and a PCA9535PW

void CheckRelays()
{
    switch (MDL.RelayControl)
    {
    case 2:
        // PCA9555 8 relays
        if (PCA9555PW_found)
        {
            for (int i = 0; i < 8; i++)
            {
                BitState = bitRead(RelayLo, i);

                if (RelayStatus[i] != BitState)
                {
                    IOpin = Relays8[i];

                    if (BitState)
                    {
                        // on
                        PCA.write(IOpin, PCA95x5::Level::L);
                    }
                    else
                    {
                        // off
                        PCA.write(IOpin, PCA95x5::Level::H);
                    }
                    RelayStatus[i] = BitState;
                }
            }
        }
        break;

    case 3:
        // PCA9555 16 relays
        if (PCA9555PW_found)
        {
            for (int i = 0; i < 16; i++)
            {
                if (i < 8)
                {
                    BitState = bitRead(RelayLo, i);
                }
                else
                {
                    BitState = bitRead(RelayHi, i - 8);
                }

                if (RelayStatus[i] != BitState)
                {
                    IOpin = Relays16[i];

                    if (BitState)
                    {
                        // on
                        PCA.write(IOpin, PCA95x5::Level::L);
                    }
                    else
                    {
                        // off
                        PCA.write(IOpin, PCA95x5::Level::H);
                    }
                    RelayStatus[i] = BitState;
                }
            }
        }
        break;

    case 4:
        // MCP23017
        if (MCP23017_found)
        {
            for (int j = 0; j < 2; j++)
            {
                if (j < 1) Rlys = RelayLo; else Rlys = RelayHi;
                for (int i = 0; i < 8; i++)
                {
                    if (bitRead(Rlys, i))
                    {
                        MCP.digitalWrite(MDL.RelayPins[i + j * 8], steerConfig.IsRelayActiveHigh);
                    }
                    else
                    {
                        MCP.digitalWrite(MDL.RelayPins[i + j * 8], !steerConfig.IsRelayActiveHigh);
                    }
                }
            }
        }
        break;

    case 5:
        // GPIOs
        for (int j = 0; j < 2; j++)
        {
            if (j < 1) Rlys = RelayLo; else Rlys = RelayHi;
            for (int i = 0; i < 8; i++)
            {
                if (MDL.RelayPins[i + j * 8] > 1) // check if relay is enabled
                {
                    if (bitRead(Rlys, i))
                    {
                        digitalWrite(MDL.RelayPins[i + j * 8], steerConfig.IsRelayActiveHigh);
                    }
                    else
                    {
                        digitalWrite(MDL.RelayPins[i + j * 8], !steerConfig.IsRelayActiveHigh);
                    }
                }
            }
        }
        break;
    }
}


