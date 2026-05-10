// Bynav C2 PASHR sentence handler
// PASHR_Handler() and SendPASHRConfig() are called from Panda.ino / Begin.ino
// Active when MDL.GPSSource == GPS_ByNav

// $PASHR,024224.00,37.186,T,0.000,-76.837,0.000,0.000,0.500,0.200,2*10
// 1    header      -   $PASHR
// 2    UTC Time    0   024224.00
// 3    heading     1   37.186
// 4    North       2   T
// 5    Roll        3   0.000
// 6    Pitch       4   0.000
// 7    Heave       5   0.000
// 8    Roll dev    6   0.000
// 9    Pitch dev   7   0.500
// 10   Yaw dev     8   0.200
// 11   status      9   2
// 12   check sum   10  10

void PASHR_Handler()
{
    char buf[12];

    // solution status field[9]: 0=invalid, 1=single point, 2=RTK
    parser.getArg(9, buf);
    if (atoi(buf) < 1) return;

    ATT_Time = millis();

    // heading: field[1], degrees True North 0-360
    parser.getArg(1, buf);
    ATT_Heading = atof(buf) * 10.0f;

    // roll: field[3], degrees
    parser.getArg(3, buf);
    ATT_Roll = atof(buf) * 10.0f;
    itoa((int16_t)ATT_Roll, attRoll, 10);

    // pitch: field[4], degrees
    parser.getArg(4, buf);
    ATT_Pitch = atof(buf) * 10.0f;
    itoa((int16_t)ATT_Pitch, attPitch, 10);
    
    itoa(0, attYawRate, 10);
}

void SendPASHRConfig(HardwareSerialIMXRT* port)
{
    delay(200);
    port->println("UNLOGALL");                     delay(100);
    port->println("RTKTYPE ROVER");                delay(100);
    port->println("WORKFREQS ALL ALL");            delay(100);
    port->println("SET OBSFREQ 10");               delay(100);
    port->println("HEADINGOFFSET 90");             delay(100);
    port->println("DUALANTENNAPOWER ON");          delay(100);
    port->println("RTKTIMEOUT 500");               delay(100);
    port->println("INTERFACEMODE COM1 AUTO AUTO"); delay(100);
    port->println("SNRCUTOFF 15");                 delay(100);
    port->println("LOG GNGGA ONTIME 0.1");         delay(100);
    port->println("LOG GNVTG ONTIME 0.1");         delay(100);
    port->println("LOG PASHR ONTIME 0.1");         delay(100);
    port->println("SAVECONFIG");                   delay(500);
}
