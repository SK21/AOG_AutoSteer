// UM982 HPR sentence handler
// HPR_Handler() and SendHPRConfig() are called from Panda.ino / Begin.ino
// Active when MDL.GPSSource == GPS_UM982

void HPR_Handler()
{
    char buf[12];

    // require RTK fixed dual-antenna solution (solQuality >= 4)
    parser.getArg(4, buf);
    if (atoi(buf) < 4) return;

    // heading: field[1], degrees True North 0-360
    parser.getArg(1, buf);
    IMU_Heading = atof(buf) * 10.0f;

    // roll: field[2] — pitch of the baseline vector; equals vehicle roll
    // when antennas are mounted side-by-side (perpendicular to travel)
    parser.getArg(2, buf);
    IMU_Roll = atof(buf) * 10.0f;
    itoa((int16_t)IMU_Roll, imuRoll, 10);

    // pitch and yaw rate not available from HPR
    IMU_Pitch = 0;
    itoa(0, imuPitch, 10);
    itoa(0, imuYawRate, 10);
}

void SendHPRConfig(HardwareSerialIMXRT* port)
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
    port->println("LOG GNHPR ONTIME 0.1");         delay(100);
    port->println("SAVECONFIG");                   delay(500);
}
