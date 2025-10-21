
bool IMU_Connected()
{
    return (millis() - IMUtime < 4000);
}

void ReadIMU()
{
    if (SerialIMUEnabled)
    {
        switch (MDL.IMUtype)
        {
        case 0:
            ReadBNO();
            break;

        case 1:
            ReadTM171();
            break;

        }
    }
}

void ReadBNO()
{
    if (rvc.read(&BNOdata))
    {
        IMUtime = millis();
        IMU_YawRate = BNOdata.z_accel;
        IMU_Heading = BNOdata.yaw;
        if (IMU_Heading < 0 && IMU_Heading >= -180) //Scale BNO085 yaw from [-180?;180?] to [0;360?]
        {
            IMU_Heading = IMU_Heading + 360;
        }
        IMU_Heading *= 10.0;

        if (SteerConfig.UseIMU_Y_Axis)
        {
            IMU_Roll = BNOdata.pitch * 10;
            IMU_Pitch = BNOdata.roll * 10;
        }
        else
        {
            IMU_Roll = BNOdata.roll * 10;
            IMU_Pitch = BNOdata.pitch * 10;
        }
    }
}

void ReadTM171()
{
    // TM171 at default settings, 50 Hz
    // will tolerate a 5 ms main loop without dropping serial data
    // even when dropping data the imu outputs are valid

    const size_t BUF_MAX = 64;
    size_t avail = (size_t)SerialIMU->available();
    if (avail > 0)
    {
        size_t toRead = (avail < BUF_MAX) ? avail : BUF_MAX;
        uint8_t buf[BUF_MAX];
        size_t actuallyRead = SerialIMU->readBytes((char*)buf, toRead);

        if (avail > BUF_MAX)
        {
            size_t toDiscard = avail - BUF_MAX;

            const size_t DISCARD_CHUNK = 32;
            uint8_t discardBuf[DISCARD_CHUNK];
            while (toDiscard)
            {
                size_t chunk = (toDiscard > DISCARD_CHUNK) ? DISCARD_CHUNK : toDiscard;
                size_t r = SerialIMU->readBytes((char*)discardBuf, chunk);
                if (r == 0) break;
                toDiscard -= r;
            }
        }

        Ep_Header header;
        for (size_t i = 0; i < actuallyRead; ++i)
        {
            char rxByte = (char)buf[i];
            if (EP_SUCC_ == eP.On_RecvPkg(&rxByte, 1, &header))
            {
                switch (header.cmd)
                {
                case EP_CMD_RPY_:
                    Ep_RPY RPY_Data;
                    if (EP_SUCC_ == eOD.Read_Ep_RPY(&RPY_Data))
                    {
                        IMUtime = millis();
                        IMU_YawRate = 0.0f;
                        IMU_Heading = fmodf(RPY_Data.yaw * 10.0f + 3600.0f, 3600.0f);

                        if (SteerConfig.UseIMU_Y_Axis)
                        {
                            IMU_Roll = RPY_Data.pitch * 10;
                            IMU_Pitch = RPY_Data.roll * 10;
                        }
                        else
                        {
                            IMU_Roll = RPY_Data.roll * 10;
                            IMU_Pitch = RPY_Data.pitch * 10;
                        }
                    }
                    break;

                case EP_CMD_COMBO_:
                    Ep_Combo ComboData;
                    if (EP_SUCC_ == eOD.Read_Ep_Combo(&ComboData))
                    {
                        IMUtime = millis();
                        IMU_YawRate = ComboData.wz * 1e-5f;
                        IMU_Heading = fmodf(ComboData.yaw * 1e-2f * 10.0f + 3600.0f, 3600.0f);

                        if (SteerConfig.UseIMU_Y_Axis)
                        {
                            // Use ComboData values (was a bug in the original)
                            IMU_Roll = ComboData.pitch * 1e-2f * 10.0f;
                            IMU_Pitch = ComboData.roll * 1e-2f * 10.0f;
                        }
                        else
                        {
                            IMU_Roll = ComboData.roll * 1e-2f * 10.0f;
                            IMU_Pitch = ComboData.pitch * 1e-2f * 10.0f;
                        }
                    }
                    break;

                }
            }
        }
    }
}


