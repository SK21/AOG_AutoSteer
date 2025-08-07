
bool IMU_Connected()
{
    return (millis() - IMUtime < 4000);
}

void ReadIMU()
{
    switch (MDL.IMUtype)
    {
    case 0:
        // BNO080
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
            if (MDL.InvertRoll) IMU_Roll *= -1.0;
        }
        break;

    case 1:
        // TM171
        while (SerialIMU->available())
        {
            IMUtime = millis();
            char rxByte = (char)SerialIMU->read();
            char* rxData = &rxByte;
            int rxSize = 1;
            Ep_Header header;
            if (EP_SUCC_ == eP.On_RecvPkg(rxData, rxSize, &header))
            {
                switch (header.cmd)
                {
                case EP_CMD_RPY_:
                    Ep_RPY RPY_Data;
                    if (EP_SUCC_ == eOD.Read_Ep_RPY(&RPY_Data))
                    {
                        IMU_YawRate = 0;
                        IMU_Heading = RPY_Data.yaw * 10.0;
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
                        if (MDL.InvertRoll) IMU_Roll *= -1.0;
                    }
                    break;

                case EP_CMD_COMBO_:
                    Ep_Combo ComboData;
                    if (EP_SUCC_ == eOD.Read_Ep_Combo(&ComboData))
                    {
                        IMU_YawRate = (ComboData.wz) * (1e-5f);         // rad/s
                        IMU_Heading = (ComboData.yaw) * (1e-2f) * 10;   // degree
                        if (SteerConfig.UseIMU_Y_Axis)
                        {
                            IMU_Roll = (ComboData.pitch) * (1e-2f) * 10;
                            IMU_Pitch = (ComboData.roll) * (1e-2f) * 10;
                        }
                        else
                        {
                            IMU_Roll = (ComboData.roll) * (1e-2f) * 10;
                            IMU_Pitch = (ComboData.pitch) * (1e-2f) * 10;
                        }
                        if (MDL.InvertRoll) IMU_Roll *= -1.0;
                    }
                    break;
                }
            }
        }
        break;
    }
}


