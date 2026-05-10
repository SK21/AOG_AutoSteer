static float   pValue = 0;                                                                                
static float   dValue = 0;
static float   lastXTE_Error = 0;                                                                           
static uint8_t dCounter = 0;

void calcSteeringPID()
{
    float errorAbs = fabsf(toolXTE_cm);

    // proportional
    pValue = toolSettings.kP * toolXTE_cm * 0.2f;
    pwmDrive = (int16_t)pValue;

    // dynamic PWM ceiling
    float   lowHighPerCM = (float)(toolSettings.highPWM - toolSettings.lowPWM) / toolSettings.lowHighDistance;
    int16_t newMax;

    if (errorAbs < toolSettings.lowHighDistance)
        newMax = (int16_t)(errorAbs * lowHighPerCM + toolSettings.lowPWM);
    else
        newMax = toolSettings.highPWM;

    if (pwmDrive > newMax) pwmDrive = newMax;
    if (pwmDrive < -newMax) pwmDrive = -newMax;

    // derivative — updated every 11 loops to reduce noise
    if (dCounter++ > 10)
    {
        dValue = (toolXTE_cm - lastXTE_Error) * 30.0f * (toolSettings.kD * 0.1f);
        lastXTE_Error = toolXTE_cm;
        dCounter = 0;
    }

    if (dValue > toolSettings.highPWM) dValue = toolSettings.highPWM;
    if (dValue < -toolSettings.highPWM) dValue = -toolSettings.highPWM;

    pwmDrive += (int16_t)dValue;

    if (pwmDrive > toolSettings.highPWM) pwmDrive = toolSettings.highPWM;
    if (pwmDrive < -toolSettings.highPWM) pwmDrive = -toolSettings.highPWM;

    // minimum PWM to overcome motor friction
    if (pwmDrive > 0) pwmDrive += toolSettings.minPWM;
    else if (pwmDrive < 0) pwmDrive -= toolSettings.minPWM;
}

void motorDrive()
{
    // manual PWM override from PGN 233
    if (manualPWM != 0)
        pwmDrive = manualPWM;

    if (toolSettings.invertActuator)
        pwmDrive *= -1;

    // hard stop at actuator travel limit
    if (fabsf(actuatorPositionPercent) > toolSettings.maxActuatorLimit)
    {
        if (actuatorPositionPercent > 0 && pwmDrive > 0) pwmDrive = 0;
        if (actuatorPositionPercent < 0 && pwmDrive < 0) pwmDrive = 0;
    }

    pwmDisplay = pwmDrive;

    // Dir + PWM — consistent with existing Steering.ino motor output
    if (MDL.DirPin < NC) digitalWrite(MDL.DirPin, pwmDrive >= 0);
    if (MDL.PWMpin < NC) analogWrite(MDL.PWMpin, abs(pwmDrive));
}

