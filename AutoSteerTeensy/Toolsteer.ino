
void UpdateWatchdog()
{
    if (watchdogTimer < WATCHDOG_FORCE_VALUE)
        watchdogTimer++;
}

bool ToolSteerEnabled()
{
    return (watchdogTimer < WATCHDOG_THRESHOLD) && (guidanceStatus & 0x01);
}

void BangBangDrive()
{
    static uint8_t valveOnCounter = 0;
    static uint8_t valveOffCounter = 0;

    float  errorAbs = fabsf(toolXTE_cm);
    int8_t dir = (toolXTE_cm > 0) ? 1 : -1;

    if (errorAbs > toolSettings.lowHighDistance)
    {
        // large error — full drive, reset pulse counters
        pwmDrive = 255 * dir;
        valveOnCounter = 0;
        valveOffCounter = 0;
    }
    else
    {
        // small error — pulse drive
        if (valveOnCounter < toolSettings.valveOnTime)
        {
            pwmDrive = 255 * dir;
            valveOnCounter++;
        }
        else if (valveOffCounter < toolSettings.valveOffTime)
        {
            pwmDrive = 0;
            valveOffCounter++;
        }
        else
        {
            valveOnCounter = 0;
            valveOffCounter = 0;
        }
    }

    motorDrive();
}
