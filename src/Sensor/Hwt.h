#include <Arduino.h>
#include "Sensor/REG.h"
#include "Sensor/wit_c_sdk.h"
#include <math.h>

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;

class HWT{
    public:
        void init();
        void read();
        float getYerror();
        float getXerror();
        float getZerror();
        float getZzerror();
        float getYaw();
        float getRoll();
        float getPitch();
    private:
        int i;
        float fAcc[3], fGyro[3], fAngle[3];
        unsigned long timeNow = 0;
        int periode = 10;
        // static void CmdProcess(void);
        // static void AutoScanSensor(void);
        // static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
        // static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
        // static void Delayms(uint16_t ucMs);
        
};