#include <Arduino.h>
#include <SimpleKalmanFilter.h>
class KALMAN {
    public:
        void getKPitch (float pitch);
        void getKRoll (float roll);
        float getYerror(void);
        float getXerror(void);
        float getZerror(void);
        float getZzerror(void);
        void init();
    private:
        
};