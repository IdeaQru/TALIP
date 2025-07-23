#include <Arduino.h>
class GYRO {
    public:
        void init();

        float getYaw();
        float getRoll();
        float getPitch();
        void read();
        float Pitch;
        float Roll;
        float Yaw;

    private:
        
        
};
