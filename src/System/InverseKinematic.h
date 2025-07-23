#include <Arduino.h>
#define Frame0 0
#define Frame1 93.1 // joint paha ke lutut
#define Frame2 93.1 // joint lutut ke angkel
class KINEMATIC {
    public:
        void getmAngle(float x, float y, float z, float Head, float *mAngle_0, float *mAngle_1, float *mAngle_2);

    private:
};