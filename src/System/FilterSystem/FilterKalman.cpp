#include "System/FilterSystem/FilterKalman.h"
        float rPitch, qPitch, rRoll, qRoll;
        float inKalmanPitch, outKalmanPitch;
        float xtPitch,  xt_updatePitch, xt_prevPitch;
        float ptPitch, pt_updatePitch;
        float ktPitch;
        float pt_prevPitch;

        float inKalmanRoll, outKalmanRoll;
        float xtRoll,  xt_updateRoll, xt_prevRoll;
        float ptRoll,   pt_updateRoll;
        float ktRoll;
        float pt_prevRoll;

        SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);
        const long SERIAL_REFRESH_TIME = 100;
        long refresh_time;

void KALMAN :: init () {
    rRoll = 20; qRoll=1; pt_prevRoll=1;
    rPitch = 10; qPitch=2; pt_prevPitch=1;
    
}
void KALMAN :: getKPitch(float pitch) {
    // inKalmanPitch = pitch;
    // xt_updatePitch = xt_prevPitch;
    // pt_updatePitch = pt_prevPitch + qPitch;
    // ktPitch = pt_updatePitch / (pt_updatePitch + rPitch);
    // xtPitch = xt_updatePitch + (ktPitch * (inKalmanPitch - xt_updatePitch));
    // ptPitch = (1-ktPitch) * pt_updatePitch;
    // xt_prevPitch = xtPitch;
    // pt_prevPitch = ptPitch;
    // outKalmanPitch = xtPitch;
    outKalmanPitch = simpleKalmanFilter.updateEstimate(pitch);
}

void KALMAN :: getKRoll(float roll) {
    // inKalmanRoll = roll;
    // xt_updateRoll = xt_prevRoll;
    // pt_updateRoll = pt_prevRoll + qRoll;
    // ktRoll = pt_updateRoll / (pt_updateRoll + rRoll);
    // xtRoll = xt_updateRoll + (ktRoll * (inKalmanRoll - xt_updateRoll));
    // ptRoll = (1-ktRoll) * pt_updateRoll;
    // xt_prevRoll = xtRoll;
    // pt_prevRoll = ptRoll;
    // outKalmanRoll = xtRoll;
    outKalmanRoll = simpleKalmanFilter.updateEstimate(roll);
}float KALMAN :: getYerror(void) {
	float Yerror;
	Yerror = 93.1 * sin (outKalmanPitch * 3.14 / 180.0);
    return Yerror;
}
float KALMAN :: getXerror(void) {
	float Xerror;
	Xerror =  93.1 * sin (outKalmanRoll * 3.14 / 180.0);
    return Xerror;
}
float KALMAN :: getZerror(void) {
	float Zerror;
	Zerror = 10 * cos (outKalmanRoll* 3.14 / 180.0);
    return Zerror;
}
float KALMAN:: getZzerror(void) {
	float Zzerror;
	Zzerror = 10.1 * sin (outKalmanPitch * 3.14 / 180.0);
    return Zzerror;
}