#include <Arduino.h>
#include "System/InverseKinematic.h"
#include "System/ArmKinematic.h"
#include "System/SyncSending.h"
#include "System/FilterSystem/FilterKalman.h"
#include "System/ControlSystem/PIDControl.h"
#include "System/ArmSmoother.h"
#include "Sensor/Gyro.h"
#include "Sensor/Hwt.h"

// Definisi konstanta
#define maxTheta 360.0
#define startColl 0
#define endColl 14 // 8 untuk kaki + 6 untuk tangan (XYZ per lengan)
#define endSteps 8
#define phi 3.14159265359
#define JUMLAH_DXL_PRTOKOL2 14

struct MOTIONROBOT
{
    // ==================== SERVO KAKI (ID 1-10) ====================
    int defPos[10] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048};
    int sendPos[10] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048};

    // ==================== SERVO TANGAN XL (ID 4-15) + KEPALA (ID 2,3) ====================
    //                                   2    3    4    5    6    7    8    9    10   11   12   13   14   15
    int defPosXL[JUMLAH_DXL_PRTOKOL2] = {512, 512, 312, 712, 612, 412, 772, 202, 792, 212, 612, 412, 512, 512};
   int sendPosXL[JUMLAH_DXL_PRTOKOL2] = {512, 512, 312, 712, 612, 412, 772, 202, 792, 212, 612, 412, 512, 512};
    int speedXL[JUMLAH_DXL_PRTOKOL2] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};

    // ==================== MOTION GABUNGAN (15 kolom) ====================
    // Kolom 0-7: Kaki (xl, yl, zl, hl, xr, yr, zr, hr)
    // Kolom 8-13: Tangan Cartesian (L_X, L_Y, L_Z, R_X, R_Y, R_Z)
    // Kolom 14: Speed
    int lastMotion[JUMLAH_DXL_PRTOKOL2] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    int standMotion[8][15] = {
        //  xl   yl   zl   hl   xr   yr   zr   hr   L_X  L_Y  L_Z  R_X  R_Y  R_Z  spd
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},

        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},
    };

    int walkMotion[8][15] = {
    //  xl   yl   zl   hl   xr   yr   zr   hr   L_X  L_Y  L_Z  R_X  R_Y  R_Z  spd
    {    0,  45,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  10},
    {  -20,  45,   0,   0,  10,  55,  30,   0,   0,   0,   0,   0,   0,   0,  15},
    {  -20,  45,   0,   0,  20,  55,  30,   0,   0,   0,   0,   0,   0,   0,  15},
    {  -20,   0,   0,   0,  20,   0,   0,   0,   0,   0,   0,   0,   0,   0,  10},
    {    0, -55,   0,   0,   0, -45,   0,   0,   0,   0,   0,   0,   0,   0,  10},
    {   10, -55,  30,   0, -20, -45,   0,   0,   0,   0,   0,   0,   0,   0,  15},
    {   20, -55,  30,   0, -20, -45,   0,   0,   0,   0,   0,   0,   0,   0,  15},
    {   20,   0,   0,   0, -20,   0,   0,   0,   0,   0,   0,   0,   0,   0,  10}
};
int walkAtPlace[8][15] = {
    //  xl   yl   zl   hl   xr   yr   zr   hr   L_X  L_Y  L_Z  R_X  R_Y  R_Z  spd
    {    0,  45,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  10},
    {    0,  45,   0,   0,   0,  55,  30,   0,   0,   0,   0,   0,   0,   0,  15},
    {    0,  45,   0,   0,   0,  55,  30,   0,   0,   0,   0,   0,   0,   0,  15},
    {    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  10},
    {    0, -55,   0,   0,   0, -45,   0,   0,   0,   0,   0,   0,   0,   0,  10},
    {    0, -55,  30,   0,   0, -45,   0,   0,   0,   0,   0,   0,   0,   0,  15},
    {    0, -55,  30,   0,   0, -45,   0,   0,   0,   0,   0,   0,   0,   0,  15},
    {    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  10}
};
    int stopMotion[8][15] = {
        //  xl   yl   zl   hl   xr   yr   zr   hr   L_X  L_Y  L_Z  R_X  R_Y  R_Z  spd
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},

        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},
    };

    int patternMotion[8][15] = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

    // Mapping ID servo XL
};

class INSTRUCTION
{
public:
    void execute(int motion); // Execute gabungan kaki + tangan
    void syncsendsservo();
    unsigned long timeNow = 0;
    void initializeArmPosition();

private:
    void set(int motion);
    void trajectory(int steps, float theta);
    void getServoDeg(float x1, float y1, float z1, float H1, float x2, float y2, float z2, float H2, float *armCartesian);
    void getServoRes(float *AngleKakiKanan, float *AngleKakiKiri);
    void getArmServoRes(float *armAngles);

    int steps;
    float theta;

    GYRO gyro;
    MOTIONROBOT motionRobot;
    KINEMATIC kinematic;
    ARM_KINEMATIC armKinematic; // Instance untuk inverse kinematics tangan
    SYNC sync;
    KALMAN kalman;
    HWT hwt;
    PID pid;
    ARM_SMOOTHER armSmoother;
};
