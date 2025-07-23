// #include <Arduino.h>
// #include "System/InverseKinematic.h"
// #include "System/ArmKinematic.h"
// #include "System/SyncSending.h"
// #include "System/FilterSystem/FilterKalman.h"
// #include "System/ControlSystem/PIDControl.h"
// #include "Sensor/Gyro.h"
// #include "Sensor/Hwt.h"

// // Definisi konstanta
// #define maxTheta 360.0
// #define startColl 0
// #define endColl 20  // 8 untuk kaki + 12 untuk tangan
// #define endSteps 8
// #define phi 3.14159265359
// #define JUMLAH_DXL_PRTOKOL2 15

// struct MOTIONROBOT
// {
//     // ==================== SERVO KAKI (ID 1-10) ====================
//     int defPos [10] = { 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048 };
//     int sendPos [10] = { 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048 };
    
//     // ==================== SERVO TANGAN XL (ID 4-15) + KEPALA (ID 123) ====================
//     int defPosXL[JUMLAH_DXL_PRTOKOL2] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048};
//     int sendPosXL[JUMLAH_DXL_PRTOKOL2] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048};
//     int speedXL[JUMLAH_DXL_PRTOKOL2] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
    
//     // ==================== MOTION GABUNGAN (21 kolom) ====================
//     // Kolom 0-7: Kaki (xl, yl, zl, hl, xr, yr, zr, hr)
//     // Kolom 8-19: Tangan (L_SP, L_SR, L_EP, L_EY, L_WP, L_WR, R_SP, R_SR, R_EP, R_EY, R_WP, R_WR)
//     // Kolom 20: Speed
//     int lastMotion[21] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    
//     int standMotion [8][21] = {
//     //  xl yl zl hl xr yr zr hr L_SP L_SR L_EP L_EY L_WP L_WR R_SP R_SR R_EP R_EY R_WP R_WR spd
//         { 0, 0, 0, 0, 0, 0, 0, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 10 },
//         { 0, 0, 0, 0, 0, 0, 0, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 10 },
//         { 0, 0, 0, 0, 0, 0, 0, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 10 },
//         { 0, 0, 0, 0, 0, 0, 0, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 10 },
//         { 0, 0, 0, 0, 0, 0, 0, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 10 },
//         { 0, 0, 0, 0, 0, 0, 0, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 10 },
//         { 0, 0, 0, 0, 0, 0, 0, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 10 },
//         { 0, 0, 0, 0, 0, 0, 0, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 10 }
//     };
    
//     int walkMotion [8][21] = {
//     //  xl yl zl hl xr yr zr hr L_SP L_SR L_EP L_EY L_WP L_WR R_SP R_SR R_EP R_EY R_WP R_WR spd
//         { 0,45, 0, 0, 0, 0, 0, 0,  20, -10,  30,   5, -15,  10, -20,  10, -30,  -5,  15, -10, 15 },
//         {-20,45,0, 0,10,45,30, 0,  30, -15,  45,  10, -25,  15, -30,  15, -45, -10,  25, -15, 15 },
//         {-20,45,0, 0,20,45,30, 0,  40, -20,  60,  15, -35,  20, -40,  20, -60, -15,  35, -20, 15 },
//         {-20, 0,0, 0,20, 0, 0, 0,  30, -15,  45, -10, -25, -15, -30, -15, -45,  10,  25,  15, 15 },
//         { 0,-47,0, 0, 0,-47,0, 0,  20, -10,  30,  -5, -15, -10, -20, -10, -30,   5,  15,  10, 15 },
//         {10,-47,30,0,-20,-47,0, 0,  10,  -5,  15, -10, -10,  -5, -10,  -5, -15,  10,  10,   5, 15 },
//         {20,-47,30,0,-20,-47,0, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 15 },
//         {20, 0, 0, 0,-20, 0, 0, 0,  10,  -5,  15,  10,  -5,   5, -10,   5, -15, -10,   5,  -5, 15 }
//     };
    
//     int patternMotion [8][21] = {
//         { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
//         { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
//         { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
//         { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
//         { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
//         { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
//         { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
//         { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
//     };
    
//     // Mapping ID servo XL
//     int servoIDsXL[13] = {4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 123};
// };

// class INSTRUCTION {
//     public:
//         void execute(int motion);  // Execute gabungan kaki + tangan
//         unsigned long timeNow = 0;
        
//     private:
//         void set(int motion);
//         void trajectory(int steps, float theta);
//         void getServoDeg(float x1, float y1, float z1, float H1, float x2, float y2, float z2, float H2, float *armData);
//         void getServoRes(float *AngleKakiKanan, float *AngleKakiKiri);
//         void getArmServoRes(float *armData);
        
//         int steps;
//         float theta;
        
//         GYRO gyro;
//         MOTIONROBOT motionRobot;
//         KINEMATIC kinematic;
//         ARM_KINEMATIC armKinematic;  // Instance untuk inverse kinematics tangan
//         SYNC sync;
//         KALMAN kalman;
//         HWT hwt;
//         PID pid;
// };
