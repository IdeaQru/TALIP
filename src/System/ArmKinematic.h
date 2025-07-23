#include <Arduino.h>

// Definisi panjang segment tangan (sesuaikan dengan robot Anda)
#define ARM_FRAME0 0      // Base offset
#define ARM_FRAME1 123.0   // Panjang lengan atas (bahu ke siku)
#define ARM_FRAME2 78.0   // Panjang lengan bawah (siku ke pergelangan)
#define ARM_FRAME3 84.0   // Panjang dari pergelangan ke ujung tangan

class ARM_KINEMATIC {
    public:
        // Inverse kinematics untuk lengan 6 DOF per sisi
        void getArmAngle6DOF(float x, float y, float z, float roll, float pitch, float yaw,
                            float *shoulder_pitch, float *shoulder_roll, 
                            float *elbow_pitch, float *elbow_yaw,
                            float *wrist_pitch, float *wrist_roll);
        
        // Untuk kedua lengan sekaligus (12 DOF total)
        void getFullArmAngles12DOF(float left_x, float left_y, float left_z,
                                  float right_x, float right_y, float right_z,
                                  float *allArmAngles);

    private:
        float constrainAngle(float angle, float min_angle, float max_angle);
        float PHI = 57.295779513082320876798154814105; // Rad to degree
};
struct ArmServoMapping {
    // ID Servo untuk tangan: 4-15 (12 servo total)
    // Asumsi: 6 servo per lengan
    
    // Lengan Kiri (ID 4,6,8,10,12,14)
    int leftShoulder1 = 4;    // Bahu kiri pitch
    int leftShoulder2 = 6;    // Bahu kiri roll  
    int leftElbow1 = 8;       // Siku kiri pitch
    int leftElbow2 = 10;      // Siku kiri yaw
    int leftWrist1 = 12;      // Pergelangan kiri pitch
    int leftWrist2 = 14;      // Pergelangan kiri roll
    
    // Lengan Kanan (ID 5,7,9,11,13,15)
    int rightShoulder1 = 5;   // Bahu kanan pitch
    int rightShoulder2 = 7;   // Bahu kanan roll
    int rightElbow1 = 9;      // Siku kanan pitch
    int rightElbow2 = 11;     // Siku kanan yaw
    int rightWrist1 = 13;     // Pergelangan kanan pitch
    int rightWrist2 = 15;     // Pergelangan kanan roll
    
    // Kepala
    int head = 123;           // Kepala
};

