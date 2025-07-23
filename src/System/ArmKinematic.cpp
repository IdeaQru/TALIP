#include "ArmKinematic.h"

void ARM_KINEMATIC::getArmAngle6DOF(
    float x, float y, float z, float roll, float pitch, float yaw,
    float *shoulder_pitch, float *shoulder_roll, 
    float *elbow_pitch, float *elbow_yaw,
    float *wrist_pitch, float *wrist_roll)
{
    const float PHI = 180.0 / M_PI; // Konversi radian ke derajat
    const float EPSILON = 1e-6;    // Untuk menghindari pembagian dengan nol
    
    // Hitung jarak horizontal dan total
    float horizontal_dist = sqrt(x*x + y*y);
    float total_dist = sqrt(horizontal_dist*horizontal_dist + z*z);
    
    // Batasi jarak maksimum yang bisa dijangkau
    float max_reach = ARM_FRAME1 + ARM_FRAME2 - EPSILON;
    if(total_dist > max_reach) {
        total_dist = max_reach;
        // Normalisasi koordinat agar tetap dalam jangkauan
        float scale = max_reach / sqrt(x*x + y*y + z*z);
        x *= scale;
        y *= scale;
        z *= scale;
        horizontal_dist = sqrt(x*x + y*y);
    }
    
    // Cek jarak minimum
    float min_reach = fabs(ARM_FRAME1 - ARM_FRAME2);
    if(total_dist < min_reach) {
        total_dist = min_reach;
    }
    
    // === SHOULDER CALCULATIONS ===
    // Shoulder Roll (rotasi base)
    *shoulder_roll = atan2(y, x) * PHI;
    
    // Shoulder Pitch menggunakan geometri 2D
    float shoulder_pitch_base = atan2(z, horizontal_dist) * PHI;
    
    // Hitung sudut siku menggunakan hukum cosinus
    float cos_elbow = (ARM_FRAME1*ARM_FRAME1 + ARM_FRAME2*ARM_FRAME2 - total_dist*total_dist) / 
                      (2.0 * ARM_FRAME1 * ARM_FRAME2);
    cos_elbow = fmax(-1.0, fmin(1.0, cos_elbow)); // Constrain ke [-1, 1]
    *elbow_pitch = acos(cos_elbow) * PHI;
    
    // Koreksi shoulder pitch berdasarkan geometri triangulasi
    if(total_dist > EPSILON) {
        float cos_alpha = (ARM_FRAME1*ARM_FRAME1 + total_dist*total_dist - ARM_FRAME2*ARM_FRAME2) / 
                          (2.0 * ARM_FRAME1 * total_dist);
        cos_alpha = fmax(-1.0, fmin(1.0, cos_alpha));
        float alpha = acos(cos_alpha) * PHI;
        *shoulder_pitch = shoulder_pitch_base + alpha;
    } else {
        *shoulder_pitch = shoulder_pitch_base;
    }
    
    // === WRIST CALCULATIONS ===
    // Distribusi orientasi ke elbow dan wrist
    *elbow_yaw = yaw * 0.3f; // 30% yaw ke elbow
    *wrist_roll = roll + (yaw * 0.7f); // 70% yaw ke wrist + roll
    
    // Wrist pitch untuk menjaga orientasi end effector
    // Kompensasi rotasi dari shoulder dan elbow
    float total_pitch_compensation = *shoulder_pitch + (*elbow_pitch - 180.0f);
    *wrist_pitch = pitch - total_pitch_compensation;
    
    // Batasi sudut dalam range yang aman untuk servo
    *shoulder_pitch = constrainAngle(*shoulder_pitch, -90.0f, 90.0f);
    *shoulder_roll  = constrainAngle(*shoulder_roll,  -90.0f, 90.0f);
    *elbow_pitch    = constrainAngle(*elbow_pitch,     0.0f, 180.0f);
    *elbow_yaw      = constrainAngle(*elbow_yaw,     -90.0f, 90.0f);
    *wrist_pitch    = constrainAngle(*wrist_pitch,   -90.0f, 90.0f);
    *wrist_roll     = constrainAngle(*wrist_roll,   -180.0f, 180.0f);
}

void ARM_KINEMATIC::getFullArmAngles12DOF(
    float left_x, float left_y, float left_z,
    float right_x, float right_y, float right_z,
    float *allArmAngles)
{
    // Default orientasi (bisa diubah sesuai kebutuhan)
    float left_roll = 0.0f, left_pitch = 0.0f, left_yaw = 0.0f;
    float right_roll = 0.0f, right_pitch = 0.0f, right_yaw = 0.0f;
    
    float leftArmAngles[6], rightArmAngles[6];
    
    // Hitung inverse kinematics untuk lengan kiri
    getArmAngle6DOF(left_x, left_y, left_z, left_roll, left_pitch, left_yaw,
                    &leftArmAngles[0], &leftArmAngles[1], &leftArmAngles[2],
                    &leftArmAngles[3], &leftArmAngles[4], &leftArmAngles[5]);
    
    // Hitung inverse kinematics untuk lengan kanan
    // Mirror koordinat Y untuk simetri
    getArmAngle6DOF(right_x, -right_y, right_z, right_roll, right_pitch, right_yaw,
                    &rightArmAngles[0], &rightArmAngles[1], &rightArmAngles[2],
                    &rightArmAngles[3], &rightArmAngles[4], &rightArmAngles[5]);
    
    // Mirror sudut tertentu untuk lengan kanan agar simetris
    rightArmAngles[1] = -rightArmAngles[1]; // Mirror shoulder roll
    rightArmAngles[3] = -rightArmAngles[3]; // Mirror elbow yaw
    rightArmAngles[5] = -rightArmAngles[5]; // Mirror wrist roll
    
    // Mapping ke array output sesuai ID servo 4-15
    // Format: [L_shoulder_pitch, R_shoulder_pitch, L_shoulder_roll, R_shoulder_roll, ...]
    allArmAngles[0]  = leftArmAngles[0];   // ID 4  - Left shoulder pitch
    allArmAngles[1]  = rightArmAngles[0];  // ID 5  - Right shoulder pitch
    allArmAngles[2]  = leftArmAngles[1];   // ID 6  - Left shoulder roll
    allArmAngles[3]  = rightArmAngles[1];  // ID 7  - Right shoulder roll
    allArmAngles[4]  = leftArmAngles[2];   // ID 8  - Left elbow pitch
    allArmAngles[5]  = rightArmAngles[2];  // ID 9  - Right elbow pitch
    allArmAngles[6]  = leftArmAngles[3];   // ID 10 - Left elbow yaw
    allArmAngles[7]  = rightArmAngles[3];  // ID 11 - Right elbow yaw
    allArmAngles[8]  = leftArmAngles[4];   // ID 12 - Left wrist pitch
    allArmAngles[9]  = rightArmAngles[4];  // ID 13 - Right wrist pitch
    allArmAngles[10] = leftArmAngles[5];   // ID 14 - Left wrist roll
    allArmAngles[11] = rightArmAngles[5];  // ID 15 - Right wrist roll
}

float ARM_KINEMATIC::constrainAngle(float angle, float min_angle, float max_angle)
{
    if (angle < min_angle) return min_angle;
    if (angle > max_angle) return max_angle;
    return angle;
}
