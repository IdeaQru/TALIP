#include "System/Process.h"

void INSTRUCTION::set(int motion)
{
    for (int row = 0; row < 8; row++)
    {
        for (int col = 0; col < 15; col++)
        {
            if (motion == 1)
                motionRobot.patternMotion[row][col] = motionRobot.standMotion[row][col];
            if (motion == 2)
                motionRobot.patternMotion[row][col] = motionRobot.walkMotion[row][col];
            if (motion == 3)
                motionRobot.patternMotion[row][col] = motionRobot.walkAtPlace[row][col];
            if (motion == 4)
                motionRobot.patternMotion[row][col] = motionRobot.stopMotion[row][col];
        }
    }
}

void INSTRUCTION::trajectory(int steps, float theta)
{
    // ==================== VARIABEL KAKI ====================
    float LengthX1, LengthX2, LengthY1, LengthY2, LengthZ1, LengthZ2, Heading1, Heading2;
    float xFoot1, xFoot2, yFoot1, yFoot2, zFoot1, zFoot2, hFoot1, hFoot2;

    // ==================== VARIABEL TANGAN CARTESIAN ====================
    float LengthArmCartesian[6];
    float armCartesian[6];

    // ==================== INTERPOLASI KAKI (kolom 0-7) ====================
    LengthX1 = (float)(motionRobot.patternMotion[steps][0] - motionRobot.lastMotion[0]);
    LengthY1 = (float)(motionRobot.patternMotion[steps][1] - motionRobot.lastMotion[1]);
    LengthZ1 = (float)(motionRobot.patternMotion[steps][2] - motionRobot.lastMotion[2]);
    Heading1 = (float)(motionRobot.patternMotion[steps][3] - motionRobot.lastMotion[3]);
    LengthX2 = (float)(motionRobot.patternMotion[steps][4] - motionRobot.lastMotion[4]);
    LengthY2 = (float)(motionRobot.patternMotion[steps][5] - motionRobot.lastMotion[5]);
    LengthZ2 = (float)(motionRobot.patternMotion[steps][6] - motionRobot.lastMotion[6]);
    Heading2 = (float)(motionRobot.patternMotion[steps][7] - motionRobot.lastMotion[7]);

    xFoot1 = (((float)theta / maxTheta) * LengthX1) + (float)motionRobot.lastMotion[0];
    yFoot1 = (((float)theta / maxTheta) * LengthY1) + (float)motionRobot.lastMotion[1];
    zFoot1 = (((float)theta / maxTheta) * LengthZ1) + (float)motionRobot.lastMotion[2];
    hFoot1 = (((float)theta / maxTheta) * Heading1) + (float)motionRobot.lastMotion[3];
    xFoot2 = (((float)theta / maxTheta) * LengthX2) + (float)motionRobot.lastMotion[4];
    yFoot2 = (((float)theta / maxTheta) * LengthY2) + (float)motionRobot.lastMotion[5];
    zFoot2 = (((float)theta / maxTheta) * LengthZ2) + (float)motionRobot.lastMotion[6];
    hFoot2 = (((float)theta / maxTheta) * Heading2) + (float)motionRobot.lastMotion[7];

    // ==================== INTERPOLASI TANGAN CARTESIAN (kolom 8-13) ====================
    for (int coord = 0; coord < 6; coord++)
    {
        LengthArmCartesian[coord] = (float)(motionRobot.patternMotion[steps][8 + coord] - motionRobot.lastMotion[8 + coord]);
        armCartesian[coord] = (((float)theta / maxTheta) * LengthArmCartesian[coord]) + (float)motionRobot.lastMotion[8 + coord];
    }

    // Panggil fungsi untuk menghitung servo dengan data gabungan
    getServoDeg(xFoot1, yFoot1, zFoot1, hFoot1, xFoot2, yFoot2, zFoot2, hFoot2, armCartesian);
}

void INSTRUCTION::getServoDeg(float Xleft, float Yleft, float Zleft, float H1,
                              float Xright, float Yright, float Zright, float H2,
                              float *armCartesian)
{
    // ==================== PROSES KAKI ====================
    float rightLegsDeg[6];
    float leftLegsDeg[6];
    float finalXright, finalYright, finalZright;
    float finalXleft, finalYleft, finalZleft;

    float DSrightX = 0.0, DSrightY = 0.0, DSrightZ = 30.0;
    float DSleftX = 0.0, DSleftY = 0.0, DSleftZ = 30.0;

    // Hitung posisi final dengan kompensasi PID
    finalXright = DSrightX + Xright;
    finalYright = DSrightY + Yright;
    finalZright = DSrightZ + Zright;
    finalXleft = DSleftX + Xleft;
    finalYleft = DSleftY + Yleft;
    finalZleft = DSleftZ + Zleft;

    // ==================== INVERSE KINEMATICS KAKI ====================
    // Kaki Kanan
    kinematic.getmAngle(finalXright, finalYright, finalZright, H2, &rightLegsDeg[1], &rightLegsDeg[2], &rightLegsDeg[3]);
    rightLegsDeg[0] = H2;
    rightLegsDeg[4] = rightLegsDeg[2] + rightLegsDeg[3];
    rightLegsDeg[5] = -rightLegsDeg[1];

    // Kaki Kiri
    kinematic.getmAngle(finalXleft, finalYleft, finalZleft, H1, &leftLegsDeg[1], &leftLegsDeg[2], &leftLegsDeg[3]);
    leftLegsDeg[0] = H1;
    leftLegsDeg[4] = leftLegsDeg[2] + leftLegsDeg[3];
    leftLegsDeg[5] = -leftLegsDeg[1];

    // ==================== SMOOTHING TANGAN ====================
    float smoothedArmCartesian[6];
    armSmoother.smoothArmMotion(armCartesian, smoothedArmCartesian);

    // ==================== INVERSE KINEMATICS TANGAN ====================
    float armAngles[12];
    armKinematic.getFullArmAngles12DOF(armCartesian[0], armCartesian[1], armCartesian[2],
                                       armCartesian[3], armCartesian[4], armCartesian[5],
                                       armAngles);

    // Kirim hasil ke servo
    getServoRes(rightLegsDeg, leftLegsDeg);
    getArmServoRes(armAngles);
}

void INSTRUCTION::getServoRes(float *rightLegsDeg, float *leftLegsDeg)
{
    // ============================================= Tumit ==================================================
    motionRobot.sendPos[0] = (int)(motionRobot.defPos[0] + (int)(((rightLegsDeg[5]) * 4095) / 360));
    motionRobot.sendPos[1] = (int)(motionRobot.defPos[1] + (int)(((leftLegsDeg[5]) * 4095) / 360));

    // ============================================= Engkel ======================================================
    motionRobot.sendPos[2] = (int)(motionRobot.defPos[2] + (int)(((rightLegsDeg[4]) * 4095) / 360));
    motionRobot.sendPos[3] = (int)(motionRobot.defPos[3] - (int)(((leftLegsDeg[4]) * 4095) / 360));

    // ============================================= Lutut ======================================================
    motionRobot.sendPos[4] = (int)(motionRobot.defPos[4] + (int)(((rightLegsDeg[3]) * 4095) / 360));
    motionRobot.sendPos[5] = (int)(motionRobot.defPos[5] - (int)(((leftLegsDeg[3]) * 4095) / 360));

    // ============================================= Paha ======================================================
    motionRobot.sendPos[6] = (int)(motionRobot.defPos[6] + (int)(((rightLegsDeg[2]) * 4095) / 360));
    motionRobot.sendPos[7] = (int)(motionRobot.defPos[7] - (int)(((leftLegsDeg[2]) * 4095) / 360));

    // ============================================= Bokong ======================================================
    motionRobot.sendPos[8] = (int)(motionRobot.defPos[8] - (int)(((rightLegsDeg[1]) * 4095) / 360));
    motionRobot.sendPos[9] = (int)(motionRobot.defPos[9] - (int)(((leftLegsDeg[1]) * 4095) / 360));

    // Kirim ke servo kaki
    sync.sendGoalPost(motionRobot.sendPos);
}

void INSTRUCTION::getArmServoRes(float *armAngles)
{
    // ============================== HEAD (ID 2 dan 3) ==============================
    motionRobot.sendPosXL[0] = motionRobot.defPosXL[0];
    motionRobot.sendPosXL[1] = motionRobot.defPosXL[1];

    // ============================== LEFT ARM =======================================
    motionRobot.sendPosXL[2] = (int)(motionRobot.defPosXL[2] + (int)((armAngles[0] * 4095) / 360));
    motionRobot.sendPosXL[4] = (int)(motionRobot.defPosXL[4] + (int)((armAngles[2] * 4095) / 360));
    motionRobot.sendPosXL[6] = (int)(motionRobot.defPosXL[6] + (int)((armAngles[4] * 4095) / 360));
    motionRobot.sendPosXL[8] = (int)(motionRobot.defPosXL[8] + (int)((armAngles[6] * 4095) / 360));
    motionRobot.sendPosXL[10] = (int)(motionRobot.defPosXL[10] + (int)((armAngles[8] * 4095) / 360));
    motionRobot.sendPosXL[12] = (int)(motionRobot.defPosXL[12] + (int)((armAngles[10] * 4095) / 360));

    // ============================== RIGHT ARM ======================================
    motionRobot.sendPosXL[3] = (int)(motionRobot.defPosXL[3] - (int)((armAngles[1] * 4095) / 360));
    motionRobot.sendPosXL[5] = (int)(motionRobot.defPosXL[5] - (int)((armAngles[3] * 4095) / 360));
    motionRobot.sendPosXL[7] = (int)(motionRobot.defPosXL[7] - (int)((armAngles[5] * 4095) / 360));
    motionRobot.sendPosXL[9] = (int)(motionRobot.defPosXL[9] - (int)((armAngles[7] * 4095) / 360));
    motionRobot.sendPosXL[11] = (int)(motionRobot.defPosXL[11] - (int)((armAngles[9] * 4095) / 360));
    motionRobot.sendPosXL[13] = (int)(motionRobot.defPosXL[13] - (int)((armAngles[11] * 4095) / 360));

    // Kirim ke servo XL
    sync.sendGoalXL(motionRobot.sendPosXL, motionRobot.speedXL);
}

void INSTRUCTION::syncsendsservo()
{
    sync.sendGoalXL(motionRobot.sendPosXL, motionRobot.speedXL);
}

// =============== EXECUTE FUNCTION - FIXED VERSION ===============
void INSTRUCTION::execute(int robotMotion)
{
    steps = 0;
    theta = 0.0;
    set(robotMotion);

    while (steps < endSteps) // endSteps = 8, jadi steps akan 0-7
    {
        // CHECK FOR EMERGENCY FLAGS (from main.cpp)
        extern bool motionChangeRequested;
        extern int requestedMotionType;
        
        // Handle emergency change request
        if (motionChangeRequested && requestedMotionType != robotMotion) {
            Serial.println("!!! EMERGENCY CHANGE DETECTED IN EXECUTE !!!");
            set(requestedMotionType);
            robotMotion = requestedMotionType;
            motionChangeRequested = false;
        }

        if (theta >= maxTheta)
        {
            // Update posisi terakhir untuk semua 14 kolom (8 kaki + 6 tangan)
            for (int col = startColl; col < endColl; col++) // 0-13 (tanpa speed)
            {
                motionRobot.lastMotion[col] = motionRobot.patternMotion[steps][col];
            }
            
            // PERBAIKAN UTAMA: Pastikan steps tidak melebihi batas array
            steps++;
            theta = 0;
            
            // Debug info
            Serial.print("Completed Step: ");
            Serial.print(steps - 1);
            Serial.print(" | Next Step: ");
            Serial.println(steps);
        }

        // SAFE ACCESS: Pastikan steps masih dalam bounds sebelum akses array
        if (steps < endSteps) {
            // Gunakan kolom speed (index 14) untuk increment theta
            theta += motionRobot.patternMotion[steps][14];
            
            // Call trajectory dengan bounds checking
            trajectory(steps, theta);
            
            // Debug info (compact)
            if (theta == motionRobot.patternMotion[steps][14]) { // First theta increment of this step
                Serial.print("S");
                Serial.print(steps);
                Serial.print("(");
                Serial.print(motionRobot.patternMotion[steps][14]);
                Serial.print(") ");
            }
        }

        if (theta >= maxTheta)
            theta = maxTheta;

        delay(10); // Critical timing preserved
    }

    // Reset untuk cycle berikutnya
    steps = 0;
    theta = 0.0;
    
    Serial.println("\n>>> Motion cycle completed <<<");
}
