// #include "System/Process.h"

// int periode = 5000;

// void INSTRUCTION::set(int motion)
// {
//     for (int row = 0; row < 8; row++)
//     {
//         for (int col = 0; col < 21; col++)  // 21 kolom total (8 kaki + 12 tangan + 1 speed)
//         {
//             if (motion == 1)
//                 motionRobot.patternMotion[row][col] = motionRobot.standMotion[row][col];
//             if (motion == 2)
//                 motionRobot.patternMotion[row][col] = motionRobot.walkMotion[row][col];
//         }
//     }
// }

// void INSTRUCTION::trajectory(int steps, float theta)
// {
//     // ==================== VARIABEL KAKI ====================
//     float LengthX1, LengthX2, LengthY1, LengthY2, LengthZ1, LengthZ2, Heading1, Heading2;
//     float xFoot1, xFoot2, yFoot1, yFoot2, zFoot1, zFoot2, hFoot1, hFoot2;
    
//     // ==================== VARIABEL TANGAN ====================
//     float LengthArm[12]; // 12 DOF tangan
//     float armPos[12];    // Posisi interpolasi tangan

//     // ==================== INTERPOLASI KAKI (kolom 0-7) ====================
//     LengthX1 = (float)(motionRobot.patternMotion[steps][0] - motionRobot.lastMotion[0]);
//     LengthY1 = (float)(motionRobot.patternMotion[steps][1] - motionRobot.lastMotion[1]);
//     LengthZ1 = (float)(motionRobot.patternMotion[steps][2] - motionRobot.lastMotion[2]);
//     Heading1 = (float)(motionRobot.patternMotion[steps][3] - motionRobot.lastMotion[3]);
//     LengthX2 = (float)(motionRobot.patternMotion[steps][4] - motionRobot.lastMotion[4]);
//     LengthY2 = (float)(motionRobot.patternMotion[steps][5] - motionRobot.lastMotion[5]);
//     LengthZ2 = (float)(motionRobot.patternMotion[steps][6] - motionRobot.lastMotion[6]);
//     Heading2 = (float)(motionRobot.patternMotion[steps][7] - motionRobot.lastMotion[7]);

//     xFoot1 = (((float)theta / maxTheta) * LengthX1) + (float)motionRobot.lastMotion[0];
//     yFoot1 = (((float)theta / maxTheta) * LengthY1) + (float)motionRobot.lastMotion[1];
//     zFoot1 = (((float)theta / maxTheta) * LengthZ1) + (float)motionRobot.lastMotion[2];
//     hFoot1 = (((float)theta / maxTheta) * Heading1) + (float)motionRobot.lastMotion[3];
//     xFoot2 = (((float)theta / maxTheta) * LengthX2) + (float)motionRobot.lastMotion[4];
//     yFoot2 = (((float)theta / maxTheta) * LengthY2) + (float)motionRobot.lastMotion[5];
//     zFoot2 = (((float)theta / maxTheta) * LengthZ2) + (float)motionRobot.lastMotion[6];
//     hFoot2 = (((float)theta / maxTheta) * Heading2) + (float)motionRobot.lastMotion[7];
    
//     // ==================== INTERPOLASI TANGAN (kolom 8-19) ====================
//     for (int dof = 0; dof < 12; dof++)
//     {
//         LengthArm[dof] = (float)(motionRobot.patternMotion[steps][8 + dof] - motionRobot.lastMotion[8 + dof]);
//         armPos[dof] = (((float)theta / maxTheta) * LengthArm[dof]) + (float)motionRobot.lastMotion[8 + dof];
//     }

//     // Panggil fungsi untuk menghitung servo dengan data gabungan
//     getServoDeg(xFoot1, yFoot1, zFoot1, hFoot1, xFoot2, yFoot2, zFoot2, hFoot2, armPos);
// }

// void INSTRUCTION::getServoDeg(float Xleft, float Yleft, float Zleft, float H1, 
//                              float Xright, float Yright, float Zright, float H2, 
//                              float *armData)
// {
//     // ==================== PROSES KAKI ====================
//     float rightLegsDeg[6];
//     float leftLegsDeg[6];
//     float finalXright, finalYright, finalZright;
//     float finalXleft, finalYleft, finalZleft;

//     float DSrightX = 0.0, DSrightY = 0.0, DSrightZ = 30.0;
//     float DSleftX = 0.0, DSleftY = 0.0, DSleftZ = 30.0;

//     // Baca sensor dan hitung PID
//     gyro.read();
//     kalman.getKRoll(gyro.getRoll());
//     kalman.getKPitch(gyro.getPitch());
//     pid.setPID(kalman.getXerror(), kalman.getYerror(), kalman.getZerror());
    
//     // Hitung posisi final dengan kompensasi PID
//     finalXright = DSrightX + Xright - pid.getPIDX();
//     finalYright = DSrightY + Yright - pid.getPIDY();
//     finalZright = DSrightZ + Zright - pid.getPIDZ();
//     finalXleft = DSleftX + Xleft - pid.getPIDX();
//     finalYleft = DSleftY + Yleft - pid.getPIDY();
//     finalZleft = DSleftZ + Zleft - pid.getPIDZ();
    
//     // ==================== INVERSE KINEMATICS KAKI ====================
//     // Kaki Kanan
//     kinematic.getmAngle(finalXright, finalYright, finalZright, H2, &rightLegsDeg[1], &rightLegsDeg[2], &rightLegsDeg[3]);
//     rightLegsDeg[0] = H2;
//     rightLegsDeg[4] = rightLegsDeg[2] + rightLegsDeg[3];
//     rightLegsDeg[5] = -rightLegsDeg[1];
    
//     // Kaki Kiri
//     kinematic.getmAngle(finalXleft, finalYleft, finalZleft, H1, &leftLegsDeg[1], &leftLegsDeg[2], &leftLegsDeg[3]);
//     leftLegsDeg[0] = H1;
//     leftLegsDeg[4] = leftLegsDeg[2] + leftLegsDeg[3];
//     leftLegsDeg[5] = -leftLegsDeg[1];
    
//     // Kirim hasil ke servo
//     getServoRes(rightLegsDeg, leftLegsDeg);
//     getArmServoRes(armData);
// }

// void INSTRUCTION::getServoRes(float *rightLegsDeg, float *leftLegsDeg)
// {
//     // ============================================= Tumit ==================================================
//     motionRobot.sendPos[0] = (int)(motionRobot.defPos[0] + (int)(((rightLegsDeg[5]) * 4095) / 360)); // ID 1
//     motionRobot.sendPos[1] = (int)(motionRobot.defPos[1] + (int)(((leftLegsDeg[5]) * 4095) / 360));  // ID 2
    
//     // ============================================= Engkel ======================================================
//     motionRobot.sendPos[2] = (int)(motionRobot.defPos[2] + (int)(((rightLegsDeg[4]) * 4095) / 360)); // ID 3
//     motionRobot.sendPos[3] = (int)(motionRobot.defPos[3] - (int)(((leftLegsDeg[4]) * 4095) / 360));  // ID 4
    
//     // ============================================= Lutut ======================================================
//     motionRobot.sendPos[4] = (int)(motionRobot.defPos[4] + (int)(((rightLegsDeg[3]) * 4095) / 360)); // ID 5
//     motionRobot.sendPos[5] = (int)(motionRobot.defPos[5] - (int)(((leftLegsDeg[3]) * 4095) / 360));  // ID 6
    
//     // ============================================= Paha ======================================================
//     motionRobot.sendPos[6] = (int)(motionRobot.defPos[6] + (int)(((rightLegsDeg[2]) * 4095) / 360)); // ID 7
//     motionRobot.sendPos[7] = (int)(motionRobot.defPos[7] - (int)(((leftLegsDeg[2]) * 4095) / 360));  // ID 8
    
//     // ============================================= Bokong ======================================================
//     motionRobot.sendPos[8] = (int)(motionRobot.defPos[8] - (int)(((rightLegsDeg[1]) * 4095) / 360)); // ID 9
//     motionRobot.sendPos[9] = (int)(motionRobot.defPos[9] - (int)(((leftLegsDeg[1]) * 4095) / 360));  // ID 10

//     // Kirim ke servo kaki
//     sync.sendGoalPost(motionRobot.sendPos);
// }

// void INSTRUCTION::getArmServoRes(float *armData)
// {
//     // Konversi data tangan ke posisi servo XL
//     // armData mapping: [0-5] = Left Arm, [6-11] = Right Arm
//     // Servo ID mapping: 4,6,8,10,12,14 = Left, 5,7,9,11,13,15 = Right
    
//     // ============================================= LEFT ARM ===============================================
//     // Left Shoulder Pitch (ID 4) - Index 0 di array XL
//     motionRobot.sendPosXL[0] = (int)(motionRobot.defPosXL[0] + (int)((armData[0] * 4095) / 360));
    
//     // Left Shoulder Roll (ID 6) - Index 2 di array XL
//     motionRobot.sendPosXL[2] = (int)(motionRobot.defPosXL[2] + (int)((armData[1] * 4095) / 360));
    
//     // Left Elbow Pitch (ID 8) - Index 4 di array XL
//     motionRobot.sendPosXL[4] = (int)(motionRobot.defPosXL[4] + (int)((armData[2] * 4095) / 360));
    
//     // Left Elbow Yaw (ID 10) - Index 6 di array XL
//     motionRobot.sendPosXL[6] = (int)(motionRobot.defPosXL[6] + (int)((armData[3] * 4095) / 360));
    
//     // Left Wrist Pitch (ID 12) - Index 8 di array XL
//     motionRobot.sendPosXL[8] = (int)(motionRobot.defPosXL[8] + (int)((armData[4] * 4095) / 360));
    
//     // Left Wrist Roll (ID 14) - Index 10 di array XL
//     motionRobot.sendPosXL[10] = (int)(motionRobot.defPosXL[10] + (int)((armData[5] * 4095) / 360));
    
//     // ============================================= RIGHT ARM ==============================================
//     // Right Shoulder Pitch (ID 5) - Index 1 di array XL
//     motionRobot.sendPosXL[1] = (int)(motionRobot.defPosXL[1] - (int)((armData[6] * 4095) / 360));
    
//     // Right Shoulder Roll (ID 7) - Index 3 di array XL
//     motionRobot.sendPosXL[3] = (int)(motionRobot.defPosXL[3] - (int)((armData[7] * 4095) / 360));
    
//     // Right Elbow Pitch (ID 9) - Index 5 di array XL
//     motionRobot.sendPosXL[5] = (int)(motionRobot.defPosXL[5] - (int)((armData[8] * 4095) / 360));
    
//     // Right Elbow Yaw (ID 11) - Index 7 di array XL
//     motionRobot.sendPosXL[7] = (int)(motionRobot.defPosXL[7] - (int)((armData[9] * 4095) / 360));
    
//     // Right Wrist Pitch (ID 13) - Index 9 di array XL
//     motionRobot.sendPosXL[9] = (int)(motionRobot.defPosXL[9] - (int)((armData[10] * 4095) / 360));
    
//     // Right Wrist Roll (ID 15) - Index 11 di array XL
//     motionRobot.sendPosXL[11] = (int)(motionRobot.defPosXL[11] - (int)((armData[11] * 4095) / 360));
    
//     // ============================================= HEAD (ID 123) =========================================
//     // Head - Index 12 di array XL
//     motionRobot.sendPosXL[12] = motionRobot.defPosXL[12]; // Posisi default atau bisa diatur terpisah
    
//     // Kirim ke servo XL
//     sync.sendGoalXL(motionRobot.sendPosXL, motionRobot.speedXL);
// }

// void INSTRUCTION::execute(int robotMotion)
// {
//     steps = 0;
//     theta = 0.0;
//     set(robotMotion);
    
//     while (steps < endSteps)
//     {
//         Serial.print("Step: ");
//         Serial.println(steps);

//         if (theta >= maxTheta)
//         {
//             // Update posisi terakhir untuk semua 20 kolom (8 kaki + 12 tangan)
//             for (int col = startColl; col < endColl; col++)  // 0-19 (tanpa speed)
//             {
//                 motionRobot.lastMotion[col] = motionRobot.patternMotion[steps][col];
//             }
//             steps++;
//             theta = 0;
//         }
        
//         // Gunakan kolom speed (index 20) untuk increment theta
//         theta += motionRobot.patternMotion[steps][20];
//         trajectory(steps, theta);

//         if (theta >= maxTheta)
//             theta = maxTheta;
            
//         delay(10); // Small delay untuk stabilitas
//     }
// }
