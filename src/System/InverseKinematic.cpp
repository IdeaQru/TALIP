#include "System/InverseKinematic.h"
void KINEMATIC :: getmAngle(float x, float y, float z, float Head, float *mAngle_0, float *mAngle_1, float *mAngle_2){ //Included in void CalculateServoAngle Function 
  float PHI = 57.295779513082320876798154814105;                             
  float Resultan0, Resultan1, Resultan2;
  float Alfa, aY, aX;
  float sA = 0, sB = 0, sC = 0;
  float sBA, sBB, sCA, sCB, sD;
  float zz;


  Resultan0 = sqrt((y*y) + (x*x));
  if(Resultan0 >= Frame1 + Frame2)Resultan0 = Frame1 + Frame2;
  
  zz = ((Frame0 + Frame1 + Frame2) - z) + Frame0;
  Alfa = (float)(atan2(y,x) * PHI) - Head;
  aY = (sin(Alfa / PHI) * Resultan0);
  aX = (cos(Alfa / PHI) * Resultan0);

  sD = (atan2(aY,zz) * PHI);
  
  Resultan1 = sqrt((zz*zz) + (aY*aY));
  if(Resultan1 >= Frame1 + Frame2)Resultan1 = Frame1 + Frame2;

  Resultan2 = sqrt((aX*aX) + (Resultan1*Resultan1));
  if(Resultan2 >= Frame1 + Frame2)Resultan2 = Frame1 + Frame2;
  
  sCA = (float)((Frame1*Frame1) + (Frame2*Frame2)) - (Resultan2*Resultan2);
  sCB = (float)(2 * Frame1 * Frame2);
  sC = (float)acos(sCA / sCB) * PHI ;
  sA = (float)atan2(Resultan1,-aX) * PHI;
  sBA = (float)Frame2 * sin(sC / PHI);
  sBB = (float)Frame1 + (Frame2 * cos(sC / PHI));
  sB = (float)atan2(sBA,sBB) * PHI;
  
  *mAngle_0 = -sD;
  *mAngle_1 = 0 - (sA - sB);
  *mAngle_2 = (180 - sC);
}
