#include "System/ControlSystem/PIDControl.h"

void PID :: setPID (double datInput1, double datInput2, double datInput3) {
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
    double input = datInput1;
    double input1 = datInput2;
    double input2 = datInput3;

    double error = Setpoint - input;
    double error1 = Setpoint - input1;
    double error2 = Setpoint - input2;

    double dInput = (input - lastInput);
    double dInput1 = (input1 - lastInput1);
    double dInput2 = (input2 - lastInput2);

    integral = (Ki * error) - (Kp * dInput);
    integral1 = (Ki1 * error1) - (Kp1 * dInput1);
    integral2 = (Ki2 * error2) - (Kp2 * dInput2);

    proporsional = Kp * error;
    proporsional1 = Kp1 * error1;
    proporsional2 = Kp2 * error2;

    derivative = Kd * dInput;
    derivative1 = Kd1 * dInput1;
    derivative2 = Kd2 * dInput2;

	finalOut = integral + proporsional - derivative;
    finalOut1 = integral1 + proporsional1 - derivative1;
    finalOut2 = integral2 + proporsional2 - derivative2;

    lastInput = input;
    lastInput1 = input1;
    lastInput2 = input2;
    lastTime = now;  
   }

//    return finalOut;
}

double PID :: getPIDX(){
    return finalOut;
}
double PID :: getPIDY(){
    return finalOut1;
}
double PID :: getPIDZ(){
    return finalOut2;
}