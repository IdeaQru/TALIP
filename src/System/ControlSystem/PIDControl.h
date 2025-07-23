#include <Arduino.h>
#include<math.h>
#define Setpoint 0.0
// #define Kp 4.43
// #define Ki 40.9
// #define Kd 0.12

#define Kp 1.0
#define Ki 0.12
#define Kd 10.0

// #define Kp 0.5
// #define Ki 0.08
// #define Kd 16.0

#define Kp1 1.2
#define Ki1 0.5
#define Kd1 6.0

#define Kp2 0.3
#define Ki2 0.05
#define Kd2 5.0
class PID {
    public :
        void setPID(double datInput1, double datInput2, double datInput3);
        double getPIDX();
        double getPIDY();
        double getPIDZ();

    private :
        unsigned long lastTime;
	    double outputSum, lastInput;
        unsigned long SampleTime;
        double outMin, outMax;
        bool inAuto, pOnE;
        double output;
        double finalOut;
        double proporsional, integral, derivative;
        
        unsigned long lastTime1;
	    double outputSum1, lastInput1;
        unsigned long SampleTime1;
        double outMin1, outMax1;
        bool inAuto1, pOnE1;
        double output1;
        double finalOut1;
        double proporsional1, integral1, derivative1;

        unsigned long lastTime2;
	    double outputSum2, lastInput2;
        unsigned long SampleTime2;
        double outMin2, outMax2;
        bool inAuto2, pOnE2;
        double output2;
        double finalOut2;
        double proporsional2, integral2, derivative2;
};