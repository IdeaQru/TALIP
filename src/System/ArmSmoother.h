#ifndef ARM_SMOOTHER_H
#define ARM_SMOOTHER_H

#include <Arduino.h>

class ARM_SMOOTHER {
private:
    // Buffer untuk menyimpan posisi sebelumnya
    float previousArmCartesian[6];
    float currentArmCartesian[6];
    float targetArmCartesian[6];
    
    // Parameter smoothing
    float smoothingFactor;
    float velocityLimit;
    float accelerationLimit;
    
    // Buffer untuk velocity dan acceleration
    float armVelocity[6];
    float armAcceleration[6];
    
    // Moving average buffer
    static const int BUFFER_SIZE = 5;
    float movingAverageBuffer[6][BUFFER_SIZE];
    int bufferIndex;
    
    // Timing
    unsigned long lastUpdateTime;
    float deltaTime;
    
public:
    ARM_SMOOTHER();
    
    // Inisialisasi smoother
    void init(float smoothFactor = 0.8f, float velLimit = 50.0f, float accelLimit = 100.0f);
    
    // Fungsi smoothing utama
    void smoothArmMotion(float *inputCartesian, float *outputCartesian);
    
    // Berbagai metode smoothing
    void exponentialSmoothing(float *input, float *output);
    void velocityLimitedSmoothing(float *input, float *output);
    void movingAverageSmoothing(float *input, float *output);
    void bezierSmoothing(float *input, float *output, float t);
    
    // Utility functions
    void setParameters(float smoothFactor, float velLimit, float accelLimit);
    void reset();
    float constrainValue(float value, float minVal, float maxVal);
    void updateTiming();
};

#endif
