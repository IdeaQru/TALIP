#include "ArmSmoother.h"

ARM_SMOOTHER::ARM_SMOOTHER() {
    init();
    reset();
}

void ARM_SMOOTHER::init(float smoothFactor, float velLimit, float accelLimit) {
    smoothingFactor = smoothFactor;
    velocityLimit = velLimit;
    accelerationLimit = accelLimit;
    bufferIndex = 0;
    lastUpdateTime = millis();
    
    // Inisialisasi buffer
    for(int i = 0; i < 6; i++) {
        previousArmCartesian[i] = 0.0f;
        currentArmCartesian[i] = 0.0f;
        targetArmCartesian[i] = 0.0f;
        armVelocity[i] = 0.0f;
        armAcceleration[i] = 0.0f;
        
        for(int j = 0; j < BUFFER_SIZE; j++) {
            movingAverageBuffer[i][j] = 0.0f;
        }
    }
}

void ARM_SMOOTHER::smoothArmMotion(float *inputCartesian, float *outputCartesian) {
    updateTiming();
    
    // Copy input ke target
    for(int i = 0; i < 6; i++) {
        targetArmCartesian[i] = inputCartesian[i];
    }
    
    // Terapkan kombinasi smoothing methods
    float tempOutput1[6], tempOutput2[6];
    
    // Step 1: Exponential smoothing
    exponentialSmoothing(targetArmCartesian, tempOutput1);
    
    // Step 2: Velocity limiting
    velocityLimitedSmoothing(tempOutput1, tempOutput2);
    
    // Step 3: Moving average untuk mengurangi noise
    movingAverageSmoothing(tempOutput2, outputCartesian);
    
    // Update posisi sebelumnya
    for(int i = 0; i < 6; i++) {
        previousArmCartesian[i] = currentArmCartesian[i];
        currentArmCartesian[i] = outputCartesian[i];
    }
}

void ARM_SMOOTHER::exponentialSmoothing(float *input, float *output) {
    for(int i = 0; i < 6; i++) {
        output[i] = (smoothingFactor * input[i]) + ((1.0f - smoothingFactor) * currentArmCartesian[i]);
    }
}

void ARM_SMOOTHER::velocityLimitedSmoothing(float *input, float *output) {
    if(deltaTime <= 0) {
        for(int i = 0; i < 6; i++) {
            output[i] = input[i];
        }
        return;
    }
    
    for(int i = 0; i < 6; i++) {
        float targetVelocity = (input[i] - currentArmCartesian[i]) / deltaTime;
        
        // Limit velocity
        targetVelocity = constrainValue(targetVelocity, -velocityLimit, velocityLimit);
        
        // Calculate acceleration
        float targetAcceleration = (targetVelocity - armVelocity[i]) / deltaTime;
        targetAcceleration = constrainValue(targetAcceleration, -accelerationLimit, accelerationLimit);
        
        // Update velocity and position
        armVelocity[i] += targetAcceleration * deltaTime;
        output[i] = currentArmCartesian[i] + (armVelocity[i] * deltaTime);
    }
}

void ARM_SMOOTHER::movingAverageSmoothing(float *input, float *output) {
    // Update buffer
    for(int i = 0; i < 6; i++) {
        movingAverageBuffer[i][bufferIndex] = input[i];
    }
    
    // Calculate moving average
    for(int i = 0; i < 6; i++) {
        float sum = 0.0f;
        for(int j = 0; j < BUFFER_SIZE; j++) {
            sum += movingAverageBuffer[i][j];
        }
        output[i] = sum / BUFFER_SIZE;
    }
    
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
}

void ARM_SMOOTHER::bezierSmoothing(float *input, float *output, float t) {
    // Cubic Bezier interpolation untuk smooth transitions
    t = constrainValue(t, 0.0f, 1.0f);
    float t2 = t * t;
    float t3 = t2 * t;
    float invT = 1.0f - t;
    float invT2 = invT * invT;
    float invT3 = invT2 * invT;
    
    for(int i = 0; i < 6; i++) {
        float p0 = previousArmCartesian[i];
        float p1 = previousArmCartesian[i] + (armVelocity[i] * 0.1f);
        float p2 = input[i] - (armVelocity[i] * 0.1f);
        float p3 = input[i];
        
        output[i] = (invT3 * p0) + (3 * invT2 * t * p1) + (3 * invT * t2 * p2) + (t3 * p3);
    }
}

void ARM_SMOOTHER::setParameters(float smoothFactor, float velLimit, float accelLimit) {
    smoothingFactor = constrainValue(smoothFactor, 0.1f, 1.0f);
    velocityLimit = velLimit;
    accelerationLimit = accelLimit;
}

void ARM_SMOOTHER::reset() {
    for(int i = 0; i < 6; i++) {
        previousArmCartesian[i] = 0.0f;
        currentArmCartesian[i] = 0.0f;
        targetArmCartesian[i] = 0.0f;
        armVelocity[i] = 0.0f;
        armAcceleration[i] = 0.0f;
    }
    bufferIndex = 0;
    lastUpdateTime = millis();
}

float ARM_SMOOTHER::constrainValue(float value, float minVal, float maxVal) {
    if(value < minVal) return minVal;
    if(value > maxVal) return maxVal;
    return value;
}

void ARM_SMOOTHER::updateTiming() {
    unsigned long currentTime = millis();
    deltaTime = (currentTime - lastUpdateTime) / 1000.0f; // Convert to seconds
    lastUpdateTime = currentTime;
    
    // Prevent division by zero and unrealistic delta times
    if(deltaTime <= 0 || deltaTime > 0.1f) {
        deltaTime = 0.01f; // Default 10ms
    }
}
