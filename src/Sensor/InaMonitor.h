#ifndef INA_MONITOR_H
#define INA_MONITOR_H

#include <Wire.h>
#include <Arduino.h>

// INA219 Register Addresses
#define INA219_ADDRESS                 0x40
#define INA219_REG_CONFIG              0x00
#define INA219_REG_SHUNTVOLTAGE        0x01
#define INA219_REG_BUSVOLTAGE          0x02
#define INA219_REG_POWER               0x03
#define INA219_REG_CURRENT             0x04
#define INA219_REG_CALIBRATION         0x05

// Configuration Register Values
#define INA219_CONFIG_RESET            0x8000
#define INA219_CONFIG_BVOLTAGERANGE_32V 0x2000
#define INA219_CONFIG_GAIN_8_320MV     0x1800
#define INA219_CONFIG_BADCRES_12BIT    0x0400
#define INA219_CONFIG_SADCRES_12BIT    0x0008
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS 0x0007

// Kalman Filter Structure
struct KalmanFilter {
    float Q;     // Process noise covariance
    float R;     // Measurement noise covariance
    float P;     // Estimation error covariance
    float K;     // Kalman gain
    float X;     // Estimated value
    float lastError;  // Last error for adaptive adjustment
    unsigned long lastUpdate;  // Last update time
};

class INA_Monitor {
private:
    uint8_t ina_address;
    float current_LSB;
    float power_LSB;
    float shunt_resistance;
    
    // Battery monitoring variables
    float batteryCapacity;
    float currentCapacity;
    float lowBatteryThreshold;
    unsigned long lastBatteryCheck;
    unsigned long batteryCheckInterval;
    bool lowBatteryWarning;
    
    // Kalman filters for different measurements
    KalmanFilter voltageFilter;
    KalmanFilter currentFilter;
    KalmanFilter powerFilter;
    
    // Private functions
    void writeRegister(uint8_t reg, uint16_t value);
    uint16_t readRegister(uint8_t reg);
    int16_t readRegisterSigned(uint8_t reg);
    
    // Kalman filter functions
    void initKalmanFilter(KalmanFilter* kf, float Q, float R, float P, float initialValue);
    float kalmanFilter(KalmanFilter* kf, float measurement);
    void autoAdjustKalmanFilter(KalmanFilter* kf, float error);
    
    // Helper functions
    float estimateCapacityFromVoltage(float voltage);
    float getRawBusVoltage();
    float getRawCurrent();
    float getRawPower();
    
public:
    INA_Monitor(uint8_t address = INA219_ADDRESS);
    bool init();
    void calibrate(float shunt_ohms = 0.1, float max_current = 3.2);
    
    // Reading functions with Kalman filtering
    float getBusVoltage();
    float getShuntVoltage();
    float getCurrent();
    float getPower();
    
    // Raw reading functions (without filtering)
    float getRawBusVoltageReading();
    float getRawCurrentReading();
    float getRawPowerReading();
    
    // Battery monitoring functions
    void setBatteryCapacity(float capacity_mAh);
    void setLowBatteryThreshold(float threshold_percent);
    void checkBatteryStatus();
    float getBatteryPercentage();
    bool isLowBattery();
    void printBatteryStatus();
    void printDetailedStatus();
    void resetBatteryCapacity();
    
    // Kalman filter control
    void resetKalmanFilters();
    void setKalmanParameters(float voltageQ, float voltageR, float currentQ, float currentR);
    void printKalmanStatus();
};

#endif
