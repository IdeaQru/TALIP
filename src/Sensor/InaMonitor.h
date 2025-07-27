#ifndef INAMONITOR_H
#define INAMONITOR_H

#include <Arduino.h>
#include <Wire.h>

// INA219 Register definitions
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNTVOLTAGE 0x01
#define INA219_REG_BUSVOLTAGE   0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_REG_CALIBRATION  0x05

// INA219 Config register values
#define INA219_CONFIG_RESET                     0x8000
#define INA219_CONFIG_BVOLTAGERANGE_32V         0x2000
#define INA219_CONFIG_GAIN_8_320MV              0x1800
#define INA219_CONFIG_BADCRES_12BIT             0x0400
#define INA219_CONFIG_SADCRES_12BIT             0x0018
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS 0x0007

// Temperature sensor pin
#define TEMP_SENSOR_PIN 34

// DS18B20 Commands
#define DS18B20_SKIP_ROM        0xCC
#define DS18B20_CONVERT_T       0x44
#define DS18B20_READ_SCRATCHPAD 0xBE
#define DS18B20_SEARCH_ROM      0xF0

// Kalman Filter structure
struct KalmanFilter {
    float Q; // Process noise
    float R; // Measurement noise
    float P; // Estimation error
    float K; // Kalman gain
    float X; // Estimated value
    float lastError;
    unsigned long lastUpdate;
};

// Decision Tree Node structure
struct DecisionNode {
    String parameter;
    float threshold;
    String condition; // ">=", "<", "==", etc.
    int leftChild;
    int rightChild;
    String classification; // For leaf nodes: "HIGH", "MEDIUM", "LOW"
    bool isLeaf;
};

// Battery Health Classification (3 classes)
enum BatteryHealthClass {
    HEALTH_HIGH,
    HEALTH_MEDIUM,
    HEALTH_LOW
};

// Temperature Classification (2 classes)
enum TemperatureClass {
    TEMP_NORMAL,
    TEMP_OVERHEAT
};

// System Performance Classification
enum PerformanceClass {
    PERFORMANCE_OPTIMAL,
    PERFORMANCE_GOOD,
    PERFORMANCE_DEGRADED,
    PERFORMANCE_CRITICAL
};

class INA_Monitor {
private:
    uint8_t ina_address;
    float current_LSB;
    float power_LSB;
    float shunt_resistance;
    
    // Battery monitoring
    float batteryCapacity;
    float currentCapacity;
    float lowBatteryThreshold;
    unsigned long lastBatteryCheck;
    unsigned long batteryCheckInterval;
    bool lowBatteryWarning;
    
    // Temperature sensor
    uint8_t tempSensorPin;
    float currentTemperature;
    
    // Kalman filters
    KalmanFilter voltageFilter;
    KalmanFilter currentFilter;
    KalmanFilter powerFilter;
    KalmanFilter temperatureFilter;
    
    // Decision Tree
    static const int MAX_NODES = 20;
    DecisionNode decisionTree[MAX_NODES];
    int nodeCount;
    
    // Classification results
    BatteryHealthClass batteryHealth;
    TemperatureClass temperatureClass;
    PerformanceClass systemPerformance;
    
    // Helper methods
    void writeRegister(uint8_t reg, uint16_t value);
    uint16_t readRegister(uint8_t reg);
    int16_t readRegisterSigned(uint8_t reg);
    float estimateCapacityFromVoltage(float voltage);
    
    // Raw DS18B20 Protocol Implementation
    bool oneWireReset();
    void oneWireWriteBit(bool bit);
    bool oneWireReadBit();
    void oneWireWriteByte(uint8_t data);
    uint8_t oneWireReadByte();
    bool ds18b20StartConversion();
    bool ds18b20ReadTemperature(float* temperature);
    
    // Kalman filter methods
    void initKalmanFilter(KalmanFilter* kf, float Q, float R, float P, float initialValue);
    float kalmanFilter(KalmanFilter* kf, float measurement);
    void autoAdjustKalmanFilter(KalmanFilter* kf, float error);
    
    // Decision Tree methods
    void initializeDecisionTree();
    String evaluateNode(int nodeIndex, float voltage, float current, float power, float temperature, float batteryPercent);
    bool evaluateCondition(String parameter, String condition, float threshold, 
                          float voltage, float current, float power, float temperature, float batteryPercent);
    
    // Classification methods
    BatteryHealthClass classifyBatteryHealth(float voltage, float percentage, float temperature);
    TemperatureClass classifyTemperature(float temperature);
    PerformanceClass classifySystemPerformance(float power, float efficiency, float temperature);
    
public:
    INA_Monitor(uint8_t address = 0x40);
    float overheatThreshold;
    
    // Basic INA219 methods
    bool init();
    void calibrate(float shunt_ohms = 0.1, float max_current = 3.2);
    
    // Raw sensor readings
    float getRawBusVoltage();
    float getRawCurrent(); 
    float getRawPower();
    float getRawTemperature();
    
    // Filtered sensor readings
    float getBusVoltage();
    float getShuntVoltage();
    float getCurrent();
    float getPower();
    float getTemperature();
    
    // Advanced metrics
    float getPowerEfficiency();
    
    // Battery management
    void setBatteryCapacity(float capacity_mAh);
    void setLowBatteryThreshold(float threshold_percent);
    void checkBatteryStatus();
    float getBatteryPercentage();
    bool isLowBattery();
    void resetBatteryCapacity();
    
    // Temperature management
    void setOverheatThreshold(float threshold = 55.0);
    bool isOverheating();
    
    // Decision Tree & Classification
    void updateClassifications();
    String getOverallSystemStatus();
    BatteryHealthClass getBatteryHealthClass();
    TemperatureClass getTemperatureClass();
    PerformanceClass getPerformanceClass();
    String getBatteryHealthString();
    String getTemperatureClassString();
    
    // Printing methods
    void printBatteryStatus();
    void printTemperatureStatus();
    void printDetailedStatus();
    void printClassificationResults();
    void printKalmanStatus();
    
    // Kalman filter management
    void resetKalmanFilters();
    void setKalmanParameters(float voltageQ, float voltageR, float currentQ, float currentR);
    
    // Raw readings for debugging
    float getRawBusVoltageReading();
    float getRawCurrentReading();
    float getRawPowerReading();
};

#endif
