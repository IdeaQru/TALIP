#include "InaMonitor.h"

INA_Monitor::INA_Monitor(uint8_t address) {
    ina_address = address;
    current_LSB = 0.0;
    power_LSB = 0.0;
    shunt_resistance = 0.1;
    
    // Default battery settings
    batteryCapacity = 2200.0;
    currentCapacity = 2200.0;
    lowBatteryThreshold = 20.0;
    lastBatteryCheck = 0;
    batteryCheckInterval = 1000;
    lowBatteryWarning = false;
    
    // Initialize Kalman filters
    initKalmanFilter(&voltageFilter, 0.01, 0.1, 1.0, 12.0);  // Voltage filter
    initKalmanFilter(&currentFilter, 0.1, 1.0, 1.0, 0.0);    // Current filter
    initKalmanFilter(&powerFilter, 0.1, 2.0, 1.0, 0.0);      // Power filter
}

bool INA_Monitor::init() {
    Wire.begin();
    
    // Reset INA219
    writeRegister(INA219_REG_CONFIG, INA219_CONFIG_RESET);
    delay(10);
    
    // Configure INA219
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                      INA219_CONFIG_GAIN_8_320MV |
                      INA219_CONFIG_BADCRES_12BIT |
                      INA219_CONFIG_SADCRES_12BIT |
                      INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
    
    writeRegister(INA219_REG_CONFIG, config);
    
    // Default calibration
    calibrate();
    
    // Reset Kalman filters with initial readings
    delay(100);
    float initialVoltage = getRawBusVoltage();
    float initialCurrent = getRawCurrent();
    float initialPower = getRawPower();
    
    voltageFilter.X = initialVoltage;
    currentFilter.X = initialCurrent;
    powerFilter.X = initialPower;
    
    Serial.println("INA219 with Kalman Filter initialized");
    Serial.println("Initial readings - V: " + String(initialVoltage, 2) + 
                   "V, I: " + String(initialCurrent, 2) + 
                   "mA, P: " + String(initialPower, 2) + "mW");
    
    return true;
}

void INA_Monitor::calibrate(float shunt_ohms, float max_current) {
    shunt_resistance = shunt_ohms;
    
    // Calculate LSB values
    current_LSB = max_current / 32767.0;
    power_LSB = 20.0 * current_LSB;
    
    // Calculate calibration value
    uint16_t cal_value = (uint16_t)((0.04096) / (current_LSB * shunt_resistance));
    
    writeRegister(INA219_REG_CALIBRATION, cal_value);
    
    Serial.println("INA219 calibrated - Shunt: " + String(shunt_ohms, 3) + 
                   "Ω, Max Current: " + String(max_current, 2) + "A");
}

void INA_Monitor::writeRegister(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(ina_address);
    Wire.write(reg);
    Wire.write((value >> 8) & 0xFF);
    Wire.write(value & 0xFF);
    Wire.endTransmission();
}

uint16_t INA_Monitor::readRegister(uint8_t reg) {
    Wire.beginTransmission(ina_address);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(ina_address, (uint8_t)2);
    uint16_t value = ((uint16_t)Wire.read() << 8) | Wire.read();
    return value;
}

int16_t INA_Monitor::readRegisterSigned(uint8_t reg) {
    return (int16_t)readRegister(reg);
}

float INA_Monitor::getRawBusVoltage() {
    int16_t value = readRegisterSigned(INA219_REG_BUSVOLTAGE);
    return (value >> 3) * 0.004; // 4mV per bit
}

float INA_Monitor::getRawCurrent() {
    int16_t value = readRegisterSigned(INA219_REG_CURRENT);
    return value * current_LSB * 1000.0; // Convert to mA
}

float INA_Monitor::getRawPower() {
    int16_t value = readRegisterSigned(INA219_REG_POWER);
    return value * power_LSB * 1000.0; // Convert to mW
}

float INA_Monitor::getBusVoltage() {
    float rawVoltage = getRawBusVoltage();
    float filteredVoltage = kalmanFilter(&voltageFilter, rawVoltage);
    
    // Auto-adjust filter based on error
    float error = abs(rawVoltage - filteredVoltage);
    autoAdjustKalmanFilter(&voltageFilter, error);
    
    return filteredVoltage;
}

float INA_Monitor::getShuntVoltage() {
    int16_t value = readRegisterSigned(INA219_REG_SHUNTVOLTAGE);
    return value * 0.00001; // 10uV per bit (no filtering for shunt voltage)
}

float INA_Monitor::getCurrent() {
    float rawCurrent = getRawCurrent();
    float filteredCurrent = kalmanFilter(&currentFilter, rawCurrent);
    
    // Auto-adjust filter based on error
    float error = abs(rawCurrent - filteredCurrent);
    autoAdjustKalmanFilter(&currentFilter, error);
    
    return filteredCurrent;
}

float INA_Monitor::getPower() {
    float rawPower = getRawPower();
    float filteredPower = kalmanFilter(&powerFilter, rawPower);
    
    // Auto-adjust filter based on error
    float error = abs(rawPower - filteredPower);
    autoAdjustKalmanFilter(&powerFilter, error);
    
    return filteredPower;
}

float INA_Monitor::getRawBusVoltageReading() {
    return getRawBusVoltage();
}

float INA_Monitor::getRawCurrentReading() {
    return getRawCurrent();
}

float INA_Monitor::getRawPowerReading() {
    return getRawPower();
}

void INA_Monitor::setBatteryCapacity(float capacity_mAh) {
    batteryCapacity = capacity_mAh;
    currentCapacity = capacity_mAh;
}

void INA_Monitor::setLowBatteryThreshold(float threshold_percent) {
    lowBatteryThreshold = threshold_percent;
}

void INA_Monitor::checkBatteryStatus() {
    if (millis() - lastBatteryCheck < batteryCheckInterval) {
        return;
    }
    
    float voltage = getBusVoltage();
    float current = getCurrent();
    
    // Calculate consumed capacity (coulomb counting)
    float timeElapsed = batteryCheckInterval / 1000.0 / 3600.0; // Convert to hours
    float consumedCapacity = current * timeElapsed; // mAh
    
    // Update current capacity
    currentCapacity -= consumedCapacity;
    
    // Ensure capacity doesn't go negative
    if (currentCapacity < 0) {
        currentCapacity = 0;
    }
    
    // Voltage-based estimation as backup
    float voltageBasedCapacity = estimateCapacityFromVoltage(voltage);
    
    // Use lower value for safety
    if (voltageBasedCapacity < currentCapacity) {
        currentCapacity = voltageBasedCapacity;
    }
    
    // Check for low battery warning
    if (getBatteryPercentage() <= lowBatteryThreshold && !lowBatteryWarning) {
        lowBatteryWarning = true;
        Serial.println("LOW BATTERY WARNING!");
    }
    
    lastBatteryCheck = millis();
}

float INA_Monitor::getBatteryPercentage() {
    return (currentCapacity / batteryCapacity) * 100.0;
}

bool INA_Monitor::isLowBattery() {
    return getBatteryPercentage() <= lowBatteryThreshold;
}

void INA_Monitor::printBatteryStatus() {
    Serial.println("=== BATTERY STATUS ===");
    Serial.println("Voltage: " + String(getBusVoltage(), 2) + " V");
    Serial.println("Current: " + String(getCurrent(), 2) + " mA");
    Serial.println("Power: " + String(getPower(), 2) + " mW");
    Serial.println("Capacity: " + String(currentCapacity, 0) + " / " + String(batteryCapacity, 0) + " mAh");
    Serial.println("Percentage: " + String(getBatteryPercentage(), 1) + " %");
    Serial.println("Status: " + String(isLowBattery() ? "LOW BATTERY" : "OK"));
    Serial.println("=====================");
}

void INA_Monitor::printDetailedStatus() {
    Serial.println("=== DETAILED STATUS ===");
    Serial.println("Raw Voltage: " + String(getRawBusVoltage(), 3) + " V");
    Serial.println("Filtered Voltage: " + String(getBusVoltage(), 3) + " V");
    Serial.println("Raw Current: " + String(getRawCurrent(), 2) + " mA");
    Serial.println("Filtered Current: " + String(getCurrent(), 2) + " mA");
    Serial.println("Raw Power: " + String(getRawPower(), 2) + " mW");
    Serial.println("Filtered Power: " + String(getPower(), 2) + " mW");
    Serial.println("Battery: " + String(getBatteryPercentage(), 1) + " %");
    printKalmanStatus();
    Serial.println("======================");
}

void INA_Monitor::resetBatteryCapacity() {
    currentCapacity = batteryCapacity;
    lowBatteryWarning = false;
    Serial.println("Battery capacity reset to full");
}

float INA_Monitor::estimateCapacityFromVoltage(float voltage) {
    // Li-Po 3S discharge curve (adjust for your battery type)
    if (voltage >= 12.6) return batteryCapacity * 1.0;      // 100%
    else if (voltage >= 12.0) return batteryCapacity * 0.8; // 80%
    else if (voltage >= 11.7) return batteryCapacity * 0.6; // 60%
    else if (voltage >= 11.4) return batteryCapacity * 0.4; // 40%
    else if (voltage >= 11.1) return batteryCapacity * 0.2; // 20%
    else if (voltage >= 10.8) return batteryCapacity * 0.1; // 10%
    else return 0.0; // 0%
}

void INA_Monitor::initKalmanFilter(KalmanFilter* kf, float Q, float R, float P, float initialValue) {
    kf->Q = Q;  // Process noise
    kf->R = R;  // Measurement noise
    kf->P = P;  // Initial estimation error
    kf->K = 0;  // Kalman gain
    kf->X = initialValue;  // Initial estimate
    kf->lastError = 0;
    kf->lastUpdate = millis();
}

float INA_Monitor::kalmanFilter(KalmanFilter* kf, float measurement) {
    unsigned long currentTime = millis();
    float dt = (currentTime - kf->lastUpdate) / 1000.0; // Time delta in seconds
    
    // Prediction step
    kf->P = kf->P + kf->Q * dt;
    
    // Update step
    kf->K = kf->P / (kf->P + kf->R);
    float error = measurement - kf->X;
    kf->X = kf->X + kf->K * error;
    kf->P = (1 - kf->K) * kf->P;
    
    kf->lastError = abs(error);
    kf->lastUpdate = currentTime;
    
    return kf->X;
}

void INA_Monitor::autoAdjustKalmanFilter(KalmanFilter* kf, float error) {
    // Auto-adjust parameters based on error magnitude and trend
    if (error > 5.0) {
        // High error - increase process noise, decrease measurement noise
        kf->Q = min(kf->Q * 1.05, 2.0);
        kf->R = max(kf->R * 0.95, 0.01);
    } else if (error < 1.0) {
        // Low error - decrease process noise, increase measurement noise
        kf->Q = max(kf->Q * 0.98, 0.001);
        kf->R = min(kf->R * 1.02, 10.0);
    }
    
    // Adaptive adjustment based on Kalman gain
    if (kf->K > 0.8) {
        // High gain - reduce measurement noise
        kf->R = max(kf->R * 0.9, 0.01);
    } else if (kf->K < 0.1) {
        // Low gain - increase process noise
        kf->Q = min(kf->Q * 1.1, 2.0);
    }
}

void INA_Monitor::resetKalmanFilters() {
    float initialVoltage = getRawBusVoltage();
    float initialCurrent = getRawCurrent();
    float initialPower = getRawPower();
    
    initKalmanFilter(&voltageFilter, 0.01, 0.1, 1.0, initialVoltage);
    initKalmanFilter(&currentFilter, 0.1, 1.0, 1.0, initialCurrent);
    initKalmanFilter(&powerFilter, 0.1, 2.0, 1.0, initialPower);
    
    Serial.println("Kalman filters reset");
}

void INA_Monitor::setKalmanParameters(float voltageQ, float voltageR, float currentQ, float currentR) {
    voltageFilter.Q = voltageQ;
    voltageFilter.R = voltageR;
    currentFilter.Q = currentQ;
    currentFilter.R = currentR;
    powerFilter.Q = currentQ;
    powerFilter.R = currentR * 2.0;
    
    Serial.println("Kalman parameters updated");
}

void INA_Monitor::printKalmanStatus() {
    Serial.println("--- KALMAN FILTER STATUS ---");
    Serial.println("Voltage Filter - Q: " + String(voltageFilter.Q, 4) + 
                   ", R: " + String(voltageFilter.R, 4) + 
                   ", K: " + String(voltageFilter.K, 4) + 
                   ", Error: " + String(voltageFilter.lastError, 3));
    Serial.println("Current Filter - Q: " + String(currentFilter.Q, 4) + 
                   ", R: " + String(currentFilter.R, 4) + 
                   ", K: " + String(currentFilter.K, 4) + 
                   ", Error: " + String(currentFilter.lastError, 3));
    Serial.println("Power Filter - Q: " + String(powerFilter.Q, 4) + 
                   ", R: " + String(powerFilter.R, 4) + 
                   ", K: " + String(powerFilter.K, 4) + 
                   ", Error: " + String(powerFilter.lastError, 3));
    Serial.println("---------------------------");
}
