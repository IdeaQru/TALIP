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
    
    // Temperature settings
    tempSensorPin = TEMP_SENSOR_PIN;
    currentTemperature = 25.0;
    overheatThreshold = 55.0;
    
    // Classifications
    batteryHealth = HEALTH_HIGH;
    temperatureClass = TEMP_NORMAL;
    systemPerformance = PERFORMANCE_OPTIMAL;
    
    nodeCount = 0;
    
    // Initialize Kalman filters
    initKalmanFilter(&voltageFilter, 0.01, 0.1, 1.0, 12.0);
    initKalmanFilter(&currentFilter, 0.1, 1.0, 1.0, 0.0);
    initKalmanFilter(&powerFilter, 0.1, 2.0, 1.0, 0.0);
    initKalmanFilter(&temperatureFilter, 0.05, 0.5, 1.0, 25.0);
}

bool INA_Monitor::init() {
    Wire.begin();
    tempSensorPin = TEMP_SENSOR_PIN;
    // Initialize temperature sensor pin
    pinMode(tempSensorPin, INPUT);
    Serial.println("DS18B20 Temperature sensor initialized (No Library)");
    
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
    
    // Initialize decision tree
    initializeDecisionTree();
    
    // Reset Kalman filters with initial readings
    delay(100);
    float initialVoltage = getRawBusVoltage();
    float initialCurrent = getRawCurrent();
    float initialPower = getRawPower();
    float initialTemp = getRawTemperature();
    
    voltageFilter.X = initialVoltage;
    currentFilter.X = initialCurrent;
    powerFilter.X = initialPower;
    temperatureFilter.X = initialTemp;
    
    Serial.println("INA219 with DS18B20 (No Library) initialized");
    Serial.println("Initial readings - V: " + String(initialVoltage, 2) + 
                   "V, I: " + String(initialCurrent, 2) + 
                   "mA, P: " + String(initialPower, 2) + 
                   "mW, T: " + String(initialTemp, 1) + "°C");
    
    return true;
}

// ========== DS18B20 RAW PROTOCOL IMPLEMENTATION (NO LIBRARY) ==========

bool INA_Monitor::oneWireReset() {
    pinMode(tempSensorPin, OUTPUT);
    digitalWrite(tempSensorPin, LOW);
    delayMicroseconds(480);  // Reset pulse
    
    pinMode(tempSensorPin, INPUT);
    delayMicroseconds(70);   // Wait for presence pulse
    
    bool presence = !digitalRead(tempSensorPin);
    delayMicroseconds(410);  // Complete the reset sequence
    
    return presence;
}

void INA_Monitor::oneWireWriteBit(bool bit) {
    pinMode(tempSensorPin, OUTPUT);
    digitalWrite(tempSensorPin, LOW);
    
    if (bit) {
        delayMicroseconds(10);  // Write 1: short low pulse
        pinMode(tempSensorPin, INPUT);
        delayMicroseconds(55);
    } else {
        delayMicroseconds(65);  // Write 0: long low pulse
        pinMode(tempSensorPin, INPUT);
    }
}

bool INA_Monitor::oneWireReadBit() {
    pinMode(tempSensorPin, OUTPUT);
    digitalWrite(tempSensorPin, LOW);
    delayMicroseconds(3);
    
    pinMode(tempSensorPin, INPUT);
    delayMicroseconds(10);
    
    bool bit = digitalRead(tempSensorPin);
    delayMicroseconds(53);
    
    return bit;
}

void INA_Monitor::oneWireWriteByte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        oneWireWriteBit(data & 0x01);
        data >>= 1;
    }
}

uint8_t INA_Monitor::oneWireReadByte() {
    uint8_t data = 0;
    
    for (int i = 0; i < 8; i++) {
        if (oneWireReadBit()) {
            data |= (1 << i);
        }
    }
    
    return data;
}

bool INA_Monitor::ds18b20StartConversion() {
    if (!oneWireReset()) {
        return false;  // No device present
    }
    
    oneWireWriteByte(DS18B20_SKIP_ROM);    // Skip ROM command
    oneWireWriteByte(DS18B20_CONVERT_T);   // Start temperature conversion
    
    return true;
}

bool INA_Monitor::ds18b20ReadTemperature(float* temperature) {
    uint8_t data[9];
    
    if (!oneWireReset()) {
        return false;  // No device present
    }
    
    oneWireWriteByte(DS18B20_SKIP_ROM);        // Skip ROM command
    oneWireWriteByte(DS18B20_READ_SCRATCHPAD); // Read scratchpad
    
    // Read 9 bytes of data
    for (int i = 0; i < 9; i++) {
        data[i] = oneWireReadByte();
    }
    
    // Calculate temperature from raw data
    int16_t raw = (data[1] << 8) | data[0];
    
    // Handle different resolution settings
    uint8_t cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw & ~7;      // 9 bit resolution
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit resolution  
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit resolution
    // 12 bit resolution requires no masking
    
    *temperature = (float)raw / 16.0;
    
    return true;
}

float INA_Monitor::getRawTemperature() {
    static unsigned long lastConversion = 0;
    static bool conversionStarted = false;
    float temperature = 25.0; // Default temperature
    
    unsigned long currentTime = millis();
    
    if (!conversionStarted || (currentTime - lastConversion > 1000)) {
        // Start new conversion
        if (ds18b20StartConversion()) {
            conversionStarted = true;
            lastConversion = currentTime;
        }
    }
    
    // Wait at least 750ms for 12-bit conversion
    if (conversionStarted && (currentTime - lastConversion > 750)) {
        if (ds18b20ReadTemperature(&temperature)) {
            conversionStarted = false; // Ready for next conversion
        }
    }
    
    return temperature;
}

// ========== STANDARD METHODS (SAMA SEPERTI SEBELUMNYA) ==========

void INA_Monitor::calibrate(float shunt_ohms, float max_current) {
    shunt_resistance = shunt_ohms;
    
    current_LSB = max_current / 32767.0;
    power_LSB = 20.0 * current_LSB;
    
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
    return (value >> 3) * 0.004;
}

float INA_Monitor::getRawCurrent() {
    int16_t value = readRegisterSigned(INA219_REG_CURRENT);
    return value * current_LSB * 1000.0;
}

float INA_Monitor::getRawPower() {
    int16_t value = readRegisterSigned(INA219_REG_POWER);
    return value * power_LSB * 1000.0;
}

float INA_Monitor::getBusVoltage() {
    float rawVoltage = getRawBusVoltage();
    float filteredVoltage = kalmanFilter(&voltageFilter, rawVoltage);
    
    float error = abs(rawVoltage - filteredVoltage);
    autoAdjustKalmanFilter(&voltageFilter, error);
    
    return filteredVoltage;
}

float INA_Monitor::getShuntVoltage() {
    int16_t value = readRegisterSigned(INA219_REG_SHUNTVOLTAGE);
    return value * 0.00001;
}

float INA_Monitor::getCurrent() {
    float rawCurrent = getRawCurrent();
    float filteredCurrent = kalmanFilter(&currentFilter, rawCurrent);
    
    float error = abs(rawCurrent - filteredCurrent);
    autoAdjustKalmanFilter(&currentFilter, error);
    
    return filteredCurrent;
}

float INA_Monitor::getPower() {
    float rawPower = getRawPower();
    float filteredPower = kalmanFilter(&powerFilter, rawPower);
    
    float error = abs(rawPower - filteredPower);
    autoAdjustKalmanFilter(&powerFilter, error);
    
    return filteredPower;
}

float INA_Monitor::getTemperature() {
    float rawTemp = getRawTemperature();
    float filteredTemp = kalmanFilter(&temperatureFilter, rawTemp);
    
    float error = abs(rawTemp - filteredTemp);
    autoAdjustKalmanFilter(&temperatureFilter, error);
    
    currentTemperature = filteredTemp;
    return filteredTemp;
}

void INA_Monitor::initializeDecisionTree() {
    nodeCount = 0;
    
    // Root node: Check temperature first (safety priority)
    decisionTree[0] = {"temperature", overheatThreshold, ">=", 1, 2, "", false};
    
    // Left branch: Overheating detected
    decisionTree[1] = {"battery_percent", 30.0, ">=", 3, 4, "", false};
    
    // Right branch: Normal temperature
    decisionTree[2] = {"battery_percent", 70.0, ">=", 5, 6, "", false};
    
    // Overheating && Battery >= 30%
    decisionTree[3] = {"power", 1500.0, "<", -1, -1, "MEDIUM", true};
    
    // Overheating && Battery < 30%
    decisionTree[4] = {"voltage", 11.0, ">=", -1, -1, "LOW", true};
    
    // Normal temp && Battery >= 70%
    decisionTree[5] = {"power", 800.0, "<", 7, 8, "", false};
    
    // Normal temp && Battery < 70%
    decisionTree[6] = {"battery_percent", 40.0, ">=", 9, 10, "", false};
    
    // High battery, normal temp, low power
    decisionTree[7] = {"current", 300.0, "<", -1, -1, "HIGH", true};
    
    // High battery, normal temp, high power
    decisionTree[8] = {"voltage", 12.0, ">=", -1, -1, "MEDIUM", true};
    
    // Medium battery, normal temp
    decisionTree[9] = {"power", 1200.0, "<", -1, -1, "MEDIUM", true};
    
    // Low battery, normal temp
    decisionTree[10] = {"voltage", 11.2, ">=", -1, -1, "LOW", true};
    
    nodeCount = 11;
    
    Serial.println("Decision Tree initialized with " + String(nodeCount) + " nodes");
}

BatteryHealthClass INA_Monitor::classifyBatteryHealth(float voltage, float percentage, float temperature) {
    if (percentage >= 60 && voltage >= 11.8 && temperature < overheatThreshold) {
        return HEALTH_HIGH;
    }
    else if (percentage >= 30 && voltage >= 11.2) {
        return HEALTH_MEDIUM;
    }
    else {
        return HEALTH_LOW;
    }
}

TemperatureClass INA_Monitor::classifyTemperature(float temperature) {
    if (temperature >= overheatThreshold) {
        return TEMP_OVERHEAT;
    } else {
        return TEMP_NORMAL;
    }
}

PerformanceClass INA_Monitor::classifySystemPerformance(float power, float efficiency, float temperature) {
    if (power < 1000 && efficiency > 85 && temperature < overheatThreshold) {
        return PERFORMANCE_OPTIMAL;
    } else if (power < 2000 && efficiency > 70 && temperature < overheatThreshold + 10) {
        return PERFORMANCE_GOOD;
    } else if (power < 3000 && temperature < overheatThreshold + 20) {
        return PERFORMANCE_DEGRADED;
    } else {
        return PERFORMANCE_CRITICAL;
    }
}

void INA_Monitor::updateClassifications() {
    float voltage = getBusVoltage();
    float current = getCurrent();
    float power = getPower();
    float temperature = getTemperature();
    float batteryPercent = getBatteryPercentage();
    
    batteryHealth = classifyBatteryHealth(voltage, batteryPercent, temperature);
    temperatureClass = classifyTemperature(temperature);
    systemPerformance = classifySystemPerformance(power, getPowerEfficiency(), temperature);
    
    String overallStatus = getOverallSystemStatus();
    
    Serial.println("Classifications updated - Overall: " + overallStatus);
}

String INA_Monitor::getOverallSystemStatus() {
    float voltage = getBusVoltage();
    float current = getCurrent();
    float power = getPower();
    float temperature = getTemperature();
    float batteryPercent = getBatteryPercentage();
    
    return evaluateNode(0, voltage, current, power, temperature, batteryPercent);
}

String INA_Monitor::evaluateNode(int nodeIndex, float voltage, float current, float power, float temperature, float batteryPercent) {
    if (nodeIndex < 0 || nodeIndex >= nodeCount) return "ERROR";
    
    DecisionNode& node = decisionTree[nodeIndex];
    
    if (node.isLeaf) {
        return node.classification;
    }
    
    bool condition = evaluateCondition(node.parameter, node.condition, node.threshold, 
                                     voltage, current, power, temperature, batteryPercent);
    
    if (condition) {
        return evaluateNode(node.leftChild, voltage, current, power, temperature, batteryPercent);
    } else {
        return evaluateNode(node.rightChild, voltage, current, power, temperature, batteryPercent);
    }
}

bool INA_Monitor::evaluateCondition(String parameter, String condition, float threshold, 
                                   float voltage, float current, float power, float temperature, float batteryPercent) {
    float value = 0.0;
    
    if (parameter == "voltage") value = voltage;
    else if (parameter == "current") value = current;
    else if (parameter == "power") value = power;
    else if (parameter == "temperature") value = temperature;
    else if (parameter == "battery_percent") value = batteryPercent;
    
    if (condition == ">=") return value >= threshold;
    else if (condition == "<") return value < threshold;
    else if (condition == ">") return value > threshold;
    else if (condition == "<=") return value <= threshold;
    else if (condition == "==") return abs(value - threshold) < 0.1;
    
    return false;
}

void INA_Monitor::setOverheatThreshold(float threshold) {
    overheatThreshold = threshold;
    Serial.println("Overheat threshold set to: " + String(threshold, 1) + "°C");
}

bool INA_Monitor::isOverheating() {
    return getTemperature() >= overheatThreshold;
}

String INA_Monitor::getBatteryHealthString() {
    switch(batteryHealth) {
        case HEALTH_HIGH: return "HIGH";
        case HEALTH_MEDIUM: return "MEDIUM";
        case HEALTH_LOW: return "LOW";
        default: return "UNKNOWN";
    }
}

String INA_Monitor::getTemperatureClassString() {
    switch(temperatureClass) {
        case TEMP_NORMAL: return "NORMAL";
        case TEMP_OVERHEAT: return "OVERHEAT";
        default: return "UNKNOWN";
    }
}

BatteryHealthClass INA_Monitor::getBatteryHealthClass() {
    return batteryHealth;
}

TemperatureClass INA_Monitor::getTemperatureClass() {
    return temperatureClass;
}

PerformanceClass INA_Monitor::getPerformanceClass() {
    return systemPerformance;
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
    
    float timeElapsed = batteryCheckInterval / 1000.0 / 3600.0;
    float consumedCapacity = current * timeElapsed;
    
    currentCapacity -= consumedCapacity;
    
    if (currentCapacity < 0) {
        currentCapacity = 0;
    }
    
    float voltageBasedCapacity = estimateCapacityFromVoltage(voltage);
    
    if (voltageBasedCapacity < currentCapacity) {
        currentCapacity = voltageBasedCapacity;
    }
    
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

float INA_Monitor::getPowerEfficiency() {
    float power = getPower();
    float voltage = getBusVoltage();
    float current = getCurrent();
    
    if (current > 0) {
        return (power / (voltage * current)) * 100.0;
    }
    return 100.0;
}

void INA_Monitor::printBatteryStatus() {
    Serial.println("=== BATTERY STATUS ===");
    Serial.println("Voltage: " + String(getBusVoltage(), 2) + " V");
    Serial.println("Current: " + String(getCurrent(), 2) + " mA");
    Serial.println("Power: " + String(getPower(), 2) + " mW");
    Serial.println("Capacity: " + String(currentCapacity, 0) + " / " + String(batteryCapacity, 0) + " mAh");
    Serial.println("Percentage: " + String(getBatteryPercentage(), 1) + " %");
    Serial.println("Health Class: " + getBatteryHealthString());
    Serial.println("Status: " + String(isLowBattery() ? "LOW BATTERY" : "OK"));
    Serial.println("=====================");
}

void INA_Monitor::printTemperatureStatus() {
    Serial.println("=== TEMPERATURE STATUS ===");
    Serial.println("Current Temperature: " + String(getTemperature(), 1) + " °C");
    Serial.println("Raw Temperature: " + String(getRawTemperature(), 1) + " °C");
    Serial.println("Overheat Threshold: " + String(overheatThreshold, 1) + " °C");
    Serial.println("Temperature Class: " + getTemperatureClassString());
    Serial.println("Status: " + String(isOverheating() ? "OVERHEATING" : "NORMAL"));
    Serial.println("========================");
}

void INA_Monitor::printClassificationResults() {
    Serial.println("=== CLASSIFICATION RESULTS ===");
    Serial.println("Battery Health: " + getBatteryHealthString());
    Serial.println("Temperature: " + getTemperatureClassString());
    Serial.println("Overall System: " + getOverallSystemStatus());
    Serial.println("Power Efficiency: " + String(getPowerEfficiency(), 1) + "%");
    Serial.println("==============================");
}

void INA_Monitor::printDetailedStatus() {
    Serial.println("=== DETAILED STATUS ===");
    Serial.println("Raw Voltage: " + String(getRawBusVoltage(), 3) + " V");
    Serial.println("Filtered Voltage: " + String(getBusVoltage(), 3) + " V");
    Serial.println("Raw Current: " + String(getRawCurrent(), 2) + " mA");
    Serial.println("Filtered Current: " + String(getCurrent(), 2) + " mA");
    Serial.println("Raw Power: " + String(getRawPower(), 2) + " mW");
    Serial.println("Filtered Power: " + String(getPower(), 2) + " mW");
    Serial.println("Raw Temperature: " + String(getRawTemperature(), 1) + " °C");
    Serial.println("Filtered Temperature: " + String(getTemperature(), 1) + " °C");
    Serial.println("Battery: " + String(getBatteryPercentage(), 1) + " %");
    printClassificationResults();
    Serial.println("======================");
}

void INA_Monitor::resetBatteryCapacity() {
    currentCapacity = batteryCapacity;
    lowBatteryWarning = false;
    Serial.println("Battery capacity reset to full");
}

float INA_Monitor::estimateCapacityFromVoltage(float voltage) {
    if (voltage >= 12.6) return batteryCapacity * 1.0;
    else if (voltage >= 12.0) return batteryCapacity * 0.8;
    else if (voltage >= 11.7) return batteryCapacity * 0.6;
    else if (voltage >= 11.4) return batteryCapacity * 0.4;
    else if (voltage >= 11.1) return batteryCapacity * 0.2;
    else if (voltage >= 10.8) return batteryCapacity * 0.1;
    else return 0.0;
}

void INA_Monitor::initKalmanFilter(KalmanFilter* kf, float Q, float R, float P, float initialValue) {
    kf->Q = Q;
    kf->R = R;
    kf->P = P;
    kf->K = 0;
    kf->X = initialValue;
    kf->lastError = 0;
    kf->lastUpdate = millis();
}

float INA_Monitor::kalmanFilter(KalmanFilter* kf, float measurement) {
    unsigned long currentTime = millis();
    float dt = (currentTime - kf->lastUpdate) / 1000.0;
    
    kf->P = kf->P + kf->Q * dt;
    
    kf->K = kf->P / (kf->P + kf->R);
    float error = measurement - kf->X;
    kf->X = kf->X + kf->K * error;
    kf->P = (1 - kf->K) * kf->P;
    
    kf->lastError = abs(error);
    kf->lastUpdate = currentTime;
    
    return kf->X;
}

void INA_Monitor::autoAdjustKalmanFilter(KalmanFilter* kf, float error) {
    if (error > 5.0) {
        kf->Q = min(kf->Q * 1.05, 2.0);
        kf->R = max(kf->R * 0.95, 0.01);
    } else if (error < 1.0) {
        kf->Q = max(kf->Q * 0.98, 0.001);
        kf->R = min(kf->R * 1.02, 10.0);
    }
    
    if (kf->K > 0.8) {
        kf->R = max(kf->R * 0.9, 0.01);
    } else if (kf->K < 0.1) {
        kf->Q = min(kf->Q * 1.1, 2.0);
    }
}

void INA_Monitor::resetKalmanFilters() {
    float initialVoltage = getRawBusVoltage();
    float initialCurrent = getRawCurrent();
    float initialPower = getRawPower();
    float initialTemp = getRawTemperature();
    
    initKalmanFilter(&voltageFilter, 0.01, 0.1, 1.0, initialVoltage);
    initKalmanFilter(&currentFilter, 0.1, 1.0, 1.0, initialCurrent);
    initKalmanFilter(&powerFilter, 0.1, 2.0, 1.0, initialPower);
    initKalmanFilter(&temperatureFilter, 0.05, 0.5, 1.0, initialTemp);
    
    Serial.println("Kalman filters reset");
}

void INA_Monitor::setKalmanParameters(float voltageQ, float voltageR, float currentQ, float currentR) {
    voltageFilter.Q = voltageQ;
    voltageFilter.R = voltageR;
    currentFilter.Q = currentQ;
    currentFilter.R = currentR;
    powerFilter.Q = currentQ;
    powerFilter.R = currentR * 2.0;
    temperatureFilter.Q = voltageQ * 5;
    temperatureFilter.R = voltageR * 5;
    
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
    Serial.println("Temperature Filter - Q: " + String(temperatureFilter.Q, 4) + 
                   ", R: " + String(temperatureFilter.R, 4) + 
                   ", K: " + String(temperatureFilter.K, 4) + 
                   ", Error: " + String(temperatureFilter.lastError, 3));
    Serial.println("---------------------------");
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
