#include <Arduino.h>
#include "System/Process.h"
#include "System/ArmSmoother.h"
#include "Sensor/InaMonitor.h"

// Instance smoother
ARM_SMOOTHER armSmoother;

#define RXD1 21
#define TXD1 19
#define RXD2 16
#define TXD2 17

#define standup 1
#define walk 2
#define atplace 3
#define stop 4
#define pinBuzzer 13

INSTRUCTION instruction;
INA_Monitor batteryMonitor;

// Global status variables (non-blocking)
bool systemHealthy = true;
bool thermalOK = true;
bool batteryOK = true;
String lastSystemStatus = "HIGH";

// Emergency flags untuk komunikasi dengan Process.cpp
bool emergencyStopRequested = false;
bool criticalShutdownRequested = false;
int requestedMotionType = atplace;
bool motionChangeRequested = false;

// Non-blocking timing variables - optimized intervals
unsigned long lastMonitoringCheck = 0;
unsigned long lastStatusPrint = 0;
unsigned long lastClassificationUpdate = 0;
unsigned long lastBuzzerUpdate = 0;
unsigned long lastEmergencyCheck = 0;

// Interval timing yang tidak akan conflict dengan execute() loop
const unsigned long MONITORING_INTERVAL = 47;      // Prime number untuk avoid sync
const unsigned long PRINT_INTERVAL = 8000;         // Tidak kelipatan dari execute timing
const unsigned long CLASSIFICATION_INTERVAL = 193; // Prime number
const unsigned long BUZZER_INTERVAL = 7;           // Fast response
const unsigned long EMERGENCY_CHECK_INTERVAL = 83; // Prime number

// Buzzer state variables
struct BuzzerState {
    int remainingBeeps = 0;
    unsigned long lastBeepTime = 0;
    bool buzzerOn = false;
    bool newBeepRequest = false;
    int requestedBeeps = 0;
} buzzerState;

// Execution state tracking
struct ExecutionState {
    bool isExecuting = false;
    unsigned long executionStartTime = 0;
    int currentMotion = atplace;
    bool executionCompleted = true;
} execState;

void nonBlockingBuzzer(int beepCount) {
    // Handle new beep requests
    if (beepCount > 0) {
        buzzerState.requestedBeeps = beepCount;
        buzzerState.newBeepRequest = true;
    }

    // Process buzzer only when it's time
    unsigned long currentTime = millis();
    if (currentTime - lastBuzzerUpdate < BUZZER_INTERVAL) {
        return; // Too early, skip this cycle
    }
    lastBuzzerUpdate = currentTime;

    // Initialize new beep sequence
    if (buzzerState.newBeepRequest) {
        buzzerState.remainingBeeps = buzzerState.requestedBeeps * 2;
        buzzerState.newBeepRequest = false;
        buzzerState.lastBeepTime = currentTime;
    }

    // Handle buzzer timing
    if (buzzerState.remainingBeeps > 0 && (currentTime - buzzerState.lastBeepTime) >= 120) {
        buzzerState.buzzerOn = !buzzerState.buzzerOn;

        if (buzzerState.buzzerOn) {
            digitalWrite(pinBuzzer, HIGH);
            Serial.print("♪");
        } else {
            digitalWrite(pinBuzzer, LOW);
        }

        buzzerState.remainingBeeps--;
        buzzerState.lastBeepTime = currentTime;

        if (buzzerState.remainingBeeps == 0) {
            Serial.print(" ");
        }
    }
}

void handleCriticalConditions() {
    static bool overheatingWarned = false;
    static bool lowBatteryWarned = false;
    static bool criticalBatteryWarned = false;
    
    unsigned long currentTime = millis();
    
    // Throttle warning checks dengan interval yang tidak sync dengan execute
    if (currentTime - lastEmergencyCheck < EMERGENCY_CHECK_INTERVAL) {
        return;
    }
    lastEmergencyCheck = currentTime;

    float batteryPercentage = batteryMonitor.getBatteryPercentage();
    float temperature = batteryMonitor.getTemperature();

    // Critical battery warning (< 5%) - SET FLAGS ONLY
    if (batteryPercentage < 5.0 && !criticalBatteryWarned) {
        Serial.println("\n*** CRITICAL BATTERY: " + String(batteryPercentage, 1) + "% ***");
        
        // Set flags untuk komunikasi dengan execute loop
        criticalShutdownRequested = true;
        requestedMotionType = stop;
        motionChangeRequested = true;
        
        nonBlockingBuzzer(10);
        criticalBatteryWarned = true;
    } else if (batteryPercentage > 10.0) {
        criticalBatteryWarned = false;
        criticalShutdownRequested = false;
    }

    // Low battery warning (< 20%) - SET FLAGS ONLY
    if (!batteryOK && !lowBatteryWarned) {
        Serial.println("\n** LOW BATTERY: " + String(batteryPercentage, 1) + "% - " +
                       batteryMonitor.getBatteryHealthString() + " **");
        
        // Set flags untuk komunikasi dengan execute loop
        emergencyStopRequested = true;
        requestedMotionType = stop;
        motionChangeRequested = true;
        
        nonBlockingBuzzer(3);
        lowBatteryWarned = true;
    } else if (batteryOK && batteryPercentage > 25.0) {
        lowBatteryWarned = false;
        emergencyStopRequested = false;
    }

    // Overheating warning - SET FLAGS ONLY
    if (!thermalOK && !overheatingWarned) {
        Serial.println("\n** OVERHEATING: " + String(temperature, 1) + "°C - " +
                       batteryMonitor.getTemperatureClassString() + " **");
        
        // Set flags untuk komunikasi dengan execute loop
        emergencyStopRequested = true;
        requestedMotionType = stop;
        motionChangeRequested = true;
        
        nonBlockingBuzzer(5);
        overheatingWarned = true;
    } else if (thermalOK && temperature < 50.0) {
        overheatingWarned = false;
        if (batteryOK) {
            emergencyStopRequested = false;
        }
    }
}

void backgroundMonitoring() {
    unsigned long currentTime = millis();

    // FAST monitoring check - offset timing untuk avoid conflict
    if (currentTime - lastMonitoringCheck >= MONITORING_INTERVAL) {
        // HANYA jika execute tidak sedang berjalan atau di safe interval
        if (!execState.isExecuting || (currentTime - execState.executionStartTime) % 50 < 5) {
            
            // Quick battery and temperature check (< 3ms execution time)
            batteryMonitor.checkBatteryStatus();

            // Update system health flags
            batteryOK = !batteryMonitor.isLowBattery() && (batteryMonitor.getBatteryPercentage() > 10.0);
            thermalOK = !batteryMonitor.isOverheating();
            systemHealthy = batteryOK && thermalOK;

            lastMonitoringCheck = currentTime;
        }
    }

    // MEDIUM frequency classification update - different timing
    if (currentTime - lastClassificationUpdate >= CLASSIFICATION_INTERVAL) {
        // HANYA update saat execute tidak di critical timing
        if (!execState.isExecuting || (currentTime - execState.executionStartTime) % 100 < 10) {
            
            // Fast classification update
            batteryMonitor.updateClassifications();
            lastSystemStatus = batteryMonitor.getOverallSystemStatus();
            lastClassificationUpdate = currentTime;
        }
    }

    // LOW frequency status printing
    if (currentTime - lastStatusPrint >= PRINT_INTERVAL) {
        Serial.println("\n=== STATUS: " + lastSystemStatus +
                       " | BAT:" + String(batteryMonitor.getBatteryPercentage(), 0) + "%" +
                       " | TEMP:" + String(batteryMonitor.getTemperature(), 0) + "°C" +
                       " | MOTION:" + String(execState.currentMotion) +
                       " | " + String(systemHealthy ? "OK" : "ALERT") + " ===");
        lastStatusPrint = currentTime;
    }

    // Handle critical conditions - independent timing
    handleCriticalConditions();
}

// Track execution state untuk avoid conflicts
void updateExecutionState(bool executing, int motionType = atplace) {
    execState.isExecuting = executing;
    execState.currentMotion = motionType;
    
    if (executing && !execState.executionCompleted) {
        execState.executionStartTime = millis();
        execState.executionCompleted = false;
    } else if (!executing) {
        execState.executionCompleted = true;
    }
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(1000000, SERIAL_8N1, RXD1, TXD1);
    delay(2000);

    Serial2.begin(1000000, SERIAL_8N1, RXD2, TXD2);

    pinMode(33, OUTPUT);
    digitalWrite(33, HIGH);
    
    pinMode(pinBuzzer, OUTPUT);
    digitalWrite(pinBuzzer, LOW);

    // Initialize battery monitor
    if (batteryMonitor.init()) {
        Serial.println("INA219 + DS18B20 initialized successfully");
        batteryMonitor.setBatteryCapacity(2200.0);
        batteryMonitor.setLowBatteryThreshold(20.0);
        batteryMonitor.setOverheatThreshold(55.0);
    } else {
        Serial.println("Failed to initialize sensors - continuing without monitoring");
    }

    armSmoother.init(0.7f, 30.0f, 80.0f);
    instruction.syncsendsservo();
    
    // Initial standup dengan tracking
    updateExecutionState(true, standup);
    instruction.execute(standup);
    updateExecutionState(false);

    Serial.println("System started - background monitoring active");
}

void loop() {
    // Background monitoring SEBELUM execute (safe timing)
    backgroundMonitoring();
    
    // Handle non-blocking buzzer
    nonBlockingBuzzer(0);
    
    // Handle motion change requests
    if (motionChangeRequested) {
        execState.currentMotion = requestedMotionType;
        motionChangeRequested = false;
        
        Serial.println(">>> MOTION CHANGE TO: " + String(requestedMotionType) + " <<<");
    }
    
    // Execute dengan state tracking
    updateExecutionState(true, execState.currentMotion);
    instruction.execute(execState.currentMotion);
    updateExecutionState(false);
    
    // Auto-recovery logic
    if (systemHealthy && execState.currentMotion == stop && 
        !emergencyStopRequested && !criticalShutdownRequested) {
        
        Serial.println(">>> SYSTEM RECOVERED - RESUMING NORMAL OPERATION <<<");
        execState.currentMotion = atplace;
    }
    
    // Critical shutdown sequence
    if (criticalShutdownRequested && batteryMonitor.getBatteryPercentage() < 3.0) {
        Serial.println(">>> CRITICAL SHUTDOWN INITIATED <<<");
        
        updateExecutionState(true, stop);
        instruction.execute(stop);
        updateExecutionState(false);
        
        delay(1000);
        Serial.println(">>> SYSTEM HALTED - MANUAL RESTART REQUIRED <<<");
        
        while(1) {
            digitalWrite(pinBuzzer, HIGH);
            delay(300);
            digitalWrite(pinBuzzer, LOW);
            delay(300);
            Serial.println("SYSTEM HALTED - BATTERY CRITICAL");
        }
    }
    
    // Small gap untuk background processing
    delay(1);
}
