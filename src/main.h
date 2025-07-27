#include <Arduino.h>
#include "System/Process.h"
#include "System/ArmSmoother.h"
#include "Sensor/InaMonitor.h"

// Instance smoother
ARM_SMOOTHER armSmoother;

// Inisialisasi di setup atau constructor


#define RXD1 21
#define TXD1 19
// #define RXD1 19
// #define TXD1 22
#define RXD2 16
#define TXD2 17

#define standup 1
#define walk 2
#define atplace 3
#define stop 4
#define  pinBuzzer 13

INSTRUCTION instruction;
INA_Monitor batteryMonitor;

// Global status variables (non-blocking)
bool systemHealthy = true;
bool thermalOK = true;
bool batteryOK = true;
String lastSystemStatus = "HIGH";

// Non-blocking timing variables dengan interval yang lebih optimal
unsigned long lastMonitoringCheck = 0;
unsigned long lastStatusPrint = 0;
unsigned long lastClassificationUpdate = 0;
unsigned long lastBuzzerUpdate = 0;
const unsigned long MONITORING_INTERVAL = 50;      // Lebih cepat: Check every 50ms
const unsigned long PRINT_INTERVAL = 10000;        // Lebih jarang: Print every 10 seconds
const unsigned long CLASSIFICATION_INTERVAL = 200; // Lebih cepat: Update classifications every 200ms
const unsigned long BUZZER_INTERVAL = 5;           // Update buzzer every 5ms untuk smoothness

// Buzzer state variables
struct BuzzerState
{
    int remainingBeeps = 0;
    unsigned long lastBeepTime = 0;
    bool buzzerOn = false;
    bool newBeepRequest = false;
    int requestedBeeps = 0;
};
BuzzerState buzzerState;

void nonBlockingBuzzer(int beepCount)
{
    // Handle new beep requests
    if (beepCount > 0)
    {
        buzzerState.requestedBeeps = beepCount;
        buzzerState.newBeepRequest = true;
    }

    // Process buzzer only when it's time
    if (millis() - lastBuzzerUpdate < BUZZER_INTERVAL)
    {
        return; // Too early, skip this cycle
    }
    lastBuzzerUpdate = millis();

    // Initialize new beep sequence
    if (buzzerState.newBeepRequest)
    {
        buzzerState.remainingBeeps = buzzerState.requestedBeeps * 2; // *2 for on/off cycles
        buzzerState.newBeepRequest = false;
        buzzerState.lastBeepTime = millis();
    }

    // Handle buzzer timing
    if (buzzerState.remainingBeeps > 0 && millis() - buzzerState.lastBeepTime >= 150)
    { // Faster beep
        buzzerState.buzzerOn = !buzzerState.buzzerOn;

        if (buzzerState.buzzerOn)
        {
            // Turn on buzzer - implement your buzzer logic here
            // digitalWrite(BUZZER_PIN, HIGH);
            Serial.print("♪");
        }
        else
        {
            // Turn off buzzer
            // digitalWrite(BUZZER_PIN, LOW);
        }

        buzzerState.remainingBeeps--;
        buzzerState.lastBeepTime = millis();

        if (buzzerState.remainingBeeps == 0)
        {
            Serial.print(" "); // Space after beep sequence
        }
    }
}

void handleCriticalConditions()
{
    static bool overheatingWarned = false;
    static bool lowBatteryWarned = false;
    static bool criticalBatteryWarned = false;
    static unsigned long lastWarningCheck = 0;

    // Limit warning checks to prevent spam
    if (millis() - lastWarningCheck < 1000)
        return; // Check warnings max once per second
    lastWarningCheck = millis();

    float batteryPercentage = batteryMonitor.getBatteryPercentage();
    float temperature = batteryMonitor.getTemperature();

    // Critical battery warning (< 5%)
    if (batteryPercentage < 5.0 && !criticalBatteryWarned)
    {
        Serial.println("\n*** CRITICAL BATTERY: " + String(batteryPercentage, 1) + "% ***");
        nonBlockingBuzzer(10); // 10 beeps warning
        criticalBatteryWarned = true;
    }
    else if (batteryPercentage > 10.0)
    {
        criticalBatteryWarned = false; // Reset warning
    }

    // Low battery warning (< 20%)
    if (!batteryOK && !lowBatteryWarned)
    {
        Serial.println("\n** LOW BATTERY: " + String(batteryPercentage, 1) + "% - " +
                       batteryMonitor.getBatteryHealthString() + " **");
        nonBlockingBuzzer(3); // 3 beeps warning
        lowBatteryWarned = true;
    }
    else if (batteryOK)
    {
        lowBatteryWarned = false; // Reset warning
    }

    // Overheating warning
    if (!thermalOK && !overheatingWarned)
    {
        Serial.println("\n** OVERHEATING: " + String(temperature, 1) + "°C - " +
                       batteryMonitor.getTemperatureClassString() + " **");
        nonBlockingBuzzer(5); // 5 beeps warning
        overheatingWarned = true;
    }
    else if (thermalOK)
    {
        overheatingWarned = false; // Reset warning
    }
}

void backgroundMonitoring()
{
    unsigned long currentTime = millis();

    // Ultra-fast monitoring check (every 50ms)
    if (currentTime - lastMonitoringCheck >= MONITORING_INTERVAL)
    {
        // Quick battery and temperature check (should be < 5ms)
        batteryMonitor.checkBatteryStatus();

        // Update system health flags
        batteryOK = !batteryMonitor.isLowBattery() && (batteryMonitor.getBatteryPercentage() > 10.0);
        thermalOK = !batteryMonitor.isOverheating();
        systemHealthy = batteryOK && thermalOK;

        lastMonitoringCheck = currentTime;
    }

    // Medium frequency classification update (every 200ms)
    if (currentTime - lastClassificationUpdate >= CLASSIFICATION_INTERVAL)
    {
        // This should be optimized to be very fast
        batteryMonitor.updateClassifications();
        lastSystemStatus = batteryMonitor.getOverallSystemStatus();
        lastClassificationUpdate = currentTime;
    }

    // Low frequency status printing (every 10 seconds)
    if (currentTime - lastStatusPrint >= PRINT_INTERVAL)
    {
        // Compact status print untuk mengurangi Serial overhead
        Serial.println("\n=== STATUS: " + lastSystemStatus +
                       " | BAT:" + String(batteryMonitor.getBatteryPercentage(), 0) + "%" +
                       " | TEMP:" + String(batteryMonitor.getTemperature(), 0) + "°C" +
                       " | " + String(systemHealthy ? "OK" : "ALERT") + " ===");
        lastStatusPrint = currentTime;
    }

    // Handle critical conditions (non-blocking warnings only)
    handleCriticalConditions();
}


