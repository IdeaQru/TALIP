#include "main.h"
#include "Sensor/InaMonitor.h"

// Create INA monitor instance
INA_Monitor batteryMonitor;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, RXD1, TXD1);
  delay(2000);

  Serial2.begin(1000000, SERIAL_8N1, RXD2, TXD2);

  pinMode(33, OUTPUT);
  digitalWrite(33, HIGH); // TRIGGER DUPLEX
  
  // Initialize INA monitor
  if (batteryMonitor.init()) {
    Serial.println("INA219 initialized successfully");
    batteryMonitor.setBatteryCapacity(2200.0); // Set your battery capacity in mAh
    batteryMonitor.setLowBatteryThreshold(20.0); // 20% threshold
  } else {
    Serial.println("Failed to initialize INA219");
  }
  
  armSmoother.init(0.7f, 30.0f, 80.0f);
  instruction.syncsendsservo();

  instruction.execute(standup);
}

void loop()
{
  // Check battery status
  batteryMonitor.checkBatteryStatus();
  
  // Print battery status every 5 seconds
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 5000) {
    batteryMonitor.printBatteryStatus();
    lastPrint = millis();
  }
  
  // Execute motion based on battery level
  if (!batteryMonitor.isLowBattery()) {
    // Battery OK, continue normal operation
    instruction.execute(atplace);
  } else {
    // Low battery, execute stop
    Serial.println("Low battery detected, executing stop...");
    instruction.execute(stop);
    loopingBuzzer();
    // Critical battery check
    if (batteryMonitor.getBatteryPercentage() < 60.0) {
      Serial.println("CRITICAL BATTERY! System shutdown...");
      // Add shutdown procedure here
      while(1) {
        delay(1000);
        Serial.println("System halted due to low battery");
      }
    }
  }
}
