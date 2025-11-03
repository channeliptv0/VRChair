#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Assign a unique ID to this sensor at the same time
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Solenoid control pins - 3 for pressure increase, 3 for pressure decrease
const int pressureIncreasePins[3] = {3, 5, 6};   // PWM pins for increasing pressure
const int pressureDecreasePins[3] = {9, 10, 11}; // PWM pins for decreasing pressure

// Cylinder configuration
// Assuming 3 air cylinders for the platform:
// Cylinder 0: Front
// Cylinder 1: Left Rear  
// Cylinder 2: Right Rear

// Variables for sensor data
float pitch = 0;
float roll = 0;

// Timing variables
unsigned long previousTime = 0;
const long interval = 50; // Update interval in milliseconds

// PID-like control variables
float previousPitch = 0;
float previousRoll = 0;
float pitchIntegral = 0;
float rollIntegral = 0;

// Structure for cylinder control
struct CylinderControl {
  int increasePin;  // Pressure increase solenoid
  int decreasePin;  // Pressure decrease solenoid
  float currentPressure; // 0-100% pressure level
  float targetPressure;  // 0-100% target pressure
  unsigned long activeTime;
  bool timedActive;
};

CylinderControl cylinders[3];

void setup(void) {
  Serial.begin(115200);
  
  // Initialize cylinder control structure
  for (int i = 0; i < 3; i++) {
    cylinders[i].increasePin = pressureIncreasePins[i];
    cylinders[i].decreasePin = pressureDecreasePins[i];
    cylinders[i].currentPressure = 50.0; // Start at 50% pressure
    cylinders[i].targetPressure = 50.0;
    cylinders[i].activeTime = 0;
    cylinders[i].timedActive = false;
    
    // Set pin modes
    pinMode(pressureIncreasePins[i], OUTPUT);
    pinMode(pressureDecreasePins[i], OUTPUT);
    
    // Ensure solenoids are off initially
    digitalWrite(pressureIncreasePins[i], LOW);
    digitalWrite(pressureDecreasePins[i], LOW);
  }
  
  // Initialize the accelerometer
  if (!accel.begin()) {
    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
    while (1);
  }
  
  accel.setRange(ADXL345_RANGE_8_G);
  Serial.println("ADXL345 initialized");
  
  Serial.println("VR Chair Platform with 6 Solenoids Ready");
  Serial.println("Commands: ");
  Serial.println("  TIME [cyl] [pressure] [duration] - Timed pressure control");
  Serial.println("  PRESSURE [cyl] [pressure] - Set target pressure");
  Serial.println("  RESET - Reset to neutral position");
}

void loop(void) {
  unsigned long currentTime = millis();
  
  // Update at specified interval
  if (currentTime - previousTime >= interval) {
    previousTime = currentTime;
    
    // Read accelerometer data
    readAccelerometer();
    
    // Calculate target pressures based on pitch and roll
    calculateTargetPressures();
    
    // Update solenoid outputs using bang-bang control with hysteresis
    updateSolenoids();
    
    // Process serial commands
    processSerialCommands();
    
    // Print debug information every 500ms
    static unsigned long lastPrint = 0;
    if (currentTime - lastPrint >= 500) {
      lastPrint = currentTime;
      printDebugInfo();
    }
  }
  
  // Check for timed activations
  checkTimedActivations(currentTime);
}

void readAccelerometer() {
  sensors_event_t event;
  accel.getEvent(&event);
  
  // Calculate pitch and roll from accelerometer data
  // Pitch: rotation around X-axis (forward/backward tilt)
  pitch = atan2(event.acceleration.y, 
               sqrt(event.acceleration.x * event.acceleration.x + 
                    event.acceleration.z * event.acceleration.z)) * 180 / PI;
  
  // Roll: rotation around Y-axis (left/right tilt)  
  roll = atan2(-event.acceleration.x, 
               event.acceleration.z) * 180 / PI;
  
  // Simple low-pass filter for smoothing
  pitch = 0.7 * pitch + 0.3 * previousPitch;
  roll = 0.7 * roll + 0.3 * previousRoll;
  
  previousPitch = pitch;
  previousRoll = roll;
}

void calculateTargetPressures() {
  // Map pitch and roll to target pressures for each cylinder
  // This creates a responsive system that counteracts tilting
  
  float pitchFactor = constrain(pitch / 30.0, -1.0, 1.0); // ±30 degrees full scale
  float rollFactor = constrain(roll / 30.0, -1.0, 1.0);   // ±30 degrees full scale
  
  // Cylinder 0 (Front) - Primarily affected by pitch
  cylinders[0].targetPressure = 50.0 + (pitchFactor * 30.0);
  
  // Cylinder 1 (Left Rear) - Affected by both pitch and roll
  cylinders[1].targetPressure = 50.0 - (pitchFactor * 15.0) + (rollFactor * 30.0);
  
  // Cylinder 2 (Right Rear) - Affected by both pitch and roll
  cylinders[2].targetPressure = 50.0 - (pitchFactor * 15.0) - (rollFactor * 30.0);
  
  // Constrain all pressures to safe limits
  for (int i = 0; i < 3; i++) {
    cylinders[i].targetPressure = constrain(cylinders[i].targetPressure, 10.0, 90.0);
  }
}

void updateSolenoids() {
  // Bang-bang control with hysteresis for each cylinder
  const float hysteresis = 2.0; // 2% hysteresis to prevent oscillation
  
  for (int i = 0; i < 3; i++) {
    if (!cylinders[i].timedActive) {
      float pressureError = cylinders[i].targetPressure - cylinders[i].currentPressure;
      
      if (pressureError > hysteresis) {
        // Need to increase pressure
        analogWrite(cylinders[i].increasePin, mapPressureToPWM(abs(pressureError)));
        digitalWrite(cylinders[i].decreasePin, LOW);
        
        // Simulate pressure change (in real system, this would be based on actual pressure feedback)
        cylinders[i].currentPressure += 0.5;
      }
      else if (pressureError < -hysteresis) {
        // Need to decrease pressure
        digitalWrite(cylinders[i].increasePin, LOW);
        analogWrite(cylinders[i].decreasePin, mapPressureToPWM(abs(pressureError)));
        
        // Simulate pressure change
        cylinders[i].currentPressure -= 0.5;
      }
      else {
        // Within hysteresis band - turn off both solenoids
        digitalWrite(cylinders[i].increasePin, LOW);
        digitalWrite(cylinders[i].decreasePin, LOW);
      }
      
      // Constrain current pressure
      cylinders[i].currentPressure = constrain(cylinders[i].currentPressure, 0.0, 100.0);
    }
  }
}

int mapPressureToPWM(float pressureError) {
  // Map pressure error to PWM value (0-255)
  // Larger errors get higher PWM for faster response
  int pwm = constrain(pressureError * 5.0, 0, 255);
  return pwm;
}

void checkTimedActivations(unsigned long currentTime) {
  for (int i = 0; i < 3; i++) {
    if (cylinders[i].timedActive && currentTime >= cylinders[i].activeTime) {
      cylinders[i].timedActive = false;
      // Return to normal control
      digitalWrite(cylinders[i].increasePin, LOW);
      digitalWrite(cylinders[i].decreasePin, LOW);
    }
  }
}

// Function to activate a specific cylinder for a set time
void activateCylinderTimed(int cylinderIndex, int pressure, unsigned long duration, bool increase) {
  if (cylinderIndex >= 0 && cylinderIndex < 3) {
    cylinders[cylinderIndex].timedActive = true;
    cylinders[cylinderIndex].activeTime = millis() + duration;
    
    if (increase) {
      analogWrite(cylinders[cylinderIndex].increasePin, map(pressure, 0, 100, 0, 255));
      digitalWrite(cylinders[cylinderIndex].decreasePin, LOW);
    } else {
      digitalWrite(cylinders[cylinderIndex].increasePin, LOW);
      analogWrite(cylinders[cylinderIndex].decreasePin, map(pressure, 0, 100, 0, 255));
    }
  }
}

void processSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Example command: "TIME 1 80 1000 INC" - activate cylinder 1 at 80% pressure for 1000ms increasing
    if (command.startsWith("TIME")) {
      int cylinder = command.substring(5, 6).toInt();
      int pressure = command.substring(7, 10).toInt();
      unsigned long duration = command.substring(11, 15).toInt();
      bool increase = command.substring(16) == "INC";
      
      activateCylinderTimed(cylinder, pressure, duration, increase);
    }
    // Example command: "PRESSURE 2 60" - set cylinder 2 target pressure to 60%
    else if (command.startsWith("PRESSURE")) {
      int cylinder = command.substring(9, 10).toInt();
      int pressure = command.substring(11).toInt();
      
      if (cylinder >= 0 && cylinder < 3) {
        cylinders[cylinder].targetPressure = constrain(pressure, 0, 100);
      }
    }
    // Reset to neutral position
    else if (command == "RESET") {
      for (int i = 0; i < 3; i++) {
        cylinders[i].targetPressure = 50.0;
      }
      Serial.println("Reset to neutral position");
    }
  }
}

void printDebugInfo() {
  Serial.print("Pitch: ");
  Serial.print(pitch, 1);
  Serial.print("° | Roll: ");
  Serial.print(roll, 1);
  Serial.print("° | Cylinders: ");
  
  for (int i = 0; i < 3; i++) {
    Serial.print("C");
    Serial.print(i);
    Serial.print(":");
    Serial.print(cylinders[i].currentPressure, 0);
    Serial.print("/");
    Serial.print(cylinders[i].targetPressure, 0);
    if (i < 2) Serial.print(" ");
  }
  Serial.println();
}

// Emergency stop function
void emergencyStop() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(cylinders[i].increasePin, LOW);
    digitalWrite(cylinders[i].decreasePin, LOW);
    cylinders[i].timedActive = false;
  }
  Serial.println("EMERGENCY STOP - All solenoids deactivated");
}

// Add emergency stop check (could be triggered by button or serial command)
void checkEmergencyStop() {
  // Example: Stop if tilt exceeds safe limits
  if (abs(pitch) > 45 || abs(roll) > 45) {
    emergencyStop();
  }
}
