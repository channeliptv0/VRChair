#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Assign a unique ID to this sensor at the same time
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Solenoid control pins - 3 for pressure increase, 3 for pressure decrease
const int pressureIncreasePins[3] = {3, 5, 6};   // PWM pins for increasing pressure
const int pressureDecreasePins[3] = {9, 10, 11}; // PWM pins for decreasing pressure

// Video sequence data structure: {time_ms, pitch, roll}
struct VideoFrame {
  unsigned long time;
  float pitch;
  float roll;
};

// Pre-programmed video sequence data
VideoFrame videoSequence[] = {
  {0, 0, 0},
  {1000, 10, 0},
  {2000, 0, 10},
  {3000, -10, 0},
  {4000, 0, -10},
  {5000, 0, 0}
};

const int sequenceLength = sizeof(videoSequence) / sizeof(videoSequence[0]);

// Operation modes
enum OperationMode {
  MODE_SENSOR,    // Use ADXL345 sensor data
  MODE_SEQUENCE,  // Use pre-programmed sequence
  MODE_CALIBRATE  // Calibration mode
};

OperationMode currentMode = MODE_SENSOR;

// Variables for sensor data
float currentPitch = 0;
float currentRoll = 0;
float targetPitch = 0;
float targetRoll = 0;

// Timing variables
unsigned long previousTime = 0;
const long controlInterval = 50; // Control update interval in milliseconds

// Sequence playback variables
unsigned long sequenceStartTime = 0;
int currentSequenceIndex = 0;
bool sequencePlaying = false;

// Structure for cylinder control
struct CylinderControl {
  int increasePin;  // Pressure increase solenoid
  int decreasePin;  // Pressure decrease solenoid
  float currentPressure; // 0-100% pressure level
  float targetPressure;  // 0-100% target pressure
};

CylinderControl cylinders[3];

// PID-like control variables
float previousPitchError = 0;
float previousRollError = 0;
float pitchIntegral = 0;
float rollIntegral = 0;

// Control gains
const float Kp = 2.0;  // Proportional gain
const float Ki = 0.1;  // Integral gain
const float Kd = 0.5;  // Derivative gain

void setup(void) {
  Serial.begin(115200);
  
  // Initialize cylinder control structure
  for (int i = 0; i < 3; i++) {
    cylinders[i].increasePin = pressureIncreasePins[i];
    cylinders[i].decreasePin = pressureDecreasePins[i];
    cylinders[i].currentPressure = 50.0; // Start at 50% pressure
    cylinders[i].targetPressure = 50.0;
    
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
  
  // Print sequence info
  Serial.println("Video Sequence Loaded:");
  for (int i = 0; i < sequenceLength; i++) {
    Serial.print("Frame ");
    Serial.print(i);
    Serial.print(": Time=");
    Serial.print(videoSequence[i].time);
    Serial.print("ms, Pitch=");
    Serial.print(videoSequence[i].pitch);
    Serial.print("°, Roll=");
    Serial.print(videoSequence[i].roll);
    Serial.println("°");
  }
  
  Serial.println("\nVR Chair Platform Ready");
  Serial.println("Commands: ");
  Serial.println("  MODE SENSOR - Use ADXL345 sensor data");
  Serial.println("  MODE SEQUENCE - Use pre-programmed sequence");
  Serial.println("  START - Start sequence playback");
  Serial.println("  STOP - Stop sequence playback");
  Serial.println("  RESET - Reset to neutral position");
  Serial.println("  CALIBRATE - Enter calibration mode");
}

void loop(void) {
  unsigned long currentTime = millis();
  
  // Process serial commands
  processSerialCommands();
  
  // Update at specified interval
  if (currentTime - previousTime >= controlInterval) {
    previousTime = currentTime;
    
    // Read current orientation from ADXL345
    readAccelerometer();
    
    // Update targets based on current mode
    updateTargets(currentTime);
    
    // Calculate target pressures based on current and target orientation
    calculateTargetPressures();
    
    // Update solenoid outputs
    updateSolenoids();
    
    // Print debug information every 200ms
    static unsigned long lastPrint = 0;
    if (currentTime - lastPrint >= 200) {
      lastPrint = currentTime;
      printDebugInfo();
    }
  }
}

void readAccelerometer() {
  sensors_event_t event;
  accel.getEvent(&event);
  
  // Calculate pitch and roll from accelerometer data
  // Pitch: rotation around X-axis (forward/backward tilt)
  currentPitch = atan2(event.acceleration.y, 
               sqrt(event.acceleration.x * event.acceleration.x + 
                    event.acceleration.z * event.acceleration.z)) * 180 / PI;
  
  // Roll: rotation around Y-axis (left/right tilt)  
  currentRoll = atan2(-event.acceleration.x, 
               event.acceleration.z) * 180 / PI;
}

void updateTargets(unsigned long currentTime) {
  switch (currentMode) {
    case MODE_SENSOR:
      // In sensor mode, target follows current orientation (neutral position)
      targetPitch = 0;
      targetRoll = 0;
      break;
      
    case MODE_SEQUENCE:
      if (sequencePlaying) {
        unsigned long sequenceTime = currentTime - sequenceStartTime;
        
        // Find current segment in sequence
        if (currentSequenceIndex < sequenceLength - 1 && 
            sequenceTime >= videoSequence[currentSequenceIndex + 1].time) {
          currentSequenceIndex++;
        }
        
        if (currentSequenceIndex < sequenceLength - 1) {
          // Interpolate between sequence points
          unsigned long segmentStart = videoSequence[currentSequenceIndex].time;
          unsigned long segmentEnd = videoSequence[currentSequenceIndex + 1].time;
          float t = float(sequenceTime - segmentStart) / float(segmentEnd - segmentStart);
          
          targetPitch = lerp(videoSequence[currentSequenceIndex].pitch, 
                            videoSequence[currentSequenceIndex + 1].pitch, t);
          targetRoll = lerp(videoSequence[currentSequenceIndex].roll, 
                           videoSequence[currentSequenceIndex + 1].roll, t);
        } else {
          // End of sequence
          targetPitch = videoSequence[sequenceLength - 1].pitch;
          targetRoll = videoSequence[sequenceLength - 1].roll;
          
          // Stop sequence if we've reached the end
          if (sequenceTime > videoSequence[sequenceLength - 1].time + 1000) {
            sequencePlaying = false;
            Serial.println("Sequence completed");
          }
        }
      } else {
        // Not playing - maintain neutral
        targetPitch = 0;
        targetRoll = 0;
      }
      break;
      
    case MODE_CALIBRATE:
      // In calibration mode, use sensor data directly
      targetPitch = currentPitch;
      targetRoll = currentRoll;
      break;
  }
}

void calculateTargetPressures() {
  // Calculate errors
  float pitchError = targetPitch - currentPitch;
  float rollError = targetRoll - currentRoll;
  
  // PID calculations
  pitchIntegral += pitchError;
  rollIntegral += rollError;
  
  float pitchDerivative = pitchError - previousPitchError;
  float rollDerivative = rollError - previousRollError;
  
  float pitchOutput = Kp * pitchError + Ki * pitchIntegral + Kd * pitchDerivative;
  float rollOutput = Kp * rollError + Ki * rollIntegral + Kd * rollDerivative;
  
  // Apply outputs to cylinder pressures
  // Cylinder 0: Front (primarily pitch)
  cylinders[0].targetPressure = 50.0 + pitchOutput * 10.0;
  
  // Cylinder 1: Left Rear (pitch and roll)
  cylinders[1].targetPressure = 50.0 - pitchOutput * 5.0 - rollOutput * 10.0;
  
  // Cylinder 2: Right Rear (pitch and roll)
  cylinders[2].targetPressure = 50.0 - pitchOutput * 5.0 + rollOutput * 10.0;
  
  // Constrain all pressures to safe limits
  for (int i = 0; i < 3; i++) {
    cylinders[i].targetPressure = constrain(cylinders[i].targetPressure, 10.0, 90.0);
  }
  
  // Store errors for next derivative calculation
  previousPitchError = pitchError;
  previousRollError = rollError;
}

void updateSolenoids() {
  // Bang-bang control with hysteresis for each cylinder
  const float hysteresis = 3.0; // 3% hysteresis to prevent oscillation
  
  for (int i = 0; i < 3; i++) {
    float pressureError = cylinders[i].targetPressure - cylinders[i].currentPressure;
    
    if (pressureError > hysteresis) {
      // Need to increase pressure
      int pwmValue = mapPressureToPWM(abs(pressureError));
      analogWrite(cylinders[i].increasePin, pwmValue);
      digitalWrite(cylinders[i].decreasePin, LOW);
      
      // Simulate pressure change (in real system, use pressure sensors)
      cylinders[i].currentPressure += 0.8;
    }
    else if (pressureError < -hysteresis) {
      // Need to decrease pressure
      int pwmValue = mapPressureToPWM(abs(pressureError));
      digitalWrite(cylinders[i].increasePin, LOW);
      analogWrite(cylinders[i].decreasePin, pwmValue);
      
      // Simulate pressure change
      cylinders[i].currentPressure -= 0.8;
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

int mapPressureToPWM(float pressureError) {
  // Map pressure error to PWM value (0-255)
  // Larger errors get higher PWM for faster response
  int pwm = constrain(pressureError * 6.0, 50, 255);
  return pwm;
}

float lerp(float a, float b, float t) {
  return a + (b - a) * t;
}

void processSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    if (command.startsWith("MODE SENSOR")) {
      currentMode = MODE_SENSOR;
      sequencePlaying = false;
      Serial.println("Mode: SENSOR (Using ADXL345 data)");
    }
    else if (command.startsWith("MODE SEQUENCE")) {
      currentMode = MODE_SEQUENCE;
      Serial.println("Mode: SEQUENCE (Using video sequence data)");
    }
    else if (command.startsWith("MODE CALIBRATE")) {
      currentMode = MODE_CALIBRATE;
      sequencePlaying = false;
      Serial.println("Mode: CALIBRATE (Direct sensor mapping)");
    }
    else if (command == "START") {
      if (currentMode == MODE_SEQUENCE) {
        sequenceStartTime = millis();
        currentSequenceIndex = 0;
        sequencePlaying = true;
        Serial.println("Sequence playback STARTED");
      } else {
        Serial.println("Start command only works in SEQUENCE mode");
      }
    }
    else if (command == "STOP") {
      sequencePlaying = false;
      Serial.println("Sequence playback STOPPED");
    }
    else if (command == "RESET") {
      // Reset to neutral position
      for (int i = 0; i < 3; i++) {
        cylinders[i].targetPressure = 50.0;
      }
      pitchIntegral = 0;
      rollIntegral = 0;
      previousPitchError = 0;
      previousRollError = 0;
      Serial.println("Reset to neutral position");
    }
    else if (command == "STATUS") {
      printDebugInfo();
    }
  }
}

void printDebugInfo() {
  Serial.print("Mode: ");
  switch (currentMode) {
    case MODE_SENSOR: Serial.print("SENSOR"); break;
    case MODE_SEQUENCE: Serial.print("SEQUENCE"); break;
    case MODE_CALIBRATE: Serial.print("CALIBRATE"); break;
  }
  
  Serial.print(" | Current: Pitch=");
  Serial.print(currentPitch, 1);
  Serial.print("°, Roll=");
  Serial.print(currentRoll, 1);
  Serial.print("°");
  
  Serial.print(" | Target: Pitch=");
  Serial.print(targetPitch, 1);
  Serial.print("°, Roll=");
  Serial.print(targetRoll, 1);
  Serial.print("°");
  
  if (currentMode == MODE_SEQUENCE && sequencePlaying) {
    Serial.print(" | Seq: ");
    Serial.print(currentSequenceIndex);
    Serial.print("/");
    Serial.print(sequenceLength - 1);
  }
  
  Serial.print(" | Pressures: ");
  for (int i = 0; i < 3; i++) {
    Serial.print("C");
    Serial.print(i);
    Serial.print(":");
    Serial.print(cylinders[i].currentPressure, 0);
    if (i < 2) Serial.print(" ");
  }
  Serial.println();
}

// Emergency stop function
void emergencyStop() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(cylinders[i].increasePin, LOW);
    digitalWrite(cylinders[i].decreasePin, LOW);
  }
  sequencePlaying = false;
  Serial.println("EMERGENCY STOP - All solenoids deactivated");
}
