Wiring:

· ADXL345: VCC→3.3V, GND→GND, SDA→A4, SCL→A5
· Increase Solenoids: Pins 3,5,6 (PWM)
· Decrease Solenoids: Pins 9,10,11 (PWM)

Dual Operation Modes:

1. SENSOR Mode: Uses real-time ADXL345 data for responsive control
2. SEQUENCE Mode: Follows pre-programmed video sequence data
3. CALIBRATE Mode: Direct sensor mapping for testing

Video Sequence Data:

· Uses the provided format: {time_ms, pitch, roll}
· Implements smooth interpolation between sequence points
· Automatic sequence playback with start/stop control

Advanced Control:

· PID Control: Proportional, Integral, Derivative control for smooth movements
· Hysteresis: Prevents solenoid oscillation
· PWM Control: Variable pressure control based on error magnitude

6-Solenoid Configuration:

· 3 Pressure Increase (pins 3,5,6)
· 3 Pressure Decrease (pins 9,10,11)
· Each cylinder has complementary control

Serial Commands:

· MODE SENSOR - Use ADXL345 sensor data
· MODE SEQUENCE - Use video sequence data
· START - Start sequence playback
· STOP - Stop sequence playback
· RESET - Return to neutral position
· CALIBRATE - Enter calibration mode
· STATUS - Show current status

Sequence Data:

The program includes your example data:

· {0, 0, 0} - Neutral at 0ms
· {1000, 10, 0} - 10° pitch at 1000ms
· {2000, 0, 10} - 10° roll at 2000ms

Plus additional points for a complete demo sequence.
