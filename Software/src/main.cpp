#include <Arduino.h>
#include <SimpleFOC.h>

// magnetic sensor instance - I2C
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);

// BLDC motor & driver instance
BLDCMotor motor1 = BLDCMotor(8);
BLDCMotor motor2 = BLDCMotor(8);
// UH, UL, VH, VL, WH, WL
BLDCDriver6PWM driver1 = BLDCDriver6PWM(8, 7, 4, 33, 6, 9);
BLDCDriver6PWM driver2 = BLDCDriver6PWM(36, 37, 29, 28, 2, 3);

Commander command = Commander(Serial);
float target_angle1 = 0;
float target_angle2 = 0;
void setTargetA(char *cmd) { command.scalar(&target_angle1, cmd); }
void setTargetB(char *cmd) { command.scalar(&target_angle2, cmd); }
void toggleEnabled(char *_) {
  if (motor1.enabled)
    motor1.disable();
  else
    motor1.enable();
  if (motor2.enabled)
    motor2.disable();
  else
    motor2.enable();
}
void reset(char *_) { SCB_AIRCR = 0x05FA0004; }

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  delay(2000); // Wait for serial.
  Serial.println("Starting...");

  // Initialize encoder sensor hardware.
  sensor1.init(&Wire1);
  sensor2.init(&Wire2);
  // Link the motors to the sensors.
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  driver1.voltage_power_supply = 5; // IMPORTANT!
  if (!driver1.init()) {
    Serial.println("driver1.init failed.");
  }

  driver2.voltage_power_supply = 5; // IMPORTANT!
  if (!driver2.init()) {
    Serial.println("driver2.init failed.");
  }

  // Link drivers.
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);

  // Aligning voltage.
  motor1.voltage_sensor_align = 3;
  motor2.voltage_sensor_align = 3;

  // Set motion control loop to be used.
  motor1.controller = MotionControlType::angle;
  motor2.controller = MotionControlType::angle;

  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);

  // Initialize motor.
  motor1.init();
  motor2.init();
  // Align sensor and start FOC.
  motor1.initFOC();
  motor2.initFOC();

  // Add serial commands.
  command.add('a', setTargetA, "target angle A");
  command.add('b', setTargetB, "target angle B");
  command.add('t', toggleEnabled, "toggle enabled");
  command.add('r', reset, "toggle enabled");

  Serial.println(F("System ready.\nAvailable commands:\n  a<angle>: set motor "
                   "1 target angle.\n  b<angle>: set "
                   "motor 2 target angle.\n  t: toggle enabled.\n  r: reset."));

  delay(1000);
}

void loop() {
  motor1.loopFOC();
  motor2.loopFOC();
  motor1.move(target_angle1);
  motor2.move(target_angle2);

  command.run();
}