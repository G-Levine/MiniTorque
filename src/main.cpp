#include "Arduino.h"
#include "Wire.h"
#include "Math.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "drivers/drv8316/drv8316.h"
#include "encoders/as5047/MagneticSensorAS5047.h"

// Motor parameters
#define POLE_PAIRS 14
#define PHASE_RESISTANCE 1.4
#define CURRENT_LIMIT 2
#define VOLTAGE_LIMIT 0.5
#define POWER_SUPPLY_VOLTAGE 12
#define SENSOR_ALIGN_VOLTAGE 0.2

// Pin definitions
// DRV8316 gate driver pins
#define DRV_HA 2
#define DRV_HB 3
#define DRV_HC 4
#define DRV_LA 5
#define DRV_LB 6
#define DRV_LC 7

// DRV8316 SPI pins
#define DRV_SPI_CS 8

// Encoder SPI pins
#define ENC_SPI_CS 10

BLDCMotor motor = BLDCMotor(POLE_PAIRS);
DRV8316Driver6PWM driver = DRV8316Driver6PWM(DRV_HA,DRV_LA,DRV_HB,DRV_LB,DRV_HC,DRV_LC,DRV_SPI_CS);
MagneticSensorAS5047 encoder = MagneticSensorAS5047(ENC_SPI_CS);

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }

void setup() {
  // initialize encoder sensor hardware
  encoder.init();
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // aligning voltage
  motor.voltage_sensor_align = SENSOR_ALIGN_VOLTAGE;

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  // set limits
  motor.phase_resistance = PHASE_RESISTANCE; // [Ohm]
  motor.current_limit = CURRENT_LIMIT;   // [Amps]
  motor.voltage_limit = VOLTAGE_LIMIT;   // [V]

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  // command.add('T', doTarget, "target current"); // - if phase resistance defined
  command.add('T', doTarget, "target voltage");

  Serial.println(F("Motor ready."));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  // user communication
  command.run();
}