#include <Arduino.h>
#include <SimpleFOC.h>
#include <encoders/smoothing/SmoothingSensor.h>

// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(4, 0.5393);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

HallSensor sensor = HallSensor(A_HALL1, A_HALL2, A_HALL3, 4);
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}
// instantiate the smoothing sensor, providing the real sensor as a constructor argument
SmoothingSensor smooth = SmoothingSensor(sensor, motor);

// instantiate the commander
float target_voltage = 5;

Commander command = Commander(Serial);

void doTarget(char *cmd){ command.scalar(&target_voltage, cmd);}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

  pinMode(A_OP3_OUT, OUTPUT);
  pinMode(A_CAN_SHDN, OUTPUT);

  #if 0
  // check if you need internal pullups
  sensor.pullup = Pullup::USE_EXTERN;
  
  // initialise encoder hardware
  sensor.init();
  // hardware interrupt enable
  sensor.enableInterrupts(doA, doB, doC);
  smooth.phase_correction = -_PI_6;
  // link the motor to the sensor
  motor.linkSensor(&smooth);
  #else
   // initialise magnetic sensor hardware
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  // set SmoothingSensor phase correction for hall sensors
  //smooth.phase_correction = -_PI_6;
  // link the motor to the sensor
  motor.linkSensor(&smooth);//smooth

  driver.voltage_power_supply = 12;
  driver.pwm_frequency = 16000;
  driver.dead_zone = 0.03;

  //driver.voltage_limit = 8;
  if ( driver.init() )
  {
    driver.enable();
  } else
  {
    return;
  }

  motor.linkDriver(&driver);

  // motor.current_limit = 0.1f; // 1.0 [Amps]
  // Orange-Red:   0.5412R
  // Brown-Red:    0.5399R
  // Brown-Orange: 0.5368R
  //motor.phase_resistance = 0.5393; // Average 0.5393 [Ohm]

  //motor.voltage_limit = 1; //[V]
  //motor.velocity_limit = 20; //[rad/s]

  // aligning voltage 
  motor.voltage_sensor_align = 1;
  motor.torque_controller = TorqueControlType::voltage;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM; // Trapezoid_120 ; SpaceVectorPWM ; Trapezoid_150
  motor.controller = MotionControlType::torque;
  //motor.voltage_limit = 8;
  motor.useMonitoring(Serial);

  // init motor hardware
  motor.init();
  // align sensor and start FOC
  motor.initFOC();
  
  //command.add('C', onPID, "my pid");
  command.add('T', doTarget, "target voltage");
  #endif

  Serial.println("Motor ready!");
  Serial.println(F("Set the target voltage using serial terminal:"));
  _delay(1000);
}

void loop()
{
  #if 0
  smooth.update();
  // display the angle and the angular velocity to the terminal
  Serial.print(smooth.getAngle());
  Serial.print("\t");
  Serial.println(smooth.getVelocity());
  delay(100);
  #else
  digitalWrite(A_OP3_OUT, HIGH);
  motor.loopFOC();
  digitalWrite(A_OP3_OUT, LOW);

  digitalWrite(A_CAN_SHDN, HIGH);
  motor.move(target_voltage);
  digitalWrite(A_CAN_SHDN, LOW);

  command.run();
  #endif
}
