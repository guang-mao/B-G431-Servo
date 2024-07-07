#include <Arduino.h>
#include <SimpleFOC.h>

// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
MagneticSensorPWM sensor(PB6, 2, 904);
// https://docs.simplefoc.com/low_side_current_sense
// LowsideCurrentSense constructor
//  - shunt_resistor  - shunt resistor value
//  - gain  - current-sense op-amp gain
//  - phA   - A phase adc pin
//  - phB   - B phase adc pin
//  - phC   - C phase adc pin (optional)
//LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// instantiate the commander
float target_voltage = 0;
float target_velocity = 0;

Commander command = Commander(Serial);

//void onPID(char* cmd){command.pid(&motor.PID_velocity, cmd);}
void doTarget(char *cmd){ command.scalar(&target_voltage, cmd);}

void doPWM()
{
  sensor.handlePWM();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //command.add('T', doTarget, "target velocity");
   // initialise magnetic sensor hardware
  sensor.init();
  // comment out to use sensor in blocking (non-interrupt) way
  sensor.enableInterrupt(doPWM);
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 10;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  //driver.voltage_limit = 10;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // motor.PID_velocity.P = 0.05;
  // motor.PID_velocity.I = 0.005;
  // motor.PID_velocity.D = 0;

  // motor.PID_velocity.output_ramp = 1000;

  // motor.LPF_velocity.Tf = 0.1;

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  //motor.voltage_limit = 4.5;   // [V]

  // Limits
  //motor.current_limit = 0.8f; // 1.0 [Amps]

  // general settings
  // motor phase resistance // I_max = V_dc/R
  motor.phase_resistance = 5.57; // 5,57 [Ohm]

  // motor KV rating [rpm/V]
  //motor.KV_rating = 197; // [rpm/volt] - default not set

  // aligning voltage 
  //motor.voltage_sensor_align = 5;
  motor.torque_controller = TorqueControlType::voltage;
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
   // set motion control loop to be used
  motor.controller = MotionControlType::torque;
  

  motor.useMonitoring(Serial);

  // init motor hardware
  motor.init();

  // align sensor and start FOC
  motor.initFOC();
  
  //command.add('C', onPID, "my pid");
  command.add('T', doTarget, "target voltage");

  Serial.println("Motor ready!");
  Serial.println(F("Set the target voltage using serial terminal:"));
  _delay(1000);
  //TIM_TypeDef *Instance = TIM1;
  //HardwareTimer *Timer = new HardwareTimer(Instance);

}

void loop()
{
  // put your main code here, to run repeatedly:
  //sensor.update();

  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  motor.loopFOC();

  motor.move(target_voltage);

  // user communication
  command.run();
}
