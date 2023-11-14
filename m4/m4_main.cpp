//====================================================
// title block
//====================================================
/*  
 *  Author: Jacob Owens - Avid Product Development
 *  Customer: Vestas
 *  Date: 08082023
 */
//====================================================
// end title block
//====================================================


//====================================================
// shared data
//====================================================
struct shared_data {
  long stepperSpeed = 0; // buffer
  int stepperCommand = 0; // stepper state : 0 = stopped, 1 = forward, 2 = backward
};
volatile struct shared_data * const xfr_ptr = (struct shared_data *)0x38001000;
//====================================================
// end shared data
//====================================================


//====================================================
// included libraries
//====================================================
#include <Arduino.h>
//====================================================
// end included libraries
//====================================================


//====================================================
// wifi and websockets definitions
//====================================================
//====================================================
// end wifi and websockets definitions
//====================================================


//====================================================
// stepper definitions
//====================================================
#include <SPI.h>
#include <HighPowerStepperDriver.h>
#include <ContinuousStepper.h>


const uint8_t Step1Pin = D6;
const uint8_t Step2Pin = D12;

const uint8_t CSPin = D7;
const uint8_t DirPin = D7;

HighPowerStepperDriver sd;
ContinuousStepper<StepperDriver> stepper1;
ContinuousStepper<StepperDriver> stepper2;

int stepperCommand = 0;
long stepperSpeed = 0;

//====================================================
// end stepper definitions
//====================================================


//====================================================
// ultrasonic definitions
//====================================================
//====================================================
// end ultrasonic definitions
//====================================================


//====================================================
// led definitions
//====================================================
//====================================================
// end led definitions
//====================================================




//====================================================
// relay definitions
//====================================================
//====================================================
// end relay definitions
//====================================================




//====================================================
// json definitions
//====================================================
//====================================================
// end json definitions
//====================================================




//====================================================
// ip address definitions
//====================================================
//====================================================
// end ip address definitions
//====================================================




//====================================================
// function prototypes
//====================================================
void setup(void);
void loop(void);
void StepperMachine(void);
//====================================================
// end function prototypes
//====================================================




//====================================================
// states
//====================================================
enum StepperStates { 
  STEPPER_STOPPED, // 0
  STEPPER_FORWARD, // 1
  STEPPER_BACKWARD // 2
};
StepperStates stepperState;
//====================================================
// end states
//====================================================



//====================================================
// objects
//====================================================
//====================================================
// end objects
//====================================================




//====================================================
// timer
//====================================================
//====================================================
// end timer
//====================================================




//====================================================
// setup
//====================================================
void setup() {
  SPI.begin();
  sd.setChipSelectPin(CSPin);

  // Give the driver some time to power up.
  delay(10);

  // Reset the driver to its default settings and clear latched status
  // conditions.
  sd.resetSettings();
  sd.clearStatus();

  // Select auto mixed decay.  TI's DRV8711 documentation recommends this mode
  // for most applications, and we find that it usually works well.
  sd.setDecayMode(HPSDDecayMode::AutoMixed);

  // Set the current limit. You should change the number here to an appropriate
  // value for your particular system.
  sd.setCurrentMilliamps36v4(750);

  // Set the number of microsteps that correspond to one full step.
  sd.setStepMode(HPSDStepMode::MicroStep4);

  // Enable the motor outputs.
  sd.enableDriver();

  // change the pin to use it for motion
  // Drive the STEP and DIR pins low initially.
  pinMode(Step1Pin, OUTPUT);
  digitalWrite(Step1Pin, LOW);
  pinMode(Step2Pin, OUTPUT);
  digitalWrite(Step2Pin, LOW);
  pinMode(DirPin, OUTPUT);
  digitalWrite(DirPin, LOW);

  stepper1.begin(Step1Pin, DirPin);
  stepper2.begin(Step2Pin, DirPin);

  stepper1.setAcceleration(6400); 
  stepper2.setAcceleration(6400); 
  // end stepper setup

}
//====================================================
// end setup
//====================================================




//====================================================
// main loop
//====================================================
void loop() {

  StepperMachine();

}
//====================================================
// end main loop
//====================================================




//====================================================
// functions
//====================================================


//====================================================
// stepper machine
//====================================================
void StepperMachine(void) {
  stepperSpeed = xfr_ptr->stepperSpeed;
  stepperCommand = xfr_ptr->stepperCommand;

  stepper1.loop();
  stepper2.loop();

  switch(stepperCommand) {
    case 0:
      stepper1.stop();
      stepper2.stop();

    break;
    case 1:
      stepper1.spin(stepperSpeed);
      stepper2.spin(stepperSpeed);

    break;
    case 2:
      stepper1.spin(-stepperSpeed);
      stepper2.spin(-stepperSpeed);

    break;
    default:
    break;
  }
  
}
//====================================================
// end stepper machine
//====================================================



//====================================================
// end functions
//====================================================