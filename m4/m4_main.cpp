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
  bool estopState = 0;         // estop state : 0 = no emergency, 1 = emergency 
};                        // estop state : 0 = sd.Enabled(), 1 = sd.Disabled()
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
#include <ContinuousStepper.h>

const uint8_t Step1Pin = D1;
const uint8_t Step2Pin = D2;
const uint8_t Step3Pin = D3;
const uint8_t Step4Pin = D4;

const uint8_t Dir1Pin = LEDB + 1 + PC_13; //gpio 0
const uint8_t Dir2Pin = LEDB + 1 + PC_15; //gpio 1
const uint8_t Dir3Pin = LEDB + 1 + PD_4;  //gpio 2
const uint8_t Dir4Pin = LEDB + 1 + PD_5;  //gpio 3

ContinuousStepper<StepperDriver> stepper1;
ContinuousStepper<StepperDriver> stepper2;
ContinuousStepper<StepperDriver> stepper3;
ContinuousStepper<StepperDriver> stepper4;

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
  // change the pin to use it for motion
  // Drive the STEP and DIR pins low initially.
  pinMode(      Step1Pin, OUTPUT);
  digitalWrite( Step1Pin, LOW);
  pinMode(      Step2Pin, OUTPUT);
  digitalWrite( Step2Pin, LOW);
  pinMode(      Step3Pin, OUTPUT);
  digitalWrite( Step3Pin, LOW);
  pinMode(      Step4Pin, OUTPUT);
  digitalWrite( Step4Pin, LOW);
//
  pinMode(      Dir1Pin, OUTPUT);
  digitalWrite( Dir1Pin, LOW);
  pinMode(      Dir2Pin, OUTPUT);
  digitalWrite( Dir2Pin, LOW);
  pinMode(      Dir3Pin, OUTPUT);
  digitalWrite( Dir3Pin, LOW);
  pinMode(      Dir4Pin, OUTPUT);
  digitalWrite( Dir4Pin, LOW);


  stepper1.begin(Step1Pin, Dir1Pin);
  stepper2.begin(Step2Pin, Dir2Pin);
  stepper3.begin(Step3Pin, Dir3Pin);
  stepper4.begin(Step4Pin, Dir4Pin);
//
  stepper1.setAcceleration(800); 
  stepper2.setAcceleration(800); 
  stepper3.setAcceleration(800); 
  stepper4.setAcceleration(800); 

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
  stepper3.loop();
  stepper4.loop();
  
  switch(stepperCommand) {
    case 0:
      stepper1.stop();
      stepper2.stop();
      stepper3.stop();
      stepper4.stop();

    break;
    case 1:
      stepper1.spin(stepperSpeed);
      stepper2.spin(stepperSpeed);
      stepper3.spin(stepperSpeed);
      stepper4.spin(stepperSpeed);

    break;
    case 2:
      stepper1.spin(-stepperSpeed);
      stepper2.spin(-stepperSpeed);
      stepper3.spin(-stepperSpeed);
      stepper4.spin(-stepperSpeed);

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
