#include <AccelStepper.h>

// Define some steppers and the pins the will use
AccelStepper stepper1(5, 2, 3, 4, 5);
// The first argument '5' indicates that we're using a 28byj-48 geared motor. The rest is just pin numbers. 
//You can still use all the other types of motors supported by accelstepper library (e.g. 4 for a normal 4 wire step motor, 8 for a halfstepped normal 4 wire motor etc.) 
AccelStepper stepper2(5, 6, 7, 8, 9);


void setup()
{  
  Serial.begin(9600);
  stepper1.setMaxSpeed(1000); //max speed of the first motor - modify if you want to
  stepper1.setAcceleration(1000); // rate at which the first motor accelerate -

  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(1000);



}

void loop()
{    

  if (stepper1.distanceToGo() == 0){
    stepper1.moveTo(300);//just an easy way to get the motors to move to random positions
    delay(30000);
  } 
  if (stepper2.distanceToGo() == 0){
    stepper2.moveTo(200);
    delay(30000);
  }
  stepper1.run();
  stepper2.run();

}
