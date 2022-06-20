#include <AccelStepper.h>
#define HALFSTEP 8
int gocdat;
bool data=false;
char sobuoc;

// Motor pin definitions
#define motorPin1  3     // IN1 on the ULN2003 driver 1
#define motorPin2  4     // IN2 on the ULN2003 driver 1
#define motorPin3  5     // IN3 on the ULN2003 driver 1
#define motorPin4  6     // IN4 on the ULN2003 driver 1

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
AccelStepper stepper1(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4);

void setup() {
  stepper1.setMaxSpeed(1000.0);
  stepper1.setAcceleration(100.0);
  stepper1.setSpeed(200);
  Serial.begin(115200);
//  stepper1.moveTo(1000);

}//--(end setup )---

void loop() {
  if(Serial.available()  > 0)
  {
     sobuoc = Serial.parseInt() 
     if( sobuoc >= 10) sobuoc 
//     gocdat = int(sobuoc);
//     Serial.println(gocdat);
  
  //Change direction when the stepper reaches the target position

     data = false;
     stepper1.moveTo(long(gocdat));
//     stepper1.run();
  }
//    stepper1.moveTo(2000);
  stepper1.run();
  Serial.println(sobuoc);
}
