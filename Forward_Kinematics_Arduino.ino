#include <Servo.h>
// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define GripperPin 6
// Servo Objects
Servo Joint1;
Servo Joint2;
Servo Joint3;
Servo Gripper;
// Starting Joint Angles
int Joint1Angle = 45;
int Joint2Angle = 45;
int Joint3Angle = 90;
int GripperOpen = 60;
int GripperClose = 180;
// Joint Angle Offsets - KEEP
int Joint1Offset = 90;
int Joint2Offset = 2;
int Joint3Offset = 23;

void setup()
{
Serial.begin(9600);
Joint1.attach(Joint1Pin);
Joint2.attach(Joint2Pin);
Joint3.attach(Joint3Pin);
Gripper.attach(GripperPin);
Joint1.write(Joint1Angle+Joint1Offset);
Joint2.write(Joint2Angle+Joint2Offset);
Joint3.write(Joint3Angle+Joint3Offset);
Gripper.write(GripperOpen); // Open gripper
delay(5000); // Wait 5 seconds before going into loop()
}
void loop()
{
Joint1.write(Joint1Angle+Joint1Offset);
Joint2.write(Joint2Angle+Joint2Offset);
Joint3.write(Joint3Angle+Joint3Offset);
delay(5);
}