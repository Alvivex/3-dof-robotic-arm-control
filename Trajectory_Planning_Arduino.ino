#include <Servo.h>

// pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define GripperPin 6

// servo definitions
Servo Joint1;
Servo Joint2;
Servo Joint3;
Servo Gripper;

// starting joint angles
int Joint1Angle = 90;
int Joint2Angle = 90;
int Joint3Angle = 90;

// gripper control
int GripperOpen = 0;
int GripperClose = 150;

// offsets
int Joint1Offset = 5;
int Joint2Offset = 12;
int Joint3Offset = 27;

// first movement

float theta1_traj_1[] = {90.00, 89.95, 89.80, 89.55, 89.20, 88.77, 88.25, 87.64, 86.95, 86.17, 85.32, 84.39, 83.39, 82.32, 81.18, 79.98, 78.72, 77.41, 76.04, 74.63, 73.19, 71.71, 70.20, 68.67, 67.14, 65.59, 64.05, 62.52, 61.00, 59.51, 58.04, 56.62, 55.23, 53.89, 52.60, 51.37, 50.19, 49.08, 48.04, 47.07, 46.16, 45.34, 44.59, 43.92, 43.33, 42.82, 42.40, 42.07, 41.83, 41.68, 41.63};

float theta2_traj_1[] = {90.00, 90.04, 90.16, 90.35, 90.61, 90.91, 91.26, 91.64, 92.04, 92.44, 92.83, 93.19, 93.50, 93.77, 93.96, 94.06, 94.06, 93.95, 93.71, 93.34, 92.81, 92.13, 91.29, 90.29, 89.13, 87.81, 86.34, 84.73, 82.99, 81.14, 79.20, 77.19, 75.11, 73.01, 70.89, 68.78, 66.70, 64.66, 62.69, 60.79, 59.00, 57.31, 55.75, 54.33, 53.05, 51.95, 51.02, 50.28, 49.73, 49.40, 49.29};

float theta3_traj_1[] = {90.00, 90.10, 90.39, 90.87, 91.51, 92.31, 93.25, 94.30, 95.46, 96.69, 98.00, 99.35, 100.73, 102.12, 103.50, 104.85, 106.17, 107.42, 108.60, 109.69, 110.67, 111.55, 112.29, 112.90, 113.36, 113.67, 113.84, 113.85, 113.71, 113.43, 113.01, 112.46, 111.79, 111.01, 110.14, 109.20, 108.19, 107.14, 106.07, 104.98, 103.90, 102.85, 101.84, 100.90, 100.03, 99.26, 98.60, 98.06, 97.66, 97.42, 97.34};

int numPoints_1 = 51;

// second movement

float theta1_traj_2[] = {90.00, 89.98, 89.90, 89.78, 89.62, 89.41, 89.16, 88.88, 88.55, 88.20, 87.80, 87.38, 86.93, 86.45, 85.94, 85.41, 84.86, 84.28, 83.69, 83.09, 82.47, 81.84, 81.20, 80.56, 79.91, 79.25, 78.60, 77.95, 77.31, 76.67, 76.04, 75.42, 74.81, 74.22, 73.65, 73.10, 72.57, 72.06, 71.58, 71.12, 70.70, 70.31, 69.95, 69.62, 69.34, 69.09, 68.88, 68.72, 68.60, 68.52, 68.50};

float theta2_traj_2[] = {90.00, 90.01, 90.03, 90.07, 90.12, 90.17, 90.22, 90.27, 90.29, 90.29, 90.26, 90.19, 90.08, 89.91, 89.67, 89.37, 88.99, 88.52, 87.97, 87.33, 86.59, 85.76, 84.83, 83.80, 82.67, 81.46, 80.15, 78.77, 77.32, 75.80, 74.23, 72.62, 70.98, 69.32, 67.65, 66.00, 64.37, 62.77, 61.22, 59.73, 58.31, 56.98, 55.75, 54.62, 53.61, 52.73, 51.99, 51.40, 50.97, 50.71, 50.62};

float theta3_traj_2[] = {90.00, 90.07, 90.27, 90.59, 91.03, 91.57, 92.21, 92.93, 93.73, 94.58, 95.47, 96.41, 97.36, 98.32, 99.29, 100.24, 101.17, 102.06, 102.91, 103.70, 104.44, 105.10, 105.70, 106.21, 106.63, 106.97, 107.21, 107.37, 107.44, 107.41, 107.30, 107.11, 106.85, 106.51, 106.12, 105.67, 105.17, 104.64, 104.09, 103.52, 102.95, 102.39, 101.84, 101.32, 100.85, 100.42, 100.06, 99.76, 99.54, 99.40, 99.35};

int numPoints_2 = 51;


void setup()
{
  Serial.begin(9600);

  Joint1.attach(Joint1Pin);
  Joint2.attach(Joint2Pin);
  Joint3.attach(Joint3Pin);
  Gripper.attach(GripperPin);

  Joint1.write(Joint1Angle + Joint1Offset);
  Joint2.write(Joint2Angle + Joint2Offset);
  Joint3.write(Joint3Angle + Joint3Offset);

  Gripper.write(GripperOpen);

  delay(5000);
}

void loop()
{

  // loop through trajectory 1
  for(int i = 0; i < numPoints_1; i++)
  {
    Joint1.write(theta1_traj_1[i] + Joint1Offset);
    Joint2.write(theta2_traj_1[i] + Joint2Offset);
    Joint3.write(theta3_traj_1[i] + Joint3Offset);
    delay(100);
  }

  delay(5000);

  // close gripper
  Gripper.write(GripperClose);
  delay(2000);

  // loop through trajectory 1 backwards (return)
  for(int i = numPoints_1 - 1; i >= 0; i--)
  {
    Joint1.write(theta1_traj_1[i] + Joint1Offset);
    Joint2.write(theta2_traj_1[i] + Joint2Offset);
    Joint3.write(theta3_traj_1[i] + Joint3Offset);
    delay(100);
  }

  delay(5000);

  // loop through trajectory 2
  for(int i = 0; i < numPoints_2; i++)
  {
    Joint1.write(theta1_traj_2[i] + Joint1Offset);
    Joint2.write(theta2_traj_2[i] + Joint2Offset);
    Joint3.write(theta3_traj_2[i] + Joint3Offset);
    delay(100);
  }

  delay(5000);

  // open gripper
  Gripper.write(GripperOpen);
  delay(2000);

  // loop through trajectory 2 backwards (return)
  for(int i = numPoints_2 - 1; i >= 0; i--)
  {
    Joint1.write(theta1_traj_2[i] + Joint1Offset);
    Joint2.write(theta2_traj_2[i] + Joint2Offset);
    Joint3.write(theta3_traj_2[i] + Joint3Offset);
    delay(100);
  }

  delay(5000);

  while(true);
}