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

// Defnining Joint Angles
int Joint1Angle;
int Joint2Angle;
int Joint3Angle;
int GripperOpen = 60;
int GripperClose = 180; // Maximum grip tightness

// Joint Angle Offsets - KEEP
int Joint1Offset = 95;
int Joint2Offset = 2;
int Joint3Offset = 23;

// inverse kinematics calculations:
void IK_3R_robot(
  float x, float y, float z,
  float L1, float L2,
  float &theta1_deg,
  float theta2_deg[2],
  float theta3_deg[2]
)
{
  // accounting for the end-effector offset
  const float L3 = 0.04;
  float L2eff = L2 + L3;

  // theta1 radians
  float theta1 = atan2(y, x);

  // definining additional parameters
  float r = sqrt(x*x + y*y);
  float rho = sqrt(r*r + z*z);
  float alpha = atan2(z, r);
  float D = (r*r + z*z + L2eff*L2eff - L1*L1) / (2.0 * L2eff);

  // reachability check
  if (fabs(D / rho) > 1.0) {
    Serial.println("Target outside workspace");
    return;
  }

  float acos_term = acos(D / rho);

  float phi[2];
  phi[0] = alpha + acos_term;   // elbow-up solution
  phi[1] = alpha - acos_term;   // elbow-down solution

  float theta2[2];
  float theta3[2];

  // iteratively accounting for multiplicity of solutions (2 in total)
  for (int i = 0; i < 2; i++) {

    theta2[i] = atan2(
      L2eff * sin(phi[i]) - z,
      r - L2eff * cos(phi[i])
    );

    theta3[i] = theta2[i] + phi[i];
  }

  // radians to degrees
  theta1_deg = theta1 * 180.0 / PI;

  for (int i = 0; i < 2; i++) {
    theta2_deg[i] = theta2[i] * 180.0 / PI;
    theta3_deg[i] = theta3[i] * 180.0 / PI;
  }
}

void setup()
{

Serial.begin(9600);

// Attach joint pins
Joint1.attach(Joint1Pin);
Joint2.attach(Joint2Pin);
Joint3.attach(Joint3Pin);
Gripper.attach(GripperPin);

// defining paramters for IK function call
float L1 = 0.095; 
float L2 = 0.1375;

float x = 0.1;
float y = 0.1;
float z = -0.2;

float theta1;
float theta2[2];
float theta3[2];

IK_3R_robot(x, y, z, L1, L2, theta1, theta2, theta3);

// displaying soluti0on multiplicity
Serial.println("Solution 1:");
Serial.println(theta1);
Serial.println(theta2[0]);
Serial.println(theta3[0]);
Serial.println("Solution 2:");
Serial.println(theta1);
Serial.println(theta2[1]);
Serial.println(theta3[1]);

// Use solution 2 (elbow up)
Joint1Angle = theta1;
Joint2Angle = theta2[0];
Joint3Angle = theta3[0];


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

