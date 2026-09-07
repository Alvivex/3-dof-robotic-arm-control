# 3-DOF Robotic Arm Control

C++/Arduino and MATLAB code for controlling a 3-DOF robotic arm: forward & inverse kinematics, trajectory generation, and playback (pick & place example). Intended for hobbyist / research setups using hobby servos and an Arduino-compatible microcontroller.

## Key features
- Forward kinematics (MATLAB + Arduino example)
- Inverse kinematics (MATLAB functions + Arduino example)
- Trajectory generation in MATLAB (cubic polynomials)
- CSV export & conversion of joint trajectories to Arduino array format
- Arduino sketch to replay precomputed trajectories and operate a gripper

## Stack
- **Language(s):** C++ (Arduino sketches), MATLAB
- **Runtime:** Arduino (Uno/Nano/compatible) + MATLAB R2019a+ (or Octave with minor edits)
- **Notable libraries:** Arduino Servo.h (standard)

## Hardware / wiring (basic)
- Recommended microcontroller: Arduino Uno / Nano (or any with sufficient PWM pins)
- Servos:
  - Joint1 -> digital pin 2
  - Joint2 -> digital pin 3
  - Joint3 -> digital pin 4
  - Gripper -> digital pin 6
- Power:
  - Use a dedicated 5V servo power supply capable of the combined servo current (do NOT power many servos from the Arduino 5V regulator).
  - Connect grounds together (Arduino GND and servo supply GND).
- Link lengths used in code (meters):
  - L1 = 0.095
  - L2 = 0.1375 (some forward-kinematics MATLAB code uses 0.1775 — see notes)
  - L3 (end-effector) = 0.04
- Units: meters for links, degrees for servo angles.

IMPORTANT: Before running, verify and calibrate these lengths and servo offsets for your mechanical assembly.

## Files overview
- Forward_Kinematics_Arduino.ino — Simple Arduino sketch showing fixed joint positions for testing.
- Forward_Kinematics_MATLAB.m — MATLAB script using DH matrices to compute forward kinematics and plot configurations.
- Inverse_Kinematics_Arduino.ino — Arduino implementation of the inverse kinematics solver (computes joint angles for a given (x,y,z) target).
- Inverse_Kinematics_Code.m — MATLAB function `inverse_kinematics_RA` (returns two solution branches for theta2/theta3).
- Inverse_Kinematics_Potentiometer_Arduino.ino — partial example for potentiometer-based position input (incomplete stub).
- Trajectory_Planning_CSV_Generation_MATLAB.m — Generates cubic polynomial trajectories between two 3D points, computes IK for each point, and writes `joint_trajectory.csv`.
- Trajectory_Planning_CSV_to_Arduino_MATLAB.m — Reads `joint_trajectory.csv` and prints C-style float arrays you can paste into an Arduino sketch.
- Trajectory_Planning_Arduino.ino — Arduino sketch that contains precomputed theta arrays (trajectory playback, gripper control, and example pick/place cycle).

## Quickstart: run the pick-and-place trajectory (MATLAB → Arduino)
1. Mechanical + wiring:
   - Mount servos, connect to pins 2,3,4,6 as above.
   - Connect servo power to a suitable 5V supply and common ground with Arduino.
   - Calibrate servo neutral positions and confirm physical joint range.

2. Generate a trajectory in MATLAB:
   - Open `Trajectory_Planning_CSV_Generation_MATLAB.m`.
   - Edit initial/final positions (x0,y0,z0 and xf,yf,zf) and link lengths (L1,L2,L3) to match your robot.
   - Run the script: it will create `joint_trajectory.csv`.

3. Convert CSV to Arduino arrays:
   - Run `Trajectory_Planning_CSV_to_Arduino_MATLAB.m` (in same folder as CSV).
   - Copy the printed arrays (`theta1_traj`, `theta2_traj`, `theta3_traj` and `numPoints`) into `Trajectory_Planning_Arduino.ino` replacing the example arrays, or create a new Arduino sketch using them.

4. Upload to Arduino:
   - Open `Trajectory_Planning_Arduino.ino` in the Arduino IDE.
   - Set board & port, then upload.
   - Ensure the servo power supply is connected before running the sketch (to avoid brownouts).

5. Run:
   - After upload, the sketch will play trajectories, operate the gripper, and loop through pick/place movements as implemented.

## Using inverse kinematics
- MATLAB:
  - Call the function: [theta1, theta2, theta3] = inverse_kinematics_RA(x,y,z,L1,L2,L3)
  - Returns theta2 and theta3 as 1x2 arrays for the two elbow solutions. Units are degrees.
- Arduino:
  - `Inverse_Kinematics_Arduino.ino` demonstrates calling a local `IK_3R_robot(...)` function with a sample (x,y,z). Edit target and link lengths, then upload.

## Notes, calibrations & gotchas
- Link-length mismatch: some MATLAB forward-kinematics code uses L2 = 0.1775, while inverse-kinematics and trajectory scripts use L2 = 0.1375. Make sure all scripts and Arduino sketches use consistent link lengths for accurate motion.
- Servo offsets vary between sketches (Joint1Offset, Joint2Offset, Joint3Offset). These are hardware-specific calibrations — update offsets to match your servo horn mounting and neutral angles.
- Z axis convention: many scripts treat negative z as "down". Check the plotting and the `set(gca, 'ZDir', 'reverse')` in MATLAB forward-kinematics (visualization uses a reversed Z).
- Workspace / reachability: IK functions check reachability and will error or print "Target outside workspace" if the target is unreachable. Validate points before commanding servos.
- Safety: keep hands clear of moving parts, use low-power testing, and consider mechanical end stops to prevent over-rotation.

## Troubleshooting
- Servos jitter / brownout: use a proper external 5V supply and common ground.
- Angles don't match physical pose: adjust JointXOffset values in the Arduino sketch and re-run.
- Trajectory playback too fast/slow: adjust `delay()` in Arduino or change the dt/time settings in the MATLAB trajectory generator (tf, dt).
- MATLAB errors: ensure the file paths are correct and that the working directory contains `joint_trajectory.csv` when running the converter script.

## Example parameters used in repo
- Example IK target in `Inverse_Kinematics_Arduino.ino`: x = 0.1, y = 0.1, z = -0.2 (meters)
- Example link lengths used in IK/trajectory scripts: L1 = 0.095, L2 = 0.1375, L3 = 0.04
- Default servo pins: 2, 3, 4, 6
- Default unit: meters for link lengths, degrees for servo angles.

## Contributing
Contributions and improvements are welcome. Consider:
- Adding a consistent configuration file (e.g., `config.h` or `config.m`) for link lengths and offsets so MATLAB and Arduino use the same parameters.
- Adding wiring diagrams, photos, and a hardware bill-of-materials (BOM).
- Adding a license (none is included currently).

## License
No license file is included in the repository. If you want this project to be reusable, consider adding an MIT or similar permissive license.

## Contact
Maintainer: repository owner (see GitHub user: Alvivex)
