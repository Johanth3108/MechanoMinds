# MechanoMinds

This repository contains the code for controlling an 2 wheel differential robot designed for specific tasks. The robot's control system is implemented using an Arduino board and various sensors and actuators.

### Contents

- **Code Files:**
  - `autonomous_robot_control.ino`: Arduino sketch file containing the main control logic and functions.
- **Libraries:**
  - `Servo.h`: Library for controlling servo motors.
  - `util/atomic.h`: Library for managing atomic operations.
- **Hardware Setup:**
  - This project assumes a specific hardware configuration. Ensure the proper connections of motors, encoders, ultrasonic sensors, and servo motors as defined in the code comments.

### Functionality

The robot is capable of performing several predefined tasks, each defined as a separate case in the `performance()` function. These tasks include:

1. Rotating 180 degrees.
2. Following the right wall at a safe distance until it reaches a certain distance from the front wall.
3. Rotating 90 degrees.
4. Following the right wall at a safe distance until it reaches a certain distance from the front wall.
5. Automatically aligning itself towards the right wall at a specific distance.
6. Actuating the robot arm.
7. Rotating 90 degrees.
8. Automatically aligning itself towards the right wall at a specific distance.
9. Rotating 90 degrees.
10. Automatically aligning itself towards the right wall at a specific distance.
11. Rotating -90 degrees.
12. Adjusting itself to move to a safe distance in front of the robot.

### Usage

1. **Hardware Setup:**
   - Connect the motors, encoders, ultrasonic sensors, and servo motors to the Arduino board following the pin configurations defined in the code.
2. **Upload Code:**
   - Upload the `autonomous_robot_control.ino` sketch to your Arduino board using the Arduino IDE or any compatible development environment.
3. **Run the Robot:**
   - Power on the robot and observe its behavior based on the predefined tasks.
4. **Task Customization:**
   - Modify the `performance()` function to define custom tasks or adjust existing ones according to your requirements.
5. **Debugging:**
   - Use serial output for debugging purposes. Uncomment `Serial.begin(9600);` in the `setup()` function if debugging is required.

### Contributors

This project was developed by Johanth PS, Darshan Bharath Muthuveeran Ramakrishnan as part of ACSE6502 Group Project at The University of Sheffield.

### Contact

For inquiries or support regarding this project, please contact johanth6600@gmail.com.
