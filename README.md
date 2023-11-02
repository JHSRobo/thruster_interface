![Image](./img/logo.png)

11/1/23 Version 1.0:


**Contributors:** Nathan Peterson '24, James Randall '24

**Editors:** James Randall '24

**Approved by:** Pending

---
This repository contains the software, electronics, and documentation for Jesuit Robotics's ROS2 thruster interface module. This repository was created to fix a common issue with [PCA9685 Clock Drift](https://forums.adafruit.com/viewtopic.php?t=89102).

This package is part of a ROS2 environment. For information about integrating it into your stack, see [ROS2 Integration](#ros2-integration).

This documentation is intended to be read and understood by other highschoolers like us, but this module will be effective for any RANGER, PIONEER, or EXPLORER team, or just anyone trying to build a low-cost ROV. If you've got questions, reach out to randallj24@student.jhs.net. Feedback is always welcome.


## Why This Repo Exists:
During the 2022-23 MATE ROV Competition Season, Jesuit Robotics encountered an issue when using the [Adafruit PCA9685 I2C Servo Driver](https://www.adafruit.com/product/815) to control our [BlueRobotics ESCs](https://bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/). The servo controller has an internal clock with a theoretical frequency of 2.5MHz, but in practice this frequency can drift ~7%.

For most people, simply adjusting the frequency of the clock in software to match its actual value is enough. We did not like this solution, because:
- On top of our production ROV, we have several test-hardware instances, so keeping track of the different clockspeeds on each unique part when we often swap parts in and out is a complex ordeal
- This solution allows parts to be replaced more quickly. It frees you from having to tweak software whenever a PCA9685 is replaced.

## ROS2 Integration