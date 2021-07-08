# Smart Intersection Prototype
Embedded software for a smart intersection prototype. Final project for MIE366 (Microprocessors and Embedded Microcontrollers). Code for traffic controller written in Python for Raspberry Pi, 
while code for small scale traffic simulation written for Arduino.

The repository includes the following files:
- documentation
  - block_diagram: system-level block diagram of the smart intersection prototype 
- simulation:
  - simulation.ino: Arduino code to actuate servo motors and traffic light LEDs based on traffic level
- systems_test:
  - systems_test.ino: Arduino code used to test and evaluate system components
- traffic_detector: 
  -  traffic_detector.py: Python code used to interface with camera sensor and run computer vision traffic detection algorithm