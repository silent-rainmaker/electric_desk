# electric_desk
Arduino/ESP32 based system to control electric desk

As my Maidesite standing desk stopped working I decided to build my own control unit.

Components:
Arduino Nano

H bridge BTS7960 https://www.handsontec.com/dataspecs/module/BTS7960%20Motor%20Driver.pdf

Original DC motor with position feedback from hall sensors (JXET-18-50, 24V, 5A, 120W)

DC motor plug pinout:
M1   M2    H1
H2   GND   +5V

![motor_plug](https://github.com/silent-rainmaker/electric_desk/assets/114312785/93f28b5b-f1b8-48cf-9db0-b6a5804daaa2)

