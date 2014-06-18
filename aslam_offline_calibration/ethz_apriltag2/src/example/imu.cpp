// Example of reading from ArduIMU (v3) via serial port
// Michael Kaess, May 2013

#include <iostream>
#include <string>
#include <stdio.h>
#include <math.h>

#include "Serial.h"

using namespace std;


void parse(string s) {
  // note that ex (the rotation matrix) and mg? (the magnetormeter
  // readings) below are optional, depending on how the ArduIMU was
  // configured

  float version;            // ArduIMU code version
  float gyroX, gyroY, gyroZ; // raw gyroscope values
  float accelX, accelY, accelZ; // raw accelerometer values
  //  int ex[9];                // rotation matrix (scaled by 10^7)
  float roll, pitch, yaw;   // Euler angles
  int imuh;                 // IMU health
  //  float mgx, mgy, mgz, mgh; // magnetometer readings
  int tow;                  // GPS time

  //  cout << s;

  // try to parse the line
  int num = sscanf(s.c_str(),
                   //"!!!VER:%g,AN0:%g,AN1:%g,AN2:%g,AN3:%g,AN4:%g,AN5:%g,EX0:%i,EX1:%i,EX2:%i,EX3:%i,EX4:%i,EX5:%i,EX6:%i,EX7:%i,EX8:%i,RLL:%g,PCH:%g,YAW:%g,IMUH:%i,TOW:%i***",
                   "!!!VER:%g,AN0:%g,AN1:%g,AN2:%g,AN3:%g,AN4:%g,AN5:%g,RLL:%g,PCH:%g,YAW:%g,IMUH:%i,TOW:%i",
                   //"!!!VER:%g,AN0:%g,AN1:%g,AN2:%g,AN3:%g,AN4:%g,AN5:%g,RLL:%g,PCH:%g,YAW:%g,IMUH:%i,MGX:%g,MGY:%g,MGZ:%g,MGH:%g,TOW:%i",
                   &version, &gyroX, &gyroY, &gyroZ, &accelX, &accelY, &accelZ,
                   //                   &ex[0], &ex[1], &ex[2], &ex[3], &ex[4], &ex[5], &ex[6], &ex[7], &ex[8],
                   &roll, &pitch, &yaw,
                   &imuh,
                   //                   &mgx, &mgy, &mgz, &mgh,
                   &tow);

  // did we read the correct number of entries, or did the line contain other information?
  if (num==12 || num==16 || num==21) {
    cout << "Euler angles: " << yaw << "," << pitch << "," << roll << endl;
  } else {
    //    cout << "Could not parse string " << num << endl;
  }

}


int main() {

  Serial serial;
  serial.open("/dev/ttyUSB0", 38400); // might need to change to your USB port

  // read and parse one line at a time
  while (true) {
    string s = serial.readBytesUntil('\n');
    parse(s);
  }
  
  return 0;
}
