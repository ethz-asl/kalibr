/**
 * @file Serial.h
 * @brief Simple serial interface, for example to talk to Arduino.
 * @author: Michael Kaess
 */

#pragma once

#include <string>


class Serial {

  int m_serialPort; // file description for the serial port
  
public:

  Serial() : m_serialPort(-1) {}

  // open a serial port connection
  void open(const std::string& port, int rate = 115200);

  // read a single character
  int read() const;

  // read until special character up to a maximum number of bytes
  std::string readBytesUntil(unsigned char until, int length = 300);

  // send a string
  void print(std::string str) const;

  // send an integer
  void print(int num) const;

  // send a double
  void print(double num) const;

  // send a float
  void print(float num) const;

};
