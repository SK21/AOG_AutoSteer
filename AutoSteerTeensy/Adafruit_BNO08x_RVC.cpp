/*!
 *  @file Adafruit_BNO08x_RVC.cpp
 *
 *  @mainpage Adafruit BNO08x RVC A simple library to use the UART-RVC mode of
 * the BNO08x sensors from Hillcrest Laboratories
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Library for the BNO08x_RVC A simple library to use
 * the UART-RVC mode of the BNO08x sensors from Hillcrest Laboratories
 *
 * 	This is a library for the Adafruit BNO08x_RVC breakout:
 * 	https://www.adafruit.com/product/4754
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 *  This library depends on the Adafruit BusIO library
 *
 *  This library depends on the Adafruit Unified Sensor library
 *
 *  @section author Author
 *
 *  Bryan Siepert for Adafruit Industries
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_BNO08x_RVC.h"

/**
 * @brief Construct a new Adafruit_BNO08x_RVC::Adafruit_BNO08x_RVC object
 *
 */
Adafruit_BNO08x_RVC::Adafruit_BNO08x_RVC(void) {}

/**
 * @brief Destroy the Adafruit_BNO08x_RVC::Adafruit_BNO08x_RVC object
 *
 */
Adafruit_BNO08x_RVC::~Adafruit_BNO08x_RVC(void) {}

/*!
 *  @brief  Setups the hardware
 *  @param  theSerial
 *          Pointer to Stream (HardwareSerial/SoftwareSerial) interface
 *  @return True
 */
bool Adafruit_BNO08x_RVC::begin(Stream *theSerial) {
  serial_dev = theSerial;
  return true;
}

/**
 * @brief Get the next available gyro and acceleration data from the sensor
 *
 * @param RVC_Data pointer to a BNO08x_RVC_Data struct to hold the measurements
 * @return true: success false: failure
 */
bool Adafruit_BNO08x_RVC::read(BNO08x_RVC_Data *RVC_Data) {
  // Each BNO085 report is prefixed with a header 0xAAAA
  byte headerByte = 0xAA; // Because the first and second bytes of the header are identical, we only define one byte to minimize processing time.
  uint8_t buffer[17]; // Each data packet is 19 bytes long, 2 of which are the header. 

  if (!RVC_Data) {
    return false;
  }

  if (!serial_dev->available()) {
    Serial.println("No bytes available to read.");
    return false;
  }

  // Are there at least 19 bytes in the buffer?
  if (serial_dev->available() < 19) {
    // Nope. 
    return false;
  }
  else { 
    // There's at least 19 bytes in the buffer.     
    //int numOfGoodPackets = 0; // For grins, keep track of how many good packets we find in the buffer.
    // Get the most recent data packet and discard any older data
    while(serial_dev->available() >= 19) {
      // Look for the first header byte
      if(serial_dev->peek() == headerByte) {
        serial_dev->read(); // Consume the first header byte

        // Is the second byte of the header correct?
        bool validHeader = true;
        if(serial_dev->peek() != headerByte) {
          validHeader = false;
        }
        else { 
          serial_dev->read(); // Consume the header byte
        }

        // If the header is valid, read the rest of the packet
        if(validHeader) {
          for(int i = 0; i < 17; i++) {
            buffer[i] = serial_dev->read();
          }
          //numOfGoodPackets++;
        }
      } 
      else {
        serial_dev->read(); // Discard the non-header byte
      }
    } // end while()
    //Serial.println("Using the most recent data from the " + String(numOfGoodPackets) + " data packets that were in the buffer.");


    // Validate and Process the Data *****************************/ 

    // Checksum
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 16; i++) {
      sum += buffer[i];
    }
    if (sum != buffer[16]) {
      return false;
    }

    // The data comes in endian'd, this solves it so it works on all platforms
    int16_t buffer_16[6];

    for (uint8_t i = 0; i < 6; i++) {

      buffer_16[i] = (buffer[1 + (i * 2)]);
      buffer_16[i] += (buffer[1 + (i * 2) + 1] << 8);
    }
    RVC_Data->yaw = (float)buffer_16[0] * DEGREE_SCALE;
    RVC_Data->pitch = (float)buffer_16[1] * DEGREE_SCALE;
    RVC_Data->roll = (float)buffer_16[2] * DEGREE_SCALE;

    RVC_Data->x_accel = (float)buffer_16[3] * MILLI_G_TO_MS2;
    RVC_Data->y_accel = (float)buffer_16[4] * MILLI_G_TO_MS2;
    RVC_Data->z_accel = (float)buffer_16[5] * MILLI_G_TO_MS2;
    return true;
  }
}
