/* Copyright 2021 Google LLC All Rights Reserved.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
  ==============================================================================
  */


/*
  OV767X - Camera Capture Raw Bytes

  This sketch reads a frame from the OmniVision OV7670 camera
  and writes the bytes to the Serial port. Use the Procesing
  sketch in the extras folder to visualize the camera output.

  Circuit:
    - Arduino Nano 33 BLE board
    - OV7670 camera module:
      - 3.3 connected to 3.3
      - GND connected GND
      - SIOC connected to A5
      - SIOD connected to A4
      - VSYNC connected to 8
      - HREF connected to A1
      - PCLK connected to A0
      - XCLK connected to 9
      - D7 connected to 4
      - D6 connected to 6
      - D5 connected to 5
      - D4 connected to 3
      - D3 connected to 2
      - D2 connected to 0 / RX
      - D1 connected to 1 / TX
      - D0 connected to 10

*/
#include <Arduino.h>
#include "ImageProvider.h"

const int kNumCols = 96;
const int kNumRows = 96;
const int kNumChannels = 1;
const int bytesPerFrame = kNumCols * kNumRows;

 // QVGA: 320x240 X 2 bytes per pixel (RGB565)
uint8_t data[kNumCols * kNumRows * kNumChannels];

void blinkLED(int pin, int amount, int delayTime) {
  for (int i = 0; i < amount; i++) {
    digitalWrite(pin, !digitalRead(pin));
    delay(delayTime);
    digitalWrite(pin, !digitalRead(pin));
    delay(delayTime);
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  blinkLED(LED_BUILTIN, 2, 400);
  
  Serial.begin(9600);
  while (!Serial);
  cameraBegin();
}

void loop() {
  getImage(kNumCols, kNumRows, kNumChannels, data);
  Serial.write(data, bytesPerFrame);
}
