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


#include "ImageProvider.h"


/*
  OV767X - Camera Capture Raw Bytes

  This sketch reads a frame from the OmniVision OV7670 camera
  and writes the bytes to the Serial port. Use the Processing
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

  This example code is in the public domain.
*/
#include <Arduino.h>

#if defined(ARDUINO_NICLA_VISION)
  #include "gc2145.h"
  GC2145 galaxyCore;
  Camera cam(galaxyCore);
  FrameBuffer fb;
#else
  #include <Arduino_OV767X.h>
#endif

constexpr int kCaptureWidth = 320;
constexpr int kCaptureHeight = 240;
constexpr int capDataLen = kCaptureWidth * kCaptureHeight * 2;

#if defined(ARDUINO_NICLA_VISION)
  uint8_t* captured_data;
#else
  byte captured_data[kCaptureWidth * kCaptureHeight * 2]; // QVGA: 320x240 X 2 bytes per pixel (RGB565)
#endif

bool cameraBegin() {
  #if defined(ARDUINO_NICLA_VISION)
    if(!cam.begin(CAMERA_R320x240, CAMERA_RGB565)){
      return false;
    }
    cam.setVerticalFlip(true);
    // cam.setHorizontalMirror(true);
    return true;
  #else
    return Camera.begin(QVGA, RGB565, 1);
  #endif
}

// Convert RGB565 color to grayscale value
float convertRGB565ToGrayscale(uint16_t color) {
  // Extract the color values (5 red bits, 6 green, 5 blue)
  uint8_t r = ((color & 0xF800) >> 11) * 8;
  uint8_t g = ((color & 0x07E0) >> 5) * 4;
  uint8_t b = ((color & 0x001F) >> 0) * 8;
  
  // Convert to grayscale by calculating luminance
  // See https://en.wikipedia.org/wiki/Grayscale for magic numbers
  return (0.2126 * r) + (0.7152 * g) + (0.0722 * b);
}

// Read RGB565 color from buffer at specified index
uint16_t readColorFromBuffer(const uint8_t* buffer, int index) {
  if (index < 0 || index >= capDataLen - 1) {
    return 0; // Return black if out of bounds
  }
  
  uint8_t high_byte = buffer[index];
  uint8_t low_byte = buffer[index + 1];
  return ((uint16_t)high_byte << 8) | low_byte;
}

// Crop image and convert it to grayscale
boolean processImage(int image_width, int image_height, uint8_t* image_data) {
  const int imgSize = 96;
  
  for (int y = 0; y < imgSize; y++) {
    for (int x = 0; x < imgSize; x++) {
      // Map destination coordinates to source coordinates for cropping
      int currentCapX = floor(map(x, 0, imgSize, 40, kCaptureWidth - 80));
      int currentCapY = floor(map(y, 0, imgSize, 0, kCaptureHeight));
      
      // Calculate indices for 2x2 pixel sampling
      int indices[4] = {
        (currentCapY * kCaptureWidth + currentCapX) * 2,                   // Top-left
        (currentCapY * kCaptureWidth + currentCapX + 1) * 2,               // Top-right
        ((currentCapY + 1) * kCaptureWidth + currentCapX) * 2,             // Bottom-left
        ((currentCapY + 1) * kCaptureWidth + currentCapX + 1) * 2          // Bottom-right
      };
      
      // Sample 2x2 grid of pixels and average their grayscale values
      float gray_value = 0.0f;
      int valid_samples = 0;
      
      for (int i = 0; i < 4; i++) {
        if (indices[i] >= 0 && indices[i] < capDataLen - 1) {
          uint16_t color = readColorFromBuffer(captured_data, indices[i]);
          gray_value += convertRGB565ToGrayscale(color);
          valid_samples++;
        }
      }
      
      // Average the grayscale values from valid samples
      if (valid_samples > 0) {
        gray_value /= valid_samples;
      }
      
      // The index of this pixel in our flat output buffer
      int output_index = y * image_width + x;
      image_data[output_index] = static_cast<int8_t>(gray_value);
    }
  }
  
  return true;
}

// Get an image from the camera module
boolean getImage(int image_width, int image_height, int channels, uint8_t* image_data) {
  #if defined(ARDUINO_NICLA_VISION)
    if(cam.grabFrame(fb, 3000) != 0) return false;
    captured_data = fb.getBuffer();
    // size_t bufferSize = cam.frameSize();
  #else
    Camera.readFrame(captured_data);
  #endif

  return processImage(image_width, image_height, image_data);
}
