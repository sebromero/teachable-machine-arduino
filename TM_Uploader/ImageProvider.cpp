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
#include <Arduino.h>

#if defined(ARDUINO_NICLA_VISION)
  #include "gc2145.h"
  GC2145 galaxyCore;
  Camera cam(galaxyCore);
  FrameBuffer fb;
#else
  #include <Arduino_OV767X.h>
#endif

#if defined(ARDUINO_NICLA_VISION)
  constexpr bool isGrayscale = false;
  // TODO Native grayscale mode doesn't work yet
  // There is some issue with the offsets
#else
  constexpr bool isGrayscale = false;
#endif

constexpr int inputImageWidth = 320;
constexpr int inputImageHeight = 240;

#if !defined(ARDUINO_NICLA_VISION)
  constexpr int inputImageChannels = isGrayscale ? 1 : 2; // 1 for grayscale, 2 for RGB565
  constexpr int inputImageDataSize = inputImageWidth * inputImageHeight * inputImageChannels;  // QVGA: 320x240 X 2 bytes per pixel (RGB565)
  byte inputImageData[inputImageDataSize];
#endif


bool cameraBegin() {
  #if defined(ARDUINO_NICLA_VISION)
    auto mode = isGrayscale ? CAMERA_GRAYSCALE : CAMERA_RGB565;
    if(!cam.begin(CAMERA_R320x240, mode)){
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
uint16_t readColorFromBuffer(const uint8_t* buffer, size_t bufferSize, int index) {
  if (index < 0 || index >= bufferSize - 1) {
    return 0; // Return black if out of bounds
  }
  
  uint8_t high_byte = buffer[index];
  uint8_t low_byte = buffer[index + 1];
  return ((uint16_t)high_byte << 8) | low_byte;
}

// Safely read a pixel value from buffer
float readPixelValue(const uint8_t* buffer, size_t bufferSize, int index, bool isGrayscale) {
  // Check bounds
  if (isGrayscale) {
    if (index < 0 || index >= bufferSize) {
      return 0.0f;
    }
    return static_cast<float>(buffer[index]);
  } else {
    if (index < 0 || index >= bufferSize - 1) {
      return 0.0f;
    }
    uint16_t color = readColorFromBuffer(buffer, bufferSize, index);
    return convertRGB565ToGrayscale(color);
  }
}

/**
 * Process the input image by cropping and converting to grayscale if needed
 * 
 * @param targetWidth Width of the output image
 * @param targetHeight Height of the output image
 * @param inputBuffer Input buffer containing the image data
 * @param inputBufferSize Size of the input buffer
 * @param outputBuffer Output buffer for processed image data
 * @param isGrayscale Whether the source image is already in grayscale format
 * @return true if processing was successful
 */
boolean processImage(int targetWidth, int targetHeight, uint8_t* inputBuffer, size_t inputBufferSize, uint8_t* outputBuffer, bool isGrayscale) {
  // Ensure image dimensions are valid
  if (targetWidth <= 0 || targetHeight <= 0 || outputBuffer == nullptr) {
    return false;
  }
  
  // For this implementation we assume square output
  const int targetSize = targetWidth;
  const int bytesPerPixel = isGrayscale ? 1 : 2;
  
  // Calculate bounds for source image sampling with proper margins
  // Keep a balanced margin on both sides of the image for a centered crop
  const int cropWidth = inputImageWidth - 120; // 40px margin on left, 80px on right
  const int cropStartX = 40;                 // Start 40px from left edge
  const int cropHeight = inputImageHeight;    // Use full height
  const int cropStartY = 0;                 // Start from top
  
  for (uint16_t y = 0; y < targetSize; y++) {
    for (uint16_t x = 0; x < targetSize; x++) {
      // Map destination coordinates to source coordinates for cropping
      // More precise mapping with proper boundaries
      uint16_t croppedX = cropStartX + (x * cropWidth / targetSize);
      uint16_t croppedY = cropStartY + (y * cropHeight / targetSize);
      
      // Sample 2x2 grid of pixels and average their values (when possible)
      float pixelValue = 0.0f;
      int validSamples = 0;
      
      // Process each pixel in the 2x2 grid
      for (int offsetY = 0; offsetY < 2; offsetY++) {
        for (int offsetX = 0; offsetX < 2; offsetX++) {
          uint16_t sourceX = croppedX + offsetX;
          uint16_t sourceY = croppedY + offsetY;
          
          // Ensure we're within source image bounds
          if (sourceX < 0 || sourceX >= inputImageWidth || 
              sourceY < 0 || sourceY >= inputImageHeight) {
            continue;
          }
          
          // Calculate buffer index based on pixel format
          uint32_t inputIndex = (sourceY * inputImageWidth + sourceX) * bytesPerPixel;
          
          // Additional bounds check for buffer access
          if (inputIndex >= 0 && inputIndex < inputBufferSize - (bytesPerPixel - 1)) {
            pixelValue += readPixelValue(inputBuffer, inputBufferSize, inputIndex, isGrayscale);
            validSamples++;
          }
        }
      }
      
      // Average the values from valid samples
      if (validSamples > 0) {
        pixelValue /= validSamples;
      }
      
      // The index of this pixel in our flat output buffer
      uint32_t outputIndex = y * targetWidth + x;
      
      // Ensure we're writing within bounds of the output buffer
      if (outputIndex < targetWidth * targetHeight) {
        outputBuffer[outputIndex] = static_cast<int8_t>(pixelValue);
      }
    }
  }
  
  return true;
}

// Get an image from the camera module
boolean getImage(int targetWidth, int targetHeight, int channels, uint8_t* outputBuffer){
  #if defined(ARDUINO_NICLA_VISION)
    if(cam.grabFrame(fb, 3000) != 0) return false;
    size_t bufferSize = cam.frameSize();
    return processImage(targetWidth, targetHeight, fb.getBuffer(), bufferSize, outputBuffer, isGrayscale);
  #else
    Camera.readFrame(inputImageData);
    return processImage(targetWidth, targetHeight, inputImageData, inputImageDataSize, outputBuffer, isGrayscale);
  #endif

}
