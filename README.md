# TV Backlight Project

This project is an ESP32-CAM based ambient TV backlight prototype.  
The system uses the camera to capture the screen image, samples colors from the edge regions of the image, and drives a WS2812 LED strip to create a dynamic backlight effect behind the screen.

## Basic Workflow

Power on
1. ESP32-CAM starts automatically
2. Camera initializes
3. Camera starts capturing screen frames
4. Edge colors of the screen image are calculated
5. WS2812B LED strip lights up according to the calculated colors

Power off
1. ESP32-CAM and LED strip lose power
2. LED strip turns off