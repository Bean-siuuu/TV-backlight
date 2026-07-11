# TV Backlight Project

This project is an ESP32-CAM based ambient TV backlight prototype.  
The system uses the camera to capture the screen image, samples colors from the edge regions of the image, and drives a WS2812 LED strip to create a dynamic backlight effect behind the screen.

## Basic Workflow

Power on
→ ESP32-CAM starts automatically
→ Camera initializes
→ Camera starts capturing screen frames
→ Edge colors of the screen image are calculated
→ WS2812B LED strip lights up according to the calculated colors

Power off
→ ESP32-CAM and LED strip lose power
→ LED strip turns off