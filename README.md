# TV Backlight Project

The project was originally designed around a standard ESP32-CAM board (seen in the `esp32-cam` branch). However, considering that this is a student coursework project and that the university (UoL) may need to order components in batches, the development board was changed to the Freenove ESP32-S3 WROOM CAM. This board provides a more modern ESP32-S3 platform while keeping the same general project concept. Other main components, including the WS2812B LED strip, external 5V power supply, capacitor, resistor, and logic-level shifting circuit, remain unchanged.

At the current stage, the basic backlight functionality has been implemented in code. The system can initialize the camera, capture image frames, calculate edge colors, and prepare the corresponding LED strip output. Two advanced lighting features have also been added: gamma correction and smooth color transition.

So far, the project has only been verified through simulation. The simulation version can be found in the `simulation` branch. Actual hardware testing has not yet been completed, so the current code should be considered a software-ready prototype rather than a fully hardware-validated system.

## Basic Workflow

Power on  
1. Freenove ESP32-S3 WROOM CAM starts automatically 
2. Camera initializes  
3. Camera starts capturing screen frames  
4. Edge colors of the screen image are calculated  
5. Gamma correction is applied  
6. Smooth transition is applied between color updates  
7. WS2812B LED strip lights up according to the calculated colors  

Power off  
1. ESP32-S3 board and LED strip lose power  
2. LED strip turns off naturally