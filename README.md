## Simulation Environment

### Overview

The project is simulated using "Wokwi through Visual Studio Code".

The physical system uses a "Freenove ESP32-S3-WROOM CAM", an "OV2640 camera", and a "WS2812B LED strip". However, Wokwi does not provide an exact simulation model for the Freenove board or the OV2640 camera used in this project.

Therefore, the simulation uses the supported "ESP32-S3-DevKitC-1" model. And since the camera cannot be simulated, the program generates synthetic RGB565 colour frames. These frames are stored in a simulated frame buffer and processed by the same edge-sampling functions used by the physical system.

### Simulation Workflow

The real hardware workflow is:
1. OV2640 camera captures a 160 × 120 RGB565 frame
2. Frame is copied into the camera buffer
3. Pixels near the four screen edges are sampled
4. An average colour is calculated for each LED
5. The colours are sent to the WS2812B LED strip


The Wokwi simulation replaces the camera input with a generated RGB565 frame:
1. Generate a synthetic 160 × 120 RGB565 frame
2. Store the frame in simulationFrame
3. Set readBuffer to the simulated frame
4. Run the original edge-sampling functions
5. Calculate the colour for each LED
6. Display the result on the virtual LED strips

### Synthetic Colour Frame

The generated simulation frame represents four coloured screen edges:

                 Top edge
          ┌──────────────────┐
          │                  │
Left edge │                  │ Right edge
          │                  │
          └──────────────────┘
               Bottom edge

If the program runs correctly, the edge colours wil rotate periodically so that the simulation can verify that:

- the main loop is running;
- the simulated frame is updating;
- LED colours are being recalculated;
- `FastLED.show()` is refreshing the virtual LED strips.

Some colour mixing may appear near the four corners. This is expected because each LED samples multiple pixels:

```cpp
#define EDGE_SAMPLES 2
#define INWARD_SAMPLES 4
```

Each LED therefore averages up to eight pixels. LEDs near a corner may sample pixels belonging to two neighbouring edge regions.


### Virtual LED Layout

The physical system uses 258 LEDs (preliminary assumption will be adjusted according to the TV dimensions during the actual experiment):
Right side  = 45 LEDs
Top side    = 84 LEDs
Left side   = 45 LEDs
Bottom side = 84 LEDs

The LED data order is: Right → Top → Left → Bottom

In `diagram.json`, the LEDs are represented by four virtual WS2812-compatible strips arranged in a rectangular layout.

The data connections in simulation are:
ESP32-S3 GPIO14
        ↓
Right strip DIN
        ↓ DOUT
Top strip DIN
        ↓ DOUT
Left strip DIN
        ↓ DOUT
Bottom strip DIN

### What the Simulation Can Test

The simulation can test:
1. ESP32-S3 program execution;
2. GPIO14 LED data output;
3. FastLED compatibility;
4. control of 258 virtual LEDs;
5. the right-to-top-to-left-to-bottom LED order;
6. LED index mapping;
7. RGB565 frame storage;
8. RGB565-to-RGB conversion;
9. screen-edge pixel sampling;
10. average colour calculation;
11. compatibility with `strip_index.h`;
12. gamma correction;
13. smooth colour transitions;

### Simulation Limitations

The Wokwi environment cannot accurately test:
1. functions related to the camera module;
2. real PSRAM performance;
3. camera and LED dual-core timing;
4. physical WS2812B signal quality;
5. the 74HCT125 level shifter;
6. external 5V power delivery;
7. voltage drop along the LED strip;
8. electrical noise or signal reflection;

The simulation only verifies the software logic and LED control sequence; it does not represent the actual electrical conditions of the physical circuit. In the real system, a series resistor is required on the LED data line to reduce signal ringing and electrical noise, while a capacitor should be connected across the 5V supply and ground to stabilise the power supply and absorb voltage fluctuations. Therefore, successful operation in Wokwi does not guarantee that the physical circuit will operate reliably without these additional components. In addition, the maximum current capacity of the power supply is another major factor in the physical implementation.