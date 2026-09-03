#include <Arduino.h>
#include <string.h>

#include "FastLED.h"

#include "esp_camera.h"
#include <esp_log.h>
#include "sensor.h"

#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

//-------------------------------------
// LED strip settings
#define LED_PIN 14 // no #CS(GPIO14) required

constexpr uint16_t MaxLEDS = 300; // which is 5m of LED strips at most
enum class StartEdge : uint8_t {
	top,
	bottom,
	right,
	left
}; // to choose the starting edge
enum class StripDirection : uint8_t{
	clockwise,
	counterclockwise
}; // installed direction of the strip
struct LedLayoutConfig {
    uint16_t sideCount;
    uint16_t horizontalCount;
    StartEdge startEdge;
    StripDirection direction;
};
LedLayoutConfig defaultV = {
    45,
    84,
    StartEdge::right,
    StripDirection::counterclockwise
}; // default layout of the LED strip, changeable
uint16_t totalCount = (defaultV.sideCount + defaultV.horizontalCount) * 2;
#define RIGHT_START 0
#define TOP_START (RIGHT_START + defaultV.sideCount)
#define LEFT_START (TOP_START + defaultV.horizontalCount)
#define BOTTOM_START (LEFT_START + defaultV.sideCount) // Right to top to left to bottom
uint16_t rightStart = 0;
uint16_t topStart = 0;
uint16_t leftStart = 0;
uint16_t bottomStart = 0;

#define EDGE_SAMPLES 2
#define INWARD_SAMPLES 4 // 8 pixels for each LED

#define CHIPSET WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS 212
float GAMMA_R = 2.0f;
float GAMMA_G = 2.0f;
float GAMMA_B = 2.0f;
CRGB leds[MaxLEDS]; // arry for store the RGB data for strip
CRGB previousLeds[MaxLEDS]; // Store the previously displayed LED colors for smooth transition
uint8_t smoothAmount = 128; //Portion of previousLeds

//Camera settings
#define IMG_WIDTH 160
#define IMG_HEIGHT 120
#define FRAME_PIXELS (IMG_WIDTH * IMG_HEIGHT)
#define FRAME_BYTES  (FRAME_PIXELS * sizeof(uint16_t))

// Double buffer for dual-core
uint16_t *bufferA = nullptr;
uint16_t *bufferB = nullptr;
uint16_t *writeBuffer = nullptr;
uint16_t *readBuffer = nullptr;

volatile bool newFrameReady = false;

SemaphoreHandle_t frameMutex = nullptr;

// FreeRTOS task handles
TaskHandle_t cameraTaskHandle = nullptr;

// Freenove ESP32-S3 WROOM CAM PIN Map
#define CAM_PIN_PWDN  -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK  15
#define CAM_PIN_SIOD  4
#define CAM_PIN_SIOC  5
#define CAM_PIN_D7    16  // CSI_Y9
#define CAM_PIN_D6    17  // CSI_Y8
#define CAM_PIN_D5    18  // CSI_Y7
#define CAM_PIN_D4    12  // CSI_Y6
#define CAM_PIN_D3    10  // CSI_Y5
#define CAM_PIN_D2    8   // CSI_Y4
#define CAM_PIN_D1    9   // CSI_Y3
#define CAM_PIN_D0    11  // CSI_Y2
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF  7
#define CAM_PIN_PCLK  13

// Log module tag
static const char *TAG = "ambilight";

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 10000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
 
    .pixel_format = PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
    
    .frame_size = FRAMESIZE_QQVGA,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .fb_count = 1,       //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .fb_location = CAMERA_FB_IN_PSRAM,
	.grab_mode = CAMERA_GRAB_LATEST, //For this project skipping some frames is acceptable, but getting the latest frame is important.
    
};

//------------------------------------------------------
// put function declarations here:
bool initFrameBuffers();
bool initCamera();
void initLedStrip();
void calculateEdgeDirection();

void cameraTask(void *pvParameters);
void copyFrameToBuffer(camera_fb_t *fb);
void swapFrameBuffers(); // read and write

void calculateLEDColors();
void calculateRight();
void calculateTop();
void calculateLeft();
void calculateBottom();
void blackBarDetection();
void whiteBalance();
int mapLedToPixel(uint16_t ledIndex, uint16_t ledCount, uint16_t pixelLength);
CRGB sampleAverageColor(
    int baseX,
    int baseY,
    int inwardDx,
    int inwardDy,
    int tangentDx,
    int tangentDy
);
CRGB rgb565ToCRGB(uint16_t pixel);

void gammaCorrection();
void smoothTransition();

//---------------------------------------------------------------------------------
bool initFrameBuffers() {
	if (psramFound())
	{
		ESP_LOGI(TAG, "PSRAM found. Allocating frame buffers in PSRAM.");
		bufferA = (uint16_t*)heap_caps_malloc(
			FRAME_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
		);
		bufferB = (uint16_t*)heap_caps_malloc(
			FRAME_BYTES,
			MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
		);
	}
	else
	{
		ESP_LOGW(TAG, "PSRAM not found. Allocating frame buffers in internal RAM.");

		bufferA = (uint16_t*)heap_caps_malloc(
			FRAME_BYTES,
			MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
		);

		bufferB = (uint16_t*)heap_caps_malloc(
			FRAME_BYTES,
			MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
		);
	}

	if (bufferA == nullptr || bufferB == nullptr)
	{
		ESP_LOGE(TAG, "Frame buffer allocation failed.");

		if (bufferA != nullptr)
		{
			heap_caps_free(bufferA);
			bufferA = nullptr;
		}

		if (bufferB != nullptr)
		{
			heap_caps_free(bufferB);
			bufferB = nullptr;
		}

		writeBuffer = nullptr;
		readBuffer = nullptr;

		return false;
	}

	memset(bufferA, 0, FRAME_BYTES);
	memset(bufferB, 0, FRAME_BYTES);

	writeBuffer = bufferA;
	readBuffer = bufferB;

	ESP_LOGI(TAG, "Frame buffers initialized successfully.");
	ESP_LOGI(TAG, "Free heap: %u", (unsigned int)ESP.getFreeHeap());
	ESP_LOGI(TAG, "PSRAM size: %u", (unsigned int)ESP.getPsramSize());
	ESP_LOGI(TAG, "Free PSRAM: %u", (unsigned int)ESP.getFreePsram());

	return true;
}
bool initCamera() {
	ESP_LOGI(TAG, "Initializing camera...");
	esp_err_t err = esp_camera_init(&camera_config);

	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Camera initialization failed. Error code: 0x%x", err);
		return false;
	}
	ESP_LOGI(TAG, "Camera driver initialized.");
	sensor_t* sensor = esp_camera_sensor_get();

	if (sensor == nullptr)
	{
		ESP_LOGE(TAG, "Failed to get camera sensor.");
		return false;
	}

	// Basic sensor settings.
	// Keep them simple for the first version.
	sensor->set_framesize(sensor, FRAMESIZE_QQVGA);
	sensor->set_brightness(sensor, 0);
	sensor->set_contrast(sensor, 0);
	sensor->set_saturation(sensor, 0);

	// Test whether the camera can capture one frame.
	camera_fb_t* testFrame = esp_camera_fb_get();

	if (testFrame == nullptr)
	{
		ESP_LOGE(TAG, "Camera test capture failed.");
		return false;
	}

	ESP_LOGI(TAG, "Camera test frame captured.");
	ESP_LOGI(TAG, "Frame width: %u", (unsigned int)testFrame->width);
	ESP_LOGI(TAG, "Frame height: %u", (unsigned int)testFrame->height);
	ESP_LOGI(TAG, "Frame length: %u bytes", (unsigned int)testFrame->len);

	esp_camera_fb_return(testFrame);

	ESP_LOGI(TAG, "Camera initialized successfully.");

	return true;
}
void initLedStrip() {
	ESP_LOGI(TAG, "Initializing LED strip...");

	FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(
		leds,
		MaxLEDS
	);
	FastLED.setBrightness(BRIGHTNESS);
	FastLED.clear();
	fill_solid(previousLeds, MaxLEDS, CRGB::Black); // Start from black to the first frame captured
	FastLED.show();

	ESP_LOGI(TAG, "LED strip initialized.");
	ESP_LOGI(TAG, "LED pin: %d", LED_PIN);
	ESP_LOGI(TAG, "Number of LEDs: %d", totalCount);
	ESP_LOGI(TAG, "Brightness: %d", BRIGHTNESS);
}
void calculateEdgeDirection()
{
	if (defaultV.direction == StripDirection::counterclockwise){
		switch (defaultV.startEdge){
        case StartEdge::right:
            rightStart = 0;
            topStart = rightStart + defaultV.sideCount;
            leftStart = topStart + defaultV.horizontalCount;
            bottomStart = leftStart + defaultV.sideCount;
            break;

        case StartEdge::top:
            topStart = 0;
            leftStart = topStart + defaultV.horizontalCount;
            bottomStart = leftStart + defaultV.sideCount;
            rightStart = bottomStart + defaultV.horizontalCount;
            break;

        case StartEdge::left:
            leftStart = 0;
            bottomStart = leftStart + defaultV.sideCount;
            rightStart = bottomStart + defaultV.horizontalCount;
            topStart = rightStart + defaultV.sideCount;
            break;

        case StartEdge::bottom:
            bottomStart = 0;
            rightStart = bottomStart + defaultV.horizontalCount;
            topStart = rightStart + defaultV.sideCount;
            leftStart = topStart + defaultV.horizontalCount;
            break;
        }
	}
	else{
		switch (defaultV.startEdge){
        case StartEdge::right:
            rightStart = 0;
            bottomStart = rightStart + defaultV.sideCount;
            leftStart = bottomStart + defaultV.horizontalCount;
            topStart = leftStart + defaultV.sideCount;
            break;

        case StartEdge::top:
            topStart = 0;
            rightStart = topStart + defaultV.horizontalCount;
            bottomStart = rightStart + defaultV.sideCount;
            leftStart = bottomStart + defaultV.horizontalCount;
            break;

        case StartEdge::left:
            leftStart = 0;
            topStart = leftStart + defaultV.sideCount;
            rightStart = topStart + defaultV.horizontalCount;
            bottomStart = rightStart + defaultV.sideCount;
            break;

        case StartEdge::bottom:
            bottomStart = 0;
            leftStart = bottomStart + defaultV.horizontalCount;
            topStart = leftStart + defaultV.sideCount;
            rightStart = topStart + defaultV.horizontalCount;
            break;
        }

	}
}

void cameraTask(void* parameter){
	ESP_LOGI(TAG, "Camera task started on Core %d", xPortGetCoreID());

	while (1)
	{
		camera_fb_t* pic = esp_camera_fb_get();

		if (pic == nullptr)
		{
			ESP_LOGE(TAG, "Failed to capture camera frame.");
			vTaskDelay(pdMS_TO_TICKS(10));
			continue;
		}

		copyFrameToBuffer(pic);
		esp_camera_fb_return(pic);
		swapFrameBuffers();
		vTaskDelay(1);
	}
}
void copyFrameToBuffer(camera_fb_t* frame){
	if (frame == nullptr)
	{
		ESP_LOGE(TAG, "copyFrameToBuffer received nullptr frame.");
		return;
	}
	if (writeBuffer == nullptr)
	{
		ESP_LOGE(TAG, "writeBuffer is nullptr.");
		return;
	}
	if (frame->len < FRAME_BYTES) //Check if the frame is complete
	{
		ESP_LOGE
    (
      TAG,"Frame length error. frame->len=%u, expected=%u",
      (unsigned int)frame->len,
      (unsigned int)FRAME_BYTES
    );
		return;
	}

	for (uint32_t i = 0; i < FRAME_PIXELS; i++)
	{
		uint32_t byteIndex = i * 2;
		writeBuffer[i] = ((uint16_t)frame->buf[byteIndex] << 8) | ((uint16_t)frame->buf[byteIndex + 1]);
	}
}
void swapFrameBuffers(){
	if (frameMutex == nullptr)
	{
		ESP_LOGE(TAG, "frameMutex is nullptr.");
		return;
	}

	if (xSemaphoreTake(frameMutex, portMAX_DELAY) == pdTRUE)
	{
    //change address
		uint16_t* temp = readBuffer;
		readBuffer = writeBuffer;
		writeBuffer = temp;
		newFrameReady = true;
		xSemaphoreGive(frameMutex);
	}
}

void calculateLEDColors(){
	if (readBuffer == nullptr)
	{
		ESP_LOGE(TAG, "readBuffer is nullptr. Cannot calculate LED colors.");
		return;
	}

	calculateRight();
	calculateTop();
	calculateLeft();
	calculateBottom();

	gammaCorrection();
	smoothTransition();
}
int mapLedToPixel(uint16_t ledIndex, uint16_t totCount, uint16_t pixelLength) {
    if (totalCount == 0 || pixelLength == 0)
    {
        return 0;
    }

    // If there is only one LED on this edge, place its sampling point at the center of the image edge.
    if (totalCount == 1)
    {
        return (pixelLength - 1) / 2;
    }

    // Map:
    // LED index 0                  -> pixel 0
    // LED index totalCount - 1       -> pixel pixelLength - 1
    //
    // Other LEDs are distributed evenly between them.
    uint32_t numerator = (uint32_t)ledIndex * (pixelLength - 1);

    uint32_t denominator = totalCount - 1;

    // Add half of the denominator to achieve rounding instead of always rounding down.
    return (numerator + denominator / 2) / denominator;
}
CRGB sampleAverageColor(int baseX, int baseY, int inwardDx, int inwardDy, int tangentDx, int tangentDy) {
	uint32_t rSum = 0;
	uint32_t gSum = 0;
	uint32_t bSum = 0;
	uint8_t sampleCount = 0;

	for (int j = 0; j < EDGE_SAMPLES; j++)
	{
		for (int k = 0; k < INWARD_SAMPLES; k++)
		{
			int sampleX = baseX + tangentDx * j + inwardDx * k;
			int sampleY = baseY + tangentDy * j + inwardDy * k;

			if (sampleX < 0 || sampleX >= IMG_WIDTH || sampleY < 0 || sampleY >= IMG_HEIGHT)
			{
				continue; // avoid reading outside the frame buffer
			}

			int index = sampleY * IMG_WIDTH + sampleX;
			uint16_t pixel565 = readBuffer[index];
			CRGB color = rgb565ToCRGB(pixel565);
			rSum += color.r;
			gSum += color.g;
			bSum += color.b;

			sampleCount++;
		}
	}

	if (sampleCount > 0)
	{
		return CRGB(rSum/sampleCount, gSum/sampleCount, bSum/sampleCount);
	}
	else
	{
		return CRGB::Black; // return black if no valid sample was collected
	}
}
CRGB rgb565ToCRGB(uint16_t pixel){
    uint8_t r = ((pixel & 0xF800) >> 11) << 3;
    uint8_t g = ((pixel & 0x07E0) >> 5) << 2;
    uint8_t b = (pixel & 0x001F) << 3;
    return CRGB(r, g, b);
}
void calculateRight(){
		int baseX = IMG_WIDTH - 1;
		int inwardDx = -1;
		int inwardDy = 0;
		int tangentDx = 0;
		int ledIndex;
		for (int i = 0; i < defaultV.sideCount; i++)
	{
		int baseY = mapLedToPixel(i, defaultV.sideCount, IMG_HEIGHT);
		int tangentDy = 1;

		if (baseY >= IMG_HEIGHT - 1)
		{
			tangentDy = -1;
		}

        if (defaultV.direction == StripDirection::clockwise){
			ledIndex = rightStart + i;
        }
        else{
			ledIndex = (rightStart + defaultV.sideCount - 1) - i;
		}

		leds[ledIndex] = sampleAverageColor(baseX, baseY, inwardDx, inwardDy, tangentDx, tangentDy);
	}
}
void calculateTop(){
	int baseY = 0;
	int inwardDx = 0;
	int inwardDy = 1;
	int tangentDy = 0;
	int ledIndex;
	for (int i = 0; i < defaultV.horizontalCount; i++)
	{
		int baseX = mapLedToPixel(i, defaultV.horizontalCount, IMG_WIDTH);
		int tangentDx = 1;

		if (baseX >= IMG_WIDTH - 1)
		{
			tangentDx = -1;
		}

        if (defaultV.direction == StripDirection::clockwise){
			ledIndex = topStart + i;
        }
        else{
			ledIndex = (topStart + defaultV.horizontalCount - 1) - i;
		}

		leds[ledIndex] = sampleAverageColor(baseX, baseY, inwardDx, inwardDy, tangentDx, tangentDy);
	}
}
void calculateLeft() {
	int baseX = 0;
	int inwardDx = 1;
	int inwardDy = 0;
	int tangentDx = 0;
	int ledIndex;
	for (int i = 0; i < defaultV.sideCount; i++)
	{
		int baseY = mapLedToPixel(i, defaultV.sideCount, IMG_HEIGHT);
		int tangentDy = 1;

		if (baseY >= IMG_HEIGHT - 1)
		{
			tangentDy = -1;
		}

        if (defaultV.direction == StripDirection::clockwise){
			ledIndex = (leftStart + defaultV.sideCount - 1) - i;
        }
        else{
			ledIndex = leftStart + i;
		}

		leds[ledIndex] = sampleAverageColor(baseX, baseY, inwardDx, inwardDy, tangentDx, tangentDy);
	}
}
void calculateBottom() {
	int baseY = IMG_HEIGHT - 1;
	int inwardDx = 0;
	int inwardDy = -1;
	int tangentDy = 0;
	int ledIndex;
	for (int i = 0; i < defaultV.sideCount; i++)
	{
		int baseX = mapLedToPixel(i, defaultV.horizontalCount, IMG_WIDTH);
		int tangentDx = 1;
		
		if (baseX >= IMG_WIDTH - 1)
		{
			tangentDx = -1;
		}

		if (defaultV.direction == StripDirection::clockwise){
			ledIndex = (bottomStart + defaultV.horizontalCount - 1) - i;
        }
        else{
			ledIndex = bottomStart + i;
		}

		leds[ledIndex] = sampleAverageColor(baseX, baseY, inwardDx, inwardDy, tangentDx, tangentDy);
	}
}
void gammaCorrection(){
	for (int i = 0; i < totalCount; i++)
	{
		leds[i] = applyGamma_video(leds[i], GAMMA_R, GAMMA_G, GAMMA_B);
	}
}
void smoothTransition(){
	for (int i = 0; i < totalCount; i++)
	{
		CRGB targetColor = leds[i];

		leds[i] = blend(
			previousLeds[i],
			targetColor,
			smoothAmount
		);

		previousLeds[i] = leds[i]; // Prepare for next loop
	}
}
void whiteBalance(){

}

void setup(){
	delay(1000);

	esp_log_level_set(TAG, ESP_LOG_INFO);
	ESP_LOGI(TAG, "System starting...");

	while (frameMutex == nullptr)
	{
		frameMutex = xSemaphoreCreateMutex();
		if (frameMutex == nullptr)
		{
			ESP_LOGE(TAG, "Failed to create frameMutex. Retrying...");
			delay(2000);
		}
	}
	ESP_LOGI(TAG, "frameMutex created successfully.");

	while (!initFrameBuffers())
	{
		ESP_LOGE(TAG, "initFrameBuffers failed. Retrying...");
		delay(2000);
	}
	ESP_LOGI(TAG, "Frame buffers initialized successfully.");

	while (!initCamera())
	{
		ESP_LOGE(TAG, "initCamera failed. Retrying...");
		esp_camera_deinit();
		delay(2000);
	}
	ESP_LOGI(TAG, "Camera initialized successfully.");

	initLedStrip();
	FastLED.clear();
	FastLED.show();
	ESP_LOGI(TAG, "LED strip initialized and cleared.");

	BaseType_t taskResult = pdFAIL;
	while (taskResult != pdPASS)
	{
		taskResult = xTaskCreatePinnedToCore(cameraTask, "CameraTask", 8192, nullptr, 2, &cameraTaskHandle, 0);
		if (taskResult != pdPASS)
		{
			ESP_LOGE(TAG, "Failed to create camera task. Retrying...");
			delay(2000);
		}
	}
	ESP_LOGI(TAG, "Camera task created successfully.");

	calculateEdgeDirection();
	ESP_LOGI(TAG, "LED strip layout completed.");

	ESP_LOGI(TAG, "Setup completed.");
}

void loop(){
	if (newFrameReady)
	{
		if (frameMutex == nullptr)
		{
			delay(1);
			return;
		}
		if (xSemaphoreTake(frameMutex, portMAX_DELAY) == pdTRUE)
		{
			if (newFrameReady && readBuffer != nullptr)
			{
				calculateLEDColors();

				newFrameReady = false;
				xSemaphoreGive(frameMutex);

				FastLED.show();
			}
			else
			{
				xSemaphoreGive(frameMutex);
			}
		}
	}
	else
	{
		delay(1);
	}
}