// Custom devices requires SinricPro ESP8266/ESP32 SDK 2.9.6 or later

// Uncomment the following line to enable serial debug output
// #define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
#define DEBUG_ESP_PORT Serial
#define NODEBUG_WEBSOCKETS
#define NDEBUG
#endif

#include <Arduino.h>
#ifdef ESP8266
#include <ESP8266WiFi.h>
#endif
#ifdef ESP32
#include <WiFi.h>
#endif

#include <driver/i2s.h>
#include <Adafruit_NeoPixel.h>
#include <SinricPro.h>
#include "TiraLEDdebajosofa.h"
#define I2S_WS 13
#define I2S_SD 11
#define I2S_SCK 12
#define I2S_PORT I2S_NUM_0
#define bufferLen 64

#define NUM_LEDS 1
#define PIN_LED 48
#define umbral 100

#define APP_KEY ""
#define APP_SECRET ""
#define DEVICE_ID ""

#define SSID ""
#define PASS ""

#define BAUD_RATE 115200

TiraLEDdebajosofa &tiraLEDdebajosofa = SinricPro[DEVICE_ID];

/*************
 * Variables *
 ***********************************************
 * Global variables to store the device states *
 ***********************************************/

// ModeController
std::map<String, String> globalModes;

// ColorController
struct Color
{
  byte r;
  byte g;
  byte b;
};

Color color;

// BrightnessController
int globalBrightness;
int brillo = 0;
Adafruit_NeoPixel pixels(NUM_LEDS, PIN_LED, NEO_RGB);
int16_t sBuffer[bufferLen];

/****************
 * FUNCIONES *
 *****************/
void i2s_install();
void i2s_setpin();
void TaskTiraLed(void *Pvparameters);
void TaskMicrofono(void *Pvparameters);

/*************
 * Callbacks *
 *************/

// ModeController
bool onSetMode(const String &deviceId, const String &instance, String &mode)
{
  Serial.printf("[Device: %s]: Modesetting for \"%s\" set to mode %s\r\n", deviceId.c_str(), instance.c_str(), mode.c_str());
  globalModes[instance] = mode;
  return true;
}

// ColorController
bool onColor(const String &deviceId, byte &r, byte &g, byte &b)
{
  Serial.printf("[Device: %s]: Color set to red=%d, green=%d, blue=%d\r\n", deviceId.c_str(), r, g, b);
  color.r = r;
  color.g = g;
  color.b = b;
  return true; // request handled properly
}

// BrightnessController
bool onBrightness(const String &deviceId, int &brightness)
{
  Serial.printf("[Device: %s]: Brightness set to %d\r\n", deviceId.c_str(), brightness);
  globalBrightness = brightness;
  return true; // request handled properly
}

bool onAdjustBrightness(const String &deviceId, int &brightnessDelta)
{
  globalBrightness += brightnessDelta; // calculate absolute brigthness
  Serial.printf("[Device: %s]: Brightness changed about %i to %d\r\n", deviceId.c_str(), brightnessDelta, globalBrightness);
  brightnessDelta = globalBrightness; // return absolute brightness
  return true;                        // request handled properly
}

/**********
 * Events *
 *************************************************
 * Examples how to update the server status when *
 * you physically interact with your device or a *
 * sensor reading changes.                       *
 *************************************************/

// ModeController
void updateMode(String instance, String mode)
{
  tiraLEDdebajosofa.sendModeEvent(instance, mode, "PHYSICAL_INTERACTION");
}

// ColorController
void updateColor(byte r, byte g, byte b)
{
  tiraLEDdebajosofa.sendColorEvent(r, g, b);
}

// BrightnessController
void updateBrightness(int brightness)
{
  tiraLEDdebajosofa.sendBrightnessEvent(brightness);
}

/*********
 * Setup *
 *********/

void setupSinricPro()
{

  // ModeController
  tiraLEDdebajosofa.onSetMode("modeInstance1", onSetMode);

  // ColorController
  tiraLEDdebajosofa.onColor(onColor);

  // BrightnessController
  tiraLEDdebajosofa.onBrightness(onBrightness);
  tiraLEDdebajosofa.onAdjustBrightness(onAdjustBrightness);
  SinricPro.onConnected([]
                        { Serial.printf("[SinricPro]: Connected\r\n"); });
  SinricPro.onDisconnected([]
                           { Serial.printf("[SinricPro]: Disconnected\r\n"); });
  SinricPro.begin(APP_KEY, APP_SECRET);
};

void setupWiFi()
{
#if defined(ESP8266)
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setAutoReconnect(true);
#elif defined(ESP32)
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
#endif

  WiFi.begin(SSID, PASS);
  Serial.printf("[WiFi]: Connecting to %s", SSID);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.printf(".");
    delay(250);
  }
  Serial.printf("connected\r\n");
}

void setup()
{
  Serial.begin(BAUD_RATE);
  setupWiFi();
  setupSinricPro();
  xTaskCreate(
      TaskMicrofono,
      "TaskMicrofono",
      10000,
      NULL,
      1,
      NULL);

  xTaskCreate(
      TaskTiraLed,
      "TaskTiraLed",
      10000,
      NULL,
      1,
      NULL);
}

/********
 * Loop *
 ********/

void loop()
{
  SinricPro.handle();
}

void TaskMicrofono(void *Pvparameters)
{
  Serial.println("Setup I2S ...");

  delay(1000);
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
  delay(500);
  for (;;)
  {
    size_t bytesIn = 0;
    esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);
    if (result == ESP_OK)
    {
      int samples_read = bytesIn / 8;
      if (samples_read > 0)
      {
        float mean = 0;
        for (int i = 0; i < samples_read; ++i)
        {
          mean += (sBuffer[i]);
        }
        mean /= samples_read;
        // Serial.println(">Media: " + String(mean));
        // Serial.println(">Umbral: " + String(umbral));
        // Serial.println(">-Umbral: " + String(-umbral));
        if (abs(mean) > umbral)
        {
          // Serial.println(">pulso: " + String(1));
          brillo = 255;
        }
        else
        {
        }
        // Serial.println(">pulso: " + String(0));
      }
    }
    // Serial.println(">Brillo: " + String(brillo));
    vTaskDelay(10);
  }
}

void i2s_install()
{
  const i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = 44100,
      .bits_per_sample = i2s_bits_per_sample_t(16),
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
      .intr_alloc_flags = 0, // default interrupt priority
      .dma_buf_count = 8,
      .dma_buf_len = bufferLen,
      .use_apll = false};

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin()
{
  const i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,
      .ws_io_num = I2S_WS,
      .data_out_num = -1,
      .data_in_num = I2S_SD};

  i2s_set_pin(I2S_PORT, &pin_config);
}

void TaskTiraLed(void *Pvparameters)
{
  pixels.begin();
  pixels.setBrightness(50);
  pixels.clear();
  pixels.show();

  for (;;)
  {
    if (brillo > 0) // Solo ejecuta si hay brillo
    {
      // Serial.println("Color: " + (String)color.r + ":" + (String)color.g + "" + (String)color.b);
      pixels.setPixelColor(0, pixels.Color(color.g*brillo/255, color.r*brillo/255, color.b*brillo/255));
      pixels.show();
      brillo--;
    }
    else
    {
      pixels.show();
    }
    vTaskDelay(5);
  }
}