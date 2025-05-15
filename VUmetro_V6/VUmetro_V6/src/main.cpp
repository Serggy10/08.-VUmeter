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

#define NUM_LEDS 200
#define PIN_LED 9
#define umbral 45
#define NUM_ESTRELLAS 50

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
// ColorController
struct Color
{
  byte r;
  byte g;
  byte b;
};

Color color = {255, 255, 255};

QueueHandle_t xQueueComandos; // Cola de comandos LED
enum LedMode
{
  LED_MODE_OFF,
  LED_MODE_BLINK,
  LED_MODE_STATIC,
  LED_MODE_RANDOM,
  LED_MODE_RAINBOW,
  LED_MODE_LIGHTNING,
  LED_MODE_MICROPHONE
};

struct LedCommand
{
  LedMode mode;
  Color color;
  int brightness;
};

LedMode modoActual = LED_MODE_MICROPHONE;

// ModeController
std::map<String, String> globalModes;

// BrightnessController
int globalBrightness = 255;
bool trigger = false;
Adafruit_NeoPixel pixels(NUM_LEDS, PIN_LED, NEO_RGB);
int16_t sBuffer[bufferLen];

/****************
 * FUNCIONES *
 *****************/
void i2s_install();
void i2s_setpin();
void TaskTiraLed(void *Pvparameters);
void TaskMicrofono(void *Pvparameters);
void TaskSinricPro(void *Pvparameters);
void sendLedCommand(LedMode mode, Color color, int brightness)
{
  LedCommand cmd = {mode, color, brightness};
  xQueueSend(xQueueComandos, &cmd, portMAX_DELAY);
}

/*************
 * Callbacks *
 *************/

// ModeController
bool onSetMode(const String &deviceId, const String &instance, String &mode)
{
  Serial.printf("[Device: %s]: Mode \"%s\" set to %s\r\n", deviceId.c_str(), instance.c_str(), mode.c_str());

  globalModes[instance] = mode;

  if (mode == "LIGHTNING")
  {
    modoActual = LED_MODE_LIGHTNING;
  }
  else if (mode == "BLINK")
  {
    modoActual = LED_MODE_BLINK;
  }
  else if (mode == "STATIC")
  {
    modoActual = LED_MODE_STATIC;
  }
  else if (mode == "RANDOM")
  {
    modoActual = LED_MODE_RANDOM;
  }
  else if (mode == "RAINBOW")
  {
    modoActual = LED_MODE_RAINBOW;
  }
  else if (mode == "MICROPHONE")
  {
    modoActual = LED_MODE_MICROPHONE;
  }
  else if (mode == "OFF")
  {
    modoActual = LED_MODE_OFF;
  }
  else
  {
    modoActual = LED_MODE_STATIC;
  }

  sendLedCommand(modoActual, color, globalBrightness);
  return true;
}

// ColorController
bool onColor(const String &deviceId, byte &r, byte &g, byte &b)
{
  Serial.printf("[Device: %s]: Color set to red=%d, green=%d, blue=%d\r\n", deviceId.c_str(), r, g, b);
  color = {r, g, b}; // Guardamos color global

  sendLedCommand(modoActual, color, globalBrightness);
  return true;
}

// BrightnessController
bool onBrightness(const String &deviceId, int &brightness)
{
  Serial.printf("[Device: %s]: Brightness set to %d\r\n", deviceId.c_str(), brightness);
  globalBrightness = brightness;

  sendLedCommand(modoActual, color, brightness);
  return true;
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
  xQueueComandos = xQueueCreate(5, sizeof(LedCommand));
  xTaskCreate(
      TaskSinricPro,
      "TaskMicrofono",
      10000,
      NULL,
      0,
      NULL);

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
  vTaskDelay(50 / portTICK_PERIOD_MS);
}

void TaskSinricPro(void *Pvparameters)
{
  for (;;)
  {
    SinricPro.handle();
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
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
        if (abs(mean) > umbral)
        {
          trigger = true;
          // Mandar un trigger para hacer un fade in de los LEDs en la tarea TaskTiraLEDs
        }
        else
        {
        }
        // float rms = 0;
        // for (int i = 0; i < samples_read; ++i)
        // {
        //   rms += sBuffer[i] * sBuffer[i];
        // }
        // rms = sqrt(rms / samples_read);

        // if (rms > umbral)
        // {
        //   trigger = true;
        //   // sendLedCommand(LED_MODE_MICROPHONE, color, globalBrightness);
        // }
      }
    }
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
  pixels.clear();
  pixels.show();

  LedCommand currentCommand = {LED_MODE_OFF, {0, 0, 0}, 0};

  bool LEDsEncendidos[NUM_ESTRELLAS] = {false};
  int pixelLEDsEstrellas[NUM_ESTRELLAS] = {-1};
  int contadorBrillo[NUM_ESTRELLAS] = {0};
  bool apagadoProgresivo[NUM_ESTRELLAS] = {false};
  unsigned long tiempoAnteriorEncendido[NUM_ESTRELLAS] = {0};
  int tiempoDelayEstrellas[NUM_ESTRELLAS] = {0};

  for (;;)
  {
    LedCommand newCommand;
    if (xQueueReceive(xQueueComandos, &newCommand, 10 / portTICK_PERIOD_MS))
    {
      currentCommand = newCommand;
    }

    switch (currentCommand.mode)
    {
    case LED_MODE_OFF:
      pixels.clear();
      pixels.show();
      break;

    case LED_MODE_BLINK:
    {
      static bool fadingIn = true;
      static int fadeValue = 0;
      static unsigned long lastChangeTime = 0;
      static enum { FADING,
                    PAUSED } state = FADING;
      int fadeStep = max(1, currentCommand.brightness / 20);

      const int pauseTime = 500; // ms

      unsigned long now = millis();

      if (state == FADING)
      {
        if (fadingIn)
        {
          fadeValue += fadeStep;
          if (fadeValue >= currentCommand.brightness)
          {
            fadeValue = currentCommand.brightness;
            fadingIn = false;
            state = PAUSED;
            lastChangeTime = now;
          }
        }
        else
        {
          fadeValue -= fadeStep;
          if (fadeValue <= 0)
          {
            fadeValue = 0;
            fadingIn = true;
            state = PAUSED;
            lastChangeTime = now;
          }
        }

        // Aplicar fade
        for (int i = 0; i < NUM_LEDS; ++i)
        {
          pixels.setPixelColor(i, pixels.Color(
                                      currentCommand.color.g * fadeValue / 255,
                                      currentCommand.color.r * fadeValue / 255,
                                      currentCommand.color.b * fadeValue / 255));
        }
        pixels.show();
      }
      else if (state == PAUSED)
      {
        if (now - lastChangeTime >= pauseTime)
        {
          state = FADING;
        }
      }

      vTaskDelay(30 / portTICK_PERIOD_MS);
      continue;
    }

    case LED_MODE_STATIC:
      for (int i = 0; i < NUM_LEDS; ++i)
      {
        pixels.setPixelColor(i, pixels.Color(
                                    currentCommand.color.g * currentCommand.brightness / 255,
                                    currentCommand.color.r * currentCommand.brightness / 255,
                                    currentCommand.color.b * currentCommand.brightness / 255));
      }
      pixels.show();
      break;

    case LED_MODE_RANDOM:
    {
      uint8_t r = currentCommand.color.r;
      uint8_t g = currentCommand.color.g;
      uint8_t b = currentCommand.color.b;
      int fadeStep = max(1, currentCommand.brightness / 20);

      for (int i = 0; i < NUM_ESTRELLAS; ++i)
      {
        if (!LEDsEncendidos[i])
        {
          if (random(100) > 96) // 4% de probabilidad por ciclo
          {
            int pixel = random(0, NUM_LEDS); // Asignamos un pixel al azar
            bool repetido = false;
            for (int j = 0; j < NUM_ESTRELLAS; ++j)
            {
              if (pixelLEDsEstrellas[j] == pixel)
              {
                repetido = true;
                break;
              }
            }

            if (!repetido)
            {
              LEDsEncendidos[i] = true;
              pixelLEDsEstrellas[i] = pixel;
              tiempoDelayEstrellas[i] = random(10, 50);
              apagadoProgresivo[i] = false;
              contadorBrillo[i] = 0;
              tiempoAnteriorEncendido[i] = millis();
            }
          }
        }
        else
        {
          unsigned long ahora = millis();
          int pixel = pixelLEDsEstrellas[i]; // Declaramos pixel aquí para asegurarnos de que esté disponible

          if (ahora - tiempoAnteriorEncendido[i] >= tiempoDelayEstrellas[i])
          {
            tiempoAnteriorEncendido[i] = ahora;

            if (!apagadoProgresivo[i])
            {
              contadorBrillo[i] += fadeStep;
              if (contadorBrillo[i] >= currentCommand.brightness)
              {
                contadorBrillo[i] = currentCommand.brightness;
                apagadoProgresivo[i] = true;
              }
            }
            else
            {
              contadorBrillo[i] -= fadeStep;
              if (contadorBrillo[i] <= 0)
              {
                contadorBrillo[i] = 0;
                LEDsEncendidos[i] = false;
                pixelLEDsEstrellas[i] = -1;
                apagadoProgresivo[i] = false;
                tiempoDelayEstrellas[i] = 0;

                pixels.setPixelColor(pixel, 0); // Esto lo apaga completamente
                continue;                       // Salimos para la siguiente iteración
              }
            }

            // Aplicar brillo actual
            pixels.setPixelColor(pixel, pixels.Color(
                                            g * contadorBrillo[i] / 255,
                                            r * contadorBrillo[i] / 255,
                                            b * contadorBrillo[i] / 255));
          }
        }
      }

      pixels.show();
      break;
    }

    case LED_MODE_RAINBOW:
    {
      static uint16_t hue = 0;
      for (int i = 0; i < NUM_LEDS; ++i)
      {
        uint32_t color = pixels.gamma32(pixels.ColorHSV(hue + (i * 65536L / NUM_LEDS)));
        color = pixels.Color(
            ((color >> 16) & 0xFF) * currentCommand.brightness / 255,
            ((color >> 8) & 0xFF) * currentCommand.brightness / 255,
            (color & 0xFF) * currentCommand.brightness / 255);
        pixels.setPixelColor(i, color);
      }
      pixels.show();
      hue += 256;
      vTaskDelay(50 / portTICK_PERIOD_MS);
      continue;
    }

    case LED_MODE_LIGHTNING:
    {
      static int rayoPos = 0;
      static int direccion = 1; // 1 = derecha, -1 = izquierda
      static unsigned long lastChangeTime = 0;
      const unsigned long rayoDelay = 1;
      const int LIGHTNING_LENGTH = 5; // Número de LEDs que forman el rayo

      unsigned long now = millis();
      if (now - lastChangeTime >= rayoDelay)
      {
        // Apaga toda la tira
        for (int i = 0; i < NUM_LEDS; ++i)
        {
          pixels.setPixelColor(i, 0);
        }

        // Enciende el "rayo" de varios LEDs
        for (int j = 0; j < LIGHTNING_LENGTH; ++j)
        {
          int pos = rayoPos - j * direccion;
          if (pos >= 0 && pos < NUM_LEDS)
          {
            pixels.setPixelColor(pos, pixels.Color(
                                          currentCommand.color.g * currentCommand.brightness / 255,
                                          currentCommand.color.r * currentCommand.brightness / 255,
                                          currentCommand.color.b * currentCommand.brightness / 255));
          }
        }

        pixels.show();

        rayoPos += direccion * 2;

        // Cambiar dirección si llega al final o principio
        if (rayoPos >= NUM_LEDS + LIGHTNING_LENGTH)
        {
          direccion = -1;
          rayoPos = NUM_LEDS + LIGHTNING_LENGTH - 1;
        }
        else if (rayoPos < -LIGHTNING_LENGTH)
        {
          direccion = 1;
          rayoPos = -LIGHTNING_LENGTH + 1;
        }

        lastChangeTime = now;
      }
      continue;
    }

    case LED_MODE_MICROPHONE:
    {
      static int brilloActual = 20; // empieza en brillo base
      static bool enFadeIn = false;
      static bool enFadeOut = false;
      static unsigned long ultimaActualizacion = 0;
    
      const int fadeStep = 15;          // cuán rápido sube/baja
      const int delayFade = 10;         // ms entre pasos
      const int brilloBase = 10;        // brillo cuando está "en reposo"
      const int brilloMaximo = currentCommand.brightness;
    
      unsigned long ahora = millis();
    
      if (trigger)
      {
        enFadeIn = true;
        enFadeOut = false;
        trigger = false;
      }
    
      if (ahora - ultimaActualizacion >= delayFade)
      {
        ultimaActualizacion = ahora;
    
        if (enFadeIn)
        {
          brilloActual += fadeStep;
          if (brilloActual >= brilloMaximo)
          {
            brilloActual = brilloMaximo;
            enFadeIn = false;
            enFadeOut = true;
          }
        }
        else if (enFadeOut)
        {
          brilloActual -= fadeStep;
          if (brilloActual <= brilloBase)
          {
            brilloActual = brilloBase;
            enFadeOut = false;
          }
        }
    
        for (int i = 0; i < NUM_LEDS; ++i)
        {
          pixels.setPixelColor(i, pixels.Color(
                                      currentCommand.color.g * brilloActual / 255,
                                      currentCommand.color.r * brilloActual / 255,
                                      currentCommand.color.b * brilloActual / 255));
        }
        pixels.show();
      }
    
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }
    

    default:
      pixels.clear();
      pixels.show();
      break;
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}