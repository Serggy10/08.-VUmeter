/////////////////////////////////////////////////////////////////
/*
  Broadcasting Your Voice with ESP32-S3 & INMP441
  For More Information: https://youtu.be/qq2FRv0lCPw
  Created by Eric N. (ThatProject)
*/
/////////////////////////////////////////////////////////////////

/*
- Device
ESP32-S3 DevKit-C
https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html

- Required Library
Arduino ESP32: 2.0.9

Arduino Websockets: 0.5.3
https://github.com/gilmaimon/ArduinoWebsockets
*/

#include <driver/i2s.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <Adafruit_NeoPixel.h>
// Configuración de los LEDs
#define PIN 9        // Pin de control de la tira de LEDs
#define NUM_LEDS 30  // Número de LEDs en la tira
Adafruit_NeoPixel strip(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

#define I2S_SD 10
#define I2S_WS 11
#define I2S_SCK 12
#define I2S_PORT I2S_NUM_0

#define bufferCnt 10
#define bufferLen 1024
int16_t sBuffer[bufferLen];

const char* ssid = "";
const char* password = "";

const char* websocket_server_host = "192.168.1.146";  // Dirección IP de tu servidor WebSocket
const uint16_t websocket_server_port = 8888;          // Puerto del servidor WebSocket

using namespace websockets;
WebsocketsClient client;
bool isWebSocketConnected;

void processAudioData(int16_t* audioBuffer, size_t bytesIn);
float filter(float input);
void setLedIntensity(float maxAmplitude);
float filterbrightness(float input);

void onEventsCallback(WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("Connnection Opened");
    isWebSocketConnected = true;
  } else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("Connnection Closed");
    isWebSocketConnected = false;
  } else if (event == WebsocketsEvent::GotPing) {
    Serial.println("Got a Ping!");
  } else if (event == WebsocketsEvent::GotPong) {
    Serial.println("Got a Pong!");
  }
}

void i2s_install() {
  // Set up I2S Processor configuration
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    //.sample_rate = 16000,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0,
    .dma_buf_count = bufferCnt,
    .dma_buf_len = bufferLen,
    .use_apll = false
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin() {
  // Set I2S pin configuration
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}

void setup() {
  Serial.begin(115200);
  pinMode(8, INPUT);
  pinMode(7, INPUT);
  //connectWiFi();
  //connectWSServer();
  strip.begin();
  strip.show();  // Inicializa todos los LEDs en "off"
  xTaskCreatePinnedToCore(micTask, "micTask", 10000, NULL, 1, NULL, 1);
}

void loop() {
}

// void connectWiFi() {
//   WiFi.begin(ssid, password);

//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("");
//   Serial.println("WiFi connected");
// }

// void connectWSServer() {
//   client.onEvent(onEventsCallback);
//   while (!client.connect(websocket_server_host, websocket_server_port, "/")) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("Websocket Connected!");
// }


// void micTask(void* parameter) {

//   i2s_install();
//   i2s_setpin();
//   i2s_start(I2S_PORT);

//   size_t bytesIn = 0;
//   while (1) {
//     esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);
//     if (result == ESP_OK && isWebSocketConnected) {
//       client.sendBinary((const char*)sBuffer, bytesIn);
//     }
//   }
// }

#define POT_PIN 8  // Pin donde se conecta el potenciómetro

void micTask(void* parameter) {
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);

  size_t bytesIn = 0;
  while (1) {
    // Leer el valor del potenciómetro para ajustar la ganancia
    int potValue = analogRead(8);

    // Mapear el valor del potenciómetro a un rango de ganancia de 0 a 5
    float gain = map(potValue, 0, 4095, 0, 100);
    //Serial.println("ganancia: " + String(gain,4));
    //Serial.println(gain, 2);  // Mostrar ganancia en el monitor serie con 2 decimales

    // Leer datos del micrófono (I2S)
    esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);

    if (result == ESP_OK) {
      // Aplicar la ganancia a cada muestra en el buffer
      int sampleCount = bytesIn / sizeof(int16_t);
      for (int i = 0; i < sampleCount; i++) {
        sBuffer[i] = sBuffer[i] * gain;  // Aplicar la ganancia
      }

      // Enviar los datos modificados a través del WebSocket si está conectado
      // if (isWebSocketConnected) {
      //   client.sendBinary((const char*)sBuffer, bytesIn);
      // }
      processAudioData(sBuffer, bytesIn);
    }
  }
}

void processAudioData(int16_t* audioBuffer, size_t bytesIn) {
  // Convertir el tamaño de bytes a cantidad de muestras
  size_t numSamples = bytesIn / sizeof(int16_t);

  float maxAmplitude = 0;

  for (size_t i = 0; i < numSamples; i++) {
    float sample = (float)audioBuffer[i] / 32768.0;  // Normalizar entre -1 y 1
    sample = filter(sample);                         // Filtrar el sample

    // Encontrar el máximo
    if (sample > maxAmplitude) {
      maxAmplitude = sample;
    }
  }

  // Mapear la amplitud máxima a un brillo y suavizarlo
  float smoothedBrightness = filterbrightness(maxAmplitude);  // Se usa el mismo filtro para suavizar el brillo
  setLedIntensity(smoothedBrightness);
}

// void setLedIntensity(float maxAmplitude) {
//   // Mapea la amplitud máxima a un rango de colores
//   int ledCount = strip.numPixels();
//   int activeLeds = (int)(maxAmplitude * ledCount);  // Mapea a la cantidad de LEDs
//   activeLeds = constrain(activeLeds, 0, ledCount);  // Limita a la cantidad de LEDs disponibles

//   // Apagar todos los LEDs
//   for (int i = 0; i < ledCount; i++) {
//     strip.setPixelColor(i, strip.Color(0, 0, 0));  // Apagar LED
//   }

//   // Encender LEDs según la amplitud
//   for (int i = 0; i < activeLeds; i++) {
//     // Se puede elegir un color diferente dependiendo de la intensidad
//     // Por ejemplo, un gradiente de verde a rojo:
//     int green = (255 * (activeLeds - i)) / activeLeds;   // Verde disminuye
//     int red = (255 * i) / activeLeds;                    // Rojo aumenta
//     strip.setPixelColor(i, strip.Color(red, green, 0));  // Color basado en la intensidad
//   }

//   strip.show();  // Mostrar cambios en los LEDs
// }

void setLedIntensity(float maxAmplitude) {
  // Mapea la amplitud máxima a un rango de brillo
  int ledCount = strip.numPixels();

  // Mapea la amplitud a un rango de brillo entre 0 y 255
  int brightness = (int)(maxAmplitude * 255);
  brightness = constrain(brightness, 0, 255);  // Limita el brillo entre 0 y 255

  // Define el color que deseas (por ejemplo, azul)
  int red = 255;
  int green = 0;
  int blue = 0;  // Azul

  // Ajusta el brillo de todos los LEDs
  for (int i = 0; i < ledCount; i++) {
    // Aplica el brillo al color
    strip.setPixelColor(i, strip.Color(red * brightness / 255, green * brightness / 255, blue * brightness / 255));
  }

  strip.show();  // Mostrar cambios en los LEDs
}


float filter(float input) {
  static float lastInput = 0;
  float output = (input + lastInput) / 2;  // Media móvil simple
  lastInput = input;
  return output;
}

// float filterbrightness(float input) {
//   static float lastInput = 0;
//   // Usamos un factor de suavizado, puedes ajustar este valor entre 0 y 1 para cambiar la suavidad
//   float smoothingFactor = 0.15;  // 0.1 significa más suavizado
//   float output = (input * smoothingFactor) + (lastInput * (1 - smoothingFactor));
//   lastInput = output;  // Actualiza lastInput para la próxima iteración
//   Serial.println(output);
//   return output;
// }


// float filterbrightness(float input) {
//   static float lastInput = 0;
//   input = constrain(input, 0, 1);
//   // Define los factores de suavizado
//   float smoothingFactorSlow = 0.015;  // Suavizado lento para entradas menores a 0.10
//   float smoothingFactorFast = 0.3;    // Suavizado rápido para entradas mayores o iguales a 0.10

//   // Selecciona el factor de suavizado basado en el valor de entrada
//   float smoothingFactor = (input < 0.10) ? smoothingFactorSlow : smoothingFactorFast;

//   // Aplicar el suavizado
//   float output = (input * smoothingFactor) + (lastInput * (1 - smoothingFactor));

//   lastInput = output;      // Actualiza lastInput para la próxima iteración
//   Serial.print(input);     // Imprime el valor de entrada
//   Serial.print(",");       // Separador
//   Serial.println(output);  // Imprime el valor de salida

//   return output;
// }
float filterbrightness(float input) {
  static float lastInput = 0;

  // Define los factores de suavizado
  float smoothingFactorUp = 0.5;      // Suavizado rápido para subidas
  float smoothingFactorDown = 0.2;    // Suavizado lento para bajadas
  float smoothingFactorSlow = 0.015;  // Suavizado lento para entradas negativas

  float output;
  const float umbral = 0.1;

  // Verifica si la entrada está aumentando o disminuyendo
  if (input > umbral) {
    if (input > lastInput) {
      // Si el nuevo input es mayor que el anterior, aplica el suavizado rápido
      output = (input * smoothingFactorUp) + (lastInput * (1 - smoothingFactorUp));
    } else {
      // Si el input es menor o igual a 0, aplica el suavizado lento
      output = (input * smoothingFactorDown) + (lastInput * (1 - smoothingFactorDown));
    }
  } else {
    // Si el nuevo input es menor o igual que el anterior, aplica el suavizado lento
    output = umbral;
  }

  lastInput = output;  // Actualiza lastInput para la próxima iteración

  // Imprime input y output para graficar
  Serial.print(input);     // Imprime el valor de entrada
  Serial.print(",");       // Separador
  Serial.println(output);  // Imprime el valor de salida

  return output;
}