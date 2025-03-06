#include <driver/i2s.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 9 // Cambia el pin a donde tienes conectada la tira de LEDs
#define NUM_LEDS 30 // Ajusta el número de LEDs en la tira

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
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
const uint16_t websocket_server_port = 8888;  // Puerto del servidor WebSocket

using namespace websockets;
WebsocketsClient client;
bool isWebSocketConnected;

void onEventsCallback(WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("Connection Opened");
    isWebSocketConnected = true;
  } else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("Connection Closed");
    isWebSocketConnected = false;
  } else if (event == WebsocketsEvent::GotPing) {
    Serial.println("Got a Ping!");
  } else if (event == WebsocketsEvent::GotPong) {
    Serial.println("Got a Pong!");
  }
}

void i2s_install() {
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
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
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}

void setup() {
  Serial.begin(9600);
  connectWiFi();
  connectWSServer();
  strip.begin();
  strip.show(); // Apaga los LEDs al inicio
  pinMode(8, INPUT); // Configura el pin 8 como entrada analógica
  xTaskCreatePinnedToCore(micTask, "micTask", 10000, NULL, 1, NULL, 1);
}

void loop() {
}

void connectWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}

void connectWSServer() {
  client.onEvent(onEventsCallback);
  while (!client.connect(websocket_server_host, websocket_server_port, "/")) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Websocket Connected!");
}

void micTask(void* parameter) {
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);

  size_t bytesIn = 0;
  const float defaultGain = 10.0;  // Valor predeterminado de ganancia
  float gain; // Variable para la ganancia leída

  float currentBrightness = 0; // Brillo actual de los LEDs
  float targetBrightness = 0; // Brillo objetivo basado en el audio
  const float smoothingFactor = 0.1; // Factor de suavizado

  while (1) {
    // Leer el valor del pin analógico y mapear a un rango de 1 a 250
    int analogValue = analogRead(8); // Lee el valor analógico
    gain = map(analogValue, 0, 4095, 0.01, 1); // Mapea el valor al rango de ganancia

    esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);
    if (result == ESP_OK && isWebSocketConnected) {
      // Aplicar ganancia
      int32_t sum = 0;  // Sumar valores para calcular RMS
      int16_t sampleCount = bytesIn / sizeof(int16_t);

      for (int i = 0; i < sampleCount; i++) {
        sBuffer[i] *= gain;
        // Asegúrate de que no exceda el rango de int16_t
        if (sBuffer[i] > INT16_MAX) sBuffer[i] = INT16_MAX;
        if (sBuffer[i] < INT16_MIN) sBuffer[i] = INT16_MIN;

        // Calcular la suma de los cuadrados
        sum += (sBuffer[i] * sBuffer[i]);
      }

      // Calcular RMS
      float rms = sqrt((float)sum / sampleCount);
      // Mapeamos el valor RMS a un brillo en un rango de 0 a 255
      targetBrightness = map(rms, 0, INT16_MAX, 0, 255); 

      // Suavizar el brillo
      currentBrightness += (targetBrightness - currentBrightness) * smoothingFactor;

      // Actualizar LEDs con el brillo suavizado
      for (int j = 0; j < NUM_LEDS; j++) {
        strip.setPixelColor(j, strip.Color(currentBrightness, 0, 0)); // Color rojo ajustando el brillo
      }
      strip.show(); // Actualizar la tira de LEDs

      // Enviar datos a través de WebSocket
      client.sendBinary((const char*)sBuffer, bytesIn);
    }
  }
}
