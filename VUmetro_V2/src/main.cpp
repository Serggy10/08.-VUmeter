#include <Arduino.h>
#include <driver/i2s.h>

#define I2S_SD 10
#define I2S_WS 11
#define I2S_SCK 12
#define I2S_PORT I2S_NUM_0
#define bufferCnt 10
#define bufferLen 1024
int16_t sBuffer[bufferLen];
size_t bytesIn = 0;

// Definir la ganancia
#define GAIN 2.0  // Puedes ajustar este valor según sea necesario

void i2s_install();
void i2s_setpin();

void setup()
{
    Serial.begin(9600);
    Serial.println("Iniciando");
    delay(1000);
    pinMode(8, INPUT);
    i2s_install();
    i2s_setpin();
    i2s_start(I2S_PORT);
}

void loop()
{
    esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);
    if (result == ESP_OK)
    {
        // Aplicar ganancia
        int16_t sampleCount = bytesIn / sizeof(int16_t);

        for (int i = 0; i < sampleCount; i++)
        {
            // Aplicar ganancia y asegurarse de que no exceda el rango de int16_t
            int32_t amplifiedSample = (int32_t)(sBuffer[i] * GAIN);

            // Limitar el valor amplificado para evitar desbordamiento
            if (amplifiedSample > INT16_MAX)
                sBuffer[i] = INT16_MAX;
            else if (amplifiedSample < INT16_MIN)
                sBuffer[i] = INT16_MIN;
            else
                sBuffer[i] = (int16_t)amplifiedSample; // Guardar el valor amplificado
        }

        // Imprimir el búfer amplificado para verificar
        Serial.println((const char*)sBuffer);
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
        .intr_alloc_flags = 0,
        .dma_buf_count = bufferCnt,
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
