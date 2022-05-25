#include "Arduino.h"
#include "Arduino_FreeRTOS.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "ZHRF24SensorProtocol.h"

#define ID 1 // Уникальный идентификатор устройства RF24 в сети.
#define PIPE 0xDDEEFF
#define CHANNEL 120
#define SLEEP_TIME 300 // В секундах.

RF24 radio(9, 10);
Adafruit_BME280 bme;

void sendSensorsValue(void *pvParameters);
float getBatteryLevelCharge(void);

void setup()
{
  ADCSRA &= ~(1 << ADEN);
  radio.begin();
  radio.setChannel(CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setPayloadSize(14);
  radio.setAddressWidth(3);
  radio.setCRCLength(RF24_CRC_8);
  radio.setRetries(15, 15);
  radio.openWritingPipe(PIPE);
  radio.powerDown();
  bme.begin(0x76);
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::FILTER_OFF);
  xTaskCreate(sendSensorsValue, "Send Sensor Values", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

void loop()
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  portENTER_CRITICAL();
  sleep_enable();
  portEXIT_CRITICAL();
  sleep_cpu();
  sleep_reset();
}

void sendSensorsValue(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    TransmittedData sensor{ID, BME280};
    bme.takeForcedMeasurement();
    sensor.value_1 = getBatteryLevelCharge() * 100; // *100 для передачи значения float в int. На стороне получателя оно преобразуется обратно в float.
    sensor.value_2 = bme.readHumidity();
    sensor.value_3 = bme.readTemperature();
    sensor.value_4 = bme.readPressure() * 0.00750062;
    radio.powerUp();
    radio.flush_tx();
    radio.write(&sensor, sizeof(struct TransmittedData));
    radio.powerDown();
    vTaskDelay(SLEEP_TIME);
  }
  vTaskDelete(NULL);
}

float getBatteryLevelCharge()
{
  ADMUX = (1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
  ADCSRA |= (1 << ADEN);
  delay(10);
  ADCSRA |= (1 << ADSC);
  while (bit_is_set(ADCSRA, ADSC))
    ;
  ADCSRA &= ~(1 << ADEN);
  float value = ((1024 * 1.1) / (ADCL + ADCH * 256));
  return value;
}