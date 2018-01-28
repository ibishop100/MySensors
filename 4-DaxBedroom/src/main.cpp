#define MY_DEBUG
#define MY_RADIO_NRF24
#define MY_RF24_CHANNEL 83
#define MY_NODE_ID 4
#define MY_REPEATER_FEATURE

#include <SPI.h>
#include <MySensors.h>
#include <DHT.h>

#define DHT_DATA_PIN 7
#define MOTION_PIN 3
#define WINDOW_PIN 2

#define CHILD_ID_TEMP 0
#define CHILD_ID_HUM 1
#define CHILD_ID_WINDOW 2
#define CHILD_ID_MOTION 3

static const uint64_t UPDATE_INTERVAL = 1000;

//--------------------- TEMP SENSOR
static const uint8_t FORCE_UPDATE_N_READS = 10;

#define SENSOR_TEMP_OFFSET 0
float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
bool metric = true;
uint8_t dhtSkips = 30;
uint8_t numberOfDHTSkips = 0;

MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgHum(CHILD_ID_HUM, V_HUM);

DHT dht;

//--------------------- Motion SENSOR
MyMessage msgMotion(CHILD_ID_MOTION, V_TRIPPED);
bool windowTrippedLastValue = false;

void presentation() {
  sendSketchInfo("DaxRoom", "1.1");

  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_WINDOW, S_DOOR);
  present(CHILD_ID_MOTION, S_MOTION);

  metric = getControllerConfig().isMetric;
}

void setup() {
  dht.setup(DHT_DATA_PIN); // set data pin of DHT sensor

  pinMode(WINDOW_PIN, INPUT);
  pinMode(MOTION_PIN, INPUT);

  sleep(dht.getMinimumSamplingPeriod());

}

void loop() {
  numberOfDHTSkips++;
  if (numberOfDHTSkips > dhtSkips) {
    Serial.println("Doing Temp");
    // Force reading sensor, so it works also after sleep()
    dht.readSensor(true);

    // Get temperature from DHT library
    float temperature = dht.getTemperature();
    if (isnan(temperature)) {
      //Serial.println("Failed reading temperature from DHT!");
    } else if (temperature != lastTemp || nNoUpdatesTemp == FORCE_UPDATE_N_READS) {
      // Only send temperature if it changed since the last measurement or if we didn't send an update for n times
      lastTemp = temperature;
      if (!metric) {
        temperature = dht.toFahrenheit(temperature);
      }
      // Reset no updates counter
      nNoUpdatesTemp = 0;
      temperature += SENSOR_TEMP_OFFSET;
      send(msgTemp.set(temperature, 1));

      #ifdef MY_DEBUG
        Serial.print("T: ");
        Serial.println(temperature);
      #endif
    } else {
      // Increase no update counter if the temperature stayed the same
      nNoUpdatesTemp++;
    }

    // Get humidity from DHT library
    float humidity = dht.getHumidity();
    if (isnan(humidity)) {
      //Serial.println("Failed reading humidity from DHT");
    } else if (humidity != lastHum || nNoUpdatesHum == FORCE_UPDATE_N_READS) {
      // Only send humidity if it changed since the last measurement or if we didn't send an update for n times
      lastHum = humidity;
      // Reset no updates counter
      nNoUpdatesHum = 0;
      send(msgHum.set(humidity, 1));

      #ifdef MY_DEBUG
      Serial.print("H: ");
      Serial.println(humidity);
      #endif
    } else {
      // Increase no update counter if the humidity stayed the same
      nNoUpdatesHum++;
    }
    numberOfDHTSkips = 0;
  }

  bool windowTripped = digitalRead(WINDOW_PIN) == HIGH;
  Serial.print(".");

  if (windowTrippedLastValue != windowTripped) {
    Serial.print("Tripped: ");
    Serial.println(windowTripped);
    send(msgMotion.set(windowTripped?"1":"0"));  // Send tripped value to gw
    windowTrippedLastValue = windowTripped;
  }

  // Sleep for a while to save energy
  wait(UPDATE_INTERVAL);
}
