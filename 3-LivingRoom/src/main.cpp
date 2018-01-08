#define MY_DEBUG
#define MY_RADIO_NRF24
#define MY_RF24_CHANNEL 83
#define MY_NODE_ID 3

#include <SPI.h>
#include <MySensors.h>
#include <DHT.h>
#include <RCSwitch.h>

#define RELAY_PIN 6
#define RCSWITCH_PIN 4
#define DHT_DATA_PIN 7
#define SENSOR_TEMP_OFFSET 0

#define CHILD_ID_TEMP 7
#define CHILD_ID_HUM 8

static const uint64_t UPDATE_INTERVAL = 60000;
static const uint8_t FORCE_UPDATE_N_READS = 10;

float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
bool metric = true;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
DHT dht;

RCSwitch mySwitch = RCSwitch();

void presentation() {
  sendSketchInfo("LivingRoom", "1.1");

  present(0, S_LIGHT);
  present(1, S_LIGHT);
  present(2, S_LIGHT);
  present(3, S_LIGHT);
  present(4, S_LIGHT);
  present(5, S_BINARY);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_HUM, S_HUM);

  metric = getControllerConfig().isMetric;
}


void setup() {
  dht.setup(DHT_DATA_PIN); // set data pin of DHT sensor
  if (UPDATE_INTERVAL <= dht.getMinimumSamplingPeriod()) {
    Serial.println("Warning: UPDATE_INTERVAL is smaller than supported by the sensor!");
  }
  sleep(dht.getMinimumSamplingPeriod());

  mySwitch.enableTransmit(RCSWITCH_PIN);

  mySwitch.setPulseLength(182);
  mySwitch.setRepeatTransmit(8);
}


void loop() {
  // Force reading sensor, so it works also after sleep()
  dht.readSensor(true);

  // Get temperature from DHT library
  float temperature = dht.getTemperature();
  if (isnan(temperature)) {
    Serial.println("Failed reading temperature from DHT!");
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
    Serial.println("Failed reading humidity from DHT");
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

  // Sleep for a while to save energy
  sleep(UPDATE_INTERVAL);
}

void receive(const MyMessage &message) {
  Serial.print("Received a message:");
  Serial.println(message.type);
  // We only expect one type of message from controller. But we better check anyway.
  if (message.type==V_STATUS) {
    //digitalWrite(message.sensor-1+RELAY_1, message.getBool()?RELAY_ON:RELAY_OFF);
    Serial.print("Incoming change for sensor:");
    Serial.print(message.sensor);
    Serial.print(", New status: ");
    Serial.println(message.getBool());

    if (message.getBool() == 0) {
      Serial.println("Turning off");

      switch (message.sensor) {
        case 0:
          mySwitch.send(5264700, 24);
          break;
        case 1:
          mySwitch.send(5264844, 24);
          break;
        case 2:
          mySwitch.send(5265164, 24);
          break;
        case 3:
          mySwitch.send(5266700, 24);
          break;
        case 4:
          mySwitch.send(5272844, 24);
          break;
        case 5;
          digitalWrite(RELAY_PIN, 0);
          break;
        default:
          Serial.println("Wrong ID");
          break;
        }
      } else {
        Serial.println("Turning on");
        switch (message.sensor) {
          case 0:
            mySwitch.send(5264691, 24);
            break;
          case 1:
            mySwitch.send(5264835, 24);
            break;
          case 2:
            mySwitch.send(5265155, 24);
            break;
          case 3:
            mySwitch.send(5266691, 24);
            break;
          case 4:
            mySwitch.send(5272835, 24);
            break;
          case 5;
            digitalWrite(RELAY_PIN, 1);
            break;
          default:
            Serial.println("Wrong ID");
          break;
          }
      }
  }
}
