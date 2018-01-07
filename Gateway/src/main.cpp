#include <Arduino.h>

#define MY_DEBUG

#define MY_RADIO_NRF24

#define MY_RF24_PA_LEVEL RF24_PA_LOW

#define MY_GATEWAY_SERIAL

#define MY_INCLUSION_MODE_FEATURE
#define MY_INCLUSION_BUTTON_FEATURE

#define MY_INCLUSION_MODE_DURATION 60
#define MY_INCLUSION_MODE_BUTTON_PIN  3
#define MY_DEFAULT_LED_BLINK_PERIOD 300

#define MY_DEFAULT_ERR_LED_PIN 4  // Error led pin
#define MY_DEFAULT_RX_LED_PIN  6  // Receive led pin
#define MY_DEFAULT_TX_LED_PIN  5  // the PCB, on board LED

#define MY_RF24_CHANNEL 83

#include <MySensors.h>

void setup()
{
    // Setup locally attached sensors
}

void presentation()
{
    // Present locally attached sensors
}

void loop()
{
    // Send locally attached sensor data here
}
