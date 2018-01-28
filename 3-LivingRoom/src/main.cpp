#define MY_DEBUG
#define MY_RADIO_NRF24
#define MY_RF24_CHANNEL 83
#define MY_NODE_ID 3

#include <SPI.h>
#include <MySensors.h>
#include <DHT.h>
#include <RCSwitch.h>

#define RCSWITCH_PIN 4
#define RELAY_PIN 6
#define DHT_DATA_PIN 7

#define CHILD_ID_SMOKE 6
#define CHILD_ID_CO 7
#define CHILD_ID_LPG 8
#define CHILD_ID_TEMP 9
#define CHILD_ID_HUM 10

static const uint64_t UPDATE_INTERVAL = 60000;

//--------------------- TEMP SENSOR
static const uint8_t FORCE_UPDATE_N_READS = 10;

#define SENSOR_TEMP_OFFSET 0
float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
bool metric = true;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);

//--------------------- SMOKE SENSOR
#define MQ_SENSOR_ANALOG_PIN 0
#define RL_VALUE 10                      //define the load resistance on the board, in kilo ohms
#define RO_CLEAN_AIR_FACTOR 9.83        //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
//which is derived from the chart in datasheet
/***********************Software Related Macros************************************/
#define CALIBARAION_SAMPLE_TIMES 50     //define how many samples you are going to take in the calibration phase
#define CALIBRATION_SAMPLE_INTERVAL 500 //define the time interal(in milisecond) between each samples in the
//cablibration phase
#define READ_SAMPLE_INTERVAL 50         //define how many samples you are going to take in normal operation
#define READ_SAMPLE_TIMES 5             //define the time interal(in milisecond) between each samples in
//normal operation
/**********************Application Related Macros**********************************/
#define GAS_LPG 0
#define GAS_CO 1
#define GAS_SMOKE 2
/*****************************Globals***********************************************/
unsigned long SLEEP_TIME = 30000; // Sleep time between reads (in milliseconds)
//VARIABLES
float Ro = 10000.0;    // this has to be tuned 10K Ohm
int val = 0;           // variable to store the value coming from the sensor
float valMQ =0.0;
float lastMQ =0.0;
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59)
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15)
float           SmokeCurve[3] = {2.3,0.53,-0.44};   //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.53), point2:(lg10000,-0.22)

MyMessage msgSmoke(CHILD_ID_SMOKE, V_LEVEL);
MyMessage msgCO(CHILD_ID_CO, V_LEVEL);
MyMessage msgLPG(CHILD_ID_LPG, V_LEVEL);

//--------------------------------

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
  present(CHILD_ID_SMOKE, S_AIR_QUALITY);
  present(CHILD_ID_CO, S_AIR_QUALITY);
  present(CHILD_ID_LPG, S_AIR_QUALITY);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_HUM, S_HUM);

  metric = getControllerConfig().isMetric;
}

void receive(const MyMessage &message) {
  //Serial.print("Received a message:");
  //Serial.println(message.type);
  // We only expect one type of message from controller. But we better check anyway.
  if (message.type==V_STATUS) {
    //digitalWrite(message.sensor-1+RELAY_1, message.getBool()?RELAY_ON:RELAY_OFF);
    //Serial.print("Incoming change for sensor:");
    //Serial.print(message.sensor);
    //Serial.print(", New status: ");
    //Serial.println(message.getBool());

    if (message.getBool() == 0) {
      //Serial.println("Turning off");

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
        case 5:
        digitalWrite(RELAY_PIN, LOW);
        break;
        // Turn on relay: 3;5;1;1;2;1
        default:
        //Serial.println("Wrong ID");
        break;
      }
    } else {
      //Serial.println("Turning on");
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
        case 5:
        digitalWrite(RELAY_PIN, HIGH);
        break;
        default:
        //Serial.println("Wrong ID");
        break;
      }
    }
  }
}

float MQResistanceCalculation(int raw_adc)  {
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

float MQCalibration(int mq_pin)  {
  int i;
  float val=0;

  for (i=0; i<CALIBARAION_SAMPLE_TIMES; i++) {          //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value

  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro
  //according to the chart in the datasheet

  return val;
}

float MQRead(int mq_pin)  {
  int i;
  float rs=0;

  for (i=0; i<READ_SAMPLE_TIMES; i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
  rs = rs/READ_SAMPLE_TIMES;
  return rs;
}

int  MQGetPercentage(float rs_ro_ratio, float *pcurve)  {
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

int MQGetGasPercentage(float rs_ro_ratio, int gas_id)  {
  if ( gas_id == GAS_LPG ) {
    return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
    return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
    return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }
  return 0;
}

void setup() {
  dht.setup(DHT_DATA_PIN); // set data pin of DHT sensor
  pinMode(RELAY_PIN, OUTPUT);

  if (UPDATE_INTERVAL <= dht.getMinimumSamplingPeriod()) {
    Serial.println("Warning: UPDATE_INTERVAL is smaller than supported by the sensor!");
  }
  sleep(dht.getMinimumSamplingPeriod());

  mySwitch.enableTransmit(RCSWITCH_PIN);

  mySwitch.setPulseLength(182);
  mySwitch.setRepeatTransmit(8);

  Ro = MQCalibration(MQ_SENSOR_ANALOG_PIN);         //Calibrating the sensor. Please make sure the sensor is in clean air
}

void loop() {
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

  //---------------------  MQ-2
  uint16_t valMQ = MQGetGasPercentage(MQRead(MQ_SENSOR_ANALOG_PIN)/Ro,GAS_CO);
  Serial.print("MQ: ");
  Serial.println(valMQ);

  //Serial.print("LPG:");
  //Serial.print(MQGetGasPercentage(MQRead(MQ_SENSOR_ANALOG_PIN)/Ro,GAS_LPG) );
  //Serial.print( "ppm" );
  //Serial.print("    ");
  //Serial.print("CO:");
  //Serial.print(MQGetGasPercentage(MQRead(MQ_SENSOR_ANALOG_PIN)/Ro,GAS_CO) );
  //Serial.print( "ppm" );
  //Serial.print("    ");
  //Serial.print("SMOKE:");
  //Serial.print(MQGetGasPercentage(MQRead(MQ_SENSOR_ANALOG_PIN)/Ro,GAS_SMOKE) );
  //Serial.print( "ppm" );
  //Serial.print("\n");

  if (valMQ != lastMQ) {
    send(msgSmoke.set(MQGetGasPercentage(MQRead(MQ_SENSOR_ANALOG_PIN)/Ro,GAS_SMOKE)));
    send(msgCO.set(MQGetGasPercentage(MQRead(MQ_SENSOR_ANALOG_PIN)/Ro,GAS_CO)));
    send(msgLPG.set(MQGetGasPercentage(MQRead(MQ_SENSOR_ANALOG_PIN)/Ro,GAS_LPG)));
    lastMQ = ceil(valMQ);
  }

  // Sleep for a while to save energy
  wait(UPDATE_INTERVAL);
}
