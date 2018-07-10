#define DHTPIN 2     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302) 
//#define MY_DEBUG
#define MY_BAUD_RATE 38400
#define MY_RADIO_NRF24

#define MH_Z19_RX 7
#define MH_Z19_TX 6

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_CO2 2
#define MY_NODE_ID 25

#include "DHT.h"
#include <SoftwareSerial.h>
#include <MySensors.h> 

SoftwareSerial co2Serial(MH_Z19_RX, MH_Z19_TX); // define MH-Z19
DHT dht(DHTPIN, DHTTYPE);
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);

MyMessage msgCO2(CHILD_ID_CO2, V_LEVEL);
MyMessage msgCO2Prefix(CHILD_ID_CO2, V_UNIT_PREFIX);

int lastPPM = 0;
float lastHumidity = 0.0;
float lastTemperature = 0.0;

void presentation()  
{ 
  // Send the sketch version information to the gateway
  sendSketchInfo("room-sensor", "1.0");

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);

  present(CHILD_ID_CO2, S_AIR_QUALITY);
  send(msgCO2Prefix.set("ppm"));
}
 
void setup() {
  Serial.begin(38400); // Init console
  Serial.println("Setup started");
  
  dht.begin();
  co2Serial.begin(9600); //Init sensor MH-Z19(14)

  unsigned long previousMillis = millis();
  while (millis() - previousMillis < 10000)
    delay(1000);
  Serial.println("Setup finished");
}
 
void loop() {

  int ppm = readCO2();

  if (ppm < 100 || ppm > 6000) {
    Serial.println("PPM not valid");
  }

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius
  float t = dht.readTemperature();
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  #ifdef MY_DEBUG
  Serial.print(ppm);
  Serial.print(",");
  Serial.print(h);
  Serial.print(",");
  Serial.println(t);
  #endif

  h = round(h);

  if (lastHumidity != h) {
    send(msgHum.set(h, 1));
    lastHumidity = h;  
  }

  t = round(t * 2) / 2;

  if (lastTemperature != t) {
    send(msgTemp.set(t, 1));
    lastTemperature = t;
  }

  ppm = round(ppm / 10) * 10;

  if (lastPPM != ppm) {
    send(msgCO2.set(ppm, 1));
    lastPPM = ppm;  
  }
  
  delay(180000);
}


int readCO2()
{

  byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  // command to ask for data
  byte response[9]; // for answer

  co2Serial.write(cmd, 9); //request PPM CO2

  // The serial stream can get out of sync. The response starts with 0xff, try to resync.
  while (co2Serial.available() > 0 && (unsigned char)co2Serial.peek() != 0xFF) {
    co2Serial.read();
  }

  memset(response, 0, 9);
  co2Serial.readBytes(response, 9);

  if (response[1] != 0x86)
  {
    Serial.println("Invalid response from co2 sensor!");
    return -1;
  }

  byte crc = 0;
  for (int i = 1; i < 8; i++) {
    crc += response[i];
  }
  crc = 255 - crc + 1;

  if (response[8] == crc) {
    int responseHigh = (int) response[2];
    int responseLow = (int) response[3];
    int ppm = (256 * responseHigh) + responseLow;
    return ppm;
  } else {
    Serial.println("CRC error!");
    return -1;
  }
}

