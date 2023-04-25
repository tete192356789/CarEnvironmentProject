#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <AHT10.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <TridentTD_LineNotify.h>
#include <PubSubClient.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>

/////LCD Setup///////////////
LiquidCrystal_I2C lcd(0x3F, 20, 4);

//////WIFI Setup/////////////
#define SSID "Nachanon"
#define PASSWORD "abcdefgh"

///////LINE Setup/////////////
#define LINE_TOKEN "kCuLkDuxCGxZ2bHlkbobBoM8NLLo1NoyGtM6KK0RKzB"

//////////MQTT Setup/////////////
const char* mqtt_server = "broker.emqx.io";
WiFiClient espClient;
PubSubClient client(espClient);
const char* topic = "MQTT_CAR_SENSE";

/////////GPS Setup///////////////////
#define RXPin (16)
#define TXPin (17)
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;

HardwareSerial ss(2);
////////bmp+aht setup/////////////
Adafruit_BMP280 bmp;
AHT10 myAHT20(AHT10_ADDRESS_0X38, AHT20_SENSOR);


/////////MQ-2//////////////////
#define MQ_PIN (35)                 //define which analog input channel you are going to use
#define RL_VALUE (5)                //define the load resistance on the board, in kilo ohms
#define RO_CLEAN_AIR_FACTOR (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO, \
                                    //which is derived from the chart in datasheet

#define CALIBARAION_SAMPLE_TIMES (50)      //define how many samples you are going to take in the calibration phase
#define CALIBRATION_SAMPLE_INTERVAL (500)  //define the time interal(in milisecond) between each samples in the \
                                           //cablibration phase
#define READ_SAMPLE_INTERVAL (50)          //define how many samples you are going to take in normal operation
#define READ_SAMPLE_TIMES (5)              //define the time interal(in milisecond) between each samples in \
                                           //normal operation


#define GAS_LPG (0)
#define GAS_CO (1)
#define GAS_SMOKE (2)

/*****************************Globals***********************************************/
float LPGCurve[3] = { 2.3, 0.21, -0.47 };    //two points are taken from the curve.
                                             //with these two points, a line is formed which is "approximately equivalent"
                                             //to the original curve.
                                             //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59)
float COCurve[3] = { 2.3, 0.72, -0.34 };     //two points are taken from the curve.
                                             //with these two points, a line is formed which is "approximately equivalent"
                                             //to the original curve.
                                             //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15)
float SmokeCurve[3] = { 2.3, 0.53, -0.44 };  //two points are taken from the curve.
                                             //with these two points, a line is formed which is "approximately equivalent"
                                             //to the original curve.
                                             //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)
float Ro = 10;                               //Ro is initialized to 10 kilo ohms
/////////////////////////////////////////////////////



int val = 0;
float MQCalibration(int mq_pin);

void callback(char* topic, byte* payload, unsigned int length) {
  // Handle MQTT message received
}

void reconnect() {

  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe("temperature");
      client.subscribe("humidity");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  lcd.begin();
  lcd.backlight();


  ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin, false);
  Serial.println(TinyGPSPlus::libraryVersion());

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  WiFi.begin(SSID, PASSWORD);
  Serial.printf("WiFi connecting ", SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.printf("\nWiFi connected\nIP : ");
  Serial.println(WiFi.localIP());
  LINE.setToken(LINE_TOKEN);

  while (myAHT20.begin() != true) {
    Serial.println(F("AHT20 not connected or fail to load calibration coefficient"));
    delay(5000);
  }
  Serial.println(F("AHT20 OK"));

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1)
      ;
  }


  Serial.print("Calibrating...\n");
  Ro = MQCalibration(MQ_PIN);  
  Serial.print("Calibration is done...\n");
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");


  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {
  val = analogRead(MQ_PIN);  
  Serial.print("val = ");    
  Serial.println(val);       

  float LPG = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_LPG);
  Serial.print("LPG:");
  Serial.print(LPG);
  Serial.print("ppm");
  Serial.print("    ");

  float CO = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_CO);
  Serial.print("CO:");
  Serial.print(CO);
  Serial.print("ppm");
  Serial.print("    ");

  float SMOKE = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_SMOKE);
  Serial.print("SMOKE:");
  Serial.print(SMOKE);
  Serial.print("ppm");
  Serial.print("\n");

  float temperature = myAHT20.readTemperature();
  Serial.print("อุณหภูมิ : ");
  Serial.print(temperature);
  Serial.println(" *C");

  float humidity = myAHT20.readHumidity();
  Serial.print("ความชื้น : ");
  Serial.print(humidity);
  Serial.println(" RH");

  float pressure = bmp.readPressure();
  Serial.print("ความกดอากาศ : ");
  Serial.print(pressure);
  Serial.println(" hPa");
  Serial.println(" ");




  double lat = 0;
  double lng = 0;

  while (ss.available() > 0)
    if (gps.encode(ss.read()))
  

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS detected: check wiring."));
    // while (true)
    //   ;
  }
  displayInfo();

  lat = gps.location.lat();

  lng = gps.location.lng();


  char location[50];
  sprintf(location, "%.4f,%+.4f", lat, lng);

  LINE.notify("Temperature: " + String(temperature) + "/" + "Humidity: " + String(humidity) + "/" + "Pressure: " + String(pressure) + "/" + "LPG : " + String(LPG) + "/" + "CO: " + String(CO) + "/" + "SMOKE: " + String(SMOKE));

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
 

  DynamicJsonDocument doc(200);
  doc["analogValue"] = analogRead(MQ_PIN);
  doc["LPG"] = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_LPG);
  doc["CO"] = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_CO);
  doc["smoke"] = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_SMOKE);
  doc["temperature"] = myAHT20.readTemperature();
  doc["humidity"] = myAHT20.readHumidity();
  doc["pressure"] = bmp.readPressure();
  char payload[200];
  serializeJson(doc, payload);

  client.publish("sensor_data/7", payload);

  DynamicJsonDocument doc2(200);
  doc2["lat"] = gps.location.lat();
  doc2["lon"] = gps.location.lng();
  char payload2[200];
  serializeJson(doc2, payload2);

  client.publish("latlon_data/7", payload2);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temperature:");
  lcd.setCursor(13, 0);
  lcd.print(temperature);
  lcd.setCursor(0, 1);
  lcd.print("Humidity:");
  lcd.setCursor(13, 1);
  lcd.print(humidity);
  lcd.setCursor(0, 2);
  lcd.print("Pressure:");
  lcd.setCursor(9, 2);
  lcd.print(pressure);
  delay(5000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LPG:");
  lcd.setCursor(13, 0);
  lcd.print(LPG);
  lcd.setCursor(0, 1);
  lcd.print("SMOKE:");
  lcd.setCursor(13, 1);
  lcd.print(SMOKE);
  lcd.setCursor(0, 2);
  lcd.print("CO:");
  lcd.setCursor(13, 2);
  lcd.print(CO);



 


  delay(1000);
}

void displayInfo() {
  Serial.print(F("Location: "));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

float MQResistanceCalculation(int raw_adc) {
  return (((float)RL_VALUE * (1023 - raw_adc) / raw_adc));
}


float MQCalibration(int mq_pin) {
  int i;
  float val = 0;

  for (i = 0; i < CALIBARAION_SAMPLE_TIMES; i++) {  //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val / CALIBARAION_SAMPLE_TIMES;  //calculate the average value

  val = val / RO_CLEAN_AIR_FACTOR;  //divided by RO_CLEAN_AIR_FACTOR yields the Ro
                                    //according to the chart in the datasheet

  return val;
}

float MQRead(int mq_pin) {
  int i;
  float rs = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs / READ_SAMPLE_TIMES;

  return rs;
}


int MQGetGasPercentage(float rs_ro_ratio, int gas_id) {
  if (gas_id == GAS_LPG) {
    return MQGetPercentage(rs_ro_ratio, LPGCurve);
  } else if (gas_id == GAS_CO) {
    return MQGetPercentage(rs_ro_ratio, COCurve);
  } else if (gas_id == GAS_SMOKE) {
    return MQGetPercentage(rs_ro_ratio, SmokeCurve);
  }

  return 0;
}


int MQGetPercentage(float rs_ro_ratio, float* pcurve) {
  return (pow(10, (((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0])));
}
