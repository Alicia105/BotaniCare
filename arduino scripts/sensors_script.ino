#include <SoftwareSerial.h>
#include <time.h>
#include <ArduinoJson.h>
#include "dht.h"

SoftwareSerial mySerial(2,3); // RX, TX ;
dht sensor;

//define PIN sensors
#define TEMP_HUM 4 //Temperature and humidity sensor 
#define IR 5 //IR motion sensor 
#define SOIL_MOIST A5 //Soil moisture sensor
#define LIGHT 7 //Light sensor
#define BUZZ 8 //Buzzer

int ir=0;
int soil=0;
int light;

float tc=0;
float h=0;
float tf=0;

int detected=0;


void setup() {
  // open serial comm
  Serial.begin(9600);
  while(!Serial) {
    ; // wait for serial port to connect
  }
  mySerial.begin(38400);
  Serial.println("Bluetooth communication starting");
  pinMode(LIGHT,INPUT);
  pinMode(IR,INPUT);
  pinMode(SOIL_MOIST,INPUT);
  pinMode(BUZZ,OUTPUT);
  digitalWrite(BUZZ,LOW);
}

void startAnimalDetection(){
  Serial.println("Detection started");
}

void startBuzzer(){
  digitalWrite(BUZZ,HIGH);
  delay(5000);
  digitalWrite(BUZZ,LOW);
}

void getMeasures(){
  sensor.read(TEMP_HUM);
  tc=sensor.temperature;
  h=sensor.humidity;
  tf=tc*1.8+32;
  ir=digitalRead(IR);
  
  soil=analogRead(SOIL_MOIST);
  light=digitalRead(LIGHT);

  Serial.print(tc);
  Serial.print("C(");
  Serial.print(tf);
  Serial.print("F)\t ");

  Serial.print(h); 
  Serial.print("\t");

  Serial.print(ir); 
  Serial.print("\t");

  Serial.print(light); 
  Serial.print("\t");

  Serial.print(soil); 
  Serial.print("\n");
}

void sendMeasures(){
  // Create a JSON object
    StaticJsonDocument<200> doc;
    doc["temperature"] = tf;
    doc["humidity"]=h;
    doc["soilMoisture"]=soil;
    doc["sunlight"]=light;
    doc["motion"]=ir;

    // Serialize the JSON object
    char jsonBuffer[256];
    size_t n = serializeJson(doc, jsonBuffer);

    // Prints on serial monitor
    Serial.print(jsonBuffer);
    // Sends to raspberry pi
    mySerial.write(jsonBuffer, n);
    Serial.println("\tData sent");
}

void readDetectionResult(){
   if (mySerial.available()) {
    detected=mySerial.read();
  }
  //detected=mySerial.read();
  Serial.print("Detection status :");
  Serial.print(detected);
  Serial.print("\n");
}

void loop() {
  getMeasures();
  sendMeasures();
  if(ir==1){
    startAnimalDetection();
  }
  readDetectionResult();
  if(detected==49){
    startBuzzer();
    detected=0;
  }
  delay(1000); // Wait for 1000 millisecond(s)
}

