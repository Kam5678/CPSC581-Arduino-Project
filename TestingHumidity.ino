
/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-temperature-humidity-sensor
 * https://www.youtube.com/watch?v=puNIT8veyWU&ab_channel=SrishtiRobotics
 * https://arduinomodules.info/ky-018-photoresistor-module/
 * https://www.instructables.com/Arduino-Interfacing-With-LCD-Without-Potentiometer/
 * https://create.arduino.cc/projecthub/hrsajjad844/lcd-display-without-potentiometer-and-resistor-0d1357
 */

#include "DHT.h"
#define DHTPIN 7
#define DHTTYPE DHT11
#include <LiquidCrystal.h>
#include <FastLED.h>


#define LED_PIN     8
#define NUM_LEDS    50
#define SINGLELED 10
#define BUTTON 22
CRGB leds[NUM_LEDS];
int Contrast = 0;
int relay = 9;
bool waterLightOn = false;
int buttonState=0;
unsigned long myTime;
unsigned long myTimePrevious=0;
unsigned long nextMaximumWater=20000;
unsigned long nextMaximumIncrementWater=20000;

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

int sensorPin = 2; //define analog pin 2
int value = 0; 

DHT dht(DHTPIN, DHTTYPE);

void setup() {
 
  pinMode(SINGLELED, OUTPUT);
  pinMode(relay, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  
  Serial.begin(9600);
  dht.begin(); // initialize the sensor
  lcd.begin(16, 2);
  analogWrite(6, 60);
  //analogWrite(10, 100);
  //lcd.begin(16, 2);
  //lcd.noAutoscroll();
  //lcd.autoscroll();

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

  
}

void loop() {
  // wait a few seconds between measurements.

  //digitalWrite(SINGLELED, HIGH);
  myTime=millis();
  Serial.print("Time: ");
  if(myTime>nextMaximumWater){
    myTimePrevious=myTime;
    nextMaximumWater+=nextMaximumIncrementWater;
    waterLightOn=true;
  }
  //digitalWrite(relay, HIGH);
  if(myTime<15000){
    digitalWrite(relay, HIGH); 
  }
  else{
   // digitalWrite(relay, LOW); 
  }
  Serial.println(myTime-myTimePrevious);
  delay(300);

  //digitalWrite(SINGLELED, HIGH);

  buttonState = digitalRead(BUTTON);

  if(waterLightOn){
    digitalWrite(SINGLELED,HIGH);
  }
  
  if(buttonState == LOW)         // If button is pressing
    digitalWrite(SINGLELED, LOW);
    waterLightOn = false;
  
  //delay(1000);
  //digitalWrite(SINGLELED, LOW);

  //digitalWrite(relay, HIGH); // turn the bulb on 
  //delay(1000);              // wait for a second
  //digitalWrite(relay, LOW); // turn the bulb off 
  //delay(1000);              // wait for a second

    value = analogRead(sensorPin); 
  //Serial.println(value, DEC); // light intensity
  Serial.println(value);
                // high values for bright environment (Actually Opposite)
                // low values for dark environment  (Actually Opposite)
  delay(100); 

  // read humidity
  float humi  = dht.readHumidity();
  // read temperature as Celsius
  float tempC = dht.readTemperature();
  // read temperature as Fahrenheit
  float tempF = dht.readTemperature(true);

  // check if any reads failed
  if (isnan(humi) || isnan(tempC) || isnan(tempF)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humi);
    Serial.print("%");

    Serial.print("  |  "); 

    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.print("°C ~ ");
    Serial.print(tempF);
    Serial.println("°F");
  }


//  String temp = "Temp: ";
//  String tempTest = "20.3";
//  String c = "C         ";
//  
//  String hum = "Humidity: ";
//  String humiTest = "15.7";  
//  String per = "%     ";
//
//  String fin = temp + tempC + c;
//
//  String fin2 = hum + humi + per;
//
//  String fin3 = fin + fin2;

String t = "T: 20.3 C";
String h = "H: 15.7% ";
String m = t + h;

  //delay(2000);

  //lcd.clear();
//  lcd.setCursor(0, 0);  
 // lcd.print(t);
  //lcd.print(fin2);
//  lcd.setCursor(0, 1);
//  lcd.print(h);

//  delay(2000);
  // delay(3000);
  //delay(2500);
  //delay(2000);
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(tempC);
  lcd.print(" C");
//
  //delay(2000);
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humi);
  lcd.print("%");
  //delay(2000);

  /*
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
  leds[1] = CRGB(0, 255, 0);
  FastLED.show();
  leds[2] = CRGB(0, 0, 255);
  FastLED.show();
  leds[5] = CRGB(150, 0, 255);
  FastLED.show();
  leds[9] = CRGB(255, 200, 20);
  FastLED.show();
  leds[14] = CRGB(85, 60, 180);
  FastLED.show();
  leds[19] = CRGB(50, 255, 20);
  FastLED.show();
  leds[25] = CRGB(50, 255, 20);
  FastLED.show();
  */
   
  if(value>700){
    
    leds[0] = CRGB(85, 60, 180);
    leds[1] = CRGB(85, 60, 180);
    leds[2] = CRGB(85, 60, 180);
    leds[3] = CRGB(85, 60, 180);
    leds[4] = CRGB(85, 60, 180);
    leds[5] = CRGB(85, 60, 180);
    leds[6] = CRGB(85, 60, 180);
    leds[7] = CRGB(85, 60, 180);
    leds[8] = CRGB(85, 60, 180);
    leds[9] = CRGB(85, 60, 180);
    leds[10] = CRGB(85, 60, 180);
    leds[11] = CRGB(85, 60, 180);
    leds[12] = CRGB(85, 60, 180);
    leds[13] = CRGB(85, 60, 180);
    leds[14] = CRGB(85, 60, 180);
    leds[15] = CRGB(85, 60, 180);
    leds[16] = CRGB(85, 60, 180);
    leds[17] = CRGB(85, 60, 180);
    leds[18] = CRGB(85, 60, 180);
    leds[19] = CRGB(85, 60, 180);
    leds[20] = CRGB(85, 60, 180);
    leds[21] = CRGB(85, 60, 180);
    leds[22] = CRGB(85, 60, 180);
    leds[23] = CRGB(85, 60, 180);
    leds[24] = CRGB(85, 60, 180);
    leds[25] = CRGB(85, 60, 180);
    leds[26] = CRGB(85, 60, 180);
    leds[27] = CRGB(85, 60, 180);
    leds[28] = CRGB(85, 60, 180);
    leds[29] = CRGB(85, 60, 180);
    leds[30] = CRGB(85, 60, 180);
    leds[31] = CRGB(85, 60, 180);
    leds[32] = CRGB(85, 60, 180);
    leds[33] = CRGB(85, 60, 180);
    leds[34] = CRGB(85, 60, 180);
    leds[35] = CRGB(85, 60, 180);
    leds[36] = CRGB(85, 60, 180);
    leds[37] = CRGB(85, 60, 180);
    leds[38] = CRGB(85, 60, 180);
    leds[39] = CRGB(85, 60, 180);
    leds[40] = CRGB(85, 60, 180);    
    leds[41] = CRGB(85, 60, 180);
    leds[42] = CRGB(85, 60, 180);
    leds[43] = CRGB(85, 60, 180);
    leds[44] = CRGB(85, 60, 180);
    leds[45] = CRGB(85, 60, 180);
    leds[46] = CRGB(85, 60, 180);
    leds[47] = CRGB(85, 60, 180);
    leds[48] = CRGB(85, 60, 180);
    leds[49] = CRGB(85, 60, 180);
    leds[50] = CRGB(85, 60, 180);  
    FastLED.show();
  }
  else{
    leds[0] = CRGB(0, 0, 0);
    leds[1] = CRGB(0, 0, 0);
    leds[2] = CRGB(0, 0, 0);
    leds[3] = CRGB(0, 0, 0);
    leds[4] = CRGB(0, 0, 0);
    leds[5] = CRGB(0, 0, 0);
    leds[6] = CRGB(0, 0, 0);
    leds[7] = CRGB(0, 0, 0);
    leds[8] = CRGB(0, 0, 0);
    leds[9] = CRGB(0, 0, 0);
    leds[10] = CRGB(0, 0, 0);
    leds[11] = CRGB(0, 0, 0);
    leds[12] = CRGB(0, 0, 0);
    leds[13] = CRGB(0, 0, 0);
    leds[14] = CRGB(0, 0, 0);
    leds[15] = CRGB(0, 0, 0);
    leds[16] = CRGB(0, 0, 0);
    leds[17] = CRGB(0, 0, 0);
    leds[18] = CRGB(0, 0, 0);
    leds[19] = CRGB(0, 0, 0);
    leds[20] = CRGB(0, 0, 0);
    leds[21] = CRGB(0, 0, 0);
    leds[22] = CRGB(0, 0, 0);
    leds[23] = CRGB(0, 0, 0);
    leds[24] = CRGB(0, 0, 0);
    leds[25] = CRGB(0, 0, 0);
    leds[26] = CRGB(0, 0, 0);
    leds[27] = CRGB(0, 0, 0);
    leds[28] = CRGB(0, 0, 0);
    leds[29] = CRGB(0, 0, 0);
    leds[30] = CRGB(0, 0, 0);
    leds[31] = CRGB(0, 0, 0);
    leds[32] = CRGB(0, 0, 0);
    leds[33] = CRGB(0, 0, 0);
    leds[34] = CRGB(0, 0, 0);
    leds[35] = CRGB(0, 0, 0);
    leds[36] = CRGB(0, 0, 0);
    leds[37] = CRGB(0, 0, 0);
    leds[38] = CRGB(0, 0, 0);
    leds[39] = CRGB(0, 0, 0);
    leds[40] = CRGB(0, 0, 0);
    leds[41] = CRGB(0, 0, 0);
    leds[42] = CRGB(0, 0, 0);
    leds[43] = CRGB(0, 0, 0);
    leds[44] = CRGB(0, 0, 0);
    leds[45] = CRGB(0, 0, 0);
    leds[46] = CRGB(0, 0, 0);
    leds[47] = CRGB(0, 0, 0);
    leds[48] = CRGB(0, 0, 0);
    leds[49] = CRGB(0, 0, 0);
    leds[50] = CRGB(0, 0, 0);    
    FastLED.show();
  
  }
    
}
