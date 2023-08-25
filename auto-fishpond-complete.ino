/*
  pH = A0, TdsSensorPin = A4, waterLevel = A10
  Temp = 33, nodeSerial(12, 11),['
  inFlowSensorPin = 2, outFlowSensorPin = 3,
  inPump = 7, outPump = 9,
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

#include <SPI.h>
#include <SD.h>
File myFile;
const int CS_PIN = 10;

#include <EEPROM.h> // For Tds Sensor
#include "GravityTDS.h"

#define pHSensorPin A0  // the pH meter Analog output is connected with the Arduino’s Analog
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is connected to the Arduino digital pin 4
#define  ONE_WIRE_BUS 33

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);
float temperature;

LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 20 chars and 4 line display

SoftwareSerial nodeSerial(12, 11);

#define TdsSensorPin A4
GravityTDS gravityTds;
float tdsTemperature = 25, tdsValue = 0, toxicTdsVal = 450.0;

unsigned long int avgValue;  //Store the average value of the sensor feedback
const float phCalValue = 14.95;
float ph;
int buf[10], temp, phToxicCount = 0, tdsToxicCount = 0, temperatureToxicCount = 0;

const byte inFlowSensorInterrupt = 5;  // 5 = digital pin 3
const byte inFlowSensorPin = 3;

const byte outFlowSensorInterrupt = 4;  // 4 = digital pin 2
const byte outFlowSensorPin = 2;

const int inPump = 7;
const int outPump = 9;

const float calibrationFactor = 4.5;

volatile byte pulseInCount, pulseOutCount;

float flowRateIn, flowRateOut;
unsigned int flowMilliLitresIn, flowMilliLitresOut;
unsigned long totalMilliLitresIn = 0, totalMilliLitresOut = 0;
bool keepPumpingIn = false, keepPumpingOut = false;

unsigned long oldTimeIn, oldTimeOut, dataTransTime;

int topWaterLevelSensorPin = A10, bottomWaterLevelSensorPin = A14, overflowWaterLevelSensorPin = A12;
String waterRising = "No";

void setup() {
  Serial.begin(115200);
  nodeSerial.begin(9600);

  Serial.println("Initializing Card");
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  if (!SD.begin(CS_PIN))  {
    Serial.println("Card Failure");
    //    return;
  }
  Serial.println("Card Ready");

  lcd.init();  // initialize the lcd
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(7, 1);
  lcd.print(F("System"));
  lcd.setCursor(6, 2);
  lcd.print(F("Starting"));
  sensors.begin();
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization
  delay(2000);

  // locate devices on the bus
  Serial.print(F("Found: "));
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(F(" Devices."));

  pinMode(inPump, OUTPUT);
  pinMode(outPump, OUTPUT);

  digitalWrite(inPump, LOW);
  digitalWrite(outPump, LOW);

  pinMode(inFlowSensorPin, INPUT);
  digitalWrite(inFlowSensorPin, HIGH);
  pinMode(outFlowSensorPin, INPUT);
  digitalWrite(outFlowSensorPin, HIGH);

  pulseInCount = 0;
  pulseOutCount = 0;
  flowRateIn = 0.0;
  flowRateOut = 0.0;
  flowMilliLitresIn = 0;
  flowMilliLitresOut = 0;
  totalMilliLitresIn = 0;
  totalMilliLitresOut = 0;
  oldTimeIn = 0;
  oldTimeOut = 0;

  attachInterrupt(inFlowSensorInterrupt, pulseCounterIn, FALLING);
  attachInterrupt(outFlowSensorInterrupt, pulseCounterOut, FALLING);
  delay(500);
  lcd.clear();
}

void loop() {
  Serial.println(F("Running . . ."));
  checkOverflowWaterLevel();
  monitorPh();
  monitorTemperature();
  measureTds();
  displayGeneralMonitor();
  saveData();
  sendDataToNode();
  delay(5000);
}

void sendDataToNode() {
  String data = String(ph, 2) + ", " + String(temperature, 1) + ", " + String(flowRateIn * 100) + ", " + String(totalMilliLitresIn * 100) + ", " + String(flowRateOut * 100) + ", " + String(totalMilliLitresOut * 100) + ", " + String(tdsValue);
  Serial.println(data);
  nodeSerial.println(data);
}

void saveData() {
  myFile = SD.open("autopond.txt", FILE_WRITE);
  if (myFile)
  {
    myFile.print(ph, 2);                        myFile.print(F(","));
    myFile.print(temperature, 1);               myFile.print(F(","));
    myFile.print(flowRateIn * 100);             myFile.print(F(","));
    myFile.print(totalMilliLitresIn * 100);     myFile.print(F(","));
    myFile.print(flowRateOut * 100);            myFile.print(F(","));
    myFile.print(totalMilliLitresOut * 100);    myFile.print(F(","));
    myFile.println(tdsValue);                   myFile.close();
    Serial.println(F("Data written to log file: autopond"));
  } else {
    Serial.println(F("Couldn't open log file: autopond"));
  }
}

void startPumpingProcedure() {
  pumpOut();
}

void measureTds() {
  gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
  gravityTds.update();  //sample and calculate
  tdsValue = gravityTds.getTdsValue();  // then get the value
  Serial.print(tdsValue, 0);
  Serial.println("ppm");

  if (tdsValue > toxicTdsVal) {
    tdsToxicCount++;
    if (tdsToxicCount >= 5) {
      Serial.println("Too much dissolved solids in the water");
      startPumpingProcedure();
    }
  } else {
    tdsToxicCount = 0;
    Serial.print("-");
  }
}

void checkOverflowWaterLevel() {
  int overflowWaterLevel = 0;
  for (int i = 0; i < 5; i++) {
    int overflowWaterLevelValue = analogRead(overflowWaterLevelSensorPin);
    Serial.print(String(overflowWaterLevelValue) + "\t");
    overflowWaterLevel += overflowWaterLevelValue;
    delay(50);
  }
  overflowWaterLevel = overflowWaterLevel / 5;
  Serial.println("\noverflowWaterLevel: " + String(overflowWaterLevel));

  if (overflowWaterLevel >= 1000) {
    Serial.println("Water level still in range");
    waterRising = "No";
  }
  else {
    Serial.println("Water overflow >>>");
    waterRising = "Yes";
    delay(500);
    pumpOutExcessWater();
  }
}

void pumpOutExcessWater() {
  keepPumpingOut = true;
  lcd.clear();
  while (keepPumpingOut == true) {
    digitalWrite(outPump, HIGH);
    if ((millis() - oldTimeOut) > 1000)  // Only process counters once per second
    {
      detachInterrupt(outFlowSensorInterrupt);
      flowRateOut = ((1000.0 / (millis() - oldTimeOut)) * pulseOutCount) / calibrationFactor;
      if (flowRateOut <= 0.00) {
        flowRateOut = 0.21;
      }
      oldTimeOut = millis();

      flowMilliLitresOut = (flowRateOut / 60) * 1000;
      totalMilliLitresOut += flowMilliLitresOut;

      Serial.println("\nFlow rate out: " + String(flowRateOut) + "L/min");
      Serial.println("Flow rate out: " + String(int(flowRateOut)) + "." + String((flowRateOut - int(flowRateOut)) * 10, DEC) + "L/min");
      Serial.println("Current Water Flowing out: " + String(flowMilliLitresOut) + "mL/Sec");  // Output separator
      Serial.println("Total Water Out: " + String(totalMilliLitresOut) + "mL");
      displayPumpingOut();
      // Reset the pulse counter so we can start incrementing again
      pulseOutCount = 0;
      saveData();
      //      sendDataToNode();

      // Enable the interrupt again now that we've finished sending output
      attachInterrupt(outFlowSensorInterrupt, pulseCounterOut, FALLING);
    }
    int topWaterLevelValue = analogRead(topWaterLevelSensorPin);
    Serial.print(" " + String(topWaterLevelValue));
    if (topWaterLevelValue >= 700) {
      //      Serial.print("Keep pumping out");
    } else {
      Serial.println("\nStop pumping out");
      keepPumpingOut = false;
      break;
    }
  }
  saveData();
  nodeSerial.flush();
  sendDataToNode();
  stopPumpOut();
}

void monitorPh() {
  for (int i = 0; i < 10; i++)  //Get 10 sample value from the sensor for smooth the value
  {
    buf[i] = analogRead(pHSensorPin);
    delay(10);
  }
  for (int i = 0; i < 9; i++)  //sort the analog from small to large
  {
    for (int j = i + 1; j < 10; j++) {
      if (buf[i] > buf[j]) {
        temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
      }
    }
  }
  avgValue = 0;
  for (int i = 2; i < 8; i++)  //take the average value of 6 center sample
    avgValue += buf[i];
  float phValue = (float)avgValue * 5.0 / 1024 / 6;  //convert the analog into millivolt
  phValue = phCalValue - (3.5 * phValue);            //convert the millivolt into pH value
  ph = abs(phValue);
  Serial.print("    pH:");
  Serial.print(ph, 2);
  Serial.println(" ");

  if (phValue < 5.0) {
    phToxicCount++;
    if (phToxicCount >= 5) {
      Serial.println("PH is now toxic: Acidic, start Pumping Procedure");
    }
  } else if (phValue > 8.0) {
    phToxicCount++;
    if (phToxicCount >= 5) {
      Serial.println("PH is now toxic: Basic, start Pumping Procedure");
    }
  } else {
    phToxicCount = 0;
  }
  delay(50);
}

void monitorTemperature() {
  sensors.requestTemperatures();
  temperature = sensors.getTempFByIndex(0);
  Serial.print(F("Fahrenheit temperature: "));
  Serial.println(temperature);

  if (temperature < 65.0) {
    temperatureToxicCount++;
    if (temperatureToxicCount >= 5) {
      Serial.println(F("Temperature is too low"));
    }
  } else if (temperature > 85.0) {
    temperatureToxicCount++;
    if (temperatureToxicCount >= 5) {
      Serial.println(F("Temperature is too high"));
    }
  } else {
    temperatureToxicCount = 0;
  }
  delay(50);
}

void pumpIn() {
  keepPumpingIn = true;
  Serial.println(F("Pumping in now"));
  digitalWrite(inPump, HIGH);
  monitorInPumpFlow();
}

void stopPumpIn() {
  keepPumpingIn = false;
  digitalWrite(inPump, LOW);
  phToxicCount = 0;
  tdsToxicCount = 0;
  flowRateIn = 0;
  flowMilliLitresIn = 0;
  totalMilliLitresIn = 0;
  Serial.println(F("Pumping in DONE, all values reset"));
  lcd.clear();
}

void pumpOut() {
  keepPumpingOut = true;
  Serial.println(F("Pumping out"));
  digitalWrite(outPump, HIGH);
  monitorOutPumpFlow();
}

void stopPumpOut() {
  keepPumpingOut = false;
  digitalWrite(outPump, LOW);
  flowRateOut = 0;
  flowMilliLitresOut = 0;
  totalMilliLitresOut = 0;
  Serial.println(F("Pumping out DONE"));
  lcd.clear();
}

void monitorInPumpFlow() {
  lcd.clear();
  while (keepPumpingIn == true) {
    if ((millis() - oldTimeIn) > 1000)  // Only process counters once per second
    {
      detachInterrupt(inFlowSensorInterrupt);
      flowRateIn = ((1000.0 / (millis() - oldTimeIn)) * pulseInCount) / calibrationFactor;
      if (flowRateIn <= 0.00) {
        flowRateIn = 0.21;
      }
      oldTimeIn = millis();

      flowMilliLitresIn = (flowRateIn / 60) * 1000;
      totalMilliLitresIn += flowMilliLitresIn;

      Serial.println("\nFlow rate out: " + String(flowRateOut) + "L/min");
      Serial.println("\nFlow rate in: " + String(int(flowRateIn)) + "." + String((flowRateIn - int(flowRateIn)) * 10, DEC) + "L/min");
      Serial.println("Current Water Flowing in: " + String(flowMilliLitresIn) + "mL/Sec");  // Output separator
      Serial.println("Total Water In: " + String(totalMilliLitresIn) + "mL");
      displayPumpingIn();
      // Reset the pulse counter so we can start incrementing again
      pulseInCount = 0;
      saveData();
      //      sendDataToNode();

      // Enable the interrupt again now that we've finished sending output
      attachInterrupt(inFlowSensorInterrupt, pulseCounterIn, FALLING);
    }
    int topWaterLevelValue = analogRead(topWaterLevelSensorPin);
    //    Serial.print(" " + String(topWaterLevelValue));
    if (topWaterLevelValue <= 700) {
      //      Serial.println("Keep pumping in");
    } else {
      Serial.println("\nStop pumping in");
      keepPumpingIn = false;
      break;
    }
  }
  saveData();
  nodeSerial.flush();
  sendDataToNode();
  stopPumpIn();
}

void monitorOutPumpFlow() {
  lcd.clear();
  while (keepPumpingOut == true) {
    digitalWrite(outPump, HIGH);
    if ((millis() - oldTimeOut) > 1000)  // Only process counters once per second
    {
      detachInterrupt(outFlowSensorInterrupt);
      flowRateOut = ((1000.0 / (millis() - oldTimeOut)) * pulseOutCount) / calibrationFactor;
      if (flowRateOut <= 0.00) {
        flowRateOut = 0.21;
      }
      oldTimeOut = millis();

      flowMilliLitresOut = (flowRateOut / 60) * 1000;
      totalMilliLitresOut += flowMilliLitresOut;

      Serial.println("\nFlow rate out: " + String(flowRateOut) + "L/min");
      Serial.println("Current Water Flowing out: " + String(flowMilliLitresOut) + "mL/Sec");  // Output separator
      Serial.println("Total Water Out: " + String(totalMilliLitresOut) + "mL");
      displayPumpingOut();
      // Reset the pulse counter so we can start incrementing again
      pulseOutCount = 0;
      saveData();
      //      sendDataToNode();

      // Enable the interrupt again now that we've finished sending output
      attachInterrupt(outFlowSensorInterrupt, pulseCounterOut, FALLING);
    }

    int bottomWaterLevel = 0;
    for (int i = 0; i < 5; i++) {
      int bottomWaterLevelValue = analogRead(bottomWaterLevelSensorPin);
      Serial.print(String(bottomWaterLevelValue) + "\t");
      bottomWaterLevel += bottomWaterLevelValue;
      delay(50);
    }
    bottomWaterLevel = bottomWaterLevel / 5;
    Serial.println("\nbottomWaterLevel: " + String(bottomWaterLevel));

    if (bottomWaterLevel >= 1020) {
      Serial.print ("Water at minimum level");
      keepPumpingOut = false;
      break;
    }
    else {
      Serial.println("- ");
    }
  }
  saveData();
  nodeSerial.flush();
  sendDataToNode();
  stopPumpOut();
  pumpIn();
}

void displayPumpingIn() {
  lcd.setCursor(0, 0);
  lcd.print(F("Volume In:"));
  lcd.setCursor(0, 1);
  lcd.print(String(totalMilliLitresIn) + "mL");

  lcd.setCursor(0, 2);
  lcd.print(F("In Flow Rate: "));
  lcd.setCursor(0, 3);
  lcd.print(String(flowRateIn, 2) + "mL/s");
}

void displayPumpingOut() {
  lcd.setCursor(0, 0);
  lcd.print(F("Volume Out:"));
  lcd.setCursor(0, 1);
  lcd.print(String(totalMilliLitresOut) + "mL");

  lcd.setCursor(0, 2);
  lcd.print(F("Out Flow Rate: "));
  lcd.setCursor(0, 3);
  lcd.print(String(flowRateOut, 2) + "mL/s");
}

void displayGeneralMonitor() {
  lcd.setCursor(0, 0);
  lcd.print(F("TDS(ppm): "));
  lcd.setCursor(10, 0);
  lcd.print(tdsValue);

  lcd.setCursor(0, 1);
  lcd.print(F("Temp(F): "));
  lcd.setCursor(9, 1);
  lcd.print(temperature, 1);

  lcd.setCursor(0, 2);
  lcd.print(F("Water PH: "));
  lcd.setCursor(10, 2);
  lcd.print(ph, 2);

  lcd.setCursor(0, 3);
  lcd.print(F("Water Rising: "));
  lcd.setCursor(14, 3);
  lcd.print(waterRising);
}

// Increment the pulse counter
void pulseCounterIn() {
  pulseInCount++;
}
void pulseCounterOut() {
  pulseOutCount++;
}
