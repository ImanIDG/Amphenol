#include <Arduino.h>

#define fwVersion "SM_UART_04L_v1.1"
#define sampleRate 2000     //sample rate in ms
#define sensorBaud 9600

#include "SoftwareSerial.h"   //arduino software serial
#define sleepPin 4 //D4, Connector Pin 3, 04L Pin 10 Sleep
#define Rx_Pin  6  //D5, Connector Pin 4, 04L Pin 9 Tx
#define Tx_Pin  5  //D6, Connector Pin 5, 04L Pin 7 Rx
SoftwareSerial mySerial(Rx_Pin, Tx_Pin); // RX, TX               // Declare serial

byte data[32] ;                   //create an array to store the response
unsigned long sampleTime;         // variable to monitor time of each sample start

void printSerialNumber();
bool GetSerialData();
unsigned int getCS(byte len);
//________________________________setup loop, runs once______________________________
void setup()
{
  digitalWrite(sleepPin, HIGH); //puts unit active (HIGH), sleep (LOW)
  pinMode(sleepPin, OUTPUT);

  Serial.begin(9600); //Opens the main serial port over USB
  mySerial.begin(sensorBaud);  //baud rate, configuration
  //opens sensor serial with baud rate, configuration, pins are preset

  Serial.println(F("Amphenol Advanced Sensors"));
  Serial.println(F(fwVersion));
  Serial.print(F("Serial Number: "));
  printSerialNumber();
  Serial.println(fwVersion);
  Serial.println(F("Advanced Sensors"));
  Serial.println(F("Amphenol"));
  Serial.println(F("PM1 Standard Smoke (µg/m³), PM2.5 Standard Smoke (µg/m³), PM10 Standard Smoke (µg/m³)"));

  sampleTime = millis();
}


//_________________________main loop runs continuously__________________________

void loop()
{
  if (millis() >= (sampleRate + sampleTime))
  {
    sampleTime = millis();  //resets timer before printing output
    //sendRequest(readPM, 5, 32); //comms request to sensor, command string, string length ex CRC, response length in bytes inc CRC
    if (GetSerialData()) {  //request to get data, if CRC OK, then print and display
      Serial.print(( data[4] & 0x3F ) << 8 | data[5]); Serial.print(", "); //PM1 Standard Smoke
      Serial.print(( data[6] & 0x3F ) << 8 | data[7]); Serial.print(", "); //PM2.5 Standard Smoke
      Serial.print(( data[8] & 0x3F ) << 8 | data[9]); Serial.print(", "); //PM10 Standard Smoke
    } else {
      Serial.print(F("No data"));
    }
    Serial.println();

  }
}


//___________________________sub routine to read value from module_________________


bool GetSerialData() {  //get lots - post sort method
  byte message[64];
  int CRC = 0;
  mySerial.readBytes(message, 64);   //read 64 streamed bytes

  for ( byte i = 0 ; i < 32 ; i++ ) {  //look for ox42, 0x4D sequence and load data from stream
    if ( message[i] == 0x42 && message[i + 1] == 0x4D ) {
      for ( byte j = 0 ; j < 32 ; j++ ) {
        data[j] = message[i];
        i++;
      }
      break;
    }
  }

  if ((data[30] * 256 + data[31]) == getCS(30)) {  //Cyclical Redundancy Check
    return true;
  }
  else {
    return false;
  }
}

unsigned int getCS(byte len)     // Compute the checksum
{
  unsigned int var = 0;
  for (int i = 0; i < len; i++) {
    var += data[i];
  }
  return var;
}

void printSerialNumber() {
  //{0x20, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x53, 0x7A};  message to call serial number
  mySerial.write((byte)0x20);
  mySerial.write((byte)0x0F);
  mySerial.write((byte)0x00);
  mySerial.write((byte)0x00);
  mySerial.write((byte)0x00);
  mySerial.write((byte)0x00);
  mySerial.write((byte)0x53);
  mySerial.write((byte)0x7A);
  mySerial.readBytes(data, 20);

Serial.print(F("Serial Number: "));
  for (int i = 0; i < 20; i++) {
    if (data[i] > 0x2E) Serial.write(data[i]);  //print serial number ASCII
    data[i] = 0x00;                             //delete data
  }
  
}
