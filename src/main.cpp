#include <Arduino.h>

#define fwVersion "SM_UART_04L_v1.1"
#define sampleRate 2000     //sample rate in ms
#define sensorBaud 9600

#include "SoftwareSerial.h"   //arduino software serial

#include <MQUnifiedsensor.h>

//Definitions
#define placa "Arduino UNO"
#define Voltage_Resolution 5
#define pin A10 //Analog input 10 of your arduino GPIO04
#define type "MQ-6" //MQ6
#define ADC_Bit_Resolution 10 // For arduino UNO/MEGA/NANO
#define RatioMQ6CleanAir 10   //RS / R0 = 10 ppm 
MQUnifiedsensor MQ6(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

#define sleepPin 4 //D4, Connector Pin 3, 04L Pin 10 Sleep
//#define Rx_Pin  6  //D5, Connector Pin 4, 04L Pin 9 Tx
//#define Tx_Pin  5  //D6, Connector Pin 5, 04L Pin 7 Rx
//SoftwareSerial Serial2(Rx_Pin, Tx_Pin); // RX, TX               // Declare serial

byte data[32] ;                   //create an array to store the response
unsigned long sampleTime;         // variable to monitor time of each sample start

void printSerialNumber();
bool GetSerialData();
unsigned int getCS(byte len);
//________________________________setup loop, runs once______________________________


// if using a CO2Meter CM-200 Sensor Development Board, set this to 1 if you want the LEDs to flash
// if you're wiring directly to the sensor, set this to 0
#define CM200 0

#if CM200
#define GREEN_LED 4
#define BLUE_LED 5
#define RED_LED 6

uint8_t ind = GREEN_LED;
#endif

// if you want the sensor to stream set this to 1
// if you want to use the sensor in polling mode, set this to 0 and set the delay_ms to desired delay time
#define STREAMING 1
const int delay_ms = 500;

//int co2 = 0;
double multiplier = 1; // 1 for 2% =20000 PPM, 10 for 20% = 200,000 PPM

const uint8_t buff_size = 64;
//uint8_t buff[buff_size];
uint8_t indexCoz = 0;

int fill_buffer();  // function prototypes here
int format_output();
double get_co2(char * str);

void setup()
{
  digitalWrite(sleepPin, HIGH); //puts unit active (HIGH), sleep (LOW)
  pinMode(sleepPin, OUTPUT);

  Serial.begin(9600); //Opens the main serial port over USB
  Serial2.begin(sensorBaud);  //baud rate, configuration
  //opens sensor serial with baud rate, configuration, pins are preset

  Serial.println(F("Amphenol Advanced Sensors"));
  Serial.println(F(fwVersion));
  Serial.print(F("Serial Number: "));
  printSerialNumber();
  Serial.println(fwVersion);
  Serial.println(F("Advanced Sensors"));
  Serial.println(F("Amphenol"));
  Serial.println(F("PM1 Standard Smoke (µg/m³), PM2.5 Standard Smoke (µg/m³), PM10 Standard Smoke (µg/m³)"));
  
  //Set math model to calculate the PPM concentration and the value of constants
  MQ6.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ6.setA(2127.2); MQ6.setB(-2.526); // Configure the equation to to calculate CH4 concentration
  MQ6.init();
  
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ6.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ6.calibrate(RatioMQ6CleanAir);
    Serial.print(".");
  }
  MQ6.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
  /*****************************  MQ CAlibration ********************************************/ 
  MQ6.serialDebug(true);
  sampleTime = millis();

  
  Serial.print("\n\n");
  Serial.println("             AN128 Ardunio to Cozir CO2 Sensor - Demonstration code 11/29/2017\n\n");

#if CM200
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(RED_LED, LOW);
#endif
  
  Serial2.begin(9600); // Start serial communications with sensor

  Serial2.println("M 6"); // send Mode for Z and z outputs
  // "Z xxxxx z xxxxx" (CO2 filtered and unfiltered)

#if STREAMING
  Serial2.println("K 1");  // set streaming mode
#else
  Serial2.println("K 2");  // set streaming mode
#endif
}


//_________________________main loop runs continuously__________________________

void loop()
{
  if (millis() >= (sampleRate + sampleTime))
  {
    sampleTime = millis();  //resets timer before printing output
    
    MQ6.update(); // Update data, the arduino will read the voltage from the analog pin
    MQ6.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
    MQ6.serialDebug(); // Will print the table on the serial port
  double co2_flt = 0.0;
  double co2_raw = 0.0;
  char buff[buff_size];

#if CM200
  // if using a CM-200 we can cycle through the LEDs to high and low so we can see that we're in the loop
  if (digitalRead(ind) == HIGH)
    digitalWrite(ind, LOW);
  else
    digitalWrite(ind, HIGH);

  ind++;
  if (ind > RED_LED)
    ind = GREEN_LED;
#endif

  Serial2.flush();
#if STREAMING == 0
  Serial2.println("Z");  // set streaming mode
#endif

  int timeout = 5000;
  while ( Serial2.available() <= 0 && timeout-- > 0) {
    Serial.print(".");
    delay(10);
  }
  Serial.println("");

  delay(10); //delay 10ms to give the Serial buffer some time to fill
  Serial.print("Sensor Response: ");
  indexCoz = 0;
  while ( Serial2.available() > 0 ) {
    char tmp = Serial2.read();
    buff[indexCoz] = tmp;
    Serial.print((char)tmp);

    if (tmp == '\r')
      break;

    indexCoz++;
    if (indexCoz >= buff_size) //let's not overflow the buffer
      break;
  }

  Serial.println("");

  char *p = buff;
  char *str;
  
  while ((str = strtok_r(p, " ", &p)) != NULL) {
    if (str[0] == 'Z') {
      str = strtok_r(p, " ", &p);
      co2_flt = get_co2(str);
    }
    if (str[0] == 'z') {
      str = strtok_r(p, " ", &p);
      co2_raw = get_co2(str);
    }
  }

  Serial.println("");
  Serial.print("***** Filtered CO2 PPM: ");
  Serial.print(co2_flt);
  Serial.print("\r\n");
#if STREAMING   // we don't need to print this out if we're not streaming
  Serial.print("***** Raw CO2 PPM: ");
  Serial.print(co2_raw);
  Serial.print("\r\n");
#endif

  delay(delay_ms);
  }
}


double get_co2(char * str) {
  for (int i = 0; i < strlen(str); i++) { //cycle through char array making sure we're all digits
    if (!isDigit(str[i])) {
     str[i] = '\0';
    }
  }

  return atof(str) * multiplier; // this is not great we could crash here if the first character is not a number
}
//___________________________sub routine to read value from module_________________


bool GetSerialData() {  //get lots - post sort method
  byte message[64];
  int CRC = 0;
  Serial2.readBytes(message, 64);   //read 64 streamed bytes

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
  Serial2.write((byte)0x20);
  Serial2.write((byte)0x0F);
  Serial2.write((byte)0x00);
  Serial2.write((byte)0x00);
  Serial2.write((byte)0x00);
  Serial2.write((byte)0x00);
  Serial2.write((byte)0x53);
  Serial2.write((byte)0x7A);
  Serial2.readBytes(data, 20);

Serial.print(F("Serial Number: "));
  for (int i = 0; i < 20; i++) {
    if (data[i] > 0x2E) Serial.write(data[i]);  //print serial number ASCII
    data[i] = 0x00;                             //delete data
  }
  
}
