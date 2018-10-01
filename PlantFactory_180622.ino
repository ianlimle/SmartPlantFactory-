#include <OneWire.h>                                        // initial Electrical conductivity sensor library
#include <Wire.h>                                           // initial setup to use RTC. library for the rtc and instructions can be found on elecrow.com
#include "RTClib.h"                                        // this bunch loads up the library you can obtain from the manufacturer  

RTC_DS1307 rtc;                                             // you can change to a different module or use a different library if you require
#include <DHT.h>                                            // loads library for DHT sensor
//#include <AFMotor.h>                                        //
#define DHTPIN 2                                            // digital pin for temp/humidity sensor
#define DHTTYPE DHT22                                       // lets library know what sensor we are using

//Consolidation of pin information*****************************************************************
/*
  A0 - CO2 SENSOR
  A1 - GAS O2 SENSOR
  A2 - PH SENSOR
  A3 - EC SENSOR
  A4 -
  A5 -
  -
  -

  D0 RX0
  D1 TX0
  D2       - TEMPERATURE/HUMIDITY SENSOR
  D3 PWM   - AF_DCMotor Fan
  D4       - EC DS18B20 TEMP SENSOR
  D5 PWM   -
  D6 PWM   -
  D7       - Pump Relay
  D8       - LED Responding Signal RespLED
  D9 PWM   - FLOAT SWITCH LED
  D10 PWM  - 
  D11 PWM  - 
  D12      - FLOAT SWITCH SENSOR
  D13      -
  .
  .
  D14 TX3 -
  D15 RX3 -
  D16 TX2 - 
  D17 RX2 -
  D18 TX1 -
  D19 RX1 - 
  D20 SDA - SDA RTC DS1307
  D21 SCL - SCL RTC DS1307
  .
  .
  D22      - B
  D23      - R
  D24      - G
  D25      - W
  D26      - B1
  D27      - R1
  D28      - G1
  D29      - W1
  D30      - B2
  D31      - R2
  D32      - G2
  D33      - W2
*/

//define for Co2
#define         MG_PIN                       (0)     //define which analog input channel you are going to use(Analog pin 0 selected)
#define         DC_GAIN                      (8.5)   //define the DC gain of amplifier
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interval(in milisecond) between each samples in 
//normal operation
#define         ZERO_POINT_VOLTAGE           (0.085) //define the output of the sensor in volts when the concentration of CO2 is 400PPM
#define         REACTION_VOLTGAE             (0.048) //define the voltage drop of the sensor when move the sensor from air into 1000ppm CO2

//define for pH Sensor
#define SensorPin A2            //pH meter Analog output to Arduino Analog Input A2
#define Offset 0.00            //deviation compensate
#define samplingInterval 20   //Check sampling when samplingTime is more than samplingInterval
//#define printInterval 800     //Shows pH value every 800ms
#define ArrayLenth  40    //times of collection

//define for Floating Switch water level sensor
#define FloatLED  9          //define digital pin 9 to be use for Float Switch LED alert
#define FloatSW  12          //define digital pin 12 to be use for Float Switch

//define EC and EC temp sensor
#include <EEPROM.h>
#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}

#define ReceivedBufferLength 20
char receivedBuffer[ReceivedBufferLength + 1]; // store the serial command
byte receivedBufferIndex = 0;

#define ecSensorPin  A3  //EC Meter analog output,pin on analog 1
#define ds18b20Pin  4    //DS18B20 signal, pin on digital 2

#define SCOUNT  100           // sum of sample point
int analogBuffer[SCOUNT];    //store the analog value read from ADC
int analogBufferIndex = 0;

#define compensationFactorAddress 8    //the address of the factor stored in the EEPROM
float compensationFactor;

#define VREF 5000  //for arduino uno, the ADC reference is the power(AVCC), that is 5000mV

boolean enterCalibrationFlag = 0;
float temperature, ECvalue, ECvalueRaw;

OneWire ds(ds18b20Pin);

// *****************************Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor******************

DHT dht(DHTPIN, DHTTYPE);                                   // initialises the sensor object in the code
/*AF_DCMotor fan(3);
unsigned long currMillis;
unsigned long prevMillis = 0;
double kp = 410;
double ki = 2*kp/20; 
double kd = kp*20/8;
double dt = 0.01;          // Specify sample time in seconds (fixed)
double dtmillis = dt*1000;  // Sample time in milliseconds
int setpoint;
float input;
float Output;
float err;
float pTerm;
int IntThresh;
float integrated_err;
float dTerm;
float iTerm;
float last_err;*/

//DECLARE CONSTANTS                                         // these constants are numbers or different data types that will be stored in the RAM and cannot be altered then the loop runs
// values here can be changed before loading onto controller if user decides to change some parameters
//Declaration for RGB Light
//**LEVEL 1**
int Blue = 22;                                                // Declare RGB LED STRIP Blue pin digital 22 relay
int Red = 23;                                              // Declare RGB LED STRIP Red pin digital 23 relay
int Green = 24;                                               // Declare RGB LED STRIP Green pin digital 24 relay
const int White = 25;                                     // digital pin 25 for sending signal to turn on white lights  ('L' Relay)
//**LEVEL 2**
int Blue1 = 26;                                                // Declare RGB LED STRIP Blue pin digital 26 relay
int Red1 = 27;                                              // Declare RGB LED STRIP Red pin digital 27 relay
int Green1 = 28;                                               // Declare RGB LED STRIP Green pin digital 28 relay
const int White1 = 29;                                     // digital pin 29 for sending signal to turn on lights  ('L' Relay)
//**LEVEL 3**
int Blue2 = 30;                                                // Declare RGB LED STRIP Blue pin digital 30 relay
int Red2 = 31;                                              // Declare RGB LED STRIP Red pin digital 31 relay
int Green2 = 32;                                               // Declare RGB LED STRIP Green pin digital 32 relay
const int White2 = 33;                                     // digital pin 33 for sending signal to turn on lights  ('L' Relay)

//const int RGB12v = *havent set pin yet*;                                      // RGB LED STRIP 12v, not required now.
boolean LedOn = false;                                      // Setting LedOn = false = LOW for user easy identify
boolean LedOff = true;                                      // Setting LedOff = true = HIGH for user easy identify
const int pumpSig = 7;                                      // digital pin for sending signal to turn on pump    ('P' Relay)
//Declaration of Time turn on light, between 0600hrs-2200hrs
const int lightStart = 6;                                   // hour for lights to turn on,   assume starting hour less than ending hour 0600hrs
const int lightStop = 22;                                   // hour for lights to turn off,  must be greater than lightStart 2200hrs

const int pumpDur = 4;                                      // duration for pump to run in minutes
const int waitDur = 6;                                      // duration for pump to rest in minutes

//DECLARE VARIABLES_________________________________________// values here are stored in the RAM and are dynamic. throughout the loop these values will change to indicate different states that the system is in

boolean pumpRun = true;                                     // this boolean (true/false) value indicates whether the pump is running to the system. see line 26
int minsCount = 0;                                          // this is a counter for the pump to check if it has reached the designated waiting/running duration before changing states. should not exceed pumpDur or waitDur (whichever is smaller) on startup
float tempAct = 24.0;                                      // Variable for actual temperature data
float temp;                                                // temporary temperature value
float humidAct = 50.0;                                      // Variable for actual humidity data
float humid;                                                // temporary humidity value
boolean dhtsample = false;                                  // flag for checking if data being sampled
int minOld = 59;                                            // 
int minNow;                                                 //
float tempCeiling = 99;                                     //
//
//
//
// Declaration for touch sensor
//int ledPin; //= LED_BUILTIN;                // Connect LED on pin 13, or use the onboard one **Not used**
//int KEY = 4;                                // Touch sensor is not used **Pin 4 has been used by LED STRIP**
//
//
//Gas Sensor declarationV2
const float VRefer = 3.3;       // voltage of adc reference
const int pinAdc   = A1;        //Select Analog pin A1 to get input
float Vout = 0;                 //set Voltage output to 0

/*//Gas Sensor declaration
  float WarningValue=19.5; //set your warning value for user
  float sensorValue, sensorVoltage, Value_o2; //declaration of names for background calculation*/

//Co2 Sensor declaration
float CO2Curve[3]  =  {2.602, ZERO_POINT_VOLTAGE, (REACTION_VOLTGAE / (2.602 - 3))};
//two points are taken from the curve.
//with these two points, a line is formed which is
//"approximately equivalent" to the original curve.
//data format:{ x, y, slope}; point1: (lg400, 0.324), point2: (lg4000, 0.280)
//slope = ( reaction voltage ) / (log400 –log1000)

//pH Sensor declaration
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex = 0;
static unsigned long samplingTime = millis(); //set samplingTime to millis
//static unsigned long printTime = millis();    //set printTime to millis
static float pHValue, voltage;                //declare usage of name
//int PhLED = 52;

//Floating Switch water level sensor declaration
boolean floatSW;          // declare floatSW to be true/false value

//Responding light signal
int RespLED = 8;
//int CounterLED = 53;

String sensor_data;                   //Initialize sensor_data variable for serial communication between Mega and RPi 

void setup() {                          //__________________// the setup function runs once when you press reset or power the board
  pinMode(Blue, OUTPUT);                                      // declare Blue pin digital 22 to be output
  pinMode(Red, OUTPUT);                                       // declare Red pin digital 23 to be output
  pinMode(Green, OUTPUT);                                     // declare Green pin digital 24 to be output
  pinMode(White, OUTPUT);                                     // declares the pin 25 for White light signal to be an output
  pinMode(Blue1, OUTPUT);                                     // declare Blue pin digital 26 to be output
  pinMode(Red1, OUTPUT);                                      // declare Red pin digital 27 to be output
  pinMode(Green1, OUTPUT);                                    // declare Green pin digital 28 to be output
  pinMode(White1, OUTPUT);                                    // declares the pin 29 for White light signal to be an output
  pinMode(Blue2, OUTPUT);                                     // declare Blue pin digital 30 to be output
  pinMode(Red2, OUTPUT);                                      // declare Red pin digital 31 to be output
  pinMode(Green2, OUTPUT);                                    // declare Green pin digital 32 to be output
  pinMode(White2, OUTPUT);                                    // declares the pin 33 for White light signal to be an output
  pinMode(pumpSig, OUTPUT);                                   // declares the pin for light signal to be an output
  pinMode(FloatSW, INPUT_PULLUP);                             // declares the pin for float switch signal to be an INPUT_PULLUP **Pull up resistor/physical resistor 10kohm is required otherwise the value will fluctuate**
  pinMode(FloatLED, OUTPUT);                                  // declares the pin for floating switch to be an input
  digitalWrite(pumpSig, LOW);                                 // sets initial state of the pump upon startup. has affinity with line 18. must be (true,HIGH) or (false,LOW)
  //pinMode(20, INPUT_PULLUP);
  //pinMode(21, INPUT_PULLUP);
  pinMode(RespLED, OUTPUT);
  //pinMode(CounterLED, OUTPUT);
  //pinMode(PhLED, OUTPUT);
  
  Serial.begin(115200);                                       //declares the Serial baud rate
  
  //Serial.print("Version:");
  //Serial.println("180530");
  // initialize all the EC readings to 0:
  readCharacteristicValues(); //read the compensationFactor

  Wire.begin();                                           // enables I2C communication via TX/RX
  rtc.begin();                                            // starts communication between RTC module and Arduino
  //rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));          // Once date/time is set, comment it out
  dht.begin();                                            // starts communication with DHT sensor
  //fan.run(RELEASE);                                       //initialize fan 
  //fan.setSpeed(0);
  
}

//LOOP START__________________________________________________// the loop function runs over and over again forever
void loop() {
  DateTime now = rtc.now();                                   // creates the object 'now' to be a DateTime class based off the current instance/moment in time with with attributes specific to that class such as:
  //Serial.print(now.hour(), DEC);                              // the recorded hour
  //Serial.print(F(":"));
  //Serial.print(now.minute(), DEC);                            // the recorded minute
  //Serial.print(F(":"));
  //Serial.println(now.second(), DEC);                          // the recorded seconds
  //Serial.print(F("Minutes Elapsed: "));                          // prints the text 'Minutes Elapsed: '
  //Serial.println(minsCount);                                  // prints how many minutes has passed, based off the counter that was set up for the pump to check

  // CODING FOR TEMPERATURE AND HUMIDTY READING
  //Serial.print(F("Temperature: "));                              // prints out the temperature and humidity reading for user
  //Serial.print(tempAct);
  //Serial.println(F(" *C "));
  //Serial.print(F("Humidity: "));
  //Serial.print(humidAct);
  //Serial.println(F(" %\t"));

  // GAS SENSOR V2
  //Serial.print("Vout =");                                 //removed, not required

  Vout = readO2Vout();                                      //set Vout to get value from readO2Vout
  //Serial.print(Vout);                                     //removed, not required
  //Serial.print(" V, Concentration of O2 is ");            //removed, not required

    //Serial.print(F("Concentration of O2: "));
    //Serial.print(readConcentration());                        //display O2 value
    //Serial.print(F("%"));                                      //in percentage
    //Serial.print(F("\n"));
  /*// GAS SENSOR OPERATION READING
    sensorValue = analogRead(A1); // read value from A1 port
    sensorVoltage = (sensorValue/1024)*5.0;
    sensorVoltage = sensorVoltage/201*10000;
    Value_o2 = (sensorVoltage/7.43);
    Serial.print("Concentration of O2 is "); //display on serial monitor
    Serial.print(Value_o2,1); //display O2 value on serial monitor
    Serial.println("%"); //display on serial monitor*/

  //Co2 SENSOR
  int ppm;
  float volts;

  volts = MGRead(MG_PIN);                               //get value from MGRead, refer to define MG_PIN
  //Serial.print( "SEN0159:" );                         //removed, not required
  //Serial.print(volts);                                //removed, not required
  //Serial.print( "V" );                                //removed, not required

  ppm = MGGetPercentage(volts, CO2Curve);        //get value from volts and Co2Curve from calculations shown at declaration
  //Serial.print(F("CO2:"));
  if (ppm == -1) {
    //Serial.print(F("<400"));
  } else {
    //Serial.print(ppm);                         //display calculation of percentage
  }
  //Serial.print(F("ppm"));                                //units
  //Serial.print( "       Time point:" );               //removed, not required
  //Serial.print(millis());                             //removed, not required
  //Serial.print(F("\n"));                                   //next line

  //pH SENSOR
  float PHValue;
  float voltage;
  if (millis() - samplingTime > samplingInterval)       //condition to begin calculations
  {
    pHArray[pHArrayIndex++] = analogRead(SensorPin);    //get analog input from SensorPin
    if (pHArrayIndex == ArrayLenth)pHArrayIndex = 0;
    voltage = avergearray(pHArray, ArrayLenth) * 5.0 / 1024; //Manufacturer calibration
    pHValue = 3.5 * voltage + Offset;                    //Manufacturer calibration
    samplingTime = millis();
  }

  if (pHValue >= 5.5 && pHValue <= 6.5) {
    //Serial.print(F("pH value: "));
    //Serial.print(pHValue, 2);                          //display pH value in 2 decimal places
    //digitalWrite(PhLED, HIGH);                           //Turn on LED indicator
    //delay(50);
    //digitalWrite(PhLED, LOW);                           //Turn off LED indicator
    //Serial.println(F(" is Neutral"));
  } else if (pHValue < 5.5) {
    //Serial.print(F("pH value: "));
    //Serial.print(pHValue, 2);                          //display pH value in 2 decimal places
    //Serial.println(F(" too much acid"));
  } else if (pHValue > 6.5) {
    //Serial.print(F("pH value: "));
    //Serial.print(pHValue, 2);                          //display pH value in 2 decimal places
    //Serial.println(F(" too much alkaline"));
  }
  /*if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
    {
     Serial.print("Voltage:");
     Serial.print(voltage,2); //2 represent how many decimal places
     Serial.print("    pH value: ");
     Serial.println(pHValue, 2);
     digitalWrite(LED,digitalRead(LED)^1);
     printTime=millis();
    }*/

  // FLOAT SWITCH WATER LEVEL SENSOR
  floatSW = digitalRead(FloatSW);                                         // declare floatSW names to take value from FloatSW "alphabetic caps matters in coding"
  if (floatSW == true) {                                                  //floatSW == true == HIGH == 1 > floatSWitch is non-straight
    //Serial.println(F("Water level low!!!"));
    digitalWrite(FloatLED, HIGH);                                         //FloatLED == HIGH == true == 1 > FloatLED light is on to alert user
  } else {
    //Serial.println(F("Water level good!"));
    digitalWrite(FloatLED, LOW);                                          //FloatLED == LOW == false = 0 > FloatLED light is off
  }

  // EC and EC temp sensor with calibration command: CALIBRATION, CONFIRM, EXIT. Change "No Line Ending" to "Both NL & CL"
  if (serialDataAvailable() > 0)
  {
    byte modeIndex = uartParse();
    ecCalibration(modeIndex);    // If the correct calibration command is received, the calibration function should be called.
  }

  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 30U) //every 30ms,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(ecSensorPin);    //read the analog value and store into the buffer,every 40ms
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }

  static unsigned long tempSampleTimepoint = millis();
  if (millis() - tempSampleTimepoint > 850U) // every 1.7s, read the temperature from DS18B20
  {
    tempSampleTimepoint = millis();
    temperature = readTemperature();  // read the current temperature from the  DS18B20
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 1000U)
  {
    printTimepoint = millis();
    float AnalogAverage = getMedianNum(analogBuffer, SCOUNT);  // read the stable value by the median filtering algorithm
    float averageVoltage = AnalogAverage * (float)VREF / 1024.0;
    if (temperature == -1000)
    {
      temperature = 25.0;      //when no temperature sensor ,temperature should be 25^C default
      //Serial.print("EC Temp: ");
      //Serial.print(temperature, 1);
      //Serial.print(F("^C(default) EC:"));
    } else {
      //Serial.print("EC Temp: ");
      //Serial.print(temperature, 1);   //current temperature
      //Serial.print(F("^C             EC:"));
    }
    float TempCoefficient = 1.0 + 0.0185 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.0185*(fTP-25.0));
    float CoefficientVolatge = (float)averageVoltage / TempCoefficient;
    /*if (CoefficientVolatge < 150)Serial.println(F("No solution!")); //25^C 1413us/cm<-->about 216mv  if the voltage(compensate)<150,that is <1ms/cm,out of the range
    else if (CoefficientVolatge > 3300)Serial.println(F("Out of the range!")); //>20ms/cm,out of the range
    else {
      if (CoefficientVolatge <= 448)ECvalue = 6.84 * CoefficientVolatge - 64.32; //1ms/cm<EC<=3ms/cm
      else if (CoefficientVolatge <= 1457)ECvalue = 6.98 * CoefficientVolatge - 127; //3ms/cm<EC<=10ms/cm
      else ECvalue = 5.3 * CoefficientVolatge + 2278;                     //10ms/cm<EC<20ms/cm
      ECvalueRaw = ECvalue / 1000.0;
      ECvalue = ECvalue / compensationFactor / 1000.0; //after compensation,convert us/cm to ms/cm
      //Serial.print(ECvalue, 2);    //two decimal
      //Serial.print(F("ms/cm"));
      if (enterCalibrationFlag)            // in calibration mode, print the voltage to user, to watch the stability of voltage
      {
        //Serial.print(F("Factor:"));
        //Serial.print(compensationFactor);
      }
      //Serial.println();
    }*/
  }


  // CONDITIONAL OPERATION OF DHT22 SENSOR. BTW DHT22 RUNS BETTER ON 3.3V, MORE ACCURATE NUMBERS
  if (now.second() % 3 == 0 && dhtsample == false) {          // the sampling runs on a 3 second cycle, on the 0th second, it calls for
    dhtsample = true;                                         // the temperature from the sensor and stores it in a temporary variable 'temp'
    temp = dht.readTemperature();                             // it changes a boolean value 'dhtsample' to flag to the arduino that it has
    humid = dht.readHumidity();                               // called the sensor so that it doesnt keep calling for the entire 0th second
    //Serial.println(F("sampling"));                               // The sensor takes a minimum 250ms to complete sampling
  }

  if (now.second() % 3 == 2 && dhtsample == true) {           // by the 2nd second of the cycle it would have recieved an accurate value
    dhtsample = false;                                        // of temperature. the code then updates the readable value 'tempAct' to be
    tempAct = temp;                                           // displayed to the user. this ensures there will always be a valid reading,
    humidAct = humid;                                         // although the readings would be 2-3 seconds old.
    //Serial.println(F("updated"));                                //
  }

  /*if (tempAct > tempCeiling)
    {                                                               // conditionals for alarm, triggers when tempAct exceeds upper defined limit 
       float input = (tempAct);                                          // Read by your sensor, y(t) ... analogRead(...)
       float err = (input - tempCeiling);                                // Compute Error, e(t) = r(t) - y(t)
       delay(200);

       // ====================== Compute Proportional term: ========================
       pTerm = kp * (err);          
       
       // ====================== Compute Integral term: ===========================
       IntThresh = .3;                      // Set value for max intergral term
       if (abs(err) > IntThresh)            // Crude way to prevent integrator 'windup'
       {                                    
           integrated_err += err;           // Accumulate the error integral
       }
       else 
       {
           integrated_err = 0;              // Zero it if out of bounds 
       }
       iTerm = ki * integrated_err;         // Compute actual Integral term

       // ====================== Compute Derivative term: ===========================
       dTerm = kd * (err - last_err)*dt;  

       // ====================== Compute PID control command, u(t): =================
       Output = constrain(pTerm + iTerm + dTerm, 0, 255);       
       fan.run(FORWARD);
       fan.setSpeed(Output);    // Use u(t) to drive the plant, i.e. analogWrite(Output), motor.run(Output), etc...

       last_err = err;      // Set last_err to err for next time through loop
    }*/


  // CODING FOR LIGHT CONTROL___________________________________// Light timer checks for hour
  //Serial.print("Current Hour: ");                             // prints the text 'Current Hour: '
  //Serial.println(now.hour(), DEC);                            // prints the numerical value of the current hour

  if (now.hour() >= lightStart && now.hour() < lightStop)     // if the current hour is within the range of hours that the lights should be on
  {
    digitalWrite(Blue, LedOn);                                // Turn on Blue pin digital 22
    digitalWrite(Red, LedOn);                                 // Turn on Red pin digital 23
    digitalWrite(Green, LedOn);                               // Turn on Green pin digital 24
    digitalWrite(White, LedOn);                               // turns on the lights by sending a LOW signal to relay pin 25
    digitalWrite(Blue1, LedOn);                               // Turn on Blue1 pin digital 26
    digitalWrite(Red1, LedOn);                                // Turn on Red1 pin digital 27
    digitalWrite(Green1, LedOn);                              // Turn on Green1 pin digital 28
    digitalWrite(White1, LedOn);                              // turns on the lights by sending a LOW signal to relay pin 29
    digitalWrite(Blue2, LedOn);                               // Turn on Blue2 pin digital 30
    digitalWrite(Red2, LedOn);                                // Turn on Red2 pin digital 31
    digitalWrite(Green2, LedOn);                              // Turn on Green2 pin digital 32
    digitalWrite(White2, LedOn);                              // turns on the lights by sending a LOW signal to relay pin 33
    //Serial.println(F("Lights ON"));                              // feedback to monitor that lights are on
  }

  else if (now.hour() >= lightStop || now.hour() < lightStart) // if the lights are beyond the scope of operating hours
  {
    if (now.hour() < 25) {
      digitalWrite(Blue, LedOff);                                // Turn off Blue pin digital 22
      digitalWrite(Red, LedOff);                                 // Turn off Red pin digital 23
      digitalWrite(Green, LedOff);                               // Turn off Green pin digital 24
      digitalWrite(White, LedOff);                               // turns off the lights by sending a LOW signal to relay pin 25
      digitalWrite(Blue1, LedOff);                               // Turn off Blue1 pin digital 26
      digitalWrite(Red1, LedOff);                                // Turn off Red1 pin digital 27
      digitalWrite(Green1, LedOff);                              // Turn off Green1 pin digital 28
      digitalWrite(White1, LedOff);                              // turns off the lights by sending a LOW signal to relay pin 29
      digitalWrite(Blue2, LedOff);                               // Turn off Blue2 pin digital 30
      digitalWrite(Red2, LedOff);                                // Turn off Red2 pin digital 31
      digitalWrite(Green2, LedOff);                              // Turn off Green2 pin digital 32
      digitalWrite(White2, LedOff);                              // turns off the lights by sending a LOW signal to relay pin 33
      //Serial.println(F("Lights OFF"));                             // feedback to monitor that lights are off
    }
  }
  // MINUTE COUNTER CODING                                      //
  minNow = now.minute();                                      // stores current minute as minNow
  if (minNow > minOld) {                                      // if the current minute is larger than previously recorded
    if (minNow < 61 && now.hour() < 25) {
      minOld = minNow;                                          // update the 'previous' value to the current value
      minsCount += 1;                                           // increment minute counter by 1 minute
    }
  }
  else if (minOld > minNow && minOld == 59) {                 // if minute from 59 rolls over to 60/00
    if (minOld < 61 && now.hour() < 25)
    {
      minOld = minNow;                                          // update 'previous' value
      minsCount += 1;                                           // increment counter by 1
    }
  }
  else if (minOld == 59) {                                    // exception case for initial conditions to prevent premature minute
    minOld = now.minute();                                    //
  }
  //Serial.println(minNow);                                     // prints current minute
  //Serial.println(minOld);
  //Serial.println(minsCount);                                    // prints the previous minute
  //Serial.print("Now.minute ");                                // prints RTC value of minute
  //Serial.println(now.minute());                               //

  // CONDITIONALS FOR PUMP TO RUN, MINUTE INTERVALS
  if (pumpRun == false && minsCount == waitDur)             //OFF to ON condition, if the pump is not running and the wait duration is reached:
  {
    digitalWrite(pumpSig, LOW);                            // turn on pump by sending HIGH to relay
    pumpRun = true;                                         // change Running state of pump to 'true'
    minsCount = 0;                                          // resets minute counter to zero
    //Serial.println(F("Pump Running"));                         // feedback to monitor that pump is running
  }

  else if ( pumpRun == true && minsCount == pumpDur)        // if pump is running and running duration is reached:
  {
    digitalWrite(pumpSig, HIGH);                             // turn off pump by sending LOW signal to relay
    pumpRun = false;                                        // change running state of pump to 'false'
    minsCount = 0;                                          // resets minute counter to zero
    //Serial.println(F("Pump Stopped"));                         // feedback to monitor that pump has stopped
  }

  // INDICATOR FOR PUMP OPERATION
  if (pumpRun == true)
  {
    //Serial.println(F("pumping"));
  }

  if (pumpRun == false)
  {
    //Serial.println(F("not pumping"));
  }
  //
  //Serial.println();                                         // blank;
  /*
    if(digitalRead(KEY)==HIGH) {      //Read Touch sensor signal
        digitalWrite(ledPin, HIGH);   // if Touch sensor is HIGH, then turn on
     }
    else{
        digitalWrite(ledPin, LOW);    // if Touch sensor is LOW, then turn off the led
     }
  */
  /*if (minsCount == 1 || minsCount == 3 || minsCount == 5) {
    digitalWrite(CounterLED, HIGH);
  } else {
    digitalWrite(CounterLED, LOW);
  }*/
  
  digitalWrite(RespLED, HIGH);        //Responding light signal
  delay(2000);
  digitalWrite(RespLED, LOW);
  delay(500);                        // waits to synchronise everything to 500ms intervals

  sensor_data = 
    String(tempAct) + "*C" + "," +
    String(humidAct) + "%" + "," +
    String(ppm) + "ppm" + "," +
    String(readConcentration()) + "%" + "," + 
    String(pHValue) + "," +
    String(ECvalue) +"ms/cm";
  
  Serial.println(sensor_data);
 }


/************************************ Write the codes for other functions below *******************/
//Gas Sensor CalibrationV2 o2
float readO2Vout()
{
  long sum = 0;
  for (int i = 0; i < 32; i++)
  {
    sum += analogRead(pinAdc);
  }

  sum >>= 5;

  float MeasuredVout = sum * (VRefer / 1023.0);
  return MeasuredVout;
}

float readConcentration()
{
  // Vout samples are with reference to 3.3V
  float MeasuredVout = readO2Vout();

  //float Concentration = FmultiMap(MeasuredVout, VoutArray,O2ConArray, 6);
  //when its output voltage is 2.0V,
  float Concentration = MeasuredVout * 0.21 / 2.0;
  float Concentration_Percentage = Concentration * 100;
  return Concentration_Percentage;
}

//Co2 Readings and calibration
float MGRead(int mg_pin)
{
  int i;
  float v = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    v += analogRead(mg_pin); //refer to define MG_PIN to select port to read
    delay(READ_SAMPLE_INTERVAL);
  }
  v = (v / READ_SAMPLE_TIMES) * 5 / 1024 ;
  return v;
}

int  MGGetPercentage(float volts, float *pcurve)
{
  if ((volts / DC_GAIN ) >= ZERO_POINT_VOLTAGE) {
    return -1;
  } else {
    return pow(10, ((volts / DC_GAIN) - pcurve[1]) / pcurve[2] + pcurve[0]);
  }
}

//pH Readings and calibration
double avergearray(int* arr, int number)
{
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0) {
    //Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if (number < 5) { //less than 5, calculated directly statistics
    for (i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  } else {
    if (arr[0] < arr[1]) {
      min = arr[0];
      max = arr[1];
    }
    else {
      min = arr[1];
      max = arr[0];
    }
    for (i = 2; i < number; i++) {
      if (arr[i] < min) {
        amount += min;      //arr<min
        min = arr[i];
      } else {
        if (arr[i] > max) {
          amount += max;  //arr>max
          max = arr[i];
        } else {
          amount += arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount / (number - 2);
  }//if
  return avg;
}

//EC and EC temp sensor
boolean serialDataAvailable(void)
{
  char receivedChar;
  static unsigned long receivedTimeOut = millis();
  while (Serial.available() > 0)
  {
    if (millis() - receivedTimeOut > 500U)
    {
      receivedBufferIndex = 0;
      memset(receivedBuffer, 0, (ReceivedBufferLength + 1));
    }
    receivedTimeOut = millis();
    receivedChar = Serial.read();
    if (receivedChar == '\n' || receivedBufferIndex == ReceivedBufferLength) {
      receivedBufferIndex = 0;
      strupr(receivedBuffer);
      return true;
    } else {
      receivedBuffer[receivedBufferIndex] = receivedChar;
      receivedBufferIndex++;
    }
  }
  return false;
}

byte uartParse()
{
  byte modeIndex = 0;
  if (strstr(receivedBuffer, "CALIBRATION") != NULL)
    modeIndex = 1;
  else if (strstr(receivedBuffer, "EXIT") != NULL)
    modeIndex = 3;
  else if (strstr(receivedBuffer, "CONFIRM") != NULL)
    modeIndex = 2;
  else if (strstr(receivedBuffer, "HELP") != NULL)
    modeIndex = 4;

  return modeIndex;
}

void ecCalibration(byte mode)
{
  char *receivedBufferPtr;
  static boolean ecCalibrationFinish = 0;
  float factorTemp;
  switch (mode)
  {
    case 0:
      if (enterCalibrationFlag)
        Serial.println(F("Command Error"));
      break;

    case 1:
      enterCalibrationFlag = 1;
      ecCalibrationFinish = 0;
      Serial.println();
      Serial.println(F(">>>Enter Calibration Mode<<<"));
      Serial.println(F(">>>Please put the probe into the 12.88ms/cm buffer solution<<<"));
      Serial.println();
      break;

    case 2:
      if (enterCalibrationFlag)
      {
        factorTemp = ECvalueRaw / 12.88;
        if ((factorTemp > 0.85) && (factorTemp < 1.15))
        {
          Serial.println();
          Serial.println(F(">>>Confirm Successful<<<"));
          Serial.println();
          compensationFactor =  factorTemp;
          ecCalibrationFinish = 1;
        }
        else {
          Serial.println();
          Serial.println(F(">>>Confirm Failed,Try Again<<<"));
          Serial.println();
          ecCalibrationFinish = 0;
        }
      }
      break;

    case 3:
      if (enterCalibrationFlag)
      {
        Serial.println();
        if (ecCalibrationFinish)
        {
          EEPROM_write(compensationFactorAddress, compensationFactor);
          Serial.print(F(">>>Calibration Successful"));
        }
        else Serial.print(F(">>>Calibration Failed"));
        Serial.println(F(",Exit Calibration Mode<<<"));
        Serial.println();
        ecCalibrationFinish = 0;
        enterCalibrationFlag = 0;
      }
      break;

    case 4:
      Serial.println();
      Serial.println();
      Serial.println(F("***ELECTRICAL CONDUCTIVITY HELP COMMAND LIST***"));
      Serial.println(F("1. CALIBRATION"));
      Serial.println(F("2. CONFIRM"));
      Serial.println(F("3. EXIT"));
      Serial.println(F("4. HELP"));
      Serial.println();
      Serial.println();
      break;
  }
}

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
  {
    bTab[i] = bArray[i];
  }
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

void readCharacteristicValues()
{
  EEPROM_read(compensationFactorAddress, compensationFactor);
  if (EEPROM.read(compensationFactorAddress) == 0xFF && EEPROM.read(compensationFactorAddress + 1) == 0xFF && EEPROM.read(compensationFactorAddress + 2) == 0xFF && EEPROM.read(compensationFactorAddress + 3) == 0xFF)
  {
    compensationFactor = 1.0;   // If the EEPROM is new, the compensationFactorAddress is 1.0(default).
    EEPROM_write(compensationFactorAddress, compensationFactor);
  }
}

//returns the temperature from one DS18B20 in DEG Celsius
float readTemperature()
{
  static byte data[12], addr[8];
  static float TemperatureSum = 25;
  static boolean ch = 0;
  if (!ch) {
    if ( !ds.search(addr)) {
      // Serial.println("no more sensors on chain, reset search!");
      ds.reset_search();
      return -1000;
    }
    if ( OneWire::crc8( addr, 7) != addr[7]) {
      //  Serial.println("CRC is not valid!");
      return -1000;
    }
    if ( addr[0] != 0x10 && addr[0] != 0x28) {
      //  Serial.print("Device is not recognized!");
      return -1000;
    }
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1); // start conversion, with parasite power on at the end
  } else {
    byte present = ds.reset();
    ds.select(addr);
    ds.write(0xBE); // Read Scratchpad
    for (int i = 0; i < 9; i++) { // we need 9 bytes
      data[i] = ds.read();
    }
    ds.reset_search();
    byte MSB = data[1];
    byte LSB = data[0];
    float tempRead = ((MSB << 8) | LSB); //using two's compliment
    TemperatureSum = tempRead / 16;
  }
  ch = !ch;
  return TemperatureSum;
}

