#include <DHT.h>
/* use of relay module to power up cooling fan when temperature is above the optimal threshold*/

#define DHTPIN 6     //change to relevant arduino pin 
#define DHTTYPE DHT22  
DHT sensor(DHTPIN, DHTTYPE); 
boolean values_sent= False;
int relay_pin = 9; //initialize digital pin for relay pin
float t= sensor.readTemperature(); //initialize temperature as a float

void setup() { 
sensor.begin(); 
pinMode(relay_pin, OUTPUT);
digitalWrite(relay_pin, HIGH);
}

void loop() { 
// Checking if the sensor is sending values or not
continueChecking();
Serial.println("Temp: ");
Serial.print(t);
Serial.print(" C");
if (t > 35){
  digitalWrite(relay_pin, LOW);
  Serial.print("Fan is ON "); 
  delay(10);
}
else{
  digitalWrite(relay_pin, HIGH);
  Serial.print("Fan is OFF "); 
}
delay(2000);
}

void continueChecking() {
  while (values_sent==False); {
  Serial.println("Waiting for sensor reading...");
  if (isnan(t)); {
    delay(1000);
  }  
  else; {
    values_sent=True;   
  }
  }
}
  
}


/*#include <AFMotor.h>
#include <Wire.h>
#include <DHT.h>

#define DHTPIN 6     //change to relevant arduino pin 
#define DHTTYPE DHT22  
DHT sensor(DHTPIN, DHTTYPE); 
AF_DCMotor fan(2);

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
float last_err;


void setup(){
  Serial.begin(9600);
  
  if (!tempsensor.begin()) {
    Serial.println("Couldn't find MCP9808!");
    while (1);
  }
  fan.run(RELEASE);
  fan.setSpeed(0);
}

void loop(){
setpoint = 30;               // Specified by user, r(t)
  float c = tempsensor.readTempC();
  Serial.print("Temp: "); 
  Serial.println(c); Serial.println("*C\t");
input = (c);                 // Read by your sensor, y(t) ... analogRead(...)
err = (input - setpoint);    // Compute Error, e(t) = r(t) - y(t)
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

