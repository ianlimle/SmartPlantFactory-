/**********************************************************************************
 *  Control Behavior:
 *    If the float switch is not floating (i.e. empty tank) then turn on the pump and led
 *    If the float switch is floating (i.e. full tank) turn off the pump and LED
 ***********************************************************************************/
//define the input/output pins
int meter[10];
byte sampleCount = 0;
byte currentSample = 0;

byte sensorReading; //reading on the water sensor either as 0 or 1  

#define FLOAT_SWITCH_PIN 1       //float pin switch
#define PUMP_PIN 2               //pin to turn off/on water flow
#define PIN_BUZZER 4             //piezo buzzer for alerts
#define LED_PIN 5                //Initialize the LED pin
#define FLOAT_SWITCH_PWR 6       //Turn on the power for the sensor  

//setup runs once
void setup()
{
  //setup input and output pins  
  pinMode(FLOAT_SWITCH_PIN, INPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT); 

  //set these as OFF first 
  digitalWrite(FLOAT_SWITCH_PWR, LOW);
  digitalWrite(PUMP_PIN, LOW);

  //Open serial monitor for debugging
  Serial.begin(9600);
}

//loop() runs indefinitely 
void loop(){
  
  long timerA = 0;
  long timerB = 0;

  if (sampleCount<10)
  {
    updateSensor();
  }
  else
  {
    blinkPattern(60);
    Serial.println("Waiting for detection");
    waitForDetection();

    //At this point, the sensor is not activated
    //Try for another 30 seconds to see if it will come back
    blinkPattern(10);

    Serial.println("No sensor detected. Confirming for 30 secs");
    timerA = millis();

    do {
      delay(500);
      updateSensor();
      if (timerA - millis() > 30000) break;
    } while (sensorReading==0);

    //Still having no detection issues?
    if (sensorReading==0)
    {
      //Alert that sensor is not detecting via a buzzer
      buzz(230);
      delay(200);
      buzz(230);

    Serial.println("WATER Turning On");
    digitalWrite(PUMP_PIN, HIGH);

    //Start timer B
    timerB = millis();

    //Monitor water level
    Serial.println("Monitoring water levels");
    do {
      delay(500);
      updateSensor();
      if (timerB - millis() > 30000)    // if sensor is not triggered within 30 seconds of the fill operation starting, water is shut off and system enters an "error" state
      {
        Serial.println("Water should be full. Detection failure");

        //Enter overflow emergency shutoff
        digitalWrite(PUMP_PIN, LOW);
        do {
          alarm(5);
        } while (true);
        break;  //requires a manual reset
      }
    }while (sensorReading==0); 

      Serial.println("Fill level reached. Shutting off water flow");
      digitalWrite(PUMP_PIN, LOW);

     } else {
      Serial.println("False positive. Continue filling");
  }
 } 
  delay(20);
}  

/*Relevant functions*/        
void waitForDetection()
{
 do {
  delay(10000);
  updateSensor();
  } while (sensorReading== 1);
}

void blinkPattern(byte nCount)
{
  for (long i = 0; i< nCount; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(10);
    digitalWrite(LED_PIN, LOW);
  }
}

void buzz(long nFreq)
{
  for (long i = 0; i< 1000; i++)
  {
    //1/2048Hz=488uS, or 244uS high and 244 uS low creates a 50% duty cycle
    digitalWrite(PIN_BUZZER, HIGH);
    delayMicroseconds(nFreq);
    digitalWrite(PIN_BUZZER, LOW);
    delayMicroseconds(nFreq);
  }
}    
       
void alarm(byte nCount)
{
  for (byte i = 0; i<nCount; i++)
  {
    buzz(230);
    buzz(270);
  }
}
 
void updateSensor()
{
  //Take a sample
  digitalWrite(FLOAT_SWITCH_PWR, HIGH);
  delay(1);
        
  byte m = digitalRead(FLOAT_SWITCH_PIN);
        
  digitalWrite(FLOAT_SWITCH_PWR, LOW);
  
  meter[currentSample] = m;
  currentSample++;
  if (currentSample > 9) currentSample = 0;
  if (sampleCount < 10)
  {
    //First 10 samples are not counted
    sampleCount++;
  } else {
    //All samples have to be the same to change the reading. This is to ensure 
    float t = 0;
    for (byte i= 0; i<10; i++)
    {
      t = t + meter[i];   //sum the values of all 10 samples
    }
  
    t = t/ 10;            //take the average of the sum 
    if (t ==0 || t==1)    //if each value of the last 10 samples are the same to avoid momentary fluctuations
    {
      sensorReading= t;
    }
   }
}
     
/***************************
 * Alternative codes: 
 * 
//define the input/output pins
#define FLOAT_SWITCH_PIN 1
#define PUMP_PIN 2
#define LED_PIN 3

//setup runs once
void setup()
{
  //setup input pins for float switch 
  //Too use a bare switch with no external pullup resistor, set the pin mode to INPUT_PULLUP to use internal pull resistors. This will invert the standard high/low behavior
  pinMode(FLOAT_SWITCH_PIN, INPUT_PULLUP);
  
  //setup output pins for relays/pumping station and LED board
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
}

//loop() runs indefinitely 
void loop()
{
  //check to see the state of the float switch. These states are assuming the pin is using an internal pullup resistor. 
  // LOW corresdponds to the float switch being at its lowest point (i.e. low water)
  if(digitalRead(FLOAT_SWITCH_PIN) == LOW)
  {
     digitalWrite(PUMP_PIN, HIGH); //turn on the pump
     digitalWrite(LED_PIN, HIGH);    //turn on the LED
  }
  
  //otherwise the float switch is HIGH
  // HIGH corresponds to the float switch being at its higest point (i.e. full water)
  else
  {
     digitalWrite(PUMP_PIN, LOW); //turn off the pump
     digitalWrite(LED_PIN, LOW);    //turn off the LED
  }
}
**********************************/
     
