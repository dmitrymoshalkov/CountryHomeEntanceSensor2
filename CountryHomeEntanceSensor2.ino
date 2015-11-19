
#include <MySensor.h>  
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Bounce2.h>
#include <avr/wdt.h>

#define COMPARE_TEMP 1 // Send temperature only if changed? 1 = Yes 0 = No
#define MAX_ATTACHED_DS18B20 16

#define NODE_ID 7

#define CHILD_ID_TEMPERATURE 1  //DHT22
#define CHILD_ID_DOOR 2
#define CHILD_ID_MOTION 3
#define BUZZER_CHILD_ID 4

#define REBOOT_CHILD_ID 100
#define DISABLE_MOTION_SENSOR_CHILD_ID 101
#define RECHECK_SENSOR_VALUES          102


#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0 // GPIO value to write to turn off attached relay

#define BUZZER_DIGITAL_PIN 8 
#define TEMPERATURE_SENSOR_DIGITAL_PIN 5
#define DOOR_SENSOR_DIGITAL_PIN 3
#define MOTION_SENSOR_DIGITAL_PIN 4


boolean metric = true;          // Celcius or fahrenheid
float lastTemp = -1;
long previousTempMillis = 0;        // last time the sensors are updated
long TempsensorInterval = 60000;     // interval at which we will take a measurement ( 30 seconds)
int oldDebouncerState=-1;
boolean lastMotion=false;

unsigned long previousMSMillis=0;
unsigned long MSsensorInterval=60000;

boolean boolMotionSensorDisabled = false;
boolean boolRecheckSensorValues = false;


OneWire oneWire(TEMPERATURE_SENSOR_DIGITAL_PIN); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 

Bounce debouncer = Bounce(); 

MySensor gw;

MyMessage TempMsg(CHILD_ID_TEMPERATURE, V_TEMP);
MyMessage DoorMsg(CHILD_ID_DOOR, V_TRIPPED);
MyMessage MotionMsg(CHILD_ID_MOTION, V_TRIPPED);
MyMessage MotionStateMsg(DISABLE_MOTION_SENSOR_CHILD_ID, V_ARMED);

void setup() {
 
  // requestTemperatures() will not block current thread
 // sensors.setWaitForConversion(true);

      Serial.begin(115200);
    Serial.println("Begin setup");
    // Initialize library and add callback for incoming messages
    gw.begin(incomingMessage, NODE_ID, false);

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Country home entrance sensor", "1.0");

   metric = gw.getConfig().isMetric;

  // Present all sensors to controller
     gw.present(CHILD_ID_TEMPERATURE, S_TEMP);
          // Fetch temperatures from Dallas sensors
          sensors.requestTemperatures(); 
          float temperature = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(0) * 10.)) / 10.;
 
  // Setup the button
  pinMode(DOOR_SENSOR_DIGITAL_PIN,INPUT);
  // Activate internal pull-up
  digitalWrite(DOOR_SENSOR_DIGITAL_PIN,HIGH);
  
  // After setting up the button, setup debouncer
  debouncer.attach(DOOR_SENSOR_DIGITAL_PIN);
  debouncer.interval(5);
 
   gw.present(CHILD_ID_DOOR, S_DOOR); 
  
  
  //Motion sensor
  pinMode(MOTION_SENSOR_DIGITAL_PIN, INPUT);      // sets the motion sensor digital pin as input
  // Register all sensors to gw (they will be created as child devices)
  gw.present(CHILD_ID_MOTION, S_MOTION);
  
  
  //buzzer
        pinMode(BUZZER_DIGITAL_PIN,OUTPUT);
     digitalWrite(BUZZER_DIGITAL_PIN,HIGH); 
     gw.present(BUZZER_CHILD_ID, S_LIGHT); 

//reboot sensor command
     gw.present(REBOOT_CHILD_ID, S_BINARY); 

//disable-enable motion sensor
     gw.present(DISABLE_MOTION_SENSOR_CHILD_ID, S_MOTION); 


//reget sensor values
  gw.present(RECHECK_SENSOR_VALUES, S_LIGHT); 

  
// Send initial state of sensors to gateway  
  debouncer.update();
  int value = debouncer.read();
  gw.send(DoorMsg.set(value==HIGH ? 1 : 0));  
  
  boolean motion = digitalRead(MOTION_SENSOR_DIGITAL_PIN) == HIGH; 
  gw.send(MotionMsg.set(motion ? "1" : "0" ));  // Send motion value to gw
  
  
    //Enable watchdog timer
    wdt_enable(WDTO_8S);
    
    Serial.println("End setup");  

}

void loop() {
  // put your main code here, to run repeatedly:


if ( !boolMotionSensorDisabled )
 {
 // Read digital motion value
  boolean motion = digitalRead(MOTION_SENSOR_DIGITAL_PIN) == HIGH; 
   if (lastMotion != motion  || boolRecheckSensorValues ) {
    Serial.println("Motion detected");
  lastMotion = motion;     
   gw.send(MotionMsg.set(motion ? "1" : "0" ));  // Send motion value to gw
  }
}
  

checkTemp();

  debouncer.update();
  // Get the update value
  int value = debouncer.read();
 
  if (value != oldDebouncerState || boolRecheckSensorValues) {
     // Send in the new value
     gw.send(DoorMsg.set(value==HIGH ? 1 : 0));
     oldDebouncerState = value;
         Serial.print("Door: ");
        Serial.println(value);
  }

  reportMotionSensorState();  

    if (boolRecheckSensorValues)
      {
       boolRecheckSensorValues = false;
      }

    // Alway process incoming messages whenever possible
    gw.process();
    
    //reset watchdog timer
        wdt_reset();
}



void checkTemp()
{

    unsigned long currentTempMillis = millis();
    if(currentTempMillis - previousTempMillis > TempsensorInterval || boolRecheckSensorValues) {
        // Save the current millis
        previousTempMillis = currentTempMillis;
        // take action here:

          // Fetch temperatures from Dallas sensors
          sensors.requestTemperatures();
    //float temperature = static_cast<float>(static_cast<int>(sensors.getTempCByIndex(0))) / 10.;
       // float temperature = static_cast<float>(static_cast<int>((gw.getConfig().isMetric?sensors.getTempCByIndex(0):sensors.getTempFByIndex(0)) * 10.)) / 10.;
        float temperature = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(0) * 10.)) / 10.;

        Serial.print("Temp: ");
        Serial.println(temperature);
        if (temperature != lastTemp) {
            gw.send(TempMsg.set(temperature,1));
            lastTemp = temperature;
        } 
        
        
    }    

  
}


void reportMotionSensorState()
{

    unsigned long currentMSMillis = millis();
    if(currentMSMillis - previousMSMillis > MSsensorInterval ) {
        // Save the current millis
        previousMSMillis = currentMSMillis;
        // take action here:

       gw.send(MotionStateMsg.set(boolMotionSensorDisabled ? "1" : "0" ));  
        
    }    

  
}


void incomingMessage(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.

    if ( message.sensor == BUZZER_CHILD_ID ) {
     digitalWrite(BUZZER_DIGITAL_PIN, message.getBool()?RELAY_OFF:RELAY_ON);

     }
    if ( message.sensor == REBOOT_CHILD_ID ) {
             wdt_enable(WDTO_30MS);
              while(1) {};

     }

     if ( message.sensor == DISABLE_MOTION_SENSOR_CHILD_ID ) {
         
         if (message.getBool() == true)
         {
            boolMotionSensorDisabled = true;
         }
         else
         {
            boolMotionSensorDisabled = false;
         }

     }
     
    if ( message.sensor == RECHECK_SENSOR_VALUES ) {
         
         if (message.getBool() == true)
         {
            boolRecheckSensorValues = true;
         }

     }   

        return;      
} 


