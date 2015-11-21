
#include <MySensor.h>  
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Bounce2.h>
#include <avr/wdt.h>


#define NODE_ID 7

#define RADIO_RESET_DELAY_TIME 20 //Задержка между сообщениями
#define MESSAGE_ACK_RETRY_COUNT 5  //количество попыток отсылки сообщения с запросом подтверждения


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

boolean gotAck=false; //подтверждение от гейта о получении сообщения 
int iCount = MESSAGE_ACK_RETRY_COUNT;

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

      //Serial.begin(115200);
    //Serial.println("Begin setup");
    // Initialize library and add callback for incoming messages
    gw.begin(incomingMessage, NODE_ID, false);

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Country home entrance sensor 2", "1.1");
        gw.wait(RADIO_RESET_DELAY_TIME);   


   metric = gw.getConfig().isMetric;

  // Present all sensors to controller
     gw.present(CHILD_ID_TEMPERATURE, S_TEMP);
        gw.wait(RADIO_RESET_DELAY_TIME);   


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
         gw.wait(RADIO_RESET_DELAY_TIME);    
  
  //Motion sensor
  pinMode(MOTION_SENSOR_DIGITAL_PIN, INPUT);      // sets the motion sensor digital pin as input
  // Register all sensors to gw (they will be created as child devices)
  gw.present(CHILD_ID_MOTION, S_MOTION);
         gw.wait(RADIO_RESET_DELAY_TIME);   

  
  //buzzer
        pinMode(BUZZER_DIGITAL_PIN,OUTPUT);
     digitalWrite(BUZZER_DIGITAL_PIN,HIGH); 
     gw.present(BUZZER_CHILD_ID, S_LIGHT); 
        gw.wait(RADIO_RESET_DELAY_TIME);   

//reboot sensor command
     gw.present(REBOOT_CHILD_ID, S_BINARY); 
        gw.wait(RADIO_RESET_DELAY_TIME);   

//disable-enable motion sensor
     gw.present(DISABLE_MOTION_SENSOR_CHILD_ID, S_MOTION); 
        gw.wait(RADIO_RESET_DELAY_TIME);   


//reget sensor values
  gw.present(RECHECK_SENSOR_VALUES, S_LIGHT); 
        gw.wait(RADIO_RESET_DELAY_TIME);   

  
// Send initial state of sensors to gateway  
  debouncer.update();
  int value = debouncer.read();
  gw.send(DoorMsg.set(value==HIGH ? 1 : 0));  
         gw.wait(RADIO_RESET_DELAY_TIME);   

  boolean motion = digitalRead(MOTION_SENSOR_DIGITAL_PIN) == HIGH; 
  gw.send(MotionMsg.set(motion ? "1" : "0" ));  // Send motion value to gw
        gw.wait(RADIO_RESET_DELAY_TIME);   

  
    //Enable watchdog timer
    wdt_enable(WDTO_8S);
    
    //Serial.println("End setup");  

}

void loop() {
  // put your main code here, to run repeatedly:


if ( !boolMotionSensorDisabled )
 {
 // Read digital motion value
  boolean motion = digitalRead(MOTION_SENSOR_DIGITAL_PIN) == HIGH; 
   if (lastMotion != motion  || boolRecheckSensorValues ) {

  //Отсылаем состояние сенсора с подтверждением получения
  iCount = MESSAGE_ACK_RETRY_COUNT;

    while( !gotAck && iCount > 0 )
    {
      gw.send(MotionMsg.set(motion ? "1" : "0" ), true);  // Send motion value to gw
      gw.wait(RADIO_RESET_DELAY_TIME);
      iCount--;
    }
    gotAck = false;

    //Serial.println("Motion detected");
  lastMotion = motion;     

  }
}
  

checkTemp();

  debouncer.update();
  // Get the update value
  int value = debouncer.read();
 
  if (value != oldDebouncerState || boolRecheckSensorValues) {

  //Отсылаем состояние сенсора с подтверждением получения
  iCount = MESSAGE_ACK_RETRY_COUNT;

    while( !gotAck && iCount > 0 )
    {
      gw.send(DoorMsg.set(value==HIGH ? 1 : 0), true);  // Send motion value to gw
      gw.wait(RADIO_RESET_DELAY_TIME);
      iCount--;
    }
    gotAck = false;

     oldDebouncerState = value;
      //   Serial.print("Door: ");
      //  Serial.println(value);
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

        //Serial.print("Temp: ");
        //Serial.println(temperature);
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

     if (message.isAck())
    {
      gotAck = true;
      return;
    }


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

         gw.send(MotionStateMsg.set(boolMotionSensorDisabled ? "1" : "0" )); 

     }
     
    if ( message.sensor == RECHECK_SENSOR_VALUES ) {
         
         if (message.getBool() == true)
         {
            boolRecheckSensorValues = true;
         }

     }   

        return;      
} 


