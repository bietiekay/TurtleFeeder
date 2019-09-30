/* Arduino code and information on how to modify a dobar or SaiDeng automatic fish pond feeder
 * (C) Daniel Kirstenpfad 2019
 * 
 * Requirements / Libraries:
 *  - Arduino IDE 1.8.8
 *  - Arduino ESP8266 library 2.5.2 - https://github.com/esp8266/Arduino
 *  - Arduino MQTT Library 2.4.1 by 256dpi - https://github.com/256dpi/arduino-mqtt
 * 
 * This also uses the Arduino OTA functionality. So when the ESP8266 successfully connected to your Wifi you will
 * be able to use the Arduino IDE over-the-air functionality.
 * 
 * 
 * What you need to configure:
 *  - WiFi SSID + PASS
 *  - MQTT Server (IP(+Username+PW))
 *  - MQTT Topic prefix
 *      
 * Commands that can be sent through mqtt to the /feed topic.
 * 
 * There are overall two MQTT topics:
 *    (1) $prefix/feeder-$chipid/state
 *        This topic will hold the current state of the feeder. It will show a number starting from 0 up.
 *        When the feeder is ready it will be 0. When it's currently feeding it will be 1 and up - counting down for
 *        every successfull turn done. 
 *        There is an safety cut-off for the motor. If the motor is longer active than configured in the MaximumMotorRuntime
 *        variable it will shut-off by itself and set the state to -1.
 *        
 *    (2) $prefix/feeder-$chipid/feed
 *        This topic acts as the command topic to start / control the feeding process. If you want to start the process
 *        you would send the number of turns you want to happen. So 1 to 5 seems reasonable. The feeder will show the
 *        progress in the /state topic.
 *        You can update the amount any time to shorten / lengthen the process.
 *        On the very first feed request after initial power-up / reboot the feeder will do a calibration run. This is to make
 *        sure that all the wheels are in the right position to work flawlessly.
 *        
 * examples (for feeder-00F3B839):
 * 
 * mosquitto_pub -t house/stappenbach/feeder/feeder-00F3B839/feed -m 3
 * -> will feed 3 times
 * 
 * mosquitto_sub -v -t house/stappenbach/feeder/feeder-00F3B839/state
 * -> will show the current state of the feeder
 */


#include <MQTT.h>
#include <sstream>

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>


#ifndef STASSID
#define STASSID "yourwifissid"
#define STAPSK  "yourwifipassword"
#endif

// mqtt
const char mqttServer[] = "mqtt-broker-ip";
const int mqttPort = 1883;
const char mqttUser[] = "yourMQTTuser";
const char mqttPassword[] = "yourMQTTpassword";
const char mqtt_topic_prefix[] = "feeder";

// will show up on topics feeder/feeder-$chipid/state and /feed
// send to topic feed a number of how many feed turns you want it to make

// MQTT
MQTTClient mqttclient;
WiFiClient wificlient;
char mqtt_node_name[255];
char mqtt_feed_topic[255];
char mqtt_feeder_topic[255];
// Timer: Auxiliary variables
unsigned long now = millis();
#define MQTTLoopTime 10000
unsigned long lastMQTTLoopActivation = 0;
long MQTTConnectRetriesUntilReboot = 10;

// Counter einbauen der maximale Laufzeit / Fütterzeit und dann sperrt
int motorPin = D2;
int switchPin = D1;

// calibration
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
bool calibrated = false;
bool bootup_started = false;
bool bootup_phase_0_done = false;


bool waiting_for_button_to_become_0 = false;
bool waiting_for_button_to_become_1 = false;

unsigned long MaximumMotorRuntime = 120000;
int MaxMotorDutyCycle = 32767; // depends on arduino used and PWM capabilities
int toFeedCounter = 0;
int motorspeed = 0;
unsigned long LastMotorCommand = 0;
unsigned long MillisSinceMotorCommand = 0;
bool MotorIsRunning = false;

const char* ssid = STASSID;
const char* password = STAPSK;

void setMotor(int speed)
{
  if (speed > 0)
    MotorIsRunning = true;
  else
    MotorIsRunning = false;
  
  Serial.print("Motor-Speed-Control: ");
  Serial.println(speed);
  motorspeed = speed;
  LastMotorCommand = millis();
  analogWrite(motorPin,motorspeed);
}

bool isCurrentlyFeeding()
{
  if (toFeedCounter > 0)
    return true;
  else
    return false;
}

void Feed()
{
  char counter[10];
  sprintf(counter,"%i",toFeedCounter);
  
  if (bootup_started)
        calibration();
        
  if (toFeedCounter > 0)
  {
    // feedCounter is bigger than 0 - we must start feeding, or we are already feeding...
    //Serial.printf("Counter %i - Button %i\n",toFeedCounter, buttonState);
    if (motorspeed > 0)
    {
      // motor is spinning, we must already be feeding!
      if (calibrated) // and it must be spinning not because we're calibrating...
      {
        // so here we are spinning the motor and are calibrated. So the Button will transistion from 0 to 1:
        if (waiting_for_button_to_become_1)
        {
          // when we switched from button not pressed to pressed recently...like right after calibration or in the usual scenario
          if (buttonState == 1)
          {
            Serial.println("Feeder moved far enough, now waiting for one turn...");
            waiting_for_button_to_become_0 = true;
            waiting_for_button_to_become_1 = false;
          }
        }
        // here we are when the button just switched 
        if (waiting_for_button_to_become_0)
        {
          // we are now waiting for the spinner to rest on the switch, turning it to ...
          if (buttonState == 0)
          {
            Serial.println("One Feed completed.");
            toFeedCounter--;
            if (mqttclient.connected())
            {

                mqttclient.publish(mqtt_feeder_topic,counter);     
            }
            else
            {
              // we are not connected anymore?
              Serial.println("MQTT connection error");
            }

            setMotor(0);
          }
         
        }        
      }
    }
    else
    {
      // if not yet calibrated, to a run...
      // Button Verhalten:
      // buttonstate = 1 --> schalter nicht gedrückt
      // buttonstate = 0 --> schalter gedrückt, bevorzugte Parkposition
      if (!calibrated)
        calibration(); // call for a calibration -> this will make sure that the buttonstate is correctly at 0 and went through a 0-1-0 cycle
      else
      {
        setMotor(MaxMotorDutyCycle);       // the motor is not yet spinning, let's get rolling!      
        waiting_for_button_to_become_0 = false; // wait...
        waiting_for_button_to_become_1 = true;
      }
    }
  }
  else // nothing to do...so check on sending out a life-signal
  {
      now = millis();
  
      if ( (now - lastMQTTLoopActivation) > MQTTLoopTime)
      {
        // send our current state
        mqttclient.publish(mqtt_feeder_topic,counter);
        lastMQTTLoopActivation = now;
      }

  }
  
  
}

void calibration()
{
  if (!calibrated)
  {    
    if (!bootup_started) {
      Serial.println("Calibrating...");
      setMotor(MaxMotorDutyCycle);
      bootup_started = true;
    }
    else
    {
      if (buttonState == 1)
      {
        if (bootup_phase_0_done == false)
        {
          bootup_phase_0_done = true;
          Serial.println("Calibration Phase #1 done...progressing...");        
        }
      }
      // when in end-phase
      if (bootup_phase_0_done)
      {
        if (buttonState == 0)
        {
          // we finished!
          calibrated = true;
          bootup_started = false;
          bootup_phase_0_done = false;
          Serial.println("Calibration finished.");
          setMotor(0);
        }
      }
    }
    delay(1000);
  }
}

void messageReceived(String &topic, String &payload) 
{
  Serial.println("--------------> mqtt incoming: " + topic + " - " + payload);
  // check for commands
  // commands:
  // number of times to feed
  toFeedCounter = payload.toInt();
  if (mqttclient.connected())
  {
    char counter[10];
    sprintf(counter,"%i",toFeedCounter);
    if (mqttclient.connected())
    {
      mqttclient.publish(mqtt_feeder_topic,counter);     
    }
  }

}


void setup() {
  char hostname[15];
  Serial.begin(115200);
  Serial.println("Booting");
  int reading = digitalRead(switchPin);
  buttonState = reading;  
  
  Serial.print("Buttonstate: ");
  Serial.println(buttonState);  
  // set-up WiFi and node-name feeder-00F3B839
  snprintf(hostname,255,"feeder-%08X", ESP.getChipId());
  Serial.print("Hostname: ");
  Serial.println(hostname);
  
  snprintf (mqtt_feed_topic, 255, "%s/%s/feed",mqtt_topic_prefix,hostname);
  snprintf (mqtt_feeder_topic, 255, "%s/%s/state",mqtt_topic_prefix,hostname);

  Serial.print("MQTT Server: ");
  Serial.println(mqttServer);
  Serial.println("MQTT Topics: ");  
  Serial.println(mqtt_feed_topic);


  
  Serial.println("TurtleFeeder starting up...");
  //Serial.println("Speed 0 to 255");

  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  pinMode(motorPin, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(switchPin), triggerSwitch, CHANGE);
  digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  // MQTT
  mqttclient.begin(mqttServer, wificlient);
  mqttclient.onMessage(messageReceived);

 int i=0;
  while (!mqttclient.connect(mqtt_node_name, mqttUser, mqttPassword)) 
  {
    delay(1000);
    Serial.println("MQTT connect");
    i++;

    if (i >= MQTTConnectRetriesUntilReboot)
    {
      Serial.println("Reached too many retries - Restarting...");
      ESP.restart();
    }
  }
  Serial.println(" MQTT connected");

  // subscribe to command topic
  //mqttclient.onMessage(messageReceived);
  mqttclient.subscribe(mqtt_feed_topic);

  if (mqttclient.connected())
  {
    mqttclient.publish(mqtt_feeder_topic,"0");   
  }
  
  // OTA =================================
  // ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname(hostname);
  // ArduinoOTA.setPassword("admin");
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
    digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%", (progress / (total / 100)));
    Serial.println();
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
// =================================================================================================================
// =================================================================================================================
// the loop function runs over and over again forever
void loop() {
  ArduinoOTA.handle();
  
int i=0;
  while (!mqttclient.connect(mqtt_node_name, mqttUser, mqttPassword)) 
  {
    delay(1000);
    Serial.println("MQTT connect");
    i++;

    if (i >= MQTTConnectRetriesUntilReboot)
    {
      Serial.println("Reached too many retries - Restarting...");
      ESP.restart();
    }
  }
  //Serial.println(" MQTT connected");

  mqttclient.loop();
  
  Feed(); // calls the feed function, won't do anything if the feedCounter is 0
  // check for MaximumMotorRuntime

  if (MotorIsRunning)
  {
    MillisSinceMotorCommand = millis() - LastMotorCommand;
    if (MillisSinceMotorCommand >= MaximumMotorRuntime)
    {
      if (mqttclient.connected())
      { // report possible error to MQTT
        mqttclient.publish(mqtt_feeder_topic,"-1");   
      }
      Serial.println("Maximum Motor Runtime reached: Stopping");
      setMotor(0);
    }    
  }

  if (Serial.available())
  {
    int feed = Serial.parseInt();
    Serial.print("Feed ");
    Serial.println(feed);
    if (feed >= 0 && feed <= 5)
    {
      toFeedCounter = feed;
    }
  }
}

// gets triggered on switch interrupt...
ICACHE_RAM_ATTR void triggerSwitch() {
  int reading = digitalRead(switchPin);
  buttonState = reading;  
  
  Serial.print("Buttonstate: ");
  Serial.println(buttonState);  
}
// =================================================================================================================
// =================================================================================================================
