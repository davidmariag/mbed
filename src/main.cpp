//#include "config.h"       //would it make sense to have a config file with the defines?

#include <WiFi.h>
#include <WiFiUdp.h>

#include <HardwareSerial.h>
#include <PubSubClient.h>     // https://github.com/knolleary/pubsubclient
#include "LittleFS.h"
#include "FS.h"
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//pins 21 and 22 SDA and SCL respectively
#define RXD2 16    
#define TXD2 17
//pins 21 and 22 SDA and SCL respectively
#define fanOnPin 26               //white wire
#define fanOffPin 18              //green wire
#define lightTogglePin 19         //blue wire
#define ambientPin 36             //ADC1_0 36 light sense  10K pho to GRN (other to 3.3V)     
#define motionSensor 39           //PIR


#define SEALEVELPRESSURE_HPA (1013.25)
#define halfSecond 500
#define oneSecond 1000
#define ten_Seconds 10000
#define ten_Min_To_Sec 600     //should 600

#include <NTPClient.h> //for time
#include <time.h> 
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -21600;
const int   daylightOffset_sec = 3600;
struct tm timeinfo;

#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

//nextion debugs
char debug3[32];
char debug2[32];
char debug1[32];

const char* ssid = "DavesFarm";                  //Enter wifi network name
const char* password = "41314131"; 
// MQTT Server Setting variables...
IPAddress mqtt_server_ip          (192,168,1,157);                     // IP Address for the MQTT Server...
const int mqtt_port =             1883;                                  // Port for the MQTT Server...
const char* mqtt_username =       "dave";                            // MQTT Server username...
const char* mqtt_password =       "mave";                            // MQTT Server password...
String base_mqtt_topic =          "mBed";    // Start of the MQTT Topic name used by this device
String mqtt_topic_Outlight =      "outLight";    // Start of the MQTT Topic name used by this device
String mqtt_topic_upOutlight =    "outUpLight";    // Start of the MQTT Topic name used by this device
String mqtt_client_id;                                                   // Will hold unique ID, so use ChipID will be assigned in void setup
// Create instances of the Wi-Fi, MQTT
WiFiClient My_WiFi_Client;
PubSubClient MQTTclient(My_WiFi_Client);

String data_from_Nex;
String msg_payload;
String msg_topic;

bool lightToTurnOff = false;
bool lightToTurnOn = false;
bool fan = false;
bool light = false;
bool lightStateInter = false;
bool snooze  = false;
bool nightTime = 1;
volatile int lightTimeout = 0;

static int lightsOn = 7;
static int lightsOut = 23;
int timeHour;

// Task handles
static TaskHandle_t light_Task = NULL;
static TaskHandle_t light_On_Inter = NULL;
static TaskHandle_t fan_Task = NULL;
static TaskHandle_t ambientLightCheck_Task = NULL;
static TaskHandle_t timeCheck_Task = NULL;
static TaskHandle_t Wifi_Task = NULL;
//static TaskHandle_t light_Off_Trigger = NULL;
static TaskHandle_t read_Time = NULL;
static TaskHandle_t data_Nextion = NULL;
static TaskHandle_t debug_Print = NULL;
static TaskHandle_t read_InTemp = NULL;

int mqttCounter;
int ambientL;
int backLight = 1;                                //initial screen light
int intHumid;
float floatTemp;

Adafruit_BME280 bme;
// Set your Static IP address
IPAddress local_IP(192, 168, 1, 10);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

void IRAM_ATTR interruptHandler1()              //Motion sensor pin drives interrupt routine
{              
  BaseType_t xtr = pdFALSE;

  if (light) 
  {
    lightTimeout = ten_Min_To_Sec-1;             //Keep light on if motion detected
  }
  else if(!nightTime && ambientL < 12)                        //was 10
  {
    lightToTurnOn = true;
    vTaskResume(light_On_Inter);
  } 
  portYIELD_FROM_ISR(xtr);
}

void setup() 
{

  pinMode(lightTogglePin, OUTPUT);
  digitalWrite(lightTogglePin,HIGH);
  pinMode(fanOffPin, OUTPUT);
  pinMode(fanOnPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(motionSensor), interruptHandler1, FALLING);
  //immediately detach
  //detachInterrupt(digitalPinToInterrupt(motionSensor));
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,password); 
  // Initialize serial communication

  //vTaskDelay(3000);
 WiFi.mode(WIFI_STA);
    // Configures static IP address
  WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   vTaskDelay(500);
  //   Serial.print(".");
  // }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Create tasks
  xTaskCreate(keepWiFiAlive, "Keep_Wifi_Alive", 5000, NULL, 1, &Wifi_Task);
  xTaskCreate(ambientLightCheckTask, "Ambient Light Check Task", 1000, NULL, 1, &ambientLightCheck_Task);
  xTaskCreate(fanTask, "Fan Task", 1000, NULL, 1, &fan_Task);
  vTaskSuspend(fan_Task);
  xTaskCreate(lightTask, "Light Task", 2000, NULL, 1, &light_Task);
  xTaskCreate(lightOnInter, "Switch light On Int", 1500, NULL, 1, &light_On_Inter);
//  xTaskCreate(lightOffTrigger, "Switch light Off", 1500, NULL, 1, &light_Off_Trigger);
  xTaskCreate(readTime, "upDate Time", 4000, NULL, 1, &read_Time);
  xTaskCreate(data_in_Nex, "Get data from Nextion", 2500, NULL, 1, &data_Nextion);
  xTaskCreate(readInTemp, "Read inside Temp", 2500, NULL, 1, &read_InTemp);
  xTaskCreate(debugPrint, "NextionDebug", 2000, NULL, 1, &debug_Print);
  
  //Suspend Tasks

  vTaskSuspend(light_On_Inter);

  bme.begin(0x76);

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  MQTTclient.setServer(mqtt_server_ip, mqtt_port);
  MQTTclient.setCallback(MQTTcallback);           // This function is called automatically whenever a message arrives on a subscribed topic
  digitalWrite(fanOffPin,HIGH);
  digitalWrite(fanOnPin,HIGH);
  pinMode(motionSensor, INPUT);
  vTaskDelete(NULL); 
}

void loop() {
  // Empty loop as FreeRTOS manages the tasks
  vTaskDelete(NULL); 
}
void keepWiFiAlive(void * parameter){
    for(;;)
    {
      //vTaskDelay(10);
      if (!MQTTclient.connected())     // Note that MQTTclient.connected() will still return 'true' until the MQTT keepalive timeout has expired (around 35 seconds for my setup) 
      {
        connect_to_MQTT();
        vTaskDelay(5000);
        mqttCounter++;
        WriteStr("page0.maxT.txt",String(mqttCounter));
        if(mqttCounter > 50)
        {
          ESP.restart();
        }
      }
      else
      {
        MQTTclient.loop();
        if(mqttCounter)
        {
          mqttCounter = 0;
          WriteStr("page0.maxT.txt",String(mqttCounter));
          debugln("reset counter");
        }
      }  
    }
   vTaskDelete(NULL);     
}

void connect_to_MQTT()
{
  debugln("Connecting to MQTT...");
  if(MQTTclient.connect(mqtt_client_id.c_str(), mqtt_username, mqtt_password, (base_mqtt_topic + "/Status").c_str(),0, 0, "Dead"))   //c_str(),0, 1, "Dead"))
  {
    vTaskDelay(halfSecond);
    // We get here if the connection was successful... 
    debugln("MQTT Connected");
       
    
    // ... and then re/subscribe to the watched topics
    MQTTclient.subscribe((base_mqtt_topic + "/mLight").c_str());   // Watch the .../LED topic for incoming MQTT messages
    MQTTclient.subscribe((base_mqtt_topic + "/mFan").c_str());   // Watch the .../LED topic for incoming MQTT messages
    MQTTclient.subscribe((base_mqtt_topic + "/States").c_str());   // Watch the .../LED topic for incoming MQTT messages 
    MQTTclient.subscribe((base_mqtt_topic + "/tempOut").c_str());   // Watch the .../LED topic for incoming MQTT messages 
    MQTTclient.subscribe((base_mqtt_topic + "/humOut").c_str());   // Watch the .../LED topic for incoming MQTT messages 
    MQTTclient.subscribe((base_mqtt_topic + "/windS").c_str());   // Watch the .../LED topic for incoming MQTT messages 
    MQTTclient.subscribe((base_mqtt_topic + "/windD").c_str());   // Watch the .../LED topic for incoming MQTT messages 
    MQTTclient.subscribe((base_mqtt_topic + "/dLamp").c_str());   // Watch the .../LED topic for incoming MQTT messages 
    MQTTclient.subscribe((base_mqtt_topic + "/mLamp").c_str());   // Watch the .../LED topic for incoming MQTT messages 
    MQTTclient.subscribe((base_mqtt_topic + "/GTB").c_str());   // Watch the .../LED topic for incoming MQTT messages 
    MQTTclient.subscribe((base_mqtt_topic + "/dLampfb").c_str());
    MQTTclient.subscribe((base_mqtt_topic + "/mLampfb").c_str());
    MQTTclient.subscribe((mqtt_topic_Outlight + "/mainDoorfb").c_str());   // Watch the .../LED topic for incoming MQTT messages
    MQTTclient.subscribe((mqtt_topic_upOutlight + "/upDoorfb").c_str());   // Watch the .../LED topic for incoming MQTT messages
 
     // Add other watched topics in here...  <--------------------- THIS IS THE BIT THAT YOU'LL FORGET TO DO!!

    // We can update the /Status topic to "Alive" now...
    
    MQTTclient.publish((base_mqtt_topic + "/Status").c_str(),"Alive",true);  //was true

  } 
  else
  {
    // We get here if the connection failed...   
    debug(F("MQTT Connection FAILED, Return Code = "));
    debugln(MQTTclient.state());
 //   debugln(); 
    /*
    MQTTclient.state return code meanings...
    -4 : MQTT_CONNECTION_TIMEOUT - the server didn't respond within the keepalive time
    -3 : MQTT_CONNECTION_LOST - the network connection was broken
    -2 : MQTT_CONNECT_FAILED - the network connection failed
    -1 : MQTT_DISCONNECTED - the client is disconnected cleanly
     0 : MQTT_CONNECTED - the client is connected
     1 : MQTT_CONNECT_BAD_PROTOCOL - the server doesn't support the requested version of MQTT
     2 : MQTT_CONNECT_BAD_CLIENT_ID - the server rejected the client identifier
     3 : MQTT_CONNECT_UNAVAILABLE - the server was unable to accept the connection
     4 : MQTT_CONNECT_BAD_CREDENTIALS - the username/password were rejected
     5 : MQTT_CONNECT_UNAUTHORIZED - the client was not authorized to connect * 
     */
  }

}

void MQTTcallback(char* topic, byte* payload, unsigned int length)
{
  debug("[");                  //debug(F("["));
  debug(topic);
  debug("] [");                //debug(F("] ["));
  for (int i=0;i<length;i++)
  {
    debug((char)payload[i]);
  }
  debugln("]");    
  
  msg_topic = String((char *)topic);
  msg_payload = String((char *)payload);
  msg_payload.remove(length); // Trim any unwanted characters off the end of the string
  // We now have two string variables, 'msg_topic' and 'msg_payload' that we can use in 'if' statements below... 
  
  if (msg_topic==base_mqtt_topic + "/GTB" )    //go to bed routine
  {
    light = 0;
    // snoozeActive = 1;
    // motionOveride = 1;
    // snoozeCounter  = 0;
    // roomMotion = 0;
    // detectedCounter = 0;
    lightTimeout = lightTimeout + 10800;
    detachInterrupt(digitalPinToInterrupt(motionSensor));
    vTaskResume(&light_Task);
  }
  
  else if (msg_topic==base_mqtt_topic + "/tempOut" )
  {
     WriteStr("page0.outT.txt", msg_payload);
  }

  else if (msg_topic==base_mqtt_topic + "/humOut" )
  {
     WriteStr("page0.outH.txt", msg_payload);
  }

  else if (msg_topic==base_mqtt_topic + "/windS" )
  {
     WriteStr("page0.windS.txt", msg_payload);
     msg_topic = "m";
  }

  else if (msg_topic==base_mqtt_topic + "/windD" )
  {
     int windDeg = atoi(msg_payload.c_str());
     if(windDeg < 23 | windDeg > 338)
     {      
       WriteNum("b0.picc",0);       //n
       debugln("wind 1"); 
     }
     else if(windDeg < 68)
     {
       WriteNum("b0.picc",1);       //se      
       debugln("wind 2");
     }
     else if(windDeg < 113)
     {
       WriteNum("b0.picc",3);         //e     
       debugln("wind 3");  
     }
     else if(windDeg < 158)
     {
       WriteNum("b0.picc",4);         //se      
       debugln("wind 4");    
     }
     else if(windDeg < 203)
     {
       WriteNum("b0.picc",5);     //s
       debugln("wind 5");   
     }
     else if(windDeg < 248)
     {
       WriteNum("b0.picc",6);    //sw
       debugln("wind 6");    
     }
     else if(windDeg < 293)
     {
       WriteNum("b0.picc",7);    //w
       debugln("wind 7");  
     }
     else
     {
       WriteNum("b0.picc",8);     //ne
       debugln("wind 8");   
     }
  }
    
  else if (msg_topic==base_mqtt_topic + "/mFan" )
  {
    mfanButton();
  }

  else if (msg_topic==base_mqtt_topic + "/mLight" )
  {
    mlightButton();    //decides how to handle message
  }
  
  else if (msg_topic==base_mqtt_topic + "/mLampfb" )
  {
    if(msg_payload == "0")
    {
      WriteNum("page0.maLamp.picc",0);
    }
    else if(msg_payload == "1")
    {
      WriteNum("page0.maLamp.picc",1);
    }
  }

  else if (msg_topic==base_mqtt_topic + "/dLampfb" )
  {
    if(msg_payload == "0")
    {
      WriteNum("page0.dvLamp.picc",0);
    }
    else if(msg_payload == "1")
    {
      WriteNum("page0.dvLamp.picc",1);
    }
  }
  
  else if (msg_topic==mqtt_topic_Outlight + "/mainDoorfb" )
  {
    //debugln("LowerOD");
    if(msg_payload == "0")
    {
      WriteNum("page2.lowerOD.picc",13);
    }
    if(msg_payload == "1")
    {
      WriteNum("page2.lowerOD.picc",14);
    }
  }

  else if (msg_topic==mqtt_topic_upOutlight + "/upDoorfb" )
  {
    //debugln("LowerOD");
    if(msg_payload == "0")
    {
      WriteNum("page2.upperOD.picc",13);
    }
    if(msg_payload == "1")
    {
      WriteNum("page2.upperOD.picc",14);
    }
  }

  else if (msg_topic==base_mqtt_topic + "/States")
  {
    if (msg_payload=="Alive") 
      {
        //put all feed back topics here for this control only
        MQTTclient.publish((base_mqtt_topic + "/mLightfb").c_str(),String(light).c_str(),true);
        MQTTclient.publish((base_mqtt_topic + "/mFanfb").c_str(),String(fan).c_str(),true);      
      }
  }
  msg_topic = "m";
  // Handle messages from other topics in here,
  // DON'T FORGET to subscribe to the topic in void MQTT_Connect()
} // End of void MQTTcallback

void ambientLightCheckTask(void* parameter) 
{
  while (1) 
  {
    vTaskDelay(oneSecond);
    ambientL = map(analogRead(ambientPin), 0.0f, 1000.0f, 1, 100);
    if(ambientL <= 2 && backLight >=2)
    {
      backLight = 1;
    }
    
    else if(ambientL < backLight - 5)
    {
      backLight = backLight - 2;
      if(backLight <= 1)
      backLight = 1;
    }
  
    else if(ambientL > backLight + 5)
    {
      backLight = backLight + 2;
    }
    Serial2.print(F("dim="));
    Serial2.print(String(backLight));
    Serial2.print(F("\xFF\xFF\xFF"));
    //debugln(ambientL);
  }
  vTaskDelete(NULL);                        //Task delete if error
}

void readInTemp(void* parameter)
{
  while (1)
  {
    //check inside temp
    floatTemp = bme.readTemperature();
    intHumid = bme.readHumidity();
    if(intHumid <= 101 && intHumid > 1)
    {
      WriteStr("page0.inT.txt", String(floatTemp,1));
      WriteStr("page0.inH.txt", String (intHumid));
      debug("temp read ");
      debugln(floatTemp);
        
      String humStr = String(intHumid);
      String tempStr = String (floatTemp,1); // or dht.readTemperature(true) for Fahrenheit
      String combinedStr = tempStr + "," + humStr;
      debugln(combinedStr);
      MQTTclient.publish((base_mqtt_topic + "/Temp").c_str(),String(combinedStr).c_str(),true);  
      combinedStr = " ";
      vTaskDelay(600000);
    }
    vTaskDelay(60000);
  }
}

void debugPrint(void *pvParameters) //loop para procesos del dosificador en Core0
{
  while (1)
  {
    vTaskDelay(oneSecond);
    {
      sprintf(debug1,"page1.t0.txt=\"HR:%u_Night:%u_TimeOut:%u\"\xFF\xFF\xFF",timeinfo.tm_hour,nightTime,lightTimeout);
      Serial2.print(debug1);
      sprintf(debug2,"page1.t2.txt=\"Snooze:%u_Ambient:%u_Backlight:%u\"\xFF\xFF\xFF",snooze,ambientL,backLight);
      Serial2.print(debug2);
      sprintf(debug3,"page1.debug.txt=\"light:%u\"\xFF\xFF\xFF",light);
      Serial2.print(debug3);      
      //WriteNum("page1.debug.txt", mLightState);
    }
  }
  vTaskDelete(NULL);
}

void mfanButton()   //sets state based on input from MQTT
{
  if (msg_payload == "toggle")
  {
    debugln(fan);
    fan = !fan;
    debugln(fan);
  }
  else if (msg_payload == "1")
  {
    fan = 1;
  }
  else if (msg_payload == "0")
  {
    fan = 0;
  }
  WriteNum("tmFan.en", fan);     //Uncomment when display update task is written
  MQTTclient.publish((base_mqtt_topic + "/mFanfb").c_str(),String(fan).c_str(),true);
  debugln("fan task starting");
  vTaskResume(fan_Task);
}


void fanTask(void* parameter) {
  while (1) {
    //////////////////////////////mqtt publish doesn't work in fan task//////////
    //debug ("fanCore ");
    //debugln (xPortGetCoreID ());
    if (!fan) 
    {
      digitalWrite(fanOffPin, LOW);
      vTaskDelay(halfSecond);
      digitalWrite(fanOffPin, HIGH);
      debugln("fanOn");
    } 
    else 
    {
      digitalWrite(fanOnPin, LOW);
      vTaskDelay(halfSecond);
      digitalWrite(fanOnPin, HIGH);
      debugln("fanOff");
    }

    vTaskSuspend(NULL);              //Need to call vTaskResume() from whatever task requests fan
  }
  vTaskDelete(NULL);                 //Task delete if error
}

void lightOnInter(void* parameter)  //initial interrupt 
{
  while (1) 
  {
    light = 1;                      //changes state
    lightTimeout = ten_Min_To_Sec;
    vTaskSuspend(NULL);                      //only runs task once
  }
  vTaskDelete(NULL);                        //Task delete if error
}

void mlightButton()   //sets state based on input from MQTT
{
  if (msg_payload == "toggle")
  {
    debugln(light);
    light = !light;
    debugln(light);
  }
  else if (msg_payload == "1")
  {
    light = 1;
  }
  else if (msg_payload == "0")
  {
    light = 0;
  }

  msg_payload = "m";

  if(light)
  {
    lightToTurnOn = true;
    lightTimeout = ten_Min_To_Sec;            // resets light counter 
  }
  else if(!lightTimeout)
  {
    //do nothing keeps from setting lightTimeout to 1
    //debugln("lightTimeOut is zero");
  }
  else
  {
    lightToTurnOn = false;
    lightTimeout = 1;
  } 

}

void lightTask(void* parameter) 
{
  while (1) 
  {   
    if(!snooze)                              //if snooze is 0       
    {
      if(lightToTurnOn) //Turn on light
      {     
        lightToTurnOn = false;
        digitalWrite(lightTogglePin, LOW);
        vTaskDelay(halfSecond);
        digitalWrite(lightTogglePin, HIGH); 
        WriteNum("mstrLt.picc", light);     //Uncomment when display update task is written
        debugln("is this turning on ?");
        MQTTclient.publish((base_mqtt_topic + "/mLightfb").c_str(),String(light).c_str(),true);
      }
      else if(lightTimeout == 1)  //use lightToTurn off to turn off light and leave decrementing
      {  
        lightToTurnOff = false;
        lightTimeout = 0;                                   //Turn off light after timeout
        digitalWrite(lightTogglePin, LOW);
        vTaskDelay(halfSecond);
        digitalWrite(lightTogglePin, HIGH);
        debugln("turnOff only??");
        if(light)                                
        {
          light = 0;
          mlightButton();
        }
        MQTTclient.publish((base_mqtt_topic + "/mLightfb").c_str(),String(light).c_str(),true);
        WriteNum("mstrLt.picc", light);     //Uncomment when display update task is written
        debugln("lightOff");
      }
    }
    else if(snooze && light)
    {
      light = 0;
      digitalWrite(lightTogglePin, LOW);
      vTaskDelay(halfSecond);
      digitalWrite(lightTogglePin, HIGH);
      debugln("turnOff for snnoozze");
      MQTTclient.publish((base_mqtt_topic + "/mLightfb").c_str(),String(light).c_str(),true);
      WriteNum("mstrLt.picc", light);     //Uncomment when display update task is written
      debugln("lightOff");
    }

    if(lightTimeout)                       //if timeout != 0
    {
      lightTimeout--;                      //decrement counter used in conjuncture with snooze flag
      //debug("lTimeout ");
      //debugln(lightTimeout);
      //debug("snooze ");
      //debugln(snooze);
      if(lightTimeout == 1)
      {
        lightToTurnOff = true;
      }
      WriteStr("page0.minT.txt",String(lightTimeout));
    }
    else if(!lightTimeout && snooze)        //if timeout is =0 & snooze is not = to 0 and night is 0              && !nightTime
    {
      
      snooze = 0;                           //turn on motion sensor.
      attachInterrupt(digitalPinToInterrupt(motionSensor), interruptHandler1, FALLING);
      debugln("interupt reattached");
    }
    vTaskDelay(oneSecond);
  }
  vTaskDelete(NULL);                        //Task delete if error
}

void WriteStr(String command, String txt) 
{
  String _component = command;
  String _strVal = txt;

    Serial2.print(_component);
    Serial2.print(F("=\""));
    Serial2.print(_strVal);
    Serial2.print(F("\""));
    Serial2.print(F("\xFF\xFF\xFF"));
    _component = "";
    _strVal = "";
}

void WriteNum(String command, int val) 
{
  String _component = command;
  int _numVal = val;

  Serial2.print(_component);
  Serial2.print(F("="));
  Serial2.print(_numVal);
  Serial2.print(F("\xFF\xFF\xFF"));
  _component = "";
  command = "";
}

void data_in_Nex(void* parameter)
{
  while(1)
  {
    vTaskDelay(10);
    while(Serial2.available())
    {
      data_from_Nex += char(Serial2.read());
      //debugln(data_from_Nex);
    }

    if(data_from_Nex == "mstLit")
    {
      debugln("MstLit");
      light = !light;
      mlightButton();
    }
    
    else if(data_from_Nex == "mstFan")
    {
      fan = !fan;
      mfanButton();
    }
    else if(data_from_Nex == "dvLamp")
    {
      MQTTclient.publish((base_mqtt_topic + "/dLamp").c_str(),"toggle",true); 
    }
    else if(data_from_Nex == "maLamp")
    {
      MQTTclient.publish((base_mqtt_topic + "/mLamp").c_str(),"toggle",true); 
    }
    else if(data_from_Nex == "zzz")
    {
      MQTTclient.publish((base_mqtt_topic + "/mLamp").c_str(),"snz",true);    //turns off auto lamps
      debugln("Motion zzz oon");
      snooze = 1;
      lightTimeout = lightTimeout + 10600;
      debug("snoozeLength ");
      debugln(lightTimeout);
      detachInterrupt(digitalPinToInterrupt(motionSensor));
    }
    else if(data_from_Nex == "xzzz")
    {  
      snooze = 0;  
      WriteNum("mstrLt.picc",light);   //write status to Nextion   
      MQTTclient.publish((base_mqtt_topic + "/mLightfb").c_str(),String(light).c_str(),true);

      MQTTclient.publish((base_mqtt_topic + "/mLightfb").c_str(),"m",true);
      
            
      MQTTclient.publish((base_mqtt_topic + "/mLamp").c_str(),"snzDn",true);    //turns off auto lamps
      debugln("Motion Xzzz");

    }
    else if(data_from_Nex == "lowOD")
    {
      MQTTclient.publish((mqtt_topic_Outlight + "/mainDoor").c_str(),"toggle",true);
    }

    else if(data_from_Nex == "upOD")
    {
      MQTTclient.publish((mqtt_topic_upOutlight + "/upDoor").c_str(),"toggle",true);
    }
    
    data_from_Nex = "";
  }
  vTaskDelete(NULL);
}

void readTime(void* parameter)
{
  while (1)
  {
    vTaskDelay(20000);
    //debugln(eTaskGetState(updateDisplay_Task));            // 1 if not suspended ??3?? if it is suspended
    if(WiFi.status() == WL_CONNECTED)
    {
      if(!getLocalTime(&timeinfo))
      {
        Serial.println("Failed to obtain time");
      }
      else
      {
        timeHour = timeinfo.tm_hour;
        vTaskDelay(5000);
        
        if ((timeHour >= lightsOn) && (timeHour < lightsOut) && nightTime)
        {
          nightTime = 0;
          debugln("day");
          //attachInterrupt(digitalPinToInterrupt(motionSensor), interruptHandler1, FALLING);
        }
        
        else if ((timeHour < lightsOn) || (timeHour >= lightsOut) && !nightTime) 
        {
          lightTimeout = 0;
          snooze = 0;
          nightTime = 1;
          debugln("night");
          //detachInterrupt(digitalPinToInterrupt(motionSensor));
        }
      
        char timeStringBuff[50]; //50 chars should be enough
  
        if(timeinfo.tm_hour >= 12)
        {
          strftime(timeStringBuff, sizeof(timeStringBuff), "%l:%M PM", &timeinfo);
          debugln(timeStringBuff);
        }
        else
        {
            strftime(timeStringBuff, sizeof(timeStringBuff), "%l:%M AM", &timeinfo);
            debugln(timeStringBuff);
        }
        WriteStr("page0.time.txt",timeStringBuff);
      }
    }
    vTaskDelay(35000);
  }

  vTaskDelete(NULL); 
}


