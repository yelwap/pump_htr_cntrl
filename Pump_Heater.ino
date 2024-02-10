
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

#define DHTTYPE DHT11   // DHT 11

//Pin numbers are based on GPIO pins
#define DHTPin 21     // Digital pin connected to the DHT sensor
#define WATER_TANK_LIGHT 20
#define WATER_LINE_LIGHT 8
#define FAN 9

//Status LEDs
#define PWR_LED 10
#define WIFI_LED 2
#define MQTT_CONN_LED 3
#define MQTT_SEND_LED 4

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPin, DHTTYPE);
 
const char* ssid     = "yelwap";
const char* password = "9a8b7c6d5e";   

// MQTTT server connection variables
const char* mqttserver = "192.168.1.32";
const char* mqttUsername = "yelwap";
const char* mqttPassword = "Spotman^93";

//Set MQTTT topic parameter
char pubTopic[] = "arduino/enviro";

//Configure JSON read buffer
char buffer[256];
StaticJsonDocument<256> output;

//Create instance for WiFi client and initialize variables
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
long fan_timer = 0;
char msg[50];
int value = 0;
int switchState = 0;
int goodDHTval = 0;
float f;
boolean light_toggle = 0;
boolean r1_status = 0;
boolean r2_status = 0;
boolean fan_status = 0;
//$$$$$$$$$$$$$$$$$$$$$$$$ END VARIABLE INITALIZATION $$$$$$$$$$$$$$$

//############################## BEGIN MAIN LOOPS ###################
void setup()
{
    pinMode(PWR_LED, OUTPUT);
    digitalWrite(PWR_LED, HIGH);
    pinMode(WIFI_LED, OUTPUT);
    pinMode(MQTT_CONN_LED, OUTPUT);
    pinMode(MQTT_SEND_LED, OUTPUT);
    
    //Serial.begin(115200);
    delay(10);
    pinMode(DHTPin, INPUT);
    pinMode(WATER_TANK_LIGHT, OUTPUT);
    digitalWrite(WATER_TANK_LIGHT, LOW);
    pinMode(WATER_LINE_LIGHT, OUTPUT);
    digitalWrite(WATER_LINE_LIGHT, LOW);
    pinMode(FAN, OUTPUT);
    digitalWrite(FAN, LOW);
    dht.begin(); 
 
    // We start by connecting to a WiFi network
    //Serial.println("--Setting up WiFi--");
    setup_wifi();
   
   //Serial.println("Connecting to MQTT Server");
   client.setServer(mqttserver, 1883);
}  

void loop()
{
  if (!client.connected()) 
  {
    reconnect();
  }
  client.loop();

     // Wait a few seconds between measurements.
    //Serial.print("Beginning DHT read loop");
    delay(5000);

    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    delay(2000);
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();
    delay(2000);
    // Read temperature as Fahrenheit (isFahrenheit = true)
    f = dht.readTemperature(true);
    // Compute heat index in Fahrenheit (the default)
    float hif = dht.computeHeatIndex(f, h);
    // Compute heat index in Celsius (isFahreheit = false)
    float hic = dht.computeHeatIndex(t, h, false);
  
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(f) || isnan(t)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    } else
    {
      goodDHTval=1;
      Serial.print(F("Temperature: "));
      Serial.print(f);
      Serial.print(" degF");      
      Serial.println();
      Serial.print(F("Humidity: "));      
      Serial.print(h); 
      Serial.print(" %");
      Serial.println();  
      Serial.print(F("Heat Index: "));
      Serial.print(hif);
      Serial.println(" degF"); 
      Serial.println(("__________"));    
    } 
    
    long now_fan = millis();
    //Turn Heat Lamp 1 On
    if ((f <= 40) and (r1_status == 0))
    {
    digitalWrite(WATER_TANK_LIGHT, HIGH);
    r1_status = 1;
    Serial.print(F("Light1 status(TURN ON TEST): "));
    Serial.println(r1_status);
    }
  
    //Turn Heat Lamp 2 On
    if ((f <= 36) and (r2_status == 0))
    {
    digitalWrite(WATER_LINE_LIGHT, HIGH);
    r2_status = 1;
    Serial.println(F("Light2 status(TURN ON TEST): "));
    Serial.println(r2_status);
    fan_timer = now_fan;
    Serial.println(fan_timer);
    light_toggle = 1;
    }
  
   //Turn Heat Lamp 2 Off
    if ((f >= 36) and (r2_status == 1))
    {
    digitalWrite(WATER_LINE_LIGHT, LOW);
    r2_status = 0;
    Serial.print(F("Light2 status(TURN OFF TEST): "));
    Serial.println(r2_status);
    fan_timer = now_fan;
    Serial.println(fan_timer);
    light_toggle = 1;
    }
  
   //Turn Heat Lamp 1 Off
    if ((f >40) and (r1_status == 1))
    {
    digitalWrite(WATER_TANK_LIGHT, LOW);
    r1_status = 0;
    Serial.print(F("Light1 status(TURN OFF TEST): "));
    Serial.println(r1_status);
    }
  
    if (light_toggle == 1)
    {
      if (now_fan - fan_timer > 30000)
      {
        if (fan_status == true)
        {
          Serial.print(F("Fan status(TURN OFF TEST): "));
          fan_status = false;
          Serial.println(fan_status);
          digitalWrite(FAN, LOW);
          light_toggle = 0;
          }
        else
          {
          Serial.print(F("Fan status(TURN ON TEST): "));
          fan_status = true;
          Serial.println(fan_status);
          digitalWrite(FAN, HIGH);
          light_toggle = 0;
        }
      }
    }

     //Store DHT data into JSON read buffer
      output["temperature"] = f;
      output["humidity"] = h;
      output["heat_index"] = hif;
      output["relay1_status"] = r1_status;
      output["relay2_status"] = r2_status;
      output["fan_status"] = fan_status;
      size_t n = serializeJson(output, buffer);
    
      long now = millis();
      if (now - lastMsg > 300000) 
      {
        digitalWrite(MQTT_SEND_LED, HIGH);
        lastMsg = now;
        //char payLoad[1];
        //itoa(f, payLoad, 10);
        //char temp_str=std::to_string(f);
        //payLoad=String(f).c_str();
        //client.publish(pubTopic, String(f).c_str());
        client.publish(pubTopic, buffer,n);
        delay(5000);
        digitalWrite(MQTT_SEND_LED, LOW);
      }
     Serial.println(("______________"));
     Serial.println();
        
  }
//############################## END MAIN LOOPS ###################

  //&&&&&&&&&&&&&&&&&&. FUNCTIONS &&&&&&&&&&&&&&&&&&&&
  
void setup_wifi() 
  {Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
 
    WiFi.begin(ssid, password);
 
    while (WiFi.status() != WL_CONNECTED) {
        digitalWrite(WIFI_LED,LOW);
        delay(500);
        Serial.print(".");
    }
    digitalWrite(WIFI_LED,HIGH);
    
    // print your board's IP address:
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    
    // print your MAC address:
    byte mac[6]; 
    WiFi.macAddress(mac);
    Serial.print("MAC: ");
    Serial.print(mac[5],HEX);
    Serial.print(":");
    Serial.print(mac[4],HEX);
    Serial.print(":");
    Serial.print(mac[3],HEX);
    Serial.print(":");
    Serial.print(mac[2],HEX);
    Serial.print(":");
    Serial.print(mac[1],HEX);
    Serial.print(":");
    Serial.println(mac[0],HEX);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("RSSI:");
    Serial.println(rssi);
  }

void reconnect() 
{
  // Loop until we're reconnected
  while (!client.connected()) 
  {
    Serial.println("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqttUsername, mqttPassword)) 
    {
      Serial.println("connected");
      digitalWrite(MQTT_CONN_LED, HIGH);
    } else 
    {
      digitalWrite(MQTT_CONN_LED, LOW);
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
  
