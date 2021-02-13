#include <WiFi.h>
#include <DHT.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>                   // For OTA Programming
#include <ArduinoJson.h>                  // JSON
#include <PubSubClient.h>                 // MQTT
#include <EmonLib.h>
#include "secrets.h"

////////////////////////////////
// Time variables
////////////////////////////////
#define REPORTING_PERIOD                  30000
#define WATCHDOG_TIMEOUT_PERIOD           86400000

// WiFi
const char* ssid =                        CONFIG_WIFI_SSID;
const char* password =                    CONFIG_WIFI_PASSWORD;

//LED
#define LED_BUILTIN                       2

////////////////////////////////
// Variables
////////////////////////////////
char hostname[32];
const int numENERGIEMonitors =            2;

////////////////////////////////
// Complex Variables
////////////////////////////////
WiFiClient client;                        // TCP Client
PubSubClient MQTTClient(client);          // MQTT

////////////////////////////////
// Util functions
////////////////////////////////
constexpr unsigned int str2int(const char* str, int h = 0) {
    return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
}

////////////////////////////////
// Enums
////////////////////////////////
typedef enum DHT_TYPES : int {
  DHT_11 = 11,  /**< DHT TYPE 11 */
  DHT_12 = 12,  /**< DHY TYPE 12 */
  DHT_22 = 22,  /**< DHT TYPE 22 */
  DHT_21 = 21,  /**< DHT TYPE 21 */
  AM_2301 = 21  /**< AM2301 */
};

typedef enum ThingFeatures : int {
  None = 0,
  EnergyConsumptionFeature = 1 << 2,
  FlowFeature = 1 << 3,
  TemperatureFeature = 1 << 4,
  HumidityFeature = 1 << 5,
  WaterTemperatureFeature = 1 << 6
};

////////////////////////////////
// Structs
////////////////////////////////
struct ENERGIEMonitor {
  private:    
    int ENERGY_TRANSFORMER_PIN, WATER_FLOW_PIN, WATER_TEMPERATURE_PIN;

    double THERMISTOR_R25 = 50000.0;
    double THERMISTOR_B = 3950.0;

    uint64_t WaterFlowPulses, WaterFlowMillis, WaterTemperatureRaw;

    int DHT_PIN = 5;
    DHT_TYPES DHT_TYPE = DHT_22;

    EnergyMonitor emon;

    DHT dht = DHT(DHT_PIN, DHT_TYPE);

    ThingFeatures Features = None;
    
    unsigned long n;

    unsigned long LAST_COMMUNICATION = 0;

    char telemetryTopic[96], commandTopic[64], commandTopicRestart[64], commandTopicStart[64], commandTopicStop[64], commandTopicRead[64], debugTopic[64], buffer[512];
  public:
    String ThingId;
  
    double Temperature, Humidity, WaterTemperature, WaterFlow, EnergyConsumption;
    
    bool Active = true, ForceRead = false;
    
    ENERGIEMonitor(String _ThingId, ThingFeatures _Features, int _ENERGY_TRANSFORMER_PIN, int _WATER_FLOW_PIN, int _WATER_TEMPERATURE_PIN, int _DHT_PIN, DHT_TYPES _DHT_TYPE = DHT_22) {
      ThingId = _ThingId;
      Features = _Features;
      ENERGY_TRANSFORMER_PIN = _ENERGY_TRANSFORMER_PIN;
      WATER_FLOW_PIN = _WATER_FLOW_PIN;
      WATER_TEMPERATURE_PIN = _WATER_TEMPERATURE_PIN;
      DHT_PIN = _DHT_PIN;
      DHT_TYPE = _DHT_TYPE;
    }

    ENERGIEMonitor(String _ThingId, ThingFeatures _Features, int _ENERGY_TRANSFORMER_PIN) {
      ThingId = _ThingId;
      Features = _Features;
      ENERGY_TRANSFORMER_PIN = _ENERGY_TRANSFORMER_PIN;
    }

    void begin(void (*f)()) {
      if (Features & FlowFeature) {
        pinMode(WATER_FLOW_PIN, INPUT_PULLUP);
        attachInterrupt(WATER_FLOW_PIN, f, RISING); 
        WaterFlowMillis = millis();
      }
    
      if (Features & EnergyConsumptionFeature) {
        pinMode(ENERGY_TRANSFORMER_PIN, INPUT_PULLDOWN); 
        emon.current(ENERGY_TRANSFORMER_PIN, 30);
      }

      if (Features & (TemperatureFeature | HumidityFeature)) {
        dht.begin();
      }

      String telemetryTopicString = "telemetry/" + ThingId + "/energymonitor";
      telemetryTopicString.toCharArray(telemetryTopic, 96);

      String commandTopicString = "command/" + ThingId + "/+";
      commandTopicString.toCharArray(commandTopic, 64);
      commandTopicString = "command/" + ThingId + "/restart";
      commandTopicString.toCharArray(commandTopicRestart, 64);
      commandTopicString = "command/" + ThingId + "/start";
      commandTopicString.toCharArray(commandTopicStart, 64);
      commandTopicString = "command/" + ThingId + "/stop";
      commandTopicString.toCharArray(commandTopicStop, 64);
      commandTopicString = "command/" + ThingId + "/read";
      commandTopicString.toCharArray(commandTopicRead, 64);
      
      String debugTopicString = "debug/" + ThingId;
      debugTopicString.toCharArray(debugTopic, 64);
    }

    void loop() {
      if (Features & EnergyConsumptionFeature) {
        EnergyConsumption += emon.calcIrms(1480); 
      }

      if (Features & WaterTemperatureFeature) {
        WaterTemperatureRaw += analogRead(WATER_TEMPERATURE_PIN);
      }
      n++;
      
      if (millis() - LAST_COMMUNICATION > REPORTING_PERIOD | ForceRead) {
        digitalWrite(LED_BUILTIN, HIGH);

        size_t capacity = JSON_OBJECT_SIZE(15);
        DynamicJsonDocument doc(capacity);

        if (Features & (TemperatureFeature | HumidityFeature)) {
          Temperature = dht.readTemperature();
          Humidity = dht.readHumidity();
        }

        if (Features & WaterTemperatureFeature) {
          double Vo = (double)WaterTemperatureRaw / (double)n;
          double R2 = 10000.0 / (4095.0 / Vo - 1.0);

          doc.clear();
          doc["type"] = "Resistance";
          doc["resistance"] = R2;
            
          serializeJson(doc, buffer);
          MQTTClient.publish(debugTopic, buffer);
  
          WaterTemperature = R2 / THERMISTOR_R25;                                                 // (R/Ro)
          WaterTemperature = log(WaterTemperature);                                               // ln(R/Ro)
          WaterTemperature /= THERMISTOR_B;                                                       // 1/B * ln(R/Ro)
          WaterTemperature += 1.0 / (25.0 + 273.15);                                              // + (1/To)
          WaterTemperature = 1.0 / WaterTemperature;                                              // Invert
          WaterTemperature -= 273.15;                                                             // To Celcius
        }

        if (Features & FlowFeature) {
          WaterFlow = (double)WaterFlowPulses;
          WaterFlow /= (double)(millis() - WaterFlowMillis);
          WaterFlowPulses = 0; WaterFlowMillis = millis();
          WaterFlow *= 1000.00;
          WaterFlow /= 6.60;
          //WaterFlow /= 60.00;
          //WaterFlow *= (double)(millis() - WaterFlowMillis);
          //WaterFlow /= 1000.00;
        }

        if (Features & EnergyConsumptionFeature) {
          EnergyConsumption /= n;
        }
        
        char *type[] = {"Temperature", "Humidity", "WaterTemperature", "Flow", "EnergyConsumption"};
        double value[] = {Temperature, Humidity, WaterTemperature, WaterFlow, EnergyConsumption};
        ThingFeatures features[] = {TemperatureFeature, HumidityFeature, WaterTemperatureFeature, FlowFeature, EnergyConsumptionFeature};

        for (int i = 0; i < 5; i++) {
          if (Features & features[i]) {
            doc.clear();
          
            doc["type"] = type[i];
            doc["value"] = value[i];
            
            serializeJson(doc, buffer);
  
            MQTTClient.publish(telemetryTopic, buffer);
          }
        }

        EnergyConsumption = 0;
        WaterTemperatureRaw = 0;
        n = 0;
        ForceRead = false;

        LAST_COMMUNICATION = millis();
        digitalWrite(LED_BUILTIN, LOW);
      }
    }

    void MQTT_Subscribe() {
      MQTTClient.subscribe(commandTopic);
      MQTTClient.publish(debugTopic, "EnergyMonitor Connected!");
    }

    bool MQTT_Command(char* topic, byte* payload, unsigned int length) {      
      if (str2int(topic) == str2int(commandTopicRestart)) {
        MQTTClient.publish(debugTopic, "Restarting EnergyMonitor!");
        ESP.restart();
      }
  
      if (str2int(topic) == str2int(commandTopicStart)) {
        MQTTClient.publish(debugTopic, "Starting EnergyMonitor!");
        Active = true;
        return true;
      }
  
      if (str2int(topic) == str2int(commandTopicStop)) {
        MQTTClient.publish(debugTopic, "Stoping EnergyMonitor!");
        Active = false;
        return true;
      }
  
      if (str2int(topic) == str2int(commandTopicRead)) {
        MQTTClient.publish(debugTopic, "Forcing EnergyMonitor Read!");
        ForceRead = true;
        return true;
      }

      return false;
    }

    void IRAM_ATTR WaterFlowInterrupt() {
      WaterFlowPulses++;
    }
};
ENERGIEMonitor ENERGIEMonitors[numENERGIEMonitors] = {ENERGIEMonitor(CONFIG_THING_ID_RC, (ThingFeatures)124, 34, 27, 32, 5),
                                                        ENERGIEMonitor(CONFIG_THING_ID_1A, (ThingFeatures)4, 35)};

////////////////////////////////
// Interrupts
////////////////////////////////
void IRAM_ATTR Interrupt_RC() {
  ENERGIEMonitors[0].WaterFlowInterrupt();
}
void IRAM_ATTR Interrupt_1A() {
  ENERGIEMonitors[1].WaterFlowInterrupt();
}

////////////////////////////////
// Setup
////////////////////////////////
void setup() {
  randomSeed(micros());

  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  setupWiFi();
  setupOTA();
  setupMQTT();

  ENERGIEMonitors[0].begin(Interrupt_RC);
  ENERGIEMonitors[1].begin(Interrupt_1A);

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  connectWiFi();
  loopMQTT();
  loopOTA();
  
  for (int ENERGIEmonitor = 0; ENERGIEmonitor < numENERGIEMonitors; ENERGIEmonitor++) {
    ENERGIEMonitors[ENERGIEmonitor].loop();
  }
}

////////////////////////////////
// WiFi
////////////////////////////////
void setupWiFi() {
  uint32_t id = 0;
  for(int i=0; i<17; i=i+8) {
    id |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  
  String hostnameString = "ENERGIEMonitor-" + String(id);
  hostnameString.toCharArray(hostname, 32);

  WiFi.mode(WIFI_STA);

  // Set your Static IP address
  IPAddress local_IP(192, 168, 1, 3);
  // Set your Gateway IP address
  IPAddress gateway(192, 168, 1, 254);
  
  IPAddress subnet(255, 255, 255, 0);
  IPAddress primaryDNS(192, 168, 1, 254);
  IPAddress secondaryDNS(8, 8, 8, 8);

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    ESP.restart();
  }

  WiFi.setHostname(hostname);

  connectWiFi();
}

bool connectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);

    const int kRetryCountWiFi = 20;
    int retryCountWiFi = 0;
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      if (retryCountWiFi++ > kRetryCountWiFi) {
        ESP.restart();
      }
    }
  }
  
  return WiFi.status() == WL_CONNECTED;
}

////////////////////////////////
// MQTT
////////////////////////////////
void setupMQTT() {
  MQTTClient.setServer(CONFIG_MQTT_SERVER, CONFIG_MQTT_SERVERPORT);
  MQTTClient.setCallback(MQTTOnMessage);

  MQTTClient.setBufferSize(2048);
  
  loopMQTT();
}

void loopMQTT() {
  if (!MQTTClient.connected()) {
    reconnectMQTT();
  }
  MQTTClient.loop();
}

void reconnectMQTT() {
  const int kRetryCountMQTT = 40;
  int retryCountMQTT = 0;
  // Loop until we're reconnected
  while (!MQTTClient.connected()) {    
    // Attempt to connect
    if (MQTTClient.connect(hostname, CONFIG_MQTT_USERNAME, CONFIG_MQTT_PASSWORD)) {
      for (int ENERGIEmonitor = 0; ENERGIEmonitor < numENERGIEMonitors; ENERGIEmonitor++) {
        ENERGIEMonitors[ENERGIEmonitor].MQTT_Subscribe();
      }
    } else {
      // Wait 5 seconds before retrying
      unsigned long now = millis();
      while (millis() - now < 5000) {
        loopOTA();
        yield();
      }
    }
    if (retryCountMQTT++ > kRetryCountMQTT) {
      ESP.restart();
    }
  }
}

void MQTTOnMessage(char* topic, byte* payload, unsigned int length) {
  for (int ENERGIEmonitor = 0; ENERGIEmonitor < numENERGIEMonitors; ENERGIEmonitor++) {
    if (ENERGIEMonitors[ENERGIEmonitor].MQTT_Command(topic, payload, length)) {
      return;
    }
  }

  String topicString(topic);
  topicString += "/reply";
  char topicChar[64];
  topicString.toCharArray(topicChar, 64);
  MQTTClient.publish(topicChar, "Invalid command received!");
}

////////////////////////////////
// OTA
////////////////////////////////
void setupOTA() {
  // Set OTA Hostname
  ArduinoOTA.setHostname(hostname);

  // Set OTA Password
  ArduinoOTA.setPassword((const char *)CONFIG_OTA_PASSWORD);

  // Init OTA
  ArduinoOTA.begin();

  // Loop OTA
  loopOTA();
}

void loopOTA() {
  ArduinoOTA.handle();
}
