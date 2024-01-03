/*
# MQTT Service: https://dash.wqtt.ru/
# Author: EmbeddevIOT (Aleksey Baranov)
# Date: (create to 31.12.22)
# Discription:
# ########### Hardware ###########
# MCU: ESP8266
# WiFi + MQTT + Yandex
# mqtt_name: u_4YVJEF
# mqtt_pass: v1HPYZgn
# mqtt_server: m5.wqtt.ru
# mqtt_port: 10073
*/

#include <Arduino.h>
#include "ESP8266WiFi.h"
#include "PubSubClient.h"
#include "ArduinoJSON.h"
#include <WiFiClientSecure.h>
#include "GRGB.h"

#define BaudSpeed 9600
static const char firmware[] = {"0.4"};

// WiFi Login and Password
const char *ssid = "AECorp2G";
const char *password = "Ae19co90$";
/**** Secure WiFi Connectivity Initialisation *****/
WiFiClientSecure espClient;

uint16_t counter = 0;
uint8_t state = 0;

GRGB led(COMMON_CATHODE, D4, D3, D2);

enum led_state
{
  led_OFF = 0,
  led_R,
  led_G,
  led_B,
};

struct Leds {
  uint8_t bright = 0;
  uint8_t state = 0;
  uint8_t colR = 0;
  uint8_t colG = 0;
  uint8_t colB = 0;
  String colHEX = "0x00";
};
Leds LED;

struct Timers
{
  uint8_t tim100 = 0;
  uint32_t tim1000 = 0;
};
Timers TIM;

// This OLD information
// // MQTT broker credentials (set to NULL if not required)
// const char *mqtt_username = "ESP8266_Test";
// const char *mqtt_password = "Ae19co90$rp";
// // Change the variable to your Raspberry Pi IP address, so it connects to your MQTT broker
// const char *mqtt_server = "9a52daac3a584ac8ab2a9e666381ee47.s2.eu.hivemq.cloud";

// MQTT broker credentials (set to NULL if not required)
const char *mqtt_username = "u_4YVJEF";
const char *mqtt_password = "v1HPYZgn";
// Change the variable to your Raspberry Pi IP address, so it connects to your MQTT broker
const char *mqtt_server = "m5.wqtt.ru";
const int mqtt_port = 10073;
/**** MQTT Client Initialisation Using WiFi Connection *****/
PubSubClient client(espClient);

//
const String leds_topic = "/leds";

struct topics {
  String cnt = "cnt"; 
  String ledState = "ledst"; 
};
topics TOPIC;


// char mqtt_message[128];


unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

/****** root certificate *********/

static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

void Task100ms();
void Task1000ms();
void setup_wifi();
void reconnect();
void callback(char *topic, byte *payload, unsigned int length);
void publishMessage(const char *topic, String payload, boolean retained);
void updateStatePins();

void setup()
{
  Serial.begin(BaudSpeed);
  Serial.println(F("MQTT_Tracker_Started"));
  Serial.printf("Firmware: %s", firmware);

  led.setBrightness(255);
  led.setCRT(true);
  led.setColor(GAqua);

  // pinMode(D4, OUTPUT); // set up LED RED
  // digitalWrite(D4, LOW);
  // pinMode(D3, OUTPUT); // set up LED GREEN
  // digitalWrite(D3, LOW);
  // pinMode(D2, OUTPUT); // set up LED BLUE
  // digitalWrite(D2, LOW);
  // pinMode(led, OUTPUT); // set up LED
  setup_wifi();

#ifdef ESP8266
  espClient.setInsecure();
#else
  espClient.setCACert(root_ca); // enable this line and the the "certificate" code for secure connection
#endif

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop()
{
  Task1000ms();

  static uint32_t tmr;
  if (millis() - tmr >= 5)
  {
    tmr = millis();
    static int8_t dir = 1;
    static int val = 0;
    val += dir;
    if (val == 255 || val == 0)
      dir = -dir; // разворачиваем
    led.setBrightness(val);
  }

  if (!client.connected())
    reconnect(); // check if client is connected
  client.loop();

  // read DHT11 temperature and humidity reading
  // delay(dht.getMinimumSamplingPeriod());
  // float humidity = dht.getHumidity();
  // float temperature = dht.getTemperature();

  // DynamicJsonDocument doc(1024);

  // doc["cnt_top"] = counter;
  // doc["ledst_top"] = state;
  // doc["humidity"] = counter;
  // doc["temperature"] = temperature;

  // serializeJson(doc, mqtt_message);

  // publishMessage("esp8266_data", mqtt_message, true);

  // delay(1000);
}

void Task100ms()
{
  if (millis() - TIM.tim100 >= 100)
  {
    TIM.tim100 += 100;
  }
}

void Task1000ms()
{
  if (millis() - TIM.tim1000 >= 1000)
  {
    TIM.tim1000 += 1000;

    counter < 1000 ? counter++ : counter = 0;

    // publishMessage("EmbedevIO", mqtt_message, true);

    publishMessage(TOPIC.cnt.c_str(), String(counter).c_str(), true);
    publishMessage(TOPIC.ledState.c_str(), String(state).c_str(), true);
    // publishMessage("led_state", String(state).c_str(), true);
  }
}

/************* Connect to WiFi ***********/
void setup_wifi()
{
  delay(10);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("\nWiFi connected\nIP address: ");
  Serial.println(WiFi.localIP());
}

/************* Connect to MQTT Broker ***********/
void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-"; // Create a random client ID
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password))
    {
      Serial.println("connected");

      client.subscribe((leds_topic + "/#").c_str());

      // client.subscribe("led_state"); // subscribe the topics here
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds"); // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/***** Call back Method for Receiving MQTT messages and Switching LED ****/
void callback(char *topic, byte *payload, unsigned int length)
{
  String data_pay;
  for (int i = 0; i < length; i++)
  {
    data_pay += String((char)payload[i]);
  }

  Serial.println(data_pay);

  if (String(topic) == leds_topic)
  {
    if (data_pay == "ON" || data_pay == "1")
      state = led_R;
    if (data_pay == "OFF" || data_pay == "0")
      state = led_OFF;
  }

  if (String(topic) == (leds_topic + "/brig"))
  {
    LED.bright = data_pay.toInt();
    Serial.println(LED.bright);
  }

  if (String(topic) == (leds_topic + "/color"))
  {
    String(LED.colHEX) = data_pay;
  }
  updateStatePins();
}

/**** Method for Publishing MQTT Messages **********/
void publishMessage(const char *topic, String payload, boolean retained)
{
  if (client.publish(topic, payload.c_str(), true))
    Serial.println("Message publised [" + String(topic) + "]: " + payload);
}

void updateStatePins(void)
{
  switch (state)
  {
  case led_OFF:
    digitalWrite(D4, LOW);
    digitalWrite(D3, LOW);
    digitalWrite(D2, LOW);
    break;
  case led_R:
    digitalWrite(D4, HIGH);
    digitalWrite(D3, LOW);
    digitalWrite(D2, LOW);
    break;
  case led_G:
    digitalWrite(D4, LOW);
    digitalWrite(D3, HIGH);
    digitalWrite(D2, LOW);
    break;
  case led_B:
    digitalWrite(D4, LOW);
    digitalWrite(D3, LOW);
    digitalWrite(D2, HIGH);
    break;

  default:
    break;
  }
}