#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <RCSwitch.h>
#include "Adafruit_BME680.h"
#include "Adafruit_CCS811.h"
// Update these with values suitable for your network.
#include "../include/settings.h"

#ifdef TLS
WiFiClientSecure espClient;
static unsigned int const mqttPort = 8883;
#else
WiFiClient espClient;
static unsigned int const mqttPort = 1883;
#endif

#define SEALEVELPRESSURE_HPA (1013.25)

PubSubClient client(espClient);
RCSwitch mySwitch = RCSwitch();
Adafruit_CCS811 ccs = Adafruit_CCS811();
Adafruit_BME680 bme = Adafruit_BME680();
const long interval = 60000; // 1 minute
unsigned long previousMillis = 0;
unsigned long currentMillis = millis();
char stgFromFloat[10];

// can be changed in platformio.ini
#ifdef TC_SWITCH_PIN
static uint8_t const tcSwitchPin = TC_SWITCH_PIN;
#else
static uint8_t const tcSwitchPin = 14;
#endif

static uint8_t const sdaPin = 2; // D4
static uint8_t const sclPin = 0; // D3

void callback(char *topic, byte *payload, unsigned int length)
{
#ifdef DEBUG
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
#endif
  char message[length];
  for (unsigned int i = 0; i < length; i++)
  {
#ifdef DEBUG
    Serial.print((char)payload[i]);
#endif
    message[i] = (char)payload[i];
  }
#ifdef DEBUG
  Serial.println();
#endif

  if (strcmp(topic, lights433topic.c_str()) == 0)
  {
    mySwitch.send(message);
  }
}

void reconnectMqtt()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
#ifdef DEBUG
    Serial.print("Attempting MQTT connection...");
#endif
    // Attempt to connect
    // client.connect(clientId.c_str(), mqttUser.c_str(), mqttPassword.c_str()); // User Auth
    if (client.connect(clientId.c_str()))
    {
      // Once connected, publish an announcement...
      // client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe(lights433topic.c_str());
    }
    else
    {
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void reconnectWiFi()
{
// We start by connecting to a WiFi network
#ifdef DEBUG
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
#endif
  WiFi.hostname(clientId);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
#ifdef DEBUG
    Serial.print(".");
#endif
  }

#ifdef DEBUG
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IPv4 address: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());
#endif
}

void setup()
{
  pinMode(internalLED, OUTPUT); // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);

  delay(5000);
  // Start sensor CSS811
  if (!ccs.begin())
  {
#ifdef DEBUG
    Serial.println("Couldn't find CSS811 sensor!");
#endif
    // TODO: Maybe a mqtt publish with error that sensor could not be started
  }

  // Start sensor BME680
  if (!bme.begin(0x76U))
  {
#ifdef DEBUG
    Serial.println("Couldn't find BME680 sensor!");
#endif
    // TODO: Maybe a mqtt publish with error that sensor could not be started
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  delay(500);
  // We start by connecting to a WiFi network
#ifdef DEBUG
  Serial.printf("Wi-Fi mode set to WIFI_STA %s\n", WiFi.mode(WIFI_STA) ? "" : "Failed!");
#else
  WiFi.mode(WIFI_STA);
#endif
  reconnectWiFi();

  client.setServer(server, mqttPort);
  client.setCallback(callback);

  mySwitch.enableTransmit(tcSwitchPin);
  mySwitch.setPulseLength(286);

  // Turn the internal LED off by making the voltage HIGH
  digitalWrite(internalLED, HIGH);

#ifdef DEBUG
  Serial.println("Setup done!");
#endif
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    reconnectWiFi();
  }
  else if (!client.connected())
  {
    reconnectMqtt();
  }

  currentMillis = millis();

  if ((currentMillis - previousMillis >= (interval * 5)) || (previousMillis == 0)) // interval is 1 minute. we want to publish every 5 minutes
  {
    previousMillis = currentMillis;
#ifdef DEBUG
    if (!bme.performReading())
    {
      Serial.println("Failed to perform reading :(");
      return;
    }
    Serial.print("Temperature = ");
    Serial.print(bme.temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bme.pressure / 100.0);
    Serial.println(" hPa");

    Serial.print("Humidity = ");
    Serial.print(bme.humidity);
    Serial.println(" %");

    Serial.print("Gas = ");
    Serial.print(bme.gas_resistance / 1000.0);
    Serial.println(" KOhms");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.println();

    if (ccs.available())
    {
      if (!ccs.readData())
      {
        Serial.print("CO2: ");
        Serial.print(ccs.geteCO2());
        Serial.print("ppm, TVOC: ");
        Serial.println(ccs.getTVOC());
      }
    }
#endif
    if (!bme.performReading())
    {
      Serial.println("Failed to perform reading :(");
      return;
    }
    dtostrf(bme.temperature, 4, 2, stgFromFloat);
    client.publish((topicBase + "temperature").c_str(), stgFromFloat, true); // Send as retained message so last vlaue can be accesed by subscriber
    dtostrf(bme.humidity, 4, 2, stgFromFloat);
    client.publish((topicBase + "humidity").c_str(), stgFromFloat, true); // Send as retained message so last vlaue can be accesed by subscriber
    dtostrf(bme.pressure / 100.0, 6, 2, stgFromFloat);
    client.publish((topicBase + "pressure").c_str(), stgFromFloat, true); // Send as retained message so last vlaue can be accesed by subscriber
    dtostrf(bme.gas_resistance / 1000.0, 4, 2, stgFromFloat);
    client.publish((topicBase + "gas").c_str(), stgFromFloat, true); // Send as retained message so last vlaue can be accesed by subscriber

    if (ccs.available())
    {
      if (!ccs.readData())
      {
        dtostrf(ccs.geteCO2(), 4, 0, stgFromFloat);
        client.publish((topicBase + "co2").c_str(), stgFromFloat, true); // Send as retained message so last vlaue can be accesed by subscriber
        dtostrf(ccs.getTVOC(), 4, 0, stgFromFloat);
        client.publish((topicBase + "tvoc").c_str(), stgFromFloat, true); // Send as retained message so last vlaue can be accesed by subscriber
      }
    }
  }

  client.loop();
}
