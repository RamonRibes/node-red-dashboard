#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ArduinoJson.h>

#define SSID "xxx"
#define PASSWORD "xxx"
#define MQTT_SERVER "192.168.0.20"
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "ESP32Client-DHT22"
#define MQTT_USER "xxx"
#define MQTT_PASSWORD "xxx"
char mqtt_out_topic_values[] = "ESP32/Values";
char mqtt_out_topic_status[] = "ESP32/Status";
char mqtt_out_topic_messages[] = "ESP32/Messages";
char mqtt_in_topic[] = "ESP32/Orders";
#define MSG_BUFFER_SIZE 200
#define PIN_LED 12
#define PIN_POLSADOR 14
#define PIN_DHT 25     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22  // DHT 22 (AM2302)
#define DELAY_MS_POLSADOR 50
#define RETARD_INTERRUPCIO 250000

bool estatLed = true;
bool interruptorPolsat = false;
bool readTempValues = true;

DHT_Unified dht(PIN_DHT, DHTTYPE);

unsigned long delayMs_DHT22 = 10000;        // ms beween DHT reads & publish to MQTT server.
unsigned long previousMillis_DHT = 0;       // will store last time DHT was read
unsigned long previousMillis_Polsador = 0;  // will store last time button was checked
unsigned long min_delayMs_DHT22;            // Minimum interval to read DHT22 next values.

WiFiClient WifiClient;
PubSubClient MQTT_Client(WifiClient);
char msg[MSG_BUFFER_SIZE];

void setup() {
  Serial.begin(112500);
  while (!Serial)
    continue;
  // Initialize sensor DHT22.
  dht.begin();
  print_sensor_details();
  // Initialize led, button and control interrupt.
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_POLSADOR, INPUT);
  digitalWrite(PIN_LED, estatLed);
  attachInterrupt(digitalPinToInterrupt(PIN_POLSADOR), isr, RISING);  //Create interruption on pin.
  // Setup MQTT connection;
  MQTT_Client.setServer(MQTT_SERVER, MQTT_PORT);
  MQTT_Client.setCallback(message_received);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connect_Wifi();
  }
  if (!MQTT_Client.connected()) {
    connect_MQTT();
  }
  MQTT_Client.loop();
  // Delay between DHT22 measurements.
  if (readTempValues && delay_exceeded(&previousMillis_DHT, delayMs_DHT22)) {
    read_and_print_sensor_values();
  }

  // Delay between button checks.
  if (delay_exceeded(&previousMillis_Polsador, DELAY_MS_POLSADOR)) {
    check_button_pressed();
  }
}

void connect_Wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("WiFi connected. IP address: ");
  Serial.println(WiFi.localIP());
}

void connect_MQTT() {
  JsonDocument messages_JsonDoc;
  JsonArray data = messages_JsonDoc["message"].to<JsonArray>();

  // Loop until we're reconnected
  while (!MQTT_Client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (MQTT_Client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      snprintf(msg, MSG_BUFFER_SIZE, "Connected to Wifi SSID: '%s'.", SSID);
      data.add(msg);
      snprintf(msg, MSG_BUFFER_SIZE, "Connected to MQTT server on '%s'", MQTT_SERVER);
      data.add(msg);
      // ... and resubscribe
      MQTT_Client.subscribe(mqtt_in_topic);
      snprintf(msg, MSG_BUFFER_SIZE, "Subscribed to MQTT topic '%s'", mqtt_in_topic);
      data.add(msg);
      send_data_to_MQTT(mqtt_out_topic_messages, &messages_JsonDoc);
      publish_status_to_MQTT();
    } else {
      Serial.print("failed, rc=");
      Serial.print(MQTT_Client.state());
      Serial.println(". Try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void message_received(char* topic, byte* payload, unsigned int length) {
  String message = "";

  Serial.print("Message arrived from MQTT[");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);
  process_message(message);
}

void process_message(String message) {
  JsonDocument messages_JsonDoc;
  char buffer[message.length() + 1];
  String message_lowercase = message;

  message_lowercase.toLowerCase();
  if (message_lowercase == "set led on") {
    estatLed = true;
    digitalWrite(PIN_LED, estatLed);
  } else if (message_lowercase == "set led off") {
    estatLed = false;
    digitalWrite(PIN_LED, estatLed);
  } else if (message_lowercase == "start reading data") {
    readTempValues = true;
  } else if (message_lowercase == "stop reading data") {
    readTempValues = false;
  } else if (message_lowercase.startsWith("set interval ") && isNaturalNumber(message.substring(13))) {
    set_DHT22_read_delay(message.substring(13).toInt());
  } else {
    message.toCharArray(buffer, message.length() + 1);
    snprintf(msg, MSG_BUFFER_SIZE, "Missatge no entés! ('%s')", buffer);
    messages_JsonDoc["message"] = msg;
    send_data_to_MQTT(mqtt_out_topic_messages, &messages_JsonDoc);
    return;
  }
  publish_status_to_MQTT();
}

boolean isNaturalNumber(String str) {
  unsigned int stringLength = str.length();

  if (stringLength == 0) {
    return false;
  }
  for (unsigned int i = 0; i < stringLength; ++i) {
    if (!isDigit(str.charAt(i))) {
      return false;
    }
  }
  return true;
}

void publish_status_to_MQTT() {
  JsonDocument status_JsonDoc;
  //const size_t capacity = JSON_OBJECT_SIZE(3);
  //StaticJsonDocument<capacity> status_JsonDoc;

  status_JsonDoc["led"] = estatLed;
  status_JsonDoc["sampling"] = readTempValues;
  status_JsonDoc["interval"] = delayMs_DHT22;
  snprintf(msg, MSG_BUFFER_SIZE, "LED status is %s. Sampling status is %s. Sampling interval is %d ms.", (estatLed ? "ON" : "OFF"), (readTempValues ? "ON" : "OFF"), delayMs_DHT22);
  Serial.println(msg);
  send_data_to_MQTT(mqtt_out_topic_status, &status_JsonDoc);
}

void check_button_pressed() {
  if (interruptorPolsat) {
    estatLed = !estatLed;
    interruptorPolsat = false;
    digitalWrite(PIN_LED, estatLed);
    publish_status_to_MQTT();
  }
}

bool delay_exceeded(unsigned long* previousMillis, unsigned long delayMS) {
  unsigned long currentMillis = millis();

  if (currentMillis - *previousMillis >= delayMS) {
    // save the last time you exceeded the delay
    *previousMillis = currentMillis;
    return true;
  } else {
    return false;
  }
}

void print_sensor_details() {
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("°C"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("°C"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("%"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("%"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set min delay between sensor readings based on sensor details.
  min_delayMs_DHT22 = sensor.min_delay / 1000;
  set_DHT22_read_delay(delayMs_DHT22);
}

void set_DHT22_read_delay(unsigned long delay_proposed) {
  if (delay_proposed < min_delayMs_DHT22) {
    delayMs_DHT22 = min_delayMs_DHT22;
  } else {
    delayMs_DHT22 = delay_proposed;
  }
}

void read_and_print_sensor_values() {
  JsonDocument values_JsonDoc;
  JsonDocument messages_JsonDoc;

  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    char errorTemp[] = "Error reading temperature!";
    Serial.println(errorTemp);
    messages_JsonDoc["message"] = errorTemp;
    send_data_to_MQTT(mqtt_out_topic_messages, &messages_JsonDoc);
  } else {
    snprintf(msg, MSG_BUFFER_SIZE, "Temperature: %.1f° C", event.temperature);
    values_JsonDoc["temp"]["parameter"] = "Temperature";
    values_JsonDoc["temp"]["value"] = event.temperature;
    values_JsonDoc["temp"]["units"] = "°C";
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    char errorHumi[] = "Error reading humidity!";
    Serial.println(errorHumi);
    messages_JsonDoc["message"] = errorHumi;
    send_data_to_MQTT(mqtt_out_topic_messages, &messages_JsonDoc);
  } else {
    snprintf(msg, MSG_BUFFER_SIZE, "Humidity: %.1f %%", event.relative_humidity);
    values_JsonDoc["humi"]["parameter"] = "Humidity";
    values_JsonDoc["humi"]["value"] = event.relative_humidity;
    values_JsonDoc["humi"]["units"] = "%";
  }
  if (!isnan(event.temperature) || !isnan(event.relative_humidity)) {
    send_data_to_MQTT(mqtt_out_topic_values, &values_JsonDoc);
  }
}

void send_data_to_MQTT(char* topic, JsonDocument* message) {
  serializeJson(*message, msg);
  Serial.print("Publish message to MQTT[");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(msg);
  MQTT_Client.publish(topic, msg);
}

//Button press interruption
void isr() {
  interruptorPolsat = true;
  delayMicroseconds(RETARD_INTERRUPCIO);
}
