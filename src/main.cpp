#include <Arduino.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include <MQTT.h>
#include <PubSubClient.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoOTA.h>
#include <crc.h>

#define EEPROM_START 0

#define MOTOR_PIN_1 2
#define MOTOR_PIN_2 3
#define MOTOR_PIN_3 5
#define MOTOR_PIN_4 4
#define REED_UP 12
#define REED_DOWN 13
#define LONG_PRESS_TIMEOUT 500

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

enum class ButtonState {BTN_UP, BTN_DOWN, BTN_NONE, BTN_UP_LONG, BTN_DOWN_LONG, BTN_RELEASE_LONG};
enum class MotorState {DRIVE_UP, DRIVE_DOWN, STOPPED};
enum class DriveDirection {DRIVE_UP, DRIVE_DOWN, DRIVE_NONE};
enum class EdgePosition {TOP, BOTTOM, MIDDLE};

String device_name = "Cover_" + String(ESP.getChipId());
String topicPrefix = "home/covers/" + device_name;

String availability_topic = topicPrefix + "/availability";
String command_topic = topicPrefix + "/set";
String position_topic = topicPrefix + "/position";

boolean setEEPROM = false;
uint32_t memcrc; 
uint8_t *p_memcrc = (uint8_t*)&memcrc;

struct eeprom_data_t {
  char mqtt_server[40];
  char mqtt_port[6];
  char motor_speed[4];
} eeprom_data;

ESP8266WebServer server(80);
WiFiManager wifiManager;

char default_mqtt_server[40] = "";
char default_mqtt_port[6] = "1883";
char default_motor_speed[4] = "300";

uint8_t position_open = 100;
uint8_t position_closed = 0;
uint8_t current_position = 50;
String payload_available = "online";
String payload_not_available = "offline";
String command_open = "OPEN";
String command_close = "CLOSE";
String command_stop = "STOP";

uint16_t motorSpeed;

IPAddress MQTTserver;
// Second and Third pins should be reversed to deal with 28BYJ-48
AccelStepper stepper(AccelStepper::HALF4WIRE, MOTOR_PIN_1, MOTOR_PIN_3, MOTOR_PIN_2, MOTOR_PIN_4);

void readSettingsESP() {
  uint16_t i;
  uint32_t datacrc;
  byte eeprom_data_tmp[sizeof(eeprom_data)];

  EEPROM.begin(sizeof(eeprom_data) + sizeof(memcrc));

  for (i = EEPROM_START; i < EEPROM_START + sizeof(eeprom_data); i++) {
    eeprom_data_tmp[i] = EEPROM.read(i);
  }

  p_memcrc[0] = EEPROM.read(i++);
  p_memcrc[1] = EEPROM.read(i++);
  p_memcrc[2] = EEPROM.read(i++);
  p_memcrc[3] = EEPROM.read(i++);

  datacrc = crc_byte(eeprom_data_tmp, sizeof(eeprom_data_tmp));

  if (memcrc == datacrc) {
    setEEPROM = true;
    memcpy(&eeprom_data, eeprom_data_tmp,  sizeof(eeprom_data));
  } else {
    strncpy(eeprom_data.mqtt_server, default_mqtt_server, sizeof(default_mqtt_server));
    strncpy(eeprom_data.mqtt_port, default_mqtt_port, sizeof(default_mqtt_port));
    strncpy(eeprom_data.motor_speed, default_motor_speed, sizeof(default_motor_speed));
  }
}

void writeSettingsESP() {
  uint16_t i;
  byte eeprom_data_tmp[sizeof(eeprom_data)];

  EEPROM.begin(sizeof(eeprom_data) + sizeof(memcrc));

  memcpy(eeprom_data_tmp, &eeprom_data, sizeof(eeprom_data));

  for (i = EEPROM_START; i < EEPROM_START+sizeof(eeprom_data); i++) {
    EEPROM.write(i, eeprom_data_tmp[i]);
  }
  memcrc = crc_byte(eeprom_data_tmp, sizeof(eeprom_data_tmp));

  EEPROM.write(i++, p_memcrc[0]);
  EEPROM.write(i++, p_memcrc[1]);
  EEPROM.write(i++, p_memcrc[2]);
  EEPROM.write(i++, p_memcrc[3]);

  EEPROM.commit();
}

void configModeCallback (WiFiManager *myWiFiManager) {}

MotorState currentMotorState = MotorState::STOPPED;
DriveDirection prevDriveDirection = DriveDirection::DRIVE_NONE;
EdgePosition rollState;

void setupStepper() {
  stepper.setMaxSpeed(1000.0);
  motorSpeed = atoi(eeprom_data.motor_speed);
}

ICACHE_RAM_ATTR void stop() {
  if (currentMotorState != MotorState::STOPPED) {
    Serial.println("Stop motor");
  }
  currentMotorState = MotorState::STOPPED;
  stepper.stop();
  stepper.disableOutputs();
}

void roll(bool isReversed) {
  int16_t speed = isReversed ? motorSpeed : -motorSpeed;
  if (current_position != 50) {
    current_position = 50;
    char buffer [3];
    mqttClient.publish(position_topic.c_str(), itoa(current_position, buffer, 10));
  }

  stepper.enableOutputs();
  stepper.setSpeed(speed);
  stepper.runSpeed();
}

void rollUp() {
  if (currentMotorState != MotorState::DRIVE_UP) {
    roll(false);
  }
  if (rollState == EdgePosition::BOTTOM) {
    rollState = EdgePosition::MIDDLE;
  }
  currentMotorState = MotorState::DRIVE_UP;
}

void rollDown() {
  if (currentMotorState != MotorState::DRIVE_DOWN) {
    roll(true);
  }
  if (rollState == EdgePosition::TOP) {
    rollState = EdgePosition::MIDDLE;
  }
  currentMotorState = MotorState::DRIVE_DOWN;
}

ICACHE_RAM_ATTR void onOpen() {
  if (currentMotorState == MotorState::DRIVE_UP) {
    stop();
  }

  rollState = EdgePosition::TOP;
  char buffer [3];
  mqttClient.publish(position_topic.c_str(), itoa(position_open, buffer, 10));
  current_position = position_open;
}

ICACHE_RAM_ATTR void onClosed() {
  if (currentMotorState == MotorState::DRIVE_DOWN) {
    stop();
  }

  rollState = EdgePosition::BOTTOM;
  char buffer [3];
  mqttClient.publish(position_topic.c_str(), itoa(position_closed, buffer, 10));
  current_position = position_closed;
}

void setupWiFi() {
  WiFi.hostname(device_name);

  WiFiManager wifiManager;
  // Debug mode on
  // wifiManager.resetSettings();
  wifiManager.setAPCallback(configModeCallback);

  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", eeprom_data.mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", eeprom_data.mqtt_port, 6);
  WiFiManagerParameter custom_motor_speed("speed", "motor speed", eeprom_data.motor_speed, 4);

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_motor_speed);

  if (!wifiManager.autoConnect(device_name.c_str(), "password")) {
    ESP.reset();
    delay(1000);
  }

  strcpy(eeprom_data.mqtt_server, custom_mqtt_server.getValue());
  strcpy(eeprom_data.mqtt_port, custom_mqtt_port.getValue());
  strcpy(eeprom_data.motor_speed, custom_motor_speed.getValue());

  WiFi.enableAP(0);
}

void mqttCallback(char* topic, byte* payload, uint16_t length) {
  String command;
  for (uint16_t i=0; i < length; i++) {
    command += (char)payload[i];
  }

  if (command == command_open) {
    rollUp();
  } else if (command == command_close) {
    rollDown();
  } else if (command == command_stop) {
    stop();
  }
}

void setupOta() {
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
    timer1_detachInterrupt();

    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](uint32_t progress, uint32_t total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
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
}

void setupMQTTClient() {
  // mqttClient.setServer(eeprom_data.mqtt_server, atoi(eeprom_data.mqtt_port));
  mqttClient.setServer("192.168.2.100", 1883);
  mqttClient.setCallback(mqttCallback);
}

void reconnectMqtt() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    Serial.print(" Current state = ");
    Serial.println(mqttClient.state());
    
    // Attempt to connect
    String client_id = device_name + "_";
    client_id += String(random(0xffff), HEX);
    if (mqttClient.connect(client_id.c_str())) {
      Serial.println("connected");

      mqttClient.publish(availability_topic.c_str(), payload_available.c_str());
      mqttClient.subscribe(command_topic.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

ButtonState prevButtonState = ButtonState::BTN_NONE;
ButtonState currentButtonState = ButtonState::BTN_NONE;
uint64_t prevStateChange = millis();
bool longPressed = false;

ButtonState getCurrentButtonState() {
  uint16_t adc = analogRead(A0);

  if (adc > 700) {
    return ButtonState::BTN_UP;
  }

  if (adc > 300) {
    return ButtonState::BTN_DOWN;
  }

  return ButtonState::BTN_NONE;
}

ButtonState detectLongPress(ButtonState btnState) {
  uint64_t now = millis();
  if (prevButtonState != btnState) {
    if (longPressed) {
      btnState = ButtonState::BTN_RELEASE_LONG;
      longPressed = false;
    }

    prevStateChange = now;
    prevButtonState = btnState;
  }
  uint64_t delta = now - prevStateChange;

  if (delta > LONG_PRESS_TIMEOUT) {
    switch (btnState) {
    case ButtonState::BTN_DOWN:
      btnState = ButtonState::BTN_DOWN_LONG;
      longPressed = true;
      break;

    case ButtonState::BTN_UP:
      btnState = ButtonState::BTN_UP_LONG;
      longPressed = true;

    default:
      break;
    }
  }

  return btnState;
}

ButtonState getControlButtonState() {
  ButtonState _state = getCurrentButtonState();
  return detectLongPress(_state);
}

void driveMotor() {
  switch (currentButtonState) {
    case ButtonState::BTN_UP:
      if (rollState != EdgePosition::TOP) rollUp();
      break;

    case ButtonState::BTN_DOWN:
      if (rollState != EdgePosition::BOTTOM) rollDown();
      break;

    case ButtonState::BTN_RELEASE_LONG:
      stop();
      break;

    case ButtonState::BTN_NONE:
    case ButtonState::BTN_DOWN_LONG:
    case ButtonState::BTN_UP_LONG:
      break;
  }

  switch (currentMotorState) {
    case MotorState::DRIVE_UP:
      roll(false);
      break;
    case MotorState::DRIVE_DOWN:
      roll(true);
      break;
    
    default:
      break;
  }
}

void ICACHE_RAM_ATTR onTimerISR() {
  currentButtonState = getControlButtonState();
  timer1_write(600000);//12us
}

void setup() {
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  motorSpeed = 2000.0;
  Serial.println("BEGIN SETUP");

  readSettingsESP();
  setupWiFi();
  writeSettingsESP();

  setupMQTTClient();
  setupOta();

  Serial.println("CONTINUE SETUP");

  // setupStepper();
  stepper.setMaxSpeed(1000.0);
  motorSpeed = 500.0;
  
  timer1_attachInterrupt(onTimerISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(600000);

  attachInterrupt(REED_UP, onOpen, FALLING);
  attachInterrupt(REED_DOWN, onClosed, FALLING);
}

void loop() {
  ArduinoOTA.handle();

  if (!mqttClient.connected()) {
    reconnectMqtt();
  }
  mqttClient.loop();

  driveMotor();
}
