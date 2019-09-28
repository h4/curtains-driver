#include <Arduino.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include <MQTT.h>
#include <PubSubClient.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#define EEPROM_START 0

#define MOTOR_PIN_1 1
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

boolean setEEPROM = false;
uint32_t memcrc; 
uint8_t *p_memcrc = (uint8_t*)&memcrc;

struct eeprom_data_t {
  char mqtt_server[40];
  char mqtt_port[6];
  char motor_speed[4];
} eeprom_data;

static  uint32_t crc_table[16] = {
  0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
  0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

unsigned long crc_update(unsigned long crc, byte data) {
  byte tbl_idx;
  tbl_idx = crc ^ (data >> (0 * 4));
  crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
  tbl_idx = crc ^ (data >> (1 * 4));
  crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
  return crc;
}

unsigned long crc_byte(byte *b, int len) {
  unsigned long crc = ~0L;
  uint16_t i;

  for (i = 0 ; i < len ; i++)
  {
    crc = crc_update(crc, *b++);
  }
  crc = ~crc;
  return crc;
}

ESP8266WebServer server(80);
WiFiManager wifiManager;

char default_mqtt_server[40] = "";
char default_mqtt_port[6] = "8080";
char default_motor_speed[4] = "300";

String device_name = "Cover_" + String(ESP.getChipId());
String topicPrefix = "home/covers/" + device_name;

String availability_topic = topicPrefix + "/availability";
String command_topic = topicPrefix + "/set";
String position_topic = topicPrefix + "/position";

// TODO: this topics are unusual
String state_topic = topicPrefix + "/buttonState";
String config_topic = topicPrefix + "/config";

int position_open = 100;
int position_closed = 0;
String command_open = "OPEN";
String command_close = "CLOSE";
String command_stop = "STOP";

float motorSpeed;

IPAddress MQTTserver;
// Second and Third pins should be reversed to deal with 28BYJ-48
AccelStepper stepper(AccelStepper::HALF4WIRE, MOTOR_PIN_1, MOTOR_PIN_3, MOTOR_PIN_2, MOTOR_PIN_4);

void readSettingsESP() {
int i;
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
  int i;
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

void setupStepper() {
  stepper.setMaxSpeed(1000.0);
  motorSpeed = atof(eeprom_data.motor_speed);
}

ICACHE_RAM_ATTR void stop() {
  if (currentMotorState != MotorState::STOPPED) {
    Serial.println("Stop motor");
  }
  currentMotorState = MotorState::STOPPED;
  return;
  stepper.stop();
  stepper.disableOutputs();
}

void roll(bool isReversed) {
  int speed = isReversed ? -motorSpeed : motorSpeed;

  Serial.print("Drive with speed ");
  Serial.println(speed);
  return;
  stepper.enableOutputs();
  stepper.setSpeed(speed);
  stepper.runSpeed();
}

void rollUp() {
  if (currentMotorState != MotorState::DRIVE_UP) {
    roll(false);
  }
  currentMotorState = MotorState::DRIVE_UP;
}

void rollDown() {
  if (currentMotorState != MotorState::DRIVE_DOWN) {
    roll(true);
  }
  currentMotorState = MotorState::DRIVE_DOWN;
}

ICACHE_RAM_ATTR void onOpen() {
  stop();
  char buffer [3];
  mqttClient.publish(position_topic.c_str(), itoa(position_open, buffer, 10));
}

ICACHE_RAM_ATTR void onClosed() {
  stop();
  char buffer [3];
  mqttClient.publish(position_topic.c_str(), itoa(position_closed, buffer, 10));
}

void setupWiFi() {
  WiFi.hostname(device_name);

  WiFiManager wifiManager;
  // Debug mode on
  // ifiManager.resetSettings();
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

void mqttCallback(char* topic, byte* payload, int length) {
  String command;
  for (int i=0; i < length; i++) {
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

      String config = "{\"state_topic\":\"" + state_topic + "\",\"availability_topic\":\"" + availability_topic + "\",\"command_topic\":\"" + command_topic + "\",\"position_topic\":\"" + position_topic + "\",\"device_class\":\"cover\",\"name\":\"" + device_name + "\"}";
      
      bool published = mqttClient.publish(config_topic.c_str(), config.c_str());
      Serial.println(config.c_str());
      Serial.println(sizeof(config.c_str()));

      Serial.println("Published Config: ");
      Serial.println(published);
      if (published) {
        Serial.println(config);
      }
      String payload = "online";
      published = mqttClient.publish(availability_topic.c_str(), payload.c_str());
      if (published) {
        Serial.println(payload);
      } else {
        Serial.println("Can't publish");
      }

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

int prevAdc;
ButtonState prevButtonState = ButtonState::BTN_NONE;
ButtonState currentButtonState = ButtonState::BTN_NONE;
long prevStateChange = millis();
bool longPressed = false;

ButtonState getCurrentButtonState() {
  int adc = analogRead(A0);

  if (adc > 700) {
    return ButtonState::BTN_UP;
  }

  if (adc > 300) {
    return ButtonState::BTN_DOWN;
  }

  return ButtonState::BTN_NONE;
}

ButtonState detectLongPress(ButtonState btnState) {
  long now = millis();
  if (prevButtonState != btnState) {
    if (longPressed) {
      btnState = ButtonState::BTN_RELEASE_LONG;
      longPressed = false;
    }

    prevStateChange = now;
    prevButtonState = btnState;
  }
  long delta = now - prevStateChange;

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
      rollUp();
      break;

    case ButtonState::BTN_DOWN:
      rollDown();
      break;

    case ButtonState::BTN_RELEASE_LONG:
      stop();
      break;

    case ButtonState::BTN_NONE:
    case ButtonState::BTN_DOWN_LONG:
    case ButtonState::BTN_UP_LONG:
      break;
  }
}

void ICACHE_RAM_ATTR onTimerISR() {
  currentButtonState = getControlButtonState();
  timer1_write(600000);//12us
}

void setup() {
  Serial.begin(115200);
  motorSpeed = 200;

//  readSettingsESP();
    setupWiFi();
    setupMQTTClient();
//  writeSettingsESP();
//  setupStepper();
  
  timer1_attachInterrupt(onTimerISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(600000);

  attachInterrupt(REED_UP, onOpen, CHANGE);
  attachInterrupt(REED_DOWN, onClosed, CHANGE);
}

void loop() {
  if (!mqttClient.connected()) {
    reconnectMqtt();
  }
  mqttClient.loop();

  driveMotor();
  return;

  switch (prevDriveDirection) {
  case DriveDirection::DRIVE_UP:
    rollUp();
    break;

  case DriveDirection::DRIVE_DOWN:
    rollDown();
    break;
  
  default:
    break;
  }
}
