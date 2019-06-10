#include <Arduino.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include <MQTT.h>
#include <PubSubClient.h>
// https://github.com/tzapu/WiFiManager
#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#define EEPROM_START 0

#define MOTOR_PIN_1 1
#define MOTOR_PIN_2 3
#define MOTOR_PIN_3 5
#define MOTOR_PIN_4 4
#define REED_UP 12
#define REED_DOWN 13

enum class DriveDirection {UP, DOWN, NONE};

boolean setEEPROM = false;
uint32_t memcrc; uint8_t *p_memcrc = (uint8_t*)&memcrc;

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

String availability_topic = "/home/covers/" + String(ESP.getChipId()) + "/availability";
String command_topic = "/home/covers/" + String(ESP.getChipId()) + "/set";
String position_topic = "/home/covers/" + String(ESP.getChipId()) + "/position";
String state_topic = "/home/covers/" + String(ESP.getChipId()) + "/state";

String state_open = "open";
String state_closed = "closed";
int position_open = 0;
int position_closed = 100;
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

void configModeCallback (WiFiManager *myWiFiManager) {
}

DriveDirection prevState = DriveDirection::NONE;
bool longPress = false;
int pressStart;

DriveDirection readConrolButtons() {
  int adc = analogRead(A0);
  DriveDirection state;

  if (adc < 500) {
    state = DriveDirection::NONE;
  } else if (adc < 900) {
    state =  DriveDirection::DOWN;
  } else {
    state =  DriveDirection::UP;
  }

  if (prevState != state && state != DriveDirection::NONE) {
    pressStart = millis();
  }

  int now = millis();

  if (prevState == state && state != DriveDirection::NONE) {
    if (now - pressStart > 500) {
      longPress = true;
    }
  }

  if (state == DriveDirection::NONE) {
    if (longPress) {
      longPress = false;
      prevState = state;
      return DriveDirection::NONE;
    } else {
      return prevState;
    }
  }

  prevState = state;

  return state;
}

void setupWiFi() {
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

  String ssid = "Curtain_" + String(ESP.getChipId());

  if (!wifiManager.autoConnect(ssid.c_str(), "password")) {
    ESP.reset();
    delay(1000);
  }

  strcpy(eeprom_data.mqtt_server, custom_mqtt_server.getValue());
  strcpy(eeprom_data.mqtt_port, custom_mqtt_port.getValue());
  strcpy(eeprom_data.motor_speed, custom_motor_speed.getValue());

  WiFi.enableAP(0);
}

void setupStepper() {
  stepper.setMaxSpeed(1000.0);
  motorSpeed = atof(eeprom_data.motor_speed);
  stepper.setSpeed(motorSpeed);
}

void rollUp() {
  stepper.enableOutputs();
  if (stepper.speed() < 0) {
    stepper.setSpeed(motorSpeed);
  }
  stepper.runSpeed();
}

void rollDown() {
  stepper.enableOutputs();
  if (stepper.speed() > 0) {
    stepper.setSpeed(-motorSpeed);
  }
  stepper.runSpeed();
}

void stop() {
  prevState = DriveDirection::NONE;
  stepper.stop();
  stepper.disableOutputs();
}

void setup() {
  readSettingsESP();
  //setupWiFi();
  writeSettingsESP();
  setupStepper();
  attachInterrupt(REED_UP, stop, CHANGE);
  attachInterrupt(REED_DOWN, stop, CHANGE);
}

void loop() {
  DriveDirection direction = readConrolButtons();

  switch (direction)
  {
  case DriveDirection::UP:
    rollUp();
    break;

  case DriveDirection::DOWN:
    rollDown();
    break;
  
  default:
    break;
  }
  //stepper.runSpeed();
  //Serial.println(readConrolButtons());
}
