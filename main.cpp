/*
Merged Demo: NMEA2000 to WiFi (NMEA0183 + AIS) AND Raymarine EV-1 Autopilot Remote (433MHz RCSwitch).
Target: ESP32-C3 (Access Point Mode, Fixed IP 192.168.4.1, Port 2222)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <nvs_flash.h>
#include <memory>
#include <RCSwitch.h>

// FreeRTOS KOPPELING
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// --- NMEA2000 & DRIVER SPECIFIEK VOOR ESP32-C3 ---
#define ESP32_CAN_TX_PIN_SET ESP32_CAN_TX_PIN
#define ESP32_CAN_RX_PIN_SET ESP32_CAN_RX_PIN
#define NMEA2000_CLK_OUT_PIN_SET GPIO_NUM_NC 

#include <NMEA2000_esp32_twai.h> 

// --- PROJECT INCLUDES ---
#include "N2kDataToNMEA0183.h"
#include "List.h"
#include "BoardSerialNumber.h"
#include "RaymarinePilot.h"
#include "N2kDeviceList.h"

// --- PIN DEFINITIES AFSTANDSBEDIENING & LED ---
#define ESP32_RCSWITCH_PIN GPIO_NUM_2  // RXB6 Ontvanger data pin
#define BUZZER_PIN GPIO_NUM_1          // Gewijzigd naar GPIO 1 (veilig op C3)
#define LED_BUILTIN 8                  // Status led C3
#define ON HIGH
#define OFF LOW

#define KEY_DELAY 300
#define BEEP_TIME 200
#define REPORTINTERVAL 10000

const uint16_t ServerPort = 2222; 
const size_t MaxClients = 10;
uint32_t last_ota_time = 0;

NMEA2000_esp32_twai NMEA2000; // Ons centrale NMEA2000 object

// Webservers initialiseren
WebServer configServer(80);    // Poort 80 voor de configuratie-pagina
WiFiServer server(ServerPort, MaxClients);

// --- WIFI ACCESS POINT CONFIG ---
const char* defaultAPSSID = "EVOPilotRemote";
const char* defaultAPPass = ""; // open

// --- GLOBALE VARIABELEN & OBJECTEN ---
unsigned long reported;
unsigned int led;
bool loggingEnabled = false;
bool AISWiFiEnabled = false;

using tWiFiClientPtr = std::shared_ptr<WiFiClient>;
LinkedList<tWiFiClientPtr> clients;

tN2kDataToNMEA0183 tN2kDataToNMEA0183(&NMEA2000, 0);
tN2kDeviceList *pN2kDeviceList;

int NodeAddress;
Preferences preferences;
RCSwitch mySwitch = RCSwitch();
short pilotSourceAddress = -1;

unsigned int beep_status;

// --- REMOTE CODES (433 MHz) ---
const unsigned long Key_Minus_1  = 8338932;
const unsigned long Key_Plus_1   = 8338936;
const unsigned long Key_Auto      = 8338929;
const unsigned long Key_Standby   = 8338930;

// --- SAMENGEVOEGDE TRANSMIT & RECEIVE PGN'S ---
const unsigned long TransmitMessages[] PROGMEM = { 
  126208UL, // Set Pilot Mode
  126720UL, // Send Key Command
  65288UL,  // Send Seatalk Alarm State
  0 
};

const unsigned long ReceiveMessages[] PROGMEM = {
  128267UL,  // Water Depth (Diepte)
  129038UL,  // AIS Class A Position Report
  129039UL,  // AIS Class B Position Report
  129794UL,  // AIS Class A Static Data
  129809UL,  // AIS Class B Static Data Part A
  129810UL,  // AIS Class B Static Data Part B
  65288UL,   // Read Seatalk Alarm State
  65379UL,   // Read Pilot Mode
  0
};

// Forward declarations
void SendNMEA0183Message(const tNMEA0183Msg &NMEA0183Msg);
void SendBufToClients(const char *buf);
void CheckConnections();
void ProcessRemoteKey(unsigned long key);
void setupConfigServer();
void Handle_AP_Remote();
void vRemoteTask(void *pvParameters);
//*****************************************************************************
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  led = OFF;

  Serial.begin(115200);
  delay(1000); 

  Serial.println("--- NMEA2000 Wifi Gateway & AP Remote ---");

  // --- START WIFI ACCESS POINT ---
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true);
  delay(200);
  
  preferences.begin("ap-config", true);
  String apSSID = preferences.getString("ap_ssid", defaultAPSSID);
  String apPass = preferences.getString("ap_pass", defaultAPPass);
  AISWiFiEnabled = preferences.getBool("ap_ais", true);
  preferences.end();

  Serial.println("\n--- ESP32C3 AP Opstarten ---");
  WiFi.mode(WIFI_AP);
  
  if (apPass.length() < 8) {
    WiFi.softAP(apSSID.c_str());
  } else {
    WiFi.softAP(apSSID.c_str(), apPass.c_str());
  }

  setupConfigServer();
  configServer.begin();
ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      if (millis() - last_ota_time > 500) {
        Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
        last_ota_time = millis();
      }
    })
    .onError([](ota_error_t error) {
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
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(apSSID, apPass);
  delay(500); 

  server.begin();
  server.setNoDelay(true);

  // --- NMEA2000 INITIALISATIE ---
  uint32_t SerialNumber = GetBoardSerialNumber();
  uint32_t uniqueID = (uint32_t)(SerialNumber & 0x1FFFFF);

  NMEA2000.SetN2kCANMsgBufSize(100);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.SetN2kCANSendFrameBufSize(150);

  if (loggingEnabled) {
    NMEA2000.SetForwardStream(&Serial);
    NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); 
  } else {
    NMEA2000.EnableForward(false);
  }

  NMEA2000.SetProductInformation("00000001", 100, "N2k->WiFi & Pilot Remote", "2.1.0", "1.0.2.0");
  NMEA2000.SetDeviceInformation(uniqueID, 132, 25, 2046);

  preferences.begin("nvs", false);
  NodeAddress = preferences.getInt("LastNodeAddress", 34);
  preferences.end();

  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, NodeAddress);
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.ExtendReceiveMessages(ReceiveMessages);
  if (AISWiFiEnabled) {
    NMEA2000.AttachMsgHandler(&tN2kDataToNMEA0183);
    tN2kDataToNMEA0183.SetSendNMEA0183MessageCallback(SendNMEA0183Message);
  }
  NMEA2000.SetMsgHandler(RaymarinePilot::HandleNMEA2000Msg);

  pN2kDeviceList = new tN2kDeviceList(&NMEA2000);
  NMEA2000.Open();

  // --- RF ONTVANGER INITIALISATIE ---
  mySwitch.enableReceive(digitalPinToInterrupt(ESP32_RCSWITCH_PIN));
  
  // Start de hoge prioriteitstaak (Prioriteit 5, loop() draait standaard op 1)
  xTaskCreate(vRemoteTask, "RemoteTask", 3072, NULL, 5, NULL);
  
  reported = millis();
}

//*****************************************************************************
void loop() {
  if (millis() > reported + REPORTINTERVAL) {
    led = led ? OFF : ON;
    digitalWrite(LED_BUILTIN, led);
    reported = millis();
  }
  ArduinoOTA.handle();
  configServer.handleClient();
  NMEA2000.ParseMessages();
  if (AISWiFiEnabled) {
    CheckConnections();
    tN2kDataToNMEA0183.Update();
  }
  Handle_AP_Remote();

  // Adreswijziging detectie
  int SourceAddress = NMEA2000.GetN2kSource();
  if (SourceAddress != NodeAddress && SourceAddress != 255) { 
    NodeAddress = SourceAddress;
    preferences.begin("nvs", false);
    preferences.putInt("LastNodeAddress", SourceAddress);
    preferences.end();
    Serial.printf("Address Change: New Address=%d\n", SourceAddress);
  }
}
unsigned long lastSignalTime;

int getDeviceSourceAddress(String model) {
  if (!pN2kDeviceList->ReadResetIsListUpdated()) return -1;
  for (uint8_t i = 0; i < N2kMaxBusDevices; i++) {
    const tNMEA2000::tDevice *device = pN2kDeviceList->FindDeviceBySource(i);
    if (device == 0) continue;

    String modelVersion = device->GetModelVersion();
    if (modelVersion.indexOf(model) >= 0) {
      Serial.printf("EV-1 Autopilot gevonden op bronadres: %d\n", device->GetSource());
      return device->GetSource();
    }
  }
  return -2;
}
unsigned long last_key = 0;

void Handle_AP_Remote(void) {
  unsigned long key = 0;

  if (pilotSourceAddress < 0) {
    pilotSourceAddress = getDeviceSourceAddress("EV-1");
    if (pilotSourceAddress > 0) {
      beep_status = 2;
    } else {
      return;
    }
  }

  if (mySwitch.available()) {
    key = mySwitch.getReceivedValue();
    Serial.println(key);
    mySwitch.resetAvailable();
  }
  if (key > 0 && millis() > lastSignalTime + KEY_DELAY) {
    lastSignalTime = millis(); // Reset timeout, button is still held down
    
    if (key == Key_Standby && key != last_key) {
      beep_status = 1;
      tN2kMsg N2kMsg;
      RaymarinePilot::SetEvoPilotMode(N2kMsg, pilotSourceAddress, PILOT_MODE_STANDBY);
      NMEA2000.SendMsg(N2kMsg);     
    }
    if (key == Key_Auto && key != last_key) {
      beep_status = 1;
      tN2kMsg N2kMsg;
      RaymarinePilot::SetEvoPilotMode(N2kMsg, pilotSourceAddress, PILOT_MODE_AUTO);
      NMEA2000.SendMsg(N2kMsg);      
    }
    if (key == Key_Plus_1) {
      tN2kMsg N2kMsg;
      RaymarinePilot::KeyCommand(N2kMsg, pilotSourceAddress, KEY_PLUS_1);
      NMEA2000.SendMsg(N2kMsg);
    }
    if (key == Key_Minus_1) {
      tN2kMsg N2kMsg;
      RaymarinePilot::KeyCommand(N2kMsg, pilotSourceAddress, KEY_MINUS_1);
      NMEA2000.SendMsg(N2kMsg);      
    }
    last_key = key;
  }
 }

// --- HOGE PRIORITEIT TASK VOOR RF & PIEP TIMING ---
void vRemoteTask(void *pvParameters) {
  unsigned long beep_on = 0;
  unsigned long beep_off = 0;

  for (;;) {
    // 2. Afhandeling Buzzer Timing (Draait met strakke 10ms intervallen)
    if (beep_status > 0) {
      if (!beep_on && (millis() - beep_off >= BEEP_TIME)) {
        digitalWrite(BUZZER_PIN, HIGH);
        beep_on = millis();
        beep_off = 0;
      }
      if (beep_on && (millis() - beep_on >= BEEP_TIME)) {
        digitalWrite(BUZZER_PIN, LOW);
        beep_status--;
        beep_on = 0;
        beep_off = millis();
      }
    }
    // 10ms rust geeft andere lagere taken (WiFi/NMEA) ook ademruimte
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// --- WIFI EN CLIENT STREAM AFHANDELING ---
#define MAX_NMEA0183_MESSAGE_SIZE 100
void SendNMEA0183Message(const tNMEA0183Msg &NMEA0183Msg) {
  char buf[MAX_NMEA0183_MESSAGE_SIZE];
  if (!NMEA0183Msg.GetMessage(buf, MAX_NMEA0183_MESSAGE_SIZE)) return;
  SendBufToClients(buf);
}

void SendBufToClients(const char *buf) {
  for (auto it = clients.begin(); it != clients.end(); it++) {
    if ((*it) != NULL && (*it)->connected()) {
      (*it)->println(buf);
    }
  }
}

void AddClient(WiFiClient &client) {
  if (loggingEnabled) Serial.println("Nieuwe navigatie client verbonden via TCP.");
  clients.push_back(tWiFiClientPtr(new WiFiClient(client)));
}

void StopClient(LinkedList<tWiFiClientPtr>::iterator &it) {
  if (loggingEnabled) Serial.println("Navigatie client verbroken.");
  (*it)->stop();
  it = clients.erase(it);
}

void CheckConnections() {
  WiFiClient client = server.available(); 
  if (client) AddClient(client);

  for (auto it = clients.begin(); it != clients.end(); it++) {
    if ((*it) != NULL) {
      if (!(*it)->connected()) {
        StopClient(it);
      } else {
        if ((*it)->available()) {
          char c = (*it)->read();
          if (c == 0x03) StopClient(it); 
        }
      }
    } else {
      it = clients.erase(it); 
    }
  }
}

// --- CONFIGURATIE SERVER ---
void setupConfigServer() {
  configServer.on("/", HTTP_GET, []() {
    preferences.begin("ap-config", true);
    String currentSSID = preferences.getString("ap_ssid", defaultAPSSID);
    preferences.end();
    String checked = AISWiFiEnabled ? "checked" : "";
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<style>body{font-family:sans-serif;margin:20px;} input{display:block;margin:10px 0;padding:8px;width:100%;max-width:300px;}</style></head>";
    html += "<body><h2>EVOPilotRemote Instellingen</h2>";
    html += "<form action='/save' method='POST'>";
    html += "AP Naam (SSID):<br><input type='text' name='ap_ssid' value='" + currentSSID + "' placeholder='Nieuwe netwerknaam' required>";
    html += "Wachtwoord (min. 8 tekens):<br><input type='password' name='ap_pass' placeholder='Wachtwoord (leeg = open)'>";
    html += "AIS data op poort 2222 <input type='checkbox' name='ap_AIS' " + checked + " >";
    html += "<input type='submit' value='Instellingen Toepassen' style='background:#28a745;color:white;border:0;'>";
    html += "</form></body></html>";
    configServer.send(200, "text/html", html);
  });

  configServer.on("/save", HTTP_POST, []() {
    String newAPSSID = configServer.arg("ap_ssid");
    String newAPPass = configServer.arg("ap_pass");
    bool aisOnWifi = configServer.hasArg("ap_AIS");
  
    if (newAPSSID != "") {
      preferences.begin("ap-config", false);
      preferences.putString("ap_ssid", newAPSSID);
      preferences.putString("ap_pass", newAPPass);
      preferences.putBool("ap_ais", aisOnWifi);
      preferences.end();
      
      configServer.send(200, "text/html", "<h3>AP Instellingen opgeslagen! De ESP32 start nu opnieuw op...</h3>");
      delay(2000);
      ESP.restart();
    } else {
      configServer.send(400, "text/plain", "SSID mag niet leeg zijn.");
    }
  });
}