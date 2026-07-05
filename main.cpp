/*
Merged Demo: NMEA2000 to WiFi (NMEA0183 + AIS) AND Raymarine EV-1 Autopilot Remote (433MHz RCSwitch).
Target: ESP32-C3 (Access Point Mode, Fixed IP 192.168.4.1, Port 2222)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

#include <Preferences.h>
#include <nvs_flash.h>
#include <memory>
#include <RCSwitch.h>

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
#define REPORTINTERVAL 10000

const uint16_t ServerPort = 2222; 
const size_t MaxClients = 10;

NMEA2000_esp32_twai NMEA2000; // Ons centrale NMEA2000 object

// Webservers initialiseren
WebServer configServer(80);    // Poort 80 voor de configuratie-pagina
WiFiServer server(ServerPort, MaxClients);

// --- WIFI ACCESS POINT CONFIG ---
// Standaard AP gegevens als er nog niks is ingesteld
const char* defaultAPSSID = "EVOPilotRemote";
const char* defaultAPPass = ""; // open

// --- GLOBALE VARIABELEN & OBJECTEN ---
unsigned long reported;
unsigned int led;
bool loggingEnabled = false;

using tWiFiClientPtr = std::shared_ptr<WiFiClient>;
LinkedList<tWiFiClientPtr> clients;

tN2kDataToNMEA0183 tN2kDataToNMEA0183(&NMEA2000, 0);
tN2kDeviceList *pN2kDeviceList;

int NodeAddress;
Preferences preferences;
RCSwitch mySwitch = RCSwitch();
unsigned long key_time = 0;
short pilotSourceAddress = -1;

int beep_status = 0;
unsigned long beep_on = 0;
unsigned long beep_off = 0;

// --- REMOTE CODES (433 MHz) ---
const unsigned long Key_Minus_10   = 1111001; 
const unsigned long Key_Plus_10    = 1111002;
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
  65288UL,   // Read Seatalk Alarm State (Nodig voor de stuurautomaat)
  65379UL,   // Read Pilot Mode (Nodig voor de stuurautomaat)
  0
};

// Forward declarations
void SendNMEA0183Message(const tNMEA0183Msg &NMEA0183Msg);
void SendBufToClients(const char *buf);
void CheckConnections();
void Handle_AP_Remote();
void handleBeep();
void beepOn(int cnt);
void setupConfigServer();

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
    // Open Preferences met namespace "ap-config"
  preferences.begin("ap-config", true);
  String apSSID = preferences.getString("ap_ssid", defaultAPSSID);
  String apPass = preferences.getString("ap_pass", defaultAPPass);
  preferences.end();

  Serial.println("\n--- ESP32C3 AP Opstarten ---");

  // Zet de ESP strikt in Access Point modus
  WiFi.mode(WIFI_AP);
  
  // Start het Access Point met de (opgeslagen) gegevens
  // Als het wachtwoord korter is dan 8 tekens, start hij als open netwerk
  if (apPass.length() < 8) {
    WiFi.softAP(apSSID.c_str());
    Serial.println("AP gestart ZONDER wachtwoord (wachtwoord te kort, min. 8 tekens).");
  } else {
    WiFi.softAP(apSSID.c_str(), apPass.c_str());
    Serial.println("AP gestart MET wachtwoord.");
  }

  Serial.print("Uitgezonden SSID: ");
  Serial.println(apSSID);
  Serial.print("IP-adres van de ESP32C3: ");
  Serial.println(WiFi.softAPIP());

  // Start servers
  setupConfigServer();
  configServer.begin();

  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(apSSID, apPass);
  delay(500); 

  server.begin();
  server.setNoDelay(true);

  // --- NMEA2000 INITIALISATIE ---
  uint32_t SerialNumber = GetBoardSerialNumber(); // Uniek nummer uit eFuse via BoardSerialNumber.cpp
  uint32_t uniqueID = (uint32_t)(SerialNumber & 0x1FFFFF); // NMEA spec max 21 bits

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

  // Open voorkeuren voor het opgeslagen NMEA Node Adres
  preferences.begin("nvs", false);
  NodeAddress = preferences.getInt("LastNodeAddress", 34);
  preferences.end();

  // Luister naar het netwerk én neem deel als eigen node (nodig om te kunnen zenden naar de EV-1)
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, NodeAddress);

  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.ExtendReceiveMessages(ReceiveMessages);

  // Handlers instellen
  NMEA2000.AttachMsgHandler(&tN2kDataToNMEA0183);
  tN2kDataToNMEA0183.SetSendNMEA0183MessageCallback(SendNMEA0183Message);
  
  // Koppel de Raymarine handler aan de library via een omweg/wrapper als dat nodig is, 
  // maar de library ondersteunt de directe callback:
  NMEA2000.SetMsgHandler(RaymarinePilot::HandleNMEA2000Msg);

  pN2kDeviceList = new tN2kDeviceList(&NMEA2000);
  NMEA2000.Open();

  // --- RF ONTVANGER START ---
  mySwitch.enableReceive(digitalPinToInterrupt(ESP32_RCSWITCH_PIN));
  
  reported = millis();
  beepOn(2); // Korte piep om aan te geven dat setup klaar is
}

//*****************************************************************************
void loop() {
  if (millis() > reported + REPORTINTERVAL) {
    led = led ? OFF : ON;
    digitalWrite(LED_BUILTIN, led);
    reported = millis();
  }
  configServer.handleClient();
  CheckConnections();
  NMEA2000.ParseMessages();
  tN2kDataToNMEA0183.Update();
  Handle_AP_Remote();
  handleBeep();

  // Adreswijziging detectie en opslag in NVS
  int SourceAddress = NMEA2000.GetN2kSource();
  if (SourceAddress != NodeAddress && SourceAddress != 255) { 
    NodeAddress = SourceAddress;
    preferences.begin("nvs", false);
    preferences.putInt("LastNodeAddress", SourceAddress);
    preferences.end();
    Serial.printf("Address Change: New Address=%d\n", SourceAddress);
  }
}

// --- AFHANDELING EN ZOEKEN NAAR EV-1 ---
int getDeviceSourceAddress(String model) {
  if (!pN2kDeviceList->ReadResetIsListUpdated()) return -1;
  for (uint8_t i = 0; i < N2kMaxBusDevices; i++) {
    const tNMEA2000::tDevice *device = pN2kDeviceList->FindDeviceBySource(i);
    if (device == 0) continue;

    String modelVersion = device->GetModelVersion();
    if (modelVersion.indexOf(model) >= 0) {
      beepOn(2); // Apparaat gevonden piep!
      Serial.printf("EV-1 Autopilot gevonden op bronadres: %d\n", device->GetSource());
      return device->GetSource();
    }
  }
  return -2;
}
unsigned long lastSignalTime;
bool isKeyPressed;

void Handle_AP_Remote(void) {
  unsigned long key = 0;

  if (pilotSourceAddress < 0) {
    pilotSourceAddress = getDeviceSourceAddress("EV-1");
  }

  if (mySwitch.available()) {
    key = mySwitch.getReceivedValue();
    Serial.println(key);
    mySwitch.resetAvailable();
  }
  if (key > 0) {
    lastSignalTime = millis(); // Reset timeout, button is still held down
    
    // First time we detect this key (Button Pressed event)
    if (!isKeyPressed) {
      isKeyPressed = true;
      if (key == Key_Standby) {
        beepOn(1);
        if (pilotSourceAddress < 0) return;
        tN2kMsg N2kMsg;
        RaymarinePilot::SetEvoPilotMode(N2kMsg, pilotSourceAddress, PILOT_MODE_STANDBY);
        NMEA2000.SendMsg(N2kMsg);     
      }
      else if (key == Key_Auto) {
        beepOn(1);
        if (pilotSourceAddress < 0) return;
        tN2kMsg N2kMsg;
        RaymarinePilot::SetEvoPilotMode(N2kMsg, pilotSourceAddress, PILOT_MODE_AUTO);
        NMEA2000.SendMsg(N2kMsg);      
      }
      else if (key == Key_Plus_1) {
        beepOn(1);
        if (pilotSourceAddress < 0) return;
        tN2kMsg N2kMsg;
        RaymarinePilot::KeyCommand(N2kMsg, pilotSourceAddress, KEY_PLUS_1);
        NMEA2000.SendMsg(N2kMsg);
      }
      else if (key == Key_Plus_10) {
        beepOn(1);
        if (pilotSourceAddress < 0) return;
        tN2kMsg N2kMsg;
        RaymarinePilot::KeyCommand(N2kMsg, pilotSourceAddress, KEY_PLUS_10);
        NMEA2000.SendMsg(N2kMsg);      
      }
      else if (key == Key_Minus_1) {
        beepOn(1);
        if (pilotSourceAddress < 0) return;
        tN2kMsg N2kMsg;
        RaymarinePilot::KeyCommand(N2kMsg, pilotSourceAddress, KEY_MINUS_1);
        NMEA2000.SendMsg(N2kMsg);      
      }
      else if (key == Key_Minus_10) {
        beepOn(1);
        if (pilotSourceAddress < 0) return;
        tN2kMsg N2kMsg;
        RaymarinePilot::KeyCommand(N2kMsg, pilotSourceAddress, KEY_MINUS_10);
        NMEA2000.SendMsg(N2kMsg);      
      }
    }
  }
  if (isKeyPressed && (millis() - lastSignalTime > 200)) {
    Serial.print("Button RELEASED: ");
    // Reset states for the next press
    isKeyPressed = false;
  }
}

// --- BUZZER / BEEP AFHANDELING ---
void beepOn(int cnt) {
  if (beep_status == 0) {
    Serial.println("beep");
    beep_status = cnt;
  }
}

void handleBeep() {
  if (beep_status > 0) {
    if (!beep_on && (millis() - beep_off >= 200)) {
      digitalWrite(BUZZER_PIN, HIGH);
      beep_on = millis();
      beep_off = 0;
    }
    if (beep_on && (millis() - beep_on >= 200)) {
      digitalWrite(BUZZER_PIN, LOW);
      beep_status--;
      beep_on = 0;
      beep_off = millis();
    }
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
// --- CONFIGURATIE SERVER (Poort 80) ---
void setupConfigServer() {
  // HTML Formulier tonen om de AP-naam aan te passen
  configServer.on("/", HTTP_GET, []() {
    preferences.begin("ap-config", true);
    String currentSSID = preferences.getString("ap_ssid", defaultAPSSID);
    preferences.end();  
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<style>body{font-family:sans-serif;margin:20px;} input{display:block;margin:10px 0;padding:8px;width:100%;max-width:300px;}</style></head>";
    html += "<body><h2>ESP32C3 AP Instellingen</h2>";
    html += "<p>Huidige SSID: <b>" + currentSSID + "</b></p>";
    html += "<form action='/save' method='POST'>";
    html += "Nieuwe AP Naam (SSID):<br><input type='text' name='ap_ssid' placeholder='Nieuwe netwerknaam' required>";
    html += "Nieuw Wachtwoord (min. 8 tekens):<br><input type='password' name='ap_pass' placeholder='Wachtwoord (leeg = open)'>";
    html += "<input type='submit' value='Instellingen Toepassen' style='background:#28a745;color:white;border:0;'>";
    html += "</form></body></html>";
    configServer.send(200, "text/html", html);
  });

  // Nieuwe AP gegevens verwerken en opslaan
  configServer.on("/save", HTTP_POST, []() {
    String newAPSSID = configServer.arg("ap_ssid");
    String newAPPass = configServer.arg("ap_pass");

    if (newAPSSID != "") {
      preferences.begin("ap-config", false);
      preferences.putString("ap_ssid", newAPSSID);
      preferences.putString("ap_pass", newAPPass);
      preferences.end();
      
      configServer.send(200, "text/html", "<h3>AP Instellingen opgeslagen! De ESP32 start nu opnieuw op met de nieuwe netwerknaam...</h3>");
      delay(2000);
      ESP.restart(); // Herstart om de nieuwe AP-naam actief te maken
    } else {
      configServer.send(400, "text/plain", "SSID mag niet leeg zijn.");
    }
  });
}
