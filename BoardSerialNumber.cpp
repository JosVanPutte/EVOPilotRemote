#include "BoardSerialNumber.h"
#include <Arduino.h>
#include "esp_mac.h" // Belangrijk! Dit lost errors op rondom esp_read_mac

uint32_t GetBoardSerialNumber() {
  uint8_t mac[6];
  // Haal het officiële, unieke fabrieks-MAC-adres op uit de eFuse
  if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
    // Combineer de laatste 4 bytes van het MAC-adres tot een uniek 32-bit getal
    return ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
  }
  
  // Terugvaloptie (fallback) als de eFuse-uitlezing om een of andere reden faalt
  return 155UL; 
}