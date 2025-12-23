#include <WiFi.h>
#include <LittleFS.h>
#include "config.h"
#include "etx_ota_update.h" 
#include "Supabase.h"
#include "uart.h" 


EspOtaState esp_ota_state = OTA_STATE_IDLE;
String current_download_url = "";
int current_cmd_id = -1;
String ota_last_msg = "";
File firmware_file;


unsigned long lastCheckTime = 0;

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

    if (!LittleFS.begin(true)) { Serial.println("LittleFS Error"); return; }

    WiFi.begin(ssid, password);
    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\n[SYSTEM] Online! IP: " + WiFi.localIP().toString());
}

void loop() {
    if (esp_ota_state == OTA_STATE_IDLE) {
        if (millis() - lastCheckTime > CHECK_INTERVAL) {
            checkCloudCommand();
            lastCheckTime = millis();
        }
        checkUartAlerts();
    }
    ota_statemachine_handler();
}