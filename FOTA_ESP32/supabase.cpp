#include "supabase.h"
#include "config.h"
#include "etx_ota_update.h" 
#include "uart.h"           
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>


extern EspOtaState esp_ota_state;
extern String current_download_url;
extern int current_cmd_id;

void updateCommandStatus(int id, String status, String message) {
    if (id == -1) return;
    HTTPClient http;
    String url = String(SUPABASE_URL) + "/rest/v1/commands?id=eq." + String(id);

    http.begin(url);
    http.addHeader("apikey", SUPABASE_KEY);
    http.addHeader("Authorization", "Bearer " + String(SUPABASE_KEY));
    http.addHeader("Content-Type", "application/json");

    String json;
    if (message.length() > 0) {
        json = "{\"status\": \"" + status + "\", \"response\": \"" + message + "\"}";
    } else {
        json = "{\"status\": \"" + status + "\"}";
    }

    int code = http.PATCH(json);
    if (code != 204 && code != 200) Serial.println("[SUPABASE] Error updating: " + String(code));
    else Serial.println("[SUPABASE] Status -> " + status + (message.length() > 0 ? " | Msg: " + message : ""));

    http.end();
}

void checkCloudCommand() {
    if (WiFi.status() != WL_CONNECTED) return;
    HTTPClient http;
    String url = String(SUPABASE_URL) + "/rest/v1/commands?status=eq.PENDING&select=*&limit=1&order=created_at.asc";

    http.begin(url);
    http.addHeader("apikey", SUPABASE_KEY);
    http.addHeader("Authorization", "Bearer " + String(SUPABASE_KEY));

    int httpCode = http.GET();
    if (httpCode == 200) {
        String payload = http.getString();
        if (payload != "[]" && payload.length() > 5) {
            DynamicJsonDocument doc(2048);
            deserializeJson(doc, payload);
            JsonObject cmd = doc[0];

            int cmd_id = cmd["id"];
            String type = cmd["type"].as<String>();
            String payloadData = cmd["payload"].as<String>();

            Serial.println("\n>>> CMD: " + type + " (ID: " + String(cmd_id) + ")");
            updateCommandStatus(cmd_id, "PROCESSING");
            current_cmd_id = cmd_id;

            if (type == "CHECK") {
                Send_CMD_Check_Version();
            } else if (type == "UPDATE") {
                Serial.println("[ACTION] Start Update...");
                current_download_url = payloadData;
                esp_ota_state = OTA_STATE_START_DOWNLOAD;
            } else if (type == "RESET") {
                Send_CMD_Reset();
            }
        }
    }
    http.end();
}