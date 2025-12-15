#include "uart.h"
#include "etx_ota_update.h" 
#include "config.h"
#include "supabase.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <FS.h>
#include <LittleFS.h>

// Extern các biến toàn cục từ .ino
extern EspOtaState esp_ota_state;
extern String current_download_url;
extern int current_cmd_id;
extern String ota_last_msg;
extern File firmware_file;

// Biến nội bộ (static)
static EspOtaState next_state_on_ack = OTA_STATE_IDLE;
static unsigned long ack_timeout_start = 0;
static uint32_t current_ack_timeout = 2000;
static uint32_t firmware_total_size = 0;
static uint32_t firmware_total_crc = 0;
static uint8_t data_packet_buffer[ETX_OTA_PACKET_MAX_SIZE];
static uint8_t file_read_buffer[ETX_OTA_DATA_MAX_SIZE];

// Hàm nội bộ
static void send_ota_command(ETX_OTA_CMD_ cmd_type);
static void send_ota_header(uint32_t size, uint32_t crc);
static void send_ota_data_chunk(uint8_t* buffer, uint16_t length);
static uint8_t wait_for_response(uint32_t timeout);
static uint32_t calculateCRC32_of_File(File& file);



uint32_t calculateCRC32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    const uint32_t poly = 0x04C11DB7;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint32_t)data[i] << 24;
        for (int j = 0; j < 8; j++) if (crc & 0x80000000) crc = (crc << 1) ^ poly; else crc <<= 1;
    }
    return crc;
}

static uint32_t calculateCRC32_of_File(File& f) {
    f.seek(0); uint32_t crc = 0xFFFFFFFF; uint8_t b[512];
    const uint32_t poly = 0x04C11DB7;
    while (f.available()) {
        size_t r = f.read(b, 512);
        for (size_t i = 0;i < r;i++) {
            crc ^= (uint32_t)b[i] << 24;
            for (int j = 0;j < 8;j++) if (crc & 0x80000000) crc = (crc << 1) ^ poly; else crc <<= 1;
        }
    }
    f.seek(0); return crc;
}

bool downloadFile(String url) {
    url.replace(" ", "%20");
    WiFiClientSecure client; client.setInsecure(); HTTPClient http;
    if (http.begin(client, url)) {
        int httpCode = http.GET();
        if (httpCode == 200) {
            File file = LittleFS.open("/update.bin", "w");
            if (file) {
                int len = http.getSize();
                uint8_t buff[1024];
                WiFiClient* stream = http.getStreamPtr();
                while (http.connected() && (len > 0 || len == -1)) {
                    size_t size = stream->available();
                    if (size) {
                        int c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));
                        file.write(buff, c);
                        if (len > 0) len -= c;
                    }
                    delay(1);
                }
                file.close(); http.end(); return true;
            }
        } else Serial.println("HTTP Error: " + String(httpCode));
        http.end();
    }
    return false;
}

static void send_ota_command(ETX_OTA_CMD_ cmd_type) {
    ETX_OTA_COMMAND_ cmd_packet;
    cmd_packet.sof = ETX_OTA_SOF;
    cmd_packet.packet_type = ETX_OTA_PACKET_TYPE_CMD;
    cmd_packet.data_len = 1;
    cmd_packet.cmd = cmd_type;
    cmd_packet.crc = calculateCRC32((uint8_t*)&cmd_packet.cmd, 1);
    cmd_packet.eof = ETX_OTA_EOF;
    Serial2.write((uint8_t*)&cmd_packet, sizeof(ETX_OTA_COMMAND_));
}

static void send_ota_header(uint32_t size, uint32_t crc) {
    ETX_OTA_HEADER_ header_packet;
    header_packet.sof = ETX_OTA_SOF;
    header_packet.packet_type = ETX_OTA_PACKET_TYPE_HEADER;
    header_packet.data_len = sizeof(meta_info);
    header_packet.meta_data.package_size = size;
    header_packet.meta_data.package_crc = crc;
    header_packet.meta_data.reserved1 = 0; header_packet.meta_data.reserved2 = 0;
    header_packet.crc = calculateCRC32((uint8_t*)&header_packet.meta_data, sizeof(meta_info));
    header_packet.eof = ETX_OTA_EOF;
    Serial2.write((uint8_t*)&header_packet, sizeof(ETX_OTA_HEADER_));
}

static void send_ota_data_chunk(uint8_t* b, uint16_t l) {
    uint16_t index = 0;
    data_packet_buffer[index++] = ETX_OTA_SOF;
    data_packet_buffer[index++] = ETX_OTA_PACKET_TYPE_DATA;
    *(uint16_t*)&data_packet_buffer[index] = l; index += 2;
    memcpy(&data_packet_buffer[index], b, l); index += l;
    uint32_t crc = calculateCRC32(b, l);
    *(uint32_t*)&data_packet_buffer[index] = crc; index += 4;
    data_packet_buffer[index++] = ETX_OTA_EOF;
    Serial2.write(data_packet_buffer, index);
}

static uint8_t wait_for_response(uint32_t t) {
    ETX_OTA_RESP_ response_packet;
    uint8_t* rx_buffer = (uint8_t*)&response_packet;
    uint8_t rx_index = 0;
    bool sof_found = false;
    unsigned long s = millis();

    while (millis() - s < t) {
        if (Serial2.available()) {
            uint8_t byte_in = Serial2.read();
            if (!sof_found) {
                if (byte_in == ETX_OTA_SOF) { sof_found = true; rx_buffer[0] = byte_in; rx_index = 1; }
                continue;
            }
            if (sof_found && rx_index < sizeof(ETX_OTA_RESP_)) {
                rx_buffer[rx_index++] = byte_in;
            }
            if (rx_index == sizeof(ETX_OTA_RESP_)) {
                if (response_packet.eof != ETX_OTA_EOF) return ETX_OTA_NACK;
                uint32_t calc_crc = calculateCRC32(&response_packet.status, 1);
                if (calc_crc != response_packet.crc) {
                    Serial.printf("CRC Fail! Calc: 0x%X, Recv: 0x%X\n", calc_crc, response_packet.crc);
                    return ETX_OTA_NACK;
                }
                return response_packet.status;
            }
        }
    }
    return ETX_OTA_NACK;
}

void Send_CMD_Check_Version() {
    Serial.println("CMD: Get Version...");
    while (Serial2.available()) { Serial2.read(); }
    Serial2.print("ver");

    unsigned long start_time = millis();
    bool received = false;
    String response = "";

    while (millis() - start_time < 1000) {
        if (Serial2.available()) {
            response = Serial2.readStringUntil('\n');
            received = true;
            break;
        }
    }

    if (received) {
        response.trim();
        Serial.print("STM32 Responded: ");
        Serial.println(response);

        if (response.startsWith("ver:")) {
            Serial.println("[RESULT] STM32 OK!");
            updateCommandStatus(current_cmd_id, "DONE", response);
        } else {
            Serial.println("[RESULT] Invalid Format!");
            updateCommandStatus(current_cmd_id, "ERROR", "Invalid Response: " + response);
        }
    } else {
        Serial.println("[RESULT] STM32 Timeout!");
        updateCommandStatus(current_cmd_id, "ERROR", "Timeout: No response from STM32");
    }
    Serial.println("-----------------------------");
}

void Send_CMD_Reset() {
    Serial.println("CMD: Sending Reset...");
    Serial2.print("rst");
    updateCommandStatus(current_cmd_id, "DONE", "STM32 will be reset");
}

// FOTA STATE MACHINE
void ota_statemachine_handler() {
    if (esp_ota_state == OTA_STATE_IDLE) return;

    switch (esp_ota_state) {

    case OTA_STATE_START_DOWNLOAD:
        Serial.println("[FOTA] Downloading firmware...");
        if (LittleFS.exists("/update.bin")) LittleFS.remove("/update.bin");

        if (downloadFile(current_download_url)) {
            Serial.println("[FOTA] Download OK. Calc CRC...");
            esp_ota_state = OTA_STATE_DOWNLOAD_COMPLETE;
        } else {
            Serial.println("[FOTA] Download FAILED.");
            ota_last_msg = "Download firmware FAILED";
            esp_ota_state = OTA_STATE_FAILED;
        }
        break;

    case OTA_STATE_DOWNLOAD_COMPLETE:
        firmware_file = LittleFS.open("/update.bin", "r");
        if (firmware_file) {
            uint32_t msp_val;
            if (firmware_file.read((uint8_t*)&msp_val, 4) == 4) {
                if (msp_val < 0x20000000 || msp_val > 0x20030000) {
                    Serial.printf("[FOTA] REJECTED! Invalid Header (MSP: 0x%08X)\n", msp_val);
                    ota_last_msg = "Invalid Header File";
                    esp_ota_state = OTA_STATE_FAILED;
                    break;
                }
            } else {
                Serial.println("[FOTA] REJECTED! File too short.");
                ota_last_msg = "File too short";
                esp_ota_state = OTA_STATE_FAILED;
                break;
            }

            firmware_total_size = firmware_file.size();
            firmware_total_crc = calculateCRC32_of_File(firmware_file);
            firmware_file.seek(0);
            Serial.printf("[FOTA] Size: %d, CRC: 0x%X\n", firmware_total_size, firmware_total_crc);

            esp_ota_state = OTA_STATE_TRIGGER_STM32_RESET;
        } else {
            ota_last_msg = "Open file FAILED";
            esp_ota_state = OTA_STATE_FAILED;
        }
        break;

    case OTA_STATE_TRIGGER_STM32_RESET:
        Serial.println("[FOTA] Sending RESET signal to STM32...");
        Serial2.print("ota");
        delay(50);
        Serial.println("[FOTA] Waiting 3s for STM32 reboot...");
        ack_timeout_start = millis();
        delay(3000);
        esp_ota_state = OTA_STATE_SEND_START_CMD;
        break;

    case OTA_STATE_SEND_START_CMD:
        while (Serial2.available()) Serial2.read();
        Serial.println("[FOTA] Sending Bootloader Handshake (START)...");
        send_ota_command(ETX_OTA_CMD_START);
        esp_ota_state = OTA_STATE_WAIT_FOR_ACK;
        next_state_on_ack = OTA_STATE_SEND_HEADER;
        current_ack_timeout = ACK_TIMEOUT_NORMAL;
        ack_timeout_start = millis();
        break;

    case OTA_STATE_SEND_HEADER:
        Serial.println("[FOTA] Sending HEADER...");
        send_ota_header(firmware_total_size, firmware_total_crc);
        esp_ota_state = OTA_STATE_WAIT_FOR_ACK;
        next_state_on_ack = OTA_STATE_SEND_DATA_CHUNK;
        current_ack_timeout = ACK_TIMEOUT_NORMAL;
        ack_timeout_start = millis();
        break;

    case OTA_STATE_SEND_DATA_CHUNK:
        if (firmware_file.available()) {
            size_t bytesRead = firmware_file.read(file_read_buffer, ETX_OTA_DATA_MAX_SIZE);
            send_ota_data_chunk(file_read_buffer, bytesRead);
            int pct = (int)(((float)firmware_file.position() / (float)firmware_total_size) * 100.0);
            if (pct % 10 == 0) Serial.println("[FOTA] Uploading: " + String(pct) + "%");
            esp_ota_state = OTA_STATE_WAIT_FOR_ACK;

            if (firmware_file.position() <= ETX_OTA_DATA_MAX_SIZE) {
                current_ack_timeout = ACK_TIMEOUT_ERASE;
            } else {
                current_ack_timeout = ACK_TIMEOUT_NORMAL;
            }

            if (firmware_file.available()) next_state_on_ack = OTA_STATE_SEND_DATA_CHUNK;
            else next_state_on_ack = OTA_STATE_SEND_END_CMD;

            ack_timeout_start = millis();
        }
        break;

    case OTA_STATE_SEND_END_CMD:
        Serial.println("[FOTA] Sending END...");
        send_ota_command(ETX_OTA_CMD_END);
        esp_ota_state = OTA_STATE_WAIT_FOR_ACK;
        next_state_on_ack = OTA_STATE_SUCCESS;
        current_ack_timeout = VERIFY_TIMEOUT_MS;
        ack_timeout_start = millis();
        break;

    case OTA_STATE_WAIT_FOR_ACK:
        if (Serial2.available() >= sizeof(ETX_OTA_RESP_)) {
            uint8_t st = wait_for_response(100);
            if (st == ETX_OTA_ACK) {
                esp_ota_state = next_state_on_ack;
            } else {
                Serial.println("[FOTA] Error: NACK / Bad Packet / CRC Fail!");
                ota_last_msg = "NACK / Bad Packet / CRC Fail!";
                esp_ota_state = OTA_STATE_FAILED;
            }
        } else if (millis() - ack_timeout_start > current_ack_timeout) {
            Serial.println("[FOTA] Error: ACK Timeout!");
            ota_last_msg = "ACK Timeout!";
            esp_ota_state = OTA_STATE_FAILED;
        }
        break;

    case OTA_STATE_SUCCESS:
        Serial.println("[FOTA] SUCCESS! STM32 Rebooting...");
        updateCommandStatus(current_cmd_id, "DONE", "New Firmware was updated");
        firmware_file.close();
        LittleFS.remove("/update.bin");
        esp_ota_state = OTA_STATE_IDLE;
        break;

    case OTA_STATE_FAILED:
        Serial.println("[FOTA] FAILED! Reason: " + ota_last_msg);
        updateCommandStatus(current_cmd_id, "ERROR", ota_last_msg);
        firmware_file.close();
        LittleFS.remove("/update.bin");
        send_ota_command(ETX_OTA_CMD_ABORT);
        esp_ota_state = OTA_STATE_IDLE;
        break;

    case OTA_STATE_IDLE: break;
    }
}