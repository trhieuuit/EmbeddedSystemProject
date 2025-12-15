#ifndef ETX_OTA_UPDATE
#define ETX_OTA_UPDATE

#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>

// --- PROTOCOL DEFINITIONS ---
#define ETX_OTA_SOF  0xAA  
#define ETX_OTA_EOF  0xBB  
#define ETX_OTA_ACK  0x00  
#define ETX_OTA_NACK 0x01  

#define ETX_OTA_DATA_MAX_SIZE   1024 
#define ETX_OTA_DATA_OVERHEAD   9
#define ETX_OTA_PACKET_MAX_SIZE (ETX_OTA_DATA_MAX_SIZE + ETX_OTA_DATA_OVERHEAD)

// --- ENUMS ---
typedef enum {
    ETX_OTA_CMD_START = 0,
    ETX_OTA_CMD_END = 1,
    ETX_OTA_CMD_ABORT = 2,
    ETX_OTA_CMD_GET_VERSION = 3,
    ETX_OTA_CMD_RESET = 4
} ETX_OTA_CMD_;

typedef enum {
    ETX_OTA_PACKET_TYPE_CMD = 0,
    ETX_OTA_PACKET_TYPE_DATA = 1,
    ETX_OTA_PACKET_TYPE_HEADER = 2,
    ETX_OTA_PACKET_TYPE_RESPONSE = 3
} ETX_OTA_PACKET_TYPE_;

// --- STRUCTS ---
typedef struct { uint32_t package_size; uint32_t package_crc; uint32_t reserved1; uint32_t reserved2; } __attribute__((packed)) meta_info;
typedef struct { uint8_t sof; uint8_t packet_type; uint16_t data_len; uint8_t cmd; uint32_t crc; uint8_t eof; } __attribute__((packed)) ETX_OTA_COMMAND_;
typedef struct { uint8_t sof; uint8_t packet_type; uint16_t data_len; meta_info meta_data; uint32_t crc; uint8_t eof; } __attribute__((packed)) ETX_OTA_HEADER_;
typedef struct { uint8_t sof; uint8_t packet_type; uint16_t data_len; uint8_t status; uint32_t crc; uint8_t eof; } __attribute__((packed)) ETX_OTA_RESP_;

// --- STATE MACHINE ENUM ---
enum EspOtaState {
    OTA_STATE_IDLE,
    OTA_STATE_START_DOWNLOAD,
    OTA_STATE_DOWNLOAD_COMPLETE,
    OTA_STATE_TRIGGER_STM32_RESET,
    OTA_STATE_SEND_START_CMD,
    OTA_STATE_SEND_HEADER,
    OTA_STATE_SEND_DATA_CHUNK,
    OTA_STATE_SEND_END_CMD,
    OTA_STATE_WAIT_FOR_ACK,
    OTA_STATE_SUCCESS,
    OTA_STATE_FAILED
};

// --- GLOBAL VARIABLES (EXTERN) ---
// Khai báo ở đây để các file .cpp nhìn thấy, định nghĩa thực sự nằm ở .ino
extern EspOtaState esp_ota_state;
extern String current_download_url;
extern int current_cmd_id;
extern String ota_last_msg;
extern File firmware_file;

#endif